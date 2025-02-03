import open3d as o3d
import numpy as np
from PIL import Image
import math

def create_dem(points, cell_size, min_x, max_x, min_y, max_y):
    """
    (x,y) 영역을 cell_size 해상도로 격자 나눠서,
    각 셀의 최소 z를 DEM(바닥 높이)으로 기록한다.
    
    :param points: Nx3 numpy array (x, y, z)
    :param cell_size: DEM 해상도 (예: 1.0 → 1m x 1m)
    :param min_x, max_x, min_y, max_y: 전체 점 구간
    :return:
        dem array (H x W),
        dem_min_x, dem_min_y,  # DEM 시작지점 좌표
        width, height          # DEM 크기
    """

    width = int(math.ceil((max_x - min_x) / cell_size))
    height = int(math.ceil((max_y - min_y) / cell_size))

    # 매우 큰 맵인 경우, width/height도 매우 커질 수 있으니 주의
    # 초기화 (np.inf로 채워두고, 실제 값이 더 작으면 업데이트)
    dem = np.full((height, width), np.inf, dtype=np.float32)

    # 각 포인트가 어느 셀에 속하는지 계산 후, 최소 z 갱신
    for (x, y, z) in points:
        cx = int((x - min_x) / cell_size)
        cy = int((y - min_y) / cell_size)
        if 0 <= cx < width and 0 <= cy < height:
            if z < dem[cy, cx]:
                dem[cy, cx] = z

    return dem, min_x, min_y, width, height

def filter_points_by_dem(points, dem, dem_min_x, dem_min_y, cell_size,
                         height_thresh=2.5):
    """
    DEM(각 셀 최소 z) 기반으로,
    point.z <= dem[cell] + height_thresh 인 점만 남긴다.
    """
    filtered_indices = []
    width = dem.shape[1]
    height = dem.shape[0]

    for i, (x, y, z) in enumerate(points):
        cx = int((x - dem_min_x) / cell_size)
        cy = int((y - dem_min_y) / cell_size)
        if 0 <= cx < width and 0 <= cy < height:
            ground_z = dem[cy, cx]
            # 만약 ground_z가 inf라면(해당 셀에 점이 없던 경우), 
            # 바닥을 모른다고 가정 → 해당 점은 필터링할 수도 있음
            # 필요하면 ground_z == np.inf 일 때 점을 살릴지 말지 결정
            if ground_z < np.inf:
                if z <= ground_z + height_thresh:
                    filtered_indices.append(i)

    return filtered_indices

def cluster_dbscan(points, eps=0.5, min_points=10):
    """
    Open3D의 DBSCAN 함수로 클러스터 라벨을 얻는다.
    :param points: Nx3 numpy array
    :return: cluster_labels (길이 N인 list, -1은 아웃라이어)
    """
    # Open3D PointCloud 객체 생성
    pcd_temp = o3d.geometry.PointCloud()
    pcd_temp.points = o3d.utility.Vector3dVector(points)

    # 클러스터링
    labels = np.array(pcd_temp.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    return labels

def trim_cluster_top(points_cluster, z_step=0.2, expand_threshold=2.0):
    """
    단일 클러스터 점들에 대하여,
    z축으로 슬라이스하면서 x-y 분산(혹은 반경)이 급증하는 지점 위로 '잎'으로 간주해 제거.
    
    :param points_cluster: (M, 3) numpy array (이 클러스터에 속하는 점들)
    :param z_step: 슬라이스 간격(m)
    :param expand_threshold: x-y 반경이 갑자기 (expand_threshold 배) 이상으로 증가하면
                             그 윗부분을 잎으로 간주해 제거
    
    :return: trimmed_points (줄기만 남긴 점들)
    """
    if len(points_cluster) == 0:
        return points_cluster

    # z 기준 정렬
    sorted_idx = np.argsort(points_cluster[:, 2])
    p_sorted = points_cluster[sorted_idx]

    z_min = p_sorted[0, 2]
    z_max = p_sorted[-1, 2]

    # 슬라이스 진행
    current_z = z_min
    previous_radius = None
    cutoff_z = None

    while current_z <= z_max:
        # 현재 슬라이스 범위
        z_low = current_z
        z_high = current_z + z_step

        slice_mask = (p_sorted[:,2] >= z_low) & (p_sorted[:,2] < z_high)
        slice_pts = p_sorted[slice_mask]
        if len(slice_pts) > 0:
            # x-y 분산 or bounding circle 반경 계산
            x_vals = slice_pts[:,0]
            y_vals = slice_pts[:,1]
            x_center = np.mean(x_vals)
            y_center = np.mean(y_vals)
            # 반경 = 평균중심에서의 최대거리 or RMS
            radius = np.mean(np.sqrt((x_vals - x_center)**2 + (y_vals - y_center)**2))

            if previous_radius is not None and previous_radius > 0:
                # 반경이 이전 슬라이스 대비 몇 배 증가했는지
                ratio = radius / previous_radius
                if ratio >= expand_threshold:
                    # 급증 구간으로 판단 → 여기서부터 위쪽은 잎
                    cutoff_z = z_low
                    break

            previous_radius = radius

        current_z += z_step

    if cutoff_z is None:
        # 잎 부분을 못 찾았거나, 계속 변화가 크지 않은 경우 → 전체 유지
        return points_cluster
    else:
        # cutoff_z 이상인 점 제거
        keep_mask = (points_cluster[:,2] < cutoff_z)
        return points_cluster[keep_mask]

def pcd_to_binary_png_dem_and_trim(
    pcd_path, png_path,
    dem_resolution=1.0,
    dem_height_thresh=2.5,
    dbscan_eps=1.0,
    dbscan_min_points=30,
    z_step=0.2,
    expand_threshold=2.0,
    png_resolution=0.1
):
    """
    1) PCD 로드
    2) DEM 기반 바닥 필터 (각 셀 최소 z + dem_height_thresh 이하만 남김)
    3) DBSCAN으로 클러스터링 → 각 클러스터마다 '상부(잎)' 잘라내기
    4) 남은 점들을 2D 투영 → PNG(장애물=0, 빈 공간=255)로 저장

    :param pcd_path: 입력 PCD 경로
    :param png_path: 출력 PNG 경로
    :param dem_resolution: DEM 생성시 (x,y) 해상도 (m)
    :param dem_height_thresh: 바닥 + 이 값(m) 이하만 남기는 필터
    :param dbscan_eps: DBSCAN 검색 반경
    :param dbscan_min_points: DBSCAN 최소 포인트
    :param z_step: 클러스터 상부 제거 시 슬라이스 크기
    :param expand_threshold: x-y 반경이 몇 배 이상 증가 시 잎으로 간주할지
    :param png_resolution: 최종 PNG 해상도 (1픽셀당 m)
    """
    # 1) PCD 로드
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        raise ValueError("PCD에 포인트가 없습니다.")
    all_points = np.asarray(pcd.points)

    # 전체 범위
    min_x, min_y, min_z = np.min(all_points, axis=0)
    max_x, max_y, max_z = np.max(all_points, axis=0)

    # 2) DEM 생성 + 바닥 + dem_height_thresh 이하 필터
    dem, dem_min_x, dem_min_y, dem_w, dem_h = create_dem(
        all_points, dem_resolution, min_x, max_x, min_y, max_y
    )
    keep_indices = filter_points_by_dem(
        all_points, dem, dem_min_x, dem_min_y, dem_resolution,
        height_thresh=dem_height_thresh
    )
    filtered_points = all_points[keep_indices]

    # 3) DBSCAN → 클러스터별 상부 잘라내기
    if len(filtered_points) == 0:
        raise ValueError("DEM 필터 후 남은 점이 없습니다.")

    labels = cluster_dbscan(filtered_points, eps=dbscan_eps, min_points=dbscan_min_points)
    unique_labels = set(labels)
    trunk_points_list = []

    for lbl in unique_labels:
        if lbl == -1:
            # DBSCAN 아웃라이어는 무시
            continue
        cluster_mask = (labels == lbl)
        cluster_pts = filtered_points[cluster_mask]
        # 줄기만 남기고 윗부분 제거
        trimmed_cluster = trim_cluster_top(
            cluster_pts, z_step=z_step, expand_threshold=expand_threshold
        )
        trunk_points_list.append(trimmed_cluster)

    if len(trunk_points_list) == 0:
        raise ValueError("클러스터링 후 남은 점이 없습니다.")

    final_points = np.concatenate(trunk_points_list, axis=0)
    if final_points.size == 0:
        raise ValueError("상부 제거 후 남은 점이 없습니다.")

    # 4) 남은 점 2D 투영 → PNG 저장 (장애물=0, 빈공간=255)
    min_x2, min_y2, _ = np.min(final_points, axis=0)
    max_x2, max_y2, _ = np.max(final_points, axis=0)

    width = int(math.ceil((max_x2 - min_x2) / png_resolution))
    height = int(math.ceil((max_y2 - min_y2) / png_resolution))

    if width <= 0 or height <= 0:
        raise ValueError("포인트 범위가 이상하거나 png_resolution이 너무 큽니다.")

    img = np.full((height, width), 255, dtype=np.uint8)

    for (x, y, z) in final_points:
        px = int((x - min_x2) / png_resolution)
        py = int((y - min_y2) / png_resolution)
        py_img = height - py - 1  # 상하반전
        if 0 <= px < width and 0 <= py_img < height:
            img[py_img, px] = 0

    Image.fromarray(img).save(png_path)
    print(f"[INFO] Saved {png_path} (size: {width}x{height}), final_points={len(final_points)}")


if __name__ == "__main__":
    # 예시 파라미터
    pcd_file = "/root/LOAM/GlobalMap.pcd"
    png_file = "/root/LOAM/projection_noground.png"

    # DEM 해상도, DEM 상에서 바닥+2.5m 이하 필터
    dem_res = 1.0
    dem_height = 2.5

    # DBSCAN 파라미터
    eps_val = 1.0
    min_pts = 30

    # 줄기-잎 분리 파라미터
    z_step_val = 0.2
    expand_thr = 2.0  # x-y 반경이 2배 이상 급증 시 잎으로 판단

    # 최종 PNG 해상도
    png_res = 0.035

    pcd_to_binary_png_dem_and_trim(
        pcd_file, png_file,
        dem_resolution=dem_res,
        dem_height_thresh=dem_height,
        dbscan_eps=eps_val,
        dbscan_min_points=min_pts,
        z_step=z_step_val,
        expand_threshold=expand_thr,
        png_resolution=png_res
    )
