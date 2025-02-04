#!/usr/bin/env python3
import open3d as o3d
import numpy as np
from PIL import Image
import math

def create_dem_with_ransac(
    points, cell_size,
    min_x, max_x, min_y, max_y,
    ransac_dist=0.5,      # 평면 거리 임계값
    ransac_n=3,           # RANSAC에서 표본 추출 개수
    num_iterations=50,    # RANSAC 반복 횟수
    inlier_min_count=5,   # 인라이어 최소 개수
    use_median=True       # 바닥 높이: True면 median, False면 min z
):
    """
    셀 단위로 나눠서, 각 셀 내 포인트를 RANSAC으로 바닥 평면 검출 후
    그 인라이어들의 높이를 DEM에 기록한다.
    """
    width = int(math.ceil((max_x - min_x) / cell_size))
    height = int(math.ceil((max_y - min_y) / cell_size))
    dem = np.full((height, width), np.inf, dtype=np.float32)
    cell_point_indices = [[] for _ in range(height * width)]

    # 1) 각 포인트를 셀에 분배
    for i, (x, y, z) in enumerate(points):
        cx = int((x - min_x) / cell_size)
        cy = int((y - min_y) / cell_size)
        if 0 <= cx < width and 0 <= cy < height:
            idx = cy * width + cx
            cell_point_indices[idx].append(i)

    # 2) 셀별 RANSAC
    for cy in range(height):
        for cx in range(width):
            idx = cy * width + cx
            if len(cell_point_indices[idx]) < ransac_n:
                continue

            cell_pts_idx = cell_point_indices[idx]
            cell_pts = points[cell_pts_idx]

            pcd_temp = o3d.geometry.PointCloud()
            pcd_temp.points = o3d.utility.Vector3dVector(cell_pts)

            plane_model, inliers = pcd_temp.segment_plane(
                distance_threshold=ransac_dist,
                ransac_n=ransac_n,
                num_iterations=num_iterations
            )
            inlier_pts = cell_pts[inliers]
            if len(inlier_pts) < inlier_min_count:
                # 인라이어가 너무 적으면 바닥으로 보기 어려움
                continue

            if use_median:
                ground_z = np.median(inlier_pts[:, 2])
            else:
                ground_z = np.min(inlier_pts[:, 2])
            dem[cy, cx] = ground_z

    return dem, min_x, min_y, width, height


def filter_points_remove_ground(
    points, dem,
    dem_min_x, dem_min_y,
    cell_size,
    height_thresh=0.5,        # 바닥+0.5m 이하 → '바닥'으로 제거
    unknown_cell_keep=False
):
    """
    DEM 바닥 + height_thresh 이하 == '바닥'으로 간주하고 제거.
    즉 z가 ground_z + height_thresh 보다 큰 점들만 남긴다.
    unknown_cell_keep=True 이면, 바닥 정보 없는(DEM=inf) 셀의 점은 제거 안 함.
    """
    filtered_indices = []
    width = dem.shape[1]
    height = dem.shape[0]

    for i, (x, y, z) in enumerate(points):
        cx = int((x - dem_min_x) / cell_size)
        cy = int((y - dem_min_y) / cell_size)
        if 0 <= cx < width and 0 <= cy < height:
            ground_z = dem[cy, cx]
            if ground_z < np.inf:
                # 바닥 정보가 있는 셀
                # z > ground_z + height_thresh → 바닥보다 더 높은 점
                if z > ground_z + height_thresh:
                    filtered_indices.append(i)
            else:
                # 바닥 정보 없는 셀(dem=inf), unknown_cell_keep=True면 유지
                if unknown_cell_keep:
                    filtered_indices.append(i)
        else:
            # 셀 범위 밖인 경우 -> 상황에 따라 유지 or 제거
            # 여기서는 일단 유지
            filtered_indices.append(i)

    return filtered_indices


def cluster_dbscan(points, eps=1.0, min_points=30):
    """
    Open3D의 DBSCAN.
    - eps: 포인트가 이웃으로 간주될 거리
    - min_points: 클러스터로 인정될 최소 포인트 수
    """
    pcd_temp = o3d.geometry.PointCloud()
    pcd_temp.points = o3d.utility.Vector3dVector(points)
    labels = np.array(pcd_temp.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    return labels


def trim_cluster_top(points_cluster, z_step=0.2, expand_threshold=2.0):
    """
    한 클러스터의 점들을 z축으로 슬라이스하며,
    x-y 반경이 급격히 증가하는 지점을 '잎'으로 간주해 그 윗부분을 제거.
    - expand_threshold 클수록 쉽게 잎으로 간주되지 않음(덜 제거).
    """
    if len(points_cluster) == 0:
        return points_cluster

    sorted_idx = np.argsort(points_cluster[:, 2])
    p_sorted = points_cluster[sorted_idx]

    z_min = p_sorted[0, 2]
    z_max = p_sorted[-1, 2]

    current_z = z_min
    previous_radius = None
    cutoff_z = None

    while current_z <= z_max:
        z_low = current_z
        z_high = current_z + z_step
        slice_mask = (p_sorted[:, 2] >= z_low) & (p_sorted[:, 2] < z_high)
        slice_pts = p_sorted[slice_mask]
        if len(slice_pts) > 0:
            x_vals = slice_pts[:, 0]
            y_vals = slice_pts[:, 1]
            x_center = np.mean(x_vals)
            y_center = np.mean(y_vals)
            radius = np.mean(np.sqrt((x_vals - x_center) ** 2 + (y_vals - y_center) ** 2))
            if previous_radius is not None and previous_radius > 0:
                ratio = radius / previous_radius
                if ratio >= expand_threshold:
                    cutoff_z = z_low
                    break
            previous_radius = radius
        current_z += z_step

    if cutoff_z is None:
        # 잎을 찾지 못했으면 전체 유지
        return points_cluster
    else:
        # cutoff_z 이상의 점 제거
        keep_mask = (points_cluster[:, 2] < cutoff_z)
        return points_cluster[keep_mask]


def pcd_to_binary_png_dem_and_trim(
    pcd_path,
    png_path,
    # DEM 생성 파라미터
    dem_resolution=1.0,
    ransac_dist=0.5,
    ransac_n=3,
    num_iterations=50,
    inlier_min_count=5,
    use_median=True,

    # 바닥 제거 파라미터
    ground_remove_thresh=0.5,   # 바닥 + 0.5m 이하 = 제거
    unknown_cell_keep=False,

    # DBSCAN 파라미터
    dbscan_eps=1.0,
    dbscan_min_points=30,

    # 상부 제거 파라미터
    z_step=0.2,
    expand_threshold=2.0,

    # 최종 2D PNG 해상도
    png_resolution=0.1
):
    """
    1) PCD 로드
    2) RANSAC 기반 DEM 생성
    3) 바닥 제거(바닥 + ground_remove_thresh 이하 전부 제거)
    4) DBSCAN으로 클러스터링, 각 클러스터 상부 제거
    5) 2D 투영(PNG) 저장 (장애물=0, 빈공간=255)

    => 결과적으로 '바닥보다 일정 높이 이상'인 물체(장애물)들만 남기게 됨.
    """
    # 1) PCD 로드
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        raise ValueError("PCD에 포인트가 없습니다.")
    all_points = np.asarray(pcd.points)

    # 전체 범위
    min_x, min_y, min_z = np.min(all_points, axis=0)
    max_x, max_y, max_z = np.max(all_points, axis=0)

    # 2) DEM 생성 (RANSAC)
    dem, dem_min_x, dem_min_y, dem_w, dem_h = create_dem_with_ransac(
        points=all_points,
        cell_size=dem_resolution,
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        ransac_dist=ransac_dist,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
        inlier_min_count=inlier_min_count,
        use_median=use_median
    )

    # 3) 바닥 제거
    keep_indices = filter_points_remove_ground(
        points=all_points,
        dem=dem,
        dem_min_x=dem_min_x,
        dem_min_y=dem_min_y,
        cell_size=dem_resolution,
        height_thresh=ground_remove_thresh,
        unknown_cell_keep=unknown_cell_keep
    )
    filtered_points = all_points[keep_indices]
    if len(filtered_points) == 0:
        raise ValueError("바닥 제거 후 남은 점이 없습니다. 파라미터를 조정해보세요.")

    # 4) DBSCAN + 상부 제거
    labels = cluster_dbscan(filtered_points, eps=dbscan_eps, min_points=dbscan_min_points)
    unique_labels = set(labels)
    trunk_points_list = []

    for lbl in unique_labels:
        if lbl == -1:
            # 아웃라이어는 무시 (필요시 보존 가능)
            continue
        cluster_mask = (labels == lbl)
        cluster_pts = filtered_points[cluster_mask]
        trimmed_cluster = trim_cluster_top(
            cluster_pts,
            z_step=z_step,
            expand_threshold=expand_threshold
        )
        trunk_points_list.append(trimmed_cluster)

    if len(trunk_points_list) == 0:
        raise ValueError("클러스터링 후 남은 점이 없습니다. 파라미터를 조정해보세요.")

    final_points = np.concatenate(trunk_points_list, axis=0)
    if final_points.size == 0:
        raise ValueError("상부 제거 후 남은 점이 없습니다. 파라미터를 조정해보세요.")

    # 5) 2D 투영 → PNG
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
        py_img = height - py - 1  # y축 상하 반전
        if 0 <= px < width and 0 <= py_img < height:
            img[py_img, px] = 0

    Image.fromarray(img).save(png_path)
    print(f"[INFO] Saved {png_path} (size: {width}x{height}), final_points={len(final_points)}")


if __name__ == "__main__":
    # 예시 파라미터
    pcd_file = "/root/LOAM/GlobalMap.pcd"
    png_file = "/root/LOAM/above_ground.png"

    pcd_to_binary_png_dem_and_trim(
        pcd_file,
        png_file,

        # DEM 생성 관련
        dem_resolution=0.2,       # 셀 크기 (m)
        ransac_dist=0.1,          # RANSAC 거리 임계값
        ransac_n=3,
        num_iterations=50,
        inlier_min_count=5,
        use_median=True,

        # 바닥 제거 (바닥 + ground_remove_thresh 이하 제거)
        ground_remove_thresh=0.2,   # 0.5m 이하 → 제거
        unknown_cell_keep=False,

        # DBSCAN
        dbscan_eps=1.0,
        dbscan_min_points=30,

        # 상부 제거
        z_step=0.2,
        expand_threshold=4.0,

        # 최종 PNG 해상도
        png_resolution=0.035
    )
