#!/usr/bin/env python3
import open3d as o3d
import numpy as np
from PIL import Image
import math
import os
import yaml

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


def create_dem_multi_resolution(
    points,
    cell_sizes,           # 큰 크기부터 작은 순서로 [4.0, 2.0, 1.0, 0.5] 등
    min_x, max_x, min_y, max_y,
    ransac_dist=0.5,      
    ransac_n=3,           
    num_iterations=50,    
    inlier_min_count=5,   
    use_median=True       
):
    """
    멀티 레벨 DEM 생성. 큰 셀부터 시작해서 점진적으로 작은 셀로 진행.
    각 레벨에서 바닥으로 판정된 점들은 다음 레벨에서 제외됨.
    """
    remaining_points = points.copy()
    ground_points = []
    
    for cell_size in cell_sizes:
        if len(remaining_points) < ransac_n:
            break
            
        # 현재 레벨의 DEM 생성
        width = int(math.ceil((max_x - min_x) / cell_size))
        height = int(math.ceil((max_y - min_y) / cell_size))
        dem = np.full((height, width), np.inf, dtype=np.float32)
        cell_point_indices = [[] for _ in range(height * width)]

        # 포인트를 셀에 분배
        for i, (x, y, z) in enumerate(remaining_points):
            cx = int((x - min_x) / cell_size)
            cy = int((y - min_y) / cell_size)
            if 0 <= cx < width and 0 <= cy < height:
                idx = cy * width + cx
                cell_point_indices[idx].append(i)

        # 셀별 RANSAC
        current_level_ground = []
        remaining_indices = []
        
        for cy in range(height):
            for cx in range(width):
                idx = cy * width + cx
                if len(cell_point_indices[idx]) < ransac_n:
                    # 포인트가 부족한 셀의 점들은 다음 레벨로
                    remaining_indices.extend(cell_point_indices[idx])
                    continue

                cell_pts_idx = cell_point_indices[idx]
                cell_pts = remaining_points[cell_pts_idx]

                pcd_temp = o3d.geometry.PointCloud()
                pcd_temp.points = o3d.utility.Vector3dVector(cell_pts)

                # RANSAC으로 평면 검출
                plane_model, inliers = pcd_temp.segment_plane(
                    distance_threshold=ransac_dist,
                    ransac_n=ransac_n,
                    num_iterations=num_iterations
                )

                if len(inliers) >= inlier_min_count:
                    # 평면이 검출된 경우
                    inlier_pts = cell_pts[inliers]
                    outlier_indices = [cell_pts_idx[i] for i in range(len(cell_pts)) if i not in inliers]
                    
                    # 법선 벡터가 수직에 가까운지 확인 (바닥 평면 검증)
                    normal = plane_model[:3]
                    angle = np.arccos(np.abs(np.dot(normal, [0, 0, 1])))
                    if angle < np.pi/6:  # 30도 이내
                        if use_median:
                            ground_z = np.median(inlier_pts[:, 2])
                        else:
                            ground_z = np.min(inlier_pts[:, 2])
                        dem[cy, cx] = ground_z
                        
                        # 바닥으로 판정된 점들 저장
                        current_level_ground.extend(inlier_pts)
                        # 나머지 점들은 다음 레벨로
                        remaining_indices.extend(outlier_indices)
                    else:
                        # 수직 평면이면 모든 점을 다음 레벨로
                        remaining_indices.extend(cell_pts_idx)
                else:
                    # 평면이 검출되지 않은 경우, 모든 점을 다음 레벨로
                    remaining_indices.extend(cell_pts_idx)

        # 이번 레벨에서 찾은 바닥 점들 저장
        if current_level_ground:
            ground_points.extend(current_level_ground)
            
        # 다음 레벨로 전달할 점들 업데이트
        if remaining_indices:
            remaining_points = remaining_points[remaining_indices]
        else:
            break

    return np.array(ground_points), remaining_points


def clean_binary_image_with_dbscan(
    binary_img,
    eps=3,              # DBSCAN 거리 파라미터 (픽셀 단위)
    min_samples=5,      # 최소 이웃 개수
    min_cluster_size=20 # 최소 클러스터 크기
):
    """
    2D 이미지에서 DBSCAN으로 노이즈(밀도가 낮은 픽셀) 제거
    - binary_img: 장애물=0(검은색), 빈공간=255(흰색)인 이미지
    """
    # 장애물 픽셀(0)의 좌표 추출
    obstacle_points = np.column_stack(np.where(binary_img == 0))
    
    if len(obstacle_points) == 0:
        return binary_img

    # DBSCAN 적용
    from sklearn.cluster import DBSCAN
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(obstacle_points)
    labels = clustering.labels_

    # 클러스터별 크기 계산
    unique_labels = set(labels)
    cluster_sizes = {label: np.sum(labels == label) for label in unique_labels}

    # 결과 이미지 생성 (모두 빈공간으로 초기화)
    cleaned_img = np.full_like(binary_img, 255)

    # 충분히 큰 클러스터의 픽셀만 장애물로 표시
    for label in unique_labels:
        if label == -1:  # 노이즈
            continue
        if cluster_sizes[label] >= min_cluster_size:
            cluster_mask = (labels == label)
            points = obstacle_points[cluster_mask]
            cleaned_img[points[:, 0], points[:, 1]] = 0

    return cleaned_img


def save_map_yaml(png_path, resolution, width, height):
    """
    지도 메타데이터를 YAML 파일로 저장
    - origin: 지도 원점의 실제 좌표 (좌하단 기준)
    """
    yaml_path = png_path.rsplit('.', 1)[0] + '.yaml'
    
    # cpp 파일과 동일한 방식으로 origin 계산
    origin_x = -resolution * width / 2
    origin_y = -resolution * height / 2
    
    yaml_content = {
        'image': os.path.basename(png_path),
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'occupied_thresh': 0.5,
        'free_thresh': 0.2,
        'negate': 0
    }
    
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_content, f, default_flow_style=False)
    
    print(f"[INFO] Saved {yaml_path}")


def pcd_to_binary_png_dem_and_trim(
    pcd_path,
    png_path,
    # DEM 생성 파라미터
    cell_sizes=[4.0, 2.0, 1.0, 0.5],  # 멀티 레벨 DEM 셀 크기
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
    png_resolution=0.1,

    # 2D 이미지 클리닝 파라미터
    clean_eps=3,           # DBSCAN 거리 (픽셀)
    clean_min_samples=5,   # 최소 이웃 개수
    clean_min_size=20,     # 최소 클러스터 크기
):
    """
    멀티 레벨 DEM을 사용하도록 수정된 메인 함수
    """
    # 1) PCD 로드
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        raise ValueError("PCD에 포인트가 없습니다.")
    all_points = np.asarray(pcd.points)

    # 전체 범위
    min_x, min_y, min_z = np.min(all_points, axis=0)
    max_x, max_y, max_z = np.max(all_points, axis=0)

    # 2) 멀티 레벨 DEM 처리
    ground_points, non_ground_points = create_dem_multi_resolution(
        points=all_points,
        cell_sizes=cell_sizes,
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

    if len(non_ground_points) == 0:
        raise ValueError("바닥 제거 후 남은 점이 없습니다. 파라미터를 조정해보세요.")

    # 3) 바닥 제거
    keep_indices = filter_points_remove_ground(
        points=non_ground_points,
        dem=ground_points,
        dem_min_x=min_x,
        dem_min_y=min_y,
        cell_size=cell_sizes[0],
        height_thresh=ground_remove_thresh,
        unknown_cell_keep=unknown_cell_keep
    )
    filtered_points = non_ground_points[keep_indices]
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

    # 초기 2D 투영
    img = np.full((height, width), 255, dtype=np.uint8)
    for (x, y, z) in final_points:
        px = int((x - min_x2) / png_resolution)
        py = int((y - min_y2) / png_resolution)
        py_img = height - py - 1  # y축 상하 반전
        if 0 <= px < width and 0 <= py_img < height:
            img[py_img, px] = 0

    # DBSCAN으로 2D 이미지 클리닝
    cleaned_img = clean_binary_image_with_dbscan(
        img,
        eps=clean_eps,
        min_samples=clean_min_samples,
        min_cluster_size=clean_min_size
    )

    Image.fromarray(cleaned_img).save(png_path)
    print(f"[INFO] Saved {png_path} (size: {width}x{height}), final_points={len(final_points)}")
    
    # YAML 파일 생성
    save_map_yaml(png_path, png_resolution, width, height)


if __name__ == "__main__":
    # 예시 파라미터
    pcd_file = "/root/LOAM/GlobalMap.pcd"
    png_file = "/root/LOAM/above_ground.png"

    pcd_to_binary_png_dem_and_trim(
        pcd_file,
        png_file,

        # DEM 생성 관련
        cell_sizes=[4.0, 2.0, 1.0, 0.5],  # 멀티 레벨 DEM 셀 크기
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

        # 2D 이미지 클리닝
        clean_eps=3,           # DBSCAN 거리 (픽셀)
        clean_min_samples=5,   # 최소 이웃 개수
        clean_min_size=20,     # 최소 클러스터 크기

        # 최종 PNG 해상도
        png_resolution=0.035
    )
