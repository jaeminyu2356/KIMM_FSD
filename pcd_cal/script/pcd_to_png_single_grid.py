#!/usr/bin/env python3
import open3d as o3d
import numpy as np
from PIL import Image
import math
import os
import yaml

def process_single_grid(
    points,
    cell_size,
    min_x, max_x, min_y, max_y,
    ransac_dist=0.3,      # 평면 거리 임계값
    ransac_n=5,           # RANSAC에서 표본 추출 개수
    num_iterations=30,    # RANSAC 반복 횟수
    inlier_min_count=10,  # 인라이어 최소 개수
    height_thresh=0.3     # 바닥 + height_thresh 이상만 남김
):
    """
    단일 그리드 크기로 포인트 클라우드를 처리
    """
    width = int(math.ceil((max_x - min_x) / cell_size))
    height = int(math.ceil((max_y - min_y) / cell_size))
    cell_point_indices = [[] for _ in range(height * width)]
    non_ground_points = []

    print(f"[INFO] Grid size: {width}x{height}, Cell size: {cell_size}m")

    # 1) 각 포인트를 셀에 분배
    print("[INFO] Distributing points to cells...")
    for i, (x, y, z) in enumerate(points):
        cx = int((x - min_x) / cell_size)
        cy = int((y - min_y) / cell_size)
        if 0 <= cx < width and 0 <= cy < height:
            idx = cy * width + cx
            cell_point_indices[idx].append(i)

    # 2) 셀별 RANSAC으로 바닥 제거
    print("[INFO] Processing cells with RANSAC...")
    total_cells = width * height
    processed_cells = 0
    
    for cy in range(height):
        for cx in range(width):
            idx = cy * width + cx
            processed_cells += 1
            if processed_cells % 1000 == 0:
                print(f"[INFO] Processed {processed_cells}/{total_cells} cells")
                
            if len(cell_point_indices[idx]) < ransac_n:
                # 포인트가 너무 적은 셀은 모든 점을 비지면으로 간주
                cell_pts_idx = cell_point_indices[idx]
                if cell_pts_idx:  # 빈 셀이 아니면
                    non_ground_points.extend(points[cell_pts_idx])
                continue

            cell_pts_idx = cell_point_indices[idx]
            cell_pts = points[cell_pts_idx]

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
                
                # 법선 벡터가 수직에 가까운지 확인 (바닥 평면 검증)
                normal = plane_model[:3]
                angle = np.arccos(np.abs(np.dot(normal, [0, 0, 1])))
                
                if angle < np.pi/6:  # 30도 이내 - 바닥 평면
                    # 바닥 높이 계산
                    ground_z = np.median(inlier_pts[:, 2])
                    
                    # 바닥보다 높은 점들만 보존
                    for pt_idx in cell_pts_idx:
                        if points[pt_idx][2] > ground_z + height_thresh:
                            non_ground_points.append(points[pt_idx])
                else:
                    # 수직 평면이면 모든 점을 보존
                    non_ground_points.extend(cell_pts)
            else:
                # 평면이 검출되지 않은 경우, 모든 점을 보존
                non_ground_points.extend(cell_pts)

    return np.array(non_ground_points)

def save_map_yaml(png_path, resolution, width, height):
    """
    지도 메타데이터를 YAML 파일로 저장
    """
    yaml_path = png_path.rsplit('.', 1)[0] + '.yaml'
    
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

def pcd_to_binary_png_single_grid(
    pcd_path,
    png_path,
    cell_size=2.0,        # 단일 그리드 크기
    ransac_dist=0.3,      # RANSAC 거리 임계값
    ransac_n=5,           # 샘플 수
    num_iterations=30,    # RANSAC 반복 횟수
    inlier_min_count=10,  # 최소 인라이어 수
    height_thresh=0.3,    # 바닥 위 30cm 이상만 남김
    png_resolution=0.05   # 최종 이미지 해상도
):
    """
    단일 그리드 크기로 처리하는 메인 함수
    """
    print(f"[INFO] Loading point cloud from {pcd_path}...")
    # 1) PCD 로드
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        raise ValueError("PCD에 포인트가 없습니다.")
    all_points = np.asarray(pcd.points)
    print(f"[INFO] Loaded {len(all_points)} points")

    # 전체 범위
    min_x, min_y, min_z = np.min(all_points, axis=0)
    max_x, max_y, max_z = np.max(all_points, axis=0)
    print(f"[INFO] Point cloud bounds: X[{min_x:.2f}, {max_x:.2f}], Y[{min_y:.2f}, {max_y:.2f}], Z[{min_z:.2f}, {max_z:.2f}]")

    # 2) 단일 그리드 처리
    non_ground_points = process_single_grid(
        points=all_points,
        cell_size=cell_size,
        min_x=min_x, max_x=max_x,
        min_y=min_y, max_y=max_y,
        ransac_dist=ransac_dist,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
        inlier_min_count=inlier_min_count,
        height_thresh=height_thresh
    )
    print(f"[INFO] Non-ground points after processing: {len(non_ground_points)}")

    if len(non_ground_points) == 0:
        raise ValueError("지면 제거 후 남은 점이 없습니다. 파라미터를 조정해보세요.")

    # 3) 2D 투영 → PNG
    print("[INFO] Creating 2D projection...")
    width = int(math.ceil((max_x - min_x) / png_resolution))
    height = int(math.ceil((max_y - min_y) / png_resolution))
    print(f"[INFO] Image dimensions: {width}x{height} pixels")

    if width <= 0 or height <= 0:
        raise ValueError("포인트 범위가 이상하거나 png_resolution이 너무 큽니다.")

    # 2D 투영
    print("[INFO] Creating binary image...")
    img = np.full((height, width), 255, dtype=np.uint8)
    for (x, y, z) in non_ground_points:
        px = int((x - min_x) / png_resolution)
        py = int((y - min_y) / png_resolution)
        py_img = height - py - 1  # y축 상하 반전
        if 0 <= px < width and 0 <= py_img < height:
            img[py_img, px] = 0

    print(f"[INFO] Saving image to {png_path}...")
    Image.fromarray(img).save(png_path)
    print(f"[INFO] Saved {png_path} (size: {width}x{height})")
    
    # YAML 파일 생성
    save_map_yaml(png_path, png_resolution, width, height)
    print("[INFO] Process completed successfully!")

if __name__ == "__main__":
    pcd_file = "/root/LOAM/GlobalMap.pcd"
    png_file = "/root/LOAM/above_ground_single_grid.png"

    pcd_to_binary_png_single_grid(
        pcd_file,
        png_file,
        cell_size=1.0,        # 2m x 2m 그리드
        ransac_dist=0.3,      # 30cm 이내 점들을 평면으로 인식
        ransac_n=5,
        num_iterations=30,
        inlier_min_count=10,
        height_thresh=0.3,    # 바닥 위 30cm 이상만 남김
        png_resolution=0.05   # 5cm 해상도
    ) 