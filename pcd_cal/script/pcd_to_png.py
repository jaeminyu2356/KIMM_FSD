import open3d as o3d
import numpy as np
from PIL import Image
import math

def pcd_to_binary_png_remove_ground(pcd_path, png_path, 
                                    resolution=0.01,
                                    distance_threshold=0.05):
    """
    1) PCD를 읽어와 RANSAC으로 바닥 평면(ground)을 추정 및 제거.
    2) 남은 점(비바닥)을 2D로 투영하여,
       장애물이 있는 픽셀=검정(0), 없는 곳=흰색(255) PNG로 저장.

    :param pcd_path:       입력 PCD 파일 경로
    :param png_path:       저장할 PNG 파일 경로
    :param resolution:     1픽셀당 가로/세로 거리(미터)
    :param distance_threshold: RANSAC으로 평면을 찾을 때의 거리 오차 허용치
                              (바닥면에 속한다고 간주할 포인트 높이 오차)
    """
    # 1) PCD 로드
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        raise ValueError("PCD에 포인트가 없습니다.")

    # 2) 바닥 평면 분리 (RANSAC)
    #    plane_model: [a, b, c, d] -> ax + by + cz + d = 0
    #    inliers: 평면(바닥)에 속하는 점들의 인덱스
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=1000
    )
    [a, b, c, d] = plane_model
    print(f"[INFO] Found plane model: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
    print(f"[INFO] Number of inlier (ground) points: {len(inliers)}")

    # 3) 바닥점(inlier) 제외하고, outlier(비바닥)만 추출
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    points = np.asarray(outlier_cloud.points)
    print(f"[INFO] Number of outlier (non-ground) points: {len(points)}")

    if points.size == 0:
        raise ValueError("바닥을 제외하고 남은 점이 없습니다. 파라미터를 조정해보세요.")

    # 4) (x, y) 최소/최대 범위로 이미지 크기 계산
    min_x, min_y, _ = np.min(points, axis=0)
    max_x, max_y, _ = np.max(points, axis=0)

    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))

    if width <= 0 or height <= 0:
        raise ValueError("포인트 범위가 이상하거나 resolution이 너무 큽니다.")

    # 5) 흰색(255)으로 초기화된 2D 배열 생성
    img = np.full((height, width), 255, dtype=np.uint8)

    # 6) 비바닥 점들을 2D로 투영 → 검정(0) 픽셀로 설정
    for (x, y, z) in points:
        px = int((x - min_x) / resolution)
        py = int((y - min_y) / resolution)

        py_img = height - py - 1  # 상하반전

        if 0 <= px < width and 0 <= py_img < height:
            img[py_img, px] = 0   # 장애물

    # 7) PNG로 저장
    im_pil = Image.fromarray(img)
    im_pil.save(png_path)
    print(f"[INFO] Saved {png_path} (size: {width}x{height})")


if __name__ == "__main__":
    # 예시
    pcd_file = "/root/map/GlobalMap.pcd"
    png_file = "/root/map/projection_noground.png"

    # 해상도(1픽셀 당 0.1m)와 평면 분리 threshold 설정 예시
    resolution = 0.1
    distance_threshold = 0.05  # 5cm 오차 내면 바닥으로 간주

    pcd_to_binary_png_remove_ground(pcd_file, png_file, resolution, distance_threshold)

