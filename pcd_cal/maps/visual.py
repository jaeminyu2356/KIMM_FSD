import numpy as np
import matplotlib.pyplot as plt

def read_pgm(pgm_path):
    with open(pgm_path, 'rb') as f:
        # PGM 파일 헤더 정보 읽기
        header = f.readline().decode()
        if header.strip() != 'P5':
            raise ValueError('Not a valid PGM file')
        
        # 주석 무시하기
        while True:
            line = f.readline().decode()
            if line[0] != '#':
                break
        
        # 이미지의 너비와 높이 읽기
        dimensions = line.strip().split()
        width, height = int(dimensions[0]), int(dimensions[1])
        
        # 최대 그레이스케일 값 읽기
        max_gray = int(f.readline().decode().strip())
        
        # 이미지 데이터 읽기
        image_data = f.read()
        image = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width))
        
        return image

def plot_pgm(pgm_path):
    image = read_pgm(pgm_path)
    plt.imshow(image, cmap='gray')
    plt.colorbar()
    plt.title("PGM Image")
    plt.show()

# 파일 경로를 지정해주세요 (예: 'path/to/image.pgm')
plot_pgm('map_tree.pgm')

