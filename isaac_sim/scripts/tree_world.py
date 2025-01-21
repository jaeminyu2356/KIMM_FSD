import omni
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
import omni.isaac.core.utils.numpy.rotations as rot_utils
from pxr import UsdGeom, Gf
from omni.isaac.core.world import World
import yaml
import random
import numpy as np
from ament_index_python.packages import get_package_share_directory

class UnstructuredTreeWorld:
    def __init__(self, stage):
        self.stage = stage
        self.world = World()

        package_name = 'isaac_ml'
        package_share_directory = get_package_share_directory(package_name)
        yaml_path = package_share_directory + "/config/unstructured_orchard.yaml"
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # Ground
        ground_config = config["Ground"]
        self.ground_usdc_path = package_share_directory + ground_config["ground_usdc_path"]
        self.ground_prim_path = ground_config["ground_prim_path"]
        self.ground_position = tuple(ground_config["ground_position"])
        self.ground_scale = tuple(ground_config["ground_scale"])
        
        # Tree
        tree_config = config["Tree"]
        tree_usdc_path1 = package_share_directory + tree_config["tree1_usdc_path"]
        print(tree_usdc_path1)
        tree_usdc_path2 = package_share_directory + tree_config["tree2_usdc_path"]
        tree_usdc_path3 = package_share_directory + tree_config["tree3_usdc_path"]
        tree_usdc_path4 = package_share_directory + tree_config["tree4_usdc_path"]
        self.tree_usdc_list = [tree_usdc_path1, tree_usdc_path2, tree_usdc_path3, tree_usdc_path4]
        
        self.x_max = tree_config["x_max"]
        self.x_min = tree_config["x_min"]
        self.y_max = tree_config["y_max"]
        self.y_min = tree_config["y_min"]
        self.y_step = tree_config["y_step"]
        self.x_spacing_range = tuple(tree_config["x_spacing_range"])
        self.x_start_range = tuple(tree_config["x_start_range"])
        self.x_end_range = [self.x_max - self.x_start_range[1], self.x_max - self.x_start_range[0]]
        
        self.skip_spawn_rate = tree_config["skip_spawn_rate"]
        
    def create_ground_from_usdc(self):
        ground_path = self.ground_prim_path
        grass_usdc_path = self.ground_usdc_path
        grass_position = self.ground_position
        
        prim_utils.create_prim(ground_path, prim_type="Xform")
        ground_prim = self.stage.DefinePrim(ground_path, "Xform")

        ground_prim.GetReferences().AddReference(grass_usdc_path)
        ground_prim.GetAttribute("xformOp:translate").Set(grass_position)

    def create_tree_from_usdc(self, position, tree_id):
        random_spawn = random.randint(0, 100)
        if random_spawn > 0:
            tree_path = f"/World/Trees/Tree_{tree_id}"
            prim_utils.create_prim(tree_path, prim_type="Xform")
            tree_prim = self.stage.DefinePrim(tree_path, "Xform")

            random_index = random.randint(0, 2)
            tree_usdc = self.tree_usdc_list[int(random_index)]
            tree_prim.GetReferences().AddReference(tree_usdc)
            tree_prim.GetAttribute("xformOp:translate").Set(position)
            
            random_yaw = random.randint(0, 360)
            quaternion_np = rot_utils.euler_angles_to_quats(np.array([0, 0, random_yaw]), degrees=True)
            quaternion = Gf.Quatd(quaternion_np[0], Gf.Vec3d(quaternion_np[1], quaternion_np[2], quaternion_np[3]))
            tree_prim.GetAttribute("xformOp:orient").Set(quaternion)
        
        
    def generate_tree_positions(self):
        positions = []
        z_fixed = 0
        line_count = int((self.y_max - self.y_min) / self.y_step)
        
        def generate_line(start_x, start_y, end_x, skip_range=None):
            line_positions = []
            current_x = start_x
            while current_x <= end_x:
                # 각 나무의 실제 위치에 랜덤성 추가
                actual_x = current_x + random.uniform(-1.0, 1.0)  # x축도 약간의 랜덤성
                actual_y = start_y + random.uniform(-2.5, 2.5)  # y축은 더 큰 랜덤성

                # skip_range가 있다면 해당 범위의 나무는 생성하지 않음
                if skip_range is None or not (skip_range[0] <= actual_x <= skip_range[1]):
                    line_positions.append((actual_x, actual_y, z_fixed))
                current_x += random.uniform(*self.x_spacing_range)
            return line_positions
        
        for current_line in range(line_count):
            current_y = self.y_min + current_line * self.y_step
            start_x = self.x_min + random.uniform(*self.x_start_range)
            end_x = random.uniform(*self.x_end_range)
            
            # 분기 확률 (15%)
            if random.random() < 0.15:
                # 메인 라인 생성 (전체)
                main_line = generate_line(start_x, current_y, end_x)
                
                if len(main_line) > 4:  # 최소 5개의 나무가 있을 때만 분기 생성
                    # 분기점 결정 (메인 라인의 중간 즈음에서)
                    branch_start_idx = len(main_line) // 2 + random.randint(-2, 2)
                    if branch_start_idx < len(main_line):
                        branch_start_x = main_line[branch_start_idx][0]  # x 좌표
                        
                        # 분기 방향 결정 (위 또는 아래로)
                        branch_y = current_y + random.choice([-0.4, 0.4]) * self.y_step
                        if self.y_min <= branch_y <= self.y_max:
                            # 분기된 라인의 끝점
                            branch_end_x = end_x + random.uniform(-5.0, 5.0)
                            
                            # 메인 라인에서 나무를 생략할 범위 계산
                            skip_start_x = branch_start_x - random.uniform(1.0, 2.0)
                            skip_end_x = end_x  # 분기점 이후는 모두 생략
                            
                            # 메인 라인 다시 생성 (분기 부분 제외)
                            main_line_positions = generate_line(start_x, current_y, end_x, 
                                                             skip_range=(skip_start_x, skip_end_x))
                            positions.extend(main_line_positions)
                            
                            # 분기된 라인 생성
                            branch_line = generate_line(branch_start_x, branch_y, end_x)
                            positions.extend(branch_line)
                        else:
                            # 분기가 불가능한 경우 원래 라인 그대로 사용
                            positions.extend(main_line)
                    else:
                        # 분기점이 유효하지 않은 경우 원래 라인 그대로 사용
                        positions.extend(main_line)
            else:
                # 일반적인 단일 라인
                positions.extend(generate_line(start_x, current_y, end_x))

        return positions

    def setup_environment(self):
        self.create_ground_from_usdc()
        tree_positions = self.generate_tree_positions()
        for tree_id, position in enumerate(tree_positions):
            print(f"Tree {tree_id} position: {position}")
            self.create_tree_from_usdc(position, tree_id)

def main():
    stage = omni.usd.get_context().get_stage()
    tree_world = UnstructuredTreeWorld(stage)
    tree_world.setup_environment()

if __name__ == "__main__":
    main()

