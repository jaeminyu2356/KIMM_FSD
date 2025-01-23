#!/usr/bin/env python3
import carb
from isaacsim import SimulationApp

# 기본 설정으로 먼저 초기화
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    "gpu_memory_limit": 2*1024*1024*1024,  # 2GB로 제한
    "window_width": 1280,
    "window_height": 720,
})

# 초기화 후에 extension 로드
import omni.kit.commands
import omni.ext
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.utils.extensions import enable_extension

# 상수 정의
REALSENSE_USD_PATH = "/Isaac/Sensors/Intel/RealSense/rsd455.usd"
REALSENSE_DEPTH_PRIM_PATH = "/Camera_Pseudo_Depth"
REALSENSE_RGB_PRIM_PATH = "/Camera_OmniVision_OV9782_Color"

# Physics 활성화를 위한 extension 로드
enable_extension("omni.physx.bundle")

# Sim Start - 필수 extension만 먼저 활성화
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.isaac.core_nodes")
enable_extension("omni.isaac.sensor")

# 나머지 코드는 동일... 