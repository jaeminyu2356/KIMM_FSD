#!/isaac-sim/kit/python/bin/python3
import carb
from isaacsim import SimulationApp
import os
from ament_index_python.packages import get_package_share_directory
# simulation_app = SimulationApp({"headless": False}) # headless mode
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    "gpu_memory_limit": 8*1024*1024*1024  # 8GB limit
})

REALSENSE_USD_PATH = "/Isaac/Sensors/Intel/RealSense/rsd455.usd"
REALSENSE_DEPTH_PRIM_PATH = "/Camera_Pseudo_Depth"
REALSENSE_RGB_PRIM_PATH = "/Camera_OmniVision_OV9782_Color"

# Sim Start
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.isaac.core_nodes")
enable_extension("omni.isaac.sensor")

# 다음 extension들은 필요한 경우에만 활성화
enable_extension("omni.kit.window.file")
enable_extension("omni.kit.window.property")
enable_extension("omni.kit.window.viewport")

# Basic Python
import numpy as np
import yaml

# Omni packages API
import omni
from omni.isaac.core import SimulationContext
from omni.isaac.core.world import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.ros2_bridge import read_camera_info
from omni.isaac.sensor import Camera, IMUSensor
from omni.isaac.core_nodes.scripts.utils import set_target_prims
import omni.syntheticdata._syntheticdata as sd
from omni.isaac.nucleus import get_assets_root_path
import omni.replicator.core as rep
import omni.graph.core as og
import omni.kit.commands
import usdrt.Sdf
from pxr import Gf, UsdLux, Sdf, UsdPhysics, UsdGeom
from omni.physx.scripts import utils
omni.timeline.get_timeline_interface().play()

# ROS2
from ament_index_python.packages import get_package_share_directory
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

class StartSim(Node):
    def __init__(self) -> None:
        super().__init__('isaac_sim')

        self.package_name = 'isaac_ml'
        self.package_share_directory = get_package_share_directory(self.package_name)

        self.assets_root_path = get_assets_root_path()

        config_file_path = self.package_share_directory + "/config/isaac_sim_settings.yaml"
        with open(config_file_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # World
        world_config = config["World"]
        self.using_simple_ground_plane = world_config["using_simple_ground_plane"]
        self.physics_freq = world_config["physics_freq"]
        self.rendering_freq = world_config["rendering_freq"]
        self.world = World(physics_dt=(1.0 / self.physics_freq), rendering_dt=(1.0 / self.rendering_freq), stage_units_in_meters = 1.0)

        self.city_usd_path = self.package_share_directory + world_config["city_usd_path"]
        print("++++++++++=" + self.city_usd_path + "+++++++++++++++++++")
        self.city_prim_path = world_config["city_prim_path"]
        self.city_name = world_config["city_name"]
        self.city_scale = tuple(world_config["city_scale"])

        self.light_prim_path = world_config["light_prim_path"]
        self.light_intensity = world_config["light_intensity"]
        self.light_colortemperature = world_config["light_colortemperature"]
        self.light_name = world_config["light_name"]

        # Drone
        drone_config = config["Drone"]
        self.drone_is_valid = drone_config["is_valid"]
        self.drone_translation = tuple(drone_config["drone_translation"])
        self.drone_scale = tuple(drone_config["drone_scale"])
        self.drone_orientation = rot_utils.euler_angles_to_quats(np.array(drone_config["drone_orientation"]), degrees=True)
        print(self.drone_orientation)

        self.drone_usd_path = self.package_share_directory + drone_config["drone_usd_path"]
        self.drone_prim_path = drone_config["drone_prim_path"]
        self.drone_name = drone_config["drone_name"]

        # Robot
        robot_config = config["Robot"]
        self.robot_is_valid = robot_config["is_valid"]
        self.robot_prim_path = robot_config["robot_prim_path"]
        self.robot_base_link_prim = robot_config["robot_base_link_prim"]
        self.robot_name = robot_config["robot_name"]
        self.robot_frame_id = robot_config["robot_frame_id"]
        self.usd_path = self.package_share_directory + robot_config["usd_path"]
        self.usd_context = omni.usd.get_context()
        self.stage = self.usd_context.get_stage()

        # 이 부분을 수정
        self.robot_spawn_position = robot_config["robot_spawn_position"]
        self.robot_spawn_rotation = robot_config["robot_spawn_rotation"]
        
        # robot_spawn_pose는 이렇게 생성
        quaternion = rot_utils.euler_angles_to_quats(self.robot_spawn_rotation, degrees=True)
        rotation_matrix = Gf.Matrix3d(Gf.Quatd(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        self.robot_spawn_pose = Gf.Matrix4d().SetRotate(rotation_matrix).SetTranslateOnly(Gf.Vec3d(*self.robot_spawn_position))
        self.graph_prim_path = robot_config["graph_prim_path"]
        self.joint_state_topic_name = robot_config["joint_state_topic_name"]
        self.joint_command_topic_name = robot_config["joint_command_topic_name"]
        self.tf_topic = robot_config["tf_topic"]

        # Camera
        camera_config = config["Camera"]
        self.camera_usd_path = self.assets_root_path + REALSENSE_USD_PATH
        self.camera_prim_path = self.robot_prim_path + camera_config["camera_prim_path"]
        self.camera_optical_prim_path = self.robot_prim_path + camera_config["camera_optical_prim_path"]
        self.camera_freq = camera_config["camera_freq"]
        self.camera_topic_name = camera_config["camera_topic_name"]
        self.camera_frame_id = camera_config["camera_frame_id"]
        self.queue_size = camera_config["queue_size"]

        camera_translation = camera_config["camera_translation"]
        camera_orientation = camera_config["camera_orientation"]
        quaternion = rot_utils.euler_angles_to_quats(camera_orientation, degrees=True)
        camera_rot_matrix = Gf.Matrix3d(Gf.Quatd(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        self.camera_transform = Gf.Matrix4d().SetRotate(Gf.Matrix3d(camera_rot_matrix)).SetTranslateOnly(Gf.Vec3d(*camera_translation))

        # LiDAR
        lidar_config = config["LiDAR"]
        self.lidar_prim_path = self.robot_prim_path + lidar_config["lidar_prim_path"]
        self.lidar_parent = lidar_config["lidar_parent"] if lidar_config["lidar_parent"] is not None else None
        self.lidar_config = lidar_config["lidar_config"]
        self.lidar_translation = tuple(lidar_config["lidar_translation"])
        orientation_temp = rot_utils.euler_angles_to_quats(np.array(lidar_config["lidar_orientation"]), degrees=True)
        self.lidar_orientation = Gf.Quatd(orientation_temp[0]) # Realvalue
        self.lidar_orientation.SetImaginary(orientation_temp[1], orientation_temp[2], orientation_temp[3])

        self.lidar_frame_id = lidar_config["lidar_frame_id"]
        self.lidar_pc_topic = lidar_config["lidar_pc_topic"]
        self.lidar_scan_topic = lidar_config["lidar_scan_topic"]
        self.lidar_frameskip_cnt = lidar_config["lidar_frameskip_cnt"]
        self.lidar_pc_node_name = lidar_config["lidar_pc_node_name"]
        self.lidar_scan_node_name = lidar_config["lidar_scan_node_name"]

        # IMU
        imu_config = config["IMU"]
        self.imu_prim_path = self.robot_prim_path + imu_config["imu_prim_path"]
        self.imu_name = imu_config["imu_name"]
        # self.imu_node_name = imu_config["imu_node_name"]
        self.imu_frequency = imu_config["imu_frequency"]
        self.imu_translation = np.array(imu_config["imu_translation"])
        self.imu_orientation = np.array(imu_config["imu_orientation"])
        self.imu_linear_acceleration_filter_size = imu_config["imu_linear_acceleration_filter_size"]
        self.imu_angular_velocity_filter_size = imu_config["imu_angular_velocity_filter_size"]
        self.imu_orientation_filter_size = imu_config["imu_orientation_filter_size"]
        self.imu_topic_name = imu_config["imu_topic_name"]
        self.imu_frame_id = imu_config["imu_frame_id"]
        # self.imu_step = imu_config["imu_step"]
        
        # GPS
        gps_config = config["GPS"]
        self.gps_topic_name = gps_config["gps_topic_name"]
        self.gps_frame_id = gps_config["gps_frame_id"]
        self.gps_frameskip_cnt = gps_config["gps_frameskip_cnt"]
        self.gps_publish_interval = self.gps_frameskip_cnt
        self.latitude_reference = gps_config["latitude_reference"]
        self.longitude_reference = gps_config["longitude_reference"]
        self.altitude_reference = gps_config["altitude_reference"]
        # ROS2 Imu
        self.imu_pub = self.create_publisher(Imu, self.imu_topic_name, 10)
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.imu_frame_id
        self.step_count = 0
        self.imu_publish_interval = 1

        # ROS2 Robot_gt_val
        # self.odom_gt_pub = self.create_publisher(PoseStamped, "/odom_gt", 10)
        # self.odom_gt_msg = PoseStamped()
        # self.odom_gt_msg.header.frame_id = "base_link"

        self.heading_pub = self.create_publisher(Float64, "/heading", 10)
        self.heading_msg = Float64()

        self.pose_pub = self.create_publisher(Float64MultiArray, "/odom_gt", 10)
        self.pose_msg = Float64MultiArray()
        self.pose_array = [0.0, 0.0]
        
        self.gps_pub = self.create_publisher(NavSatFix, self.gps_topic_name, 10)
        self.gps_msg = NavSatFix()
        self.gps_msg.header.frame_id = self.gps_frame_id
        self.gps_msg.position_covariance_type = 0
        self.gps_msg.position_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gps_msg.status.status = 0
        self.gps_msg.status.service = 0

    def set_world(self):
        if not self.using_simple_ground_plane:
            print(f"Loading city from: {self.city_usd_path}")
            # Add reference to the stage
            add_reference_to_stage(usd_path=self.city_usd_path, prim_path=self.city_prim_path)
            
            # Scale down the city
            city_prim = self.stage.GetPrimAtPath(self.city_prim_path)
            if city_prim:
                print(f"Successfully loaded city at: {self.city_prim_path}")
                from pxr import UsdGeom, Gf, UsdLux  # 여기에 import 추가
                xform = UsdGeom.Xformable(city_prim)
                scale_op = xform.AddScaleOp()
                scale_op.Set(Gf.Vec3d(*self.city_scale))
                print(f"Applied scale: {self.city_scale}")
            else:
                print(f"Failed to find city prim at: {self.city_prim_path}")

            # 조명 설정 추가
            light_prim = UsdLux.DistantLight.Define(self.stage, self.light_prim_path)
            light_prim.CreateIntensityAttr(self.light_intensity)
            light_prim.CreateColorTemperatureAttr(self.light_colortemperature)
        
        if self.drone_is_valid:
            add_reference_to_stage(usd_path=self.drone_usd_path, prim_path=self.drone_prim_path)
            self.drone = XFormPrim(
                prim_path=self.drone_prim_path,
                name=self.drone_name,
                translation=self.drone_translation,
                orientation=self.drone_orientation,
                scale=self.drone_scale
                )
            self.world.scene.add(self.drone)
            
        self.world.add_physics_callback("physics_cb", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size):
        self.step_count += 1
        if (self.step_count % self.imu_publish_interval == 0) and self.robot_is_valid:
            self.update_imu_data()
            self.update_odom_gt()
            self.imu_pub.publish(self.imu_msg)
            self.pose_pub.publish(self.pose_msg)
            self.heading_pub.publish(self.heading_msg)
            # self.odom_gt_pub.publish(self.odom_gt_msg)
            
        if (self.step_count % self.gps_publish_interval == 0) and self.robot_is_valid:
            self.update_gps_data()
            self.gps_pub.publish(self.gps_msg)
            self.step_count = 0
    
    def update_odom_gt(self):
        robot_prim = self.stage.GetPrimAtPath(self.robot_base_link_prim)
        robot_xformable = UsdGeom.Xformable(robot_prim)
        robot_transform = robot_xformable.GetLocalTransformation(0)  # 0은 현재 프레임의 변환을 의미합니다.

        # 위치와 회전 정보 추출
        self.robot_position = robot_transform.ExtractTranslation()
        self.robot_rotation = robot_transform.ExtractRotation()

        self.pose_array[0] = float(self.robot_position[0])
        self.pose_array[1] = float(self.robot_position[1])

        euler_angles = self.robot_rotation.Decompose(Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))

        self.heading_msg.data = np.radians(euler_angles[2])
        self.pose_msg.data = self.pose_array

        # self.odom_gt_msg.pose.position.x = robot_position[0]
        # self.odom_gt_msg.pose.position.y = robot_position[1]
        # self.odom_gt_msg.pose.position.z = robot_position[2]

        # quat_imaginary = robot_rotation.GetQuat().GetImaginary()  # (x, y, z)
        # quat_real = robot_rotation.GetQuat().GetReal()  # w
        # self.odom_gt_msg.pose.orientation.x = quat_imaginary[0]
        # self.odom_gt_msg.pose.orientation.y = quat_imaginary[1]
        # self.odom_gt_msg.pose.orientation.z = quat_imaginary[2]
        # self.odom_gt_msg.pose.orientation.w = quat_real

        # self.odom_gt_msg.header.stamp = self.timestamp

    def update_gps_data(self):
        # Constants for Earth WGS84
        a = 6378137.0  # Equatorial radius in meters
        b = 6356752.3  # Polar radius in meters
        ab = a * b

        # Calculate heading sine and cosine
        heading_offset = 0
        heading_sine = np.sin(heading_offset)
        heading_cosine = np.cos(heading_offset)

        # Calculate latitude and longitude reference in radians
        latitude_ref_rad = np.radians(self.latitude_reference)
        longitude_ref_rad = np.radians(self.longitude_reference)

        # Calculate radius of curvature
        cos_lat = np.cos(latitude_ref_rad)
        sin_lat = np.sin(latitude_ref_rad)
        denom = (a * cos_lat) ** 2 + (b * sin_lat) ** 2
        radius_meridional = ab ** 2 / denom / np.sqrt(denom)
        radius_normal = a ** 2 / np.sqrt(denom)

        # Convert local coordinates to spherical
        east = self.robot_position[0] * heading_cosine - self.robot_position[1] * heading_sine
        north = self.robot_position[0] * heading_sine + self.robot_position[1] * heading_cosine

        delta_latitude = north / radius_meridional
        delta_longitude = east / radius_normal

        # Update GPS message
        self.gps_msg.latitude = self.latitude_reference + np.degrees(delta_latitude)
        self.gps_msg.longitude = self.longitude_reference + np.degrees(delta_longitude)
        self.gps_msg.altitude = self.altitude_reference + self.robot_position[2]

    def update_imu_data(self):
        imu_data = self.imu.get_current_frame()

        robot_pose_orientation = imu_data['orientation'].astype(float)
        robot_lin_acc = imu_data['lin_acc'].astype(float)
        robot_ang_vel = imu_data['ang_vel'].astype(float)
        sim_time = imu_data['time']

        seconds = int(sim_time)
        nanoseconds = int((sim_time - seconds) * 1e9)
        self.timestamp = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
        self.imu_msg.header.stamp = self.timestamp

        self.imu_msg.orientation.x = robot_pose_orientation[1]
        self.imu_msg.orientation.y = robot_pose_orientation[2]
        self.imu_msg.orientation.z = robot_pose_orientation[3]
        self.imu_msg.orientation.w = robot_pose_orientation[0]

        self.imu_msg.linear_acceleration.x = robot_lin_acc[0]
        self.imu_msg.linear_acceleration.y = robot_lin_acc[1]
        self.imu_msg.linear_acceleration.z = robot_lin_acc[2]

        self.imu_msg.angular_velocity.x = robot_ang_vel[0]
        self.imu_msg.angular_velocity.y = robot_ang_vel[1]
        self.imu_msg.angular_velocity.z = robot_ang_vel[2]

    def spawn_robot(self):
        if self.robot_is_valid:
            # Add reference to the stage
            add_reference_to_stage(usd_path=self.usd_path, prim_path=self.robot_prim_path)
            
            # Print debug info
            print(f"Spawning robot at position: {self.robot_spawn_position}")
            print(f"Robot USD path: {self.usd_path}")
            
            # USD 파일 존재 확인
            if not os.path.exists(self.usd_path):
                raise FileNotFoundError(f"Robot USD file not found: {self.usd_path}")
            
            # Define the robot prim
            prim = self.stage.DefinePrim(self.robot_prim_path, "Xform")
            prim.GetReferences().AddReference(self.usd_path)

            # Apply transform
            omni.kit.commands.execute(
                "TransformPrimCommand",
                path=prim.GetPath(),
                old_transform_matrix=None,
                new_transform_matrix=self.robot_spawn_pose,
            )

            # Add to scene after transform
            self.robot = Robot(prim_path=self.robot_prim_path, name=self.robot_name)
            self.world.scene.add(self.robot)

            # Get the base_link prim and validate
            base_link_prim = self.stage.GetPrimAtPath(self.robot_base_link_prim)
            
            # Debug: Print prim path and check if it exists
            print(f"Checking base_link_prim at path: {self.robot_base_link_prim}")
            print(f"Base link prim exists: {base_link_prim.IsValid() if base_link_prim else False}")
            
            # Print all available prims under robot path for debugging
            robot_prim = self.stage.GetPrimAtPath(self.robot_prim_path)
            print("\nAvailable prims under robot:")
            for child in robot_prim.GetChildren():
                print(f"- {child.GetPath()}")

            # Only try to apply mass if prim exists
            if base_link_prim and base_link_prim.IsValid():
                try:
                    mass_api = UsdPhysics.MassAPI.Apply(base_link_prim)
                    mass_api.CreateMassAttr(700)
                    print(f"Successfully applied MassAPI to prim at {self.robot_base_link_prim}.")
                except Exception as e:
                    print(f"Failed to apply MassAPI: {e}")
            else:
                print(f"Warning: Could not find valid prim at {self.robot_base_link_prim}")

    def set_camera(self):
        add_reference_to_stage(usd_path=self.camera_usd_path, prim_path=self.robot_prim_path)

        # Caculate base_link to camera_link transformation
        camera_link_prim = self.stage.GetPrimAtPath(self.camera_prim_path)
        camera_link_xformable = UsdGeom.Xformable(camera_link_prim)
        camera_link_transform = camera_link_xformable.GetLocalTransformation(0)

        camera_link_translation = camera_link_transform.ExtractTranslation()
        camera_link_rotation = camera_link_transform.ExtractRotation()
        camera_current_T = Gf.Matrix4d().SetRotate(Gf.Matrix3d(camera_link_rotation)).SetTranslateOnly(Gf.Vec3d(*camera_link_translation))

        # Apply relative trans 
        last_camera_transform = self.camera_transform * camera_current_T 

        prim = self.stage.GetPrimAtPath(self.camera_optical_prim_path)
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=prim.GetPath(),
            new_transform_matrix=last_camera_transform,
        )

        from_prim = self.stage.GetPrimAtPath(self.camera_optical_prim_path)
        to_prim = self.stage.GetPrimAtPath(self.camera_prim_path)
        omni.kit.commands.execute(
            "CreateJointCommand",
            stage=self.stage,
            joint_type="Fixed",
            from_prim=from_prim,
            to_prim=to_prim,
        )

    def publish_camera_info(self):
        # The following code will link the camera's render product and publish the data to the specified topic name.
        render_product = self.camera._render_product_path
        step_size = int(60/self.camera_freq)
        topic_name = self.camera_topic_name+"/camera_info"
        queue_size = self.queue_size
        node_namespace = ""
        # frame_id = self.camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.
        frame_id = self.camera_frame_id

        writer = rep.writers.get("ROS2PublishCameraInfo")
        camera_info = read_camera_info(render_product_path=render_product)
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name,
            width=camera_info["width"],
            height=camera_info["height"],
            projectionType=camera_info["projectionType"],
            k=camera_info["k"].reshape([1, 9]),
            r=camera_info["r"].reshape([1, 9]),
            p=camera_info["p"].reshape([1, 12]),
            physicalDistortionModel=camera_info["physicalDistortionModel"],
            physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
        )
        writer.attach([render_product])

        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            "PostProcessDispatch" + "IsaacSimulationGate", render_product
        )

        # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
        return

    def publish_rgb(self):
        # The following code will link the camera's render product and publish the data to the specified topic name.
        render_product = self.camera._render_product_path
        step_size = int(60/self.camera_freq)
        topic_name = self.camera_topic_name+"/image_raw"
        queue_size = self.queue_size
        node_namespace = ""
        # frame_id = self.camera.prim_path.split("/")[-1] # This matches what the TF tree is publishing.
        frame_id = self.camera_frame_id

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace=node_namespace,
            queueSize=queue_size,
            topicName=topic_name
        )
        writer.attach([render_product])

        # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            rv + "IsaacSimulationGate", render_product
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

        return

    def set_lidar(self):
        _, self.lidar = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=self.lidar_prim_path,
            parent=self.lidar_parent,
            config=self.lidar_config,
            translation=self.lidar_translation,
            orientation=self.lidar_orientation
        )

    def set_imu(self):
        self.imu = IMUSensor(
            prim_path=self.imu_prim_path,
            name=self.imu_name,
            frequency=self.imu_frequency,
            translation=self.imu_translation,
            orientation=self.imu_orientation,
            linear_acceleration_filter_size = self.imu_linear_acceleration_filter_size,
            angular_velocity_filter_size = self.imu_angular_velocity_filter_size,
            orientation_filter_size = self.imu_orientation_filter_size,
        )

    def ros2_bridge(self):
        og.Controller.edit(
            {"graph_path": self.graph_prim_path, "evaluator_name": "execution",
             "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ]
            },
        )
    
    def ros2_camera(self):
        og.Controller.edit(
            self.graph_prim_path,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("createrenderproductRgb", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("createrenderproductDepth", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
                    ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    (self.graph_prim_path+"/OnPlaybackTick.outputs:tick", "createrenderproductRgb.inputs:execIn"),
                    (self.graph_prim_path+"/OnPlaybackTick.outputs:tick", "createrenderproductDepth.inputs:execIn"),
                    (self.graph_prim_path+"/Context.outputs:context", "cameraHelperRgb.inputs:context"),
                    (self.graph_prim_path+"/Context.outputs:context", "cameraHelperInfo.inputs:context"),
                    (self.graph_prim_path+"/Context.outputs:context", "cameraHelperDepth.inputs:context"),
                    ("createrenderproductRgb.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("createrenderproductRgb.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    ("createrenderproductDepth.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                    ("createrenderproductRgb.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                    ("createrenderproductRgb.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ("createrenderproductDepth.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # ("createrenderproductRgb.inputs:height", 1200),
                    # ("createrenderproductRgb.inputs:width", 1920),
                    # ("createrenderproductDepth.inputs:height", 1200),
                    # ("createrenderproductDepth.inputs:width", 1920),
                    ("createrenderproductRgb.inputs:height", 480),
                    ("createrenderproductRgb.inputs:width", 640),
                    ("createrenderproductDepth.inputs:height", 480),
                    ("createrenderproductDepth.inputs:width", 640),
                    ("cameraHelperRgb.inputs:frameId", self.camera_frame_id),
                    ("cameraHelperRgb.inputs:topicName", self.camera_topic_name+"/image_raw"),
                    ("cameraHelperRgb.inputs:frameSkipCount", self.lidar_frameskip_cnt),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    ("cameraHelperInfo.inputs:frameId", self.camera_frame_id),
                    ("cameraHelperInfo.inputs:topicName", self.camera_topic_name+"/camera_info"),
                    ("cameraHelperDepth.inputs:frameId", self.camera_frame_id),
                    ("cameraHelperDepth.inputs:topicName", self.camera_topic_name+"/depth_raw"),
                    ("cameraHelperDepth.inputs:frameSkipCount", self.lidar_frameskip_cnt),
                    ("cameraHelperDepth.inputs:type", "depth"),
                    ("createrenderproductRgb.inputs:cameraPrim", [usdrt.Sdf.Path(self.camera_optical_prim_path + REALSENSE_DEPTH_PRIM_PATH)]),
                    ("createrenderproductDepth.inputs:cameraPrim", [usdrt.Sdf.Path(self.camera_optical_prim_path + REALSENSE_RGB_PRIM_PATH)]),
                ],
            },
        )
        # self.publish_camera_info()
        # self.publish_rgb()

    def ros2_lidar(self):    
        og.Controller.edit(
            self.graph_prim_path,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("IsaacCreateRender", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("IsaacRunOneFrame", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                    ("RTXLidarPointCloudPub", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                    ("RTXLidarLaserScanPub", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    (self.graph_prim_path+"/OnPlaybackTick.outputs:tick", "IsaacRunOneFrame.inputs:execIn"),
                    ("IsaacRunOneFrame.outputs:step", "IsaacCreateRender.inputs:execIn"),
                    (self.graph_prim_path+"/Context.outputs:context", "RTXLidarPointCloudPub.inputs:context"),
                    (self.graph_prim_path+"/Context.outputs:context", "RTXLidarLaserScanPub.inputs:context"),
                    ("IsaacCreateRender.outputs:execOut", "RTXLidarPointCloudPub.inputs:execIn"),
                    ("IsaacCreateRender.outputs:renderProductPath", "RTXLidarPointCloudPub.inputs:renderProductPath"),
                    ("IsaacCreateRender.outputs:execOut", "RTXLidarLaserScanPub.inputs:execIn"),
                    ("IsaacCreateRender.outputs:renderProductPath", "RTXLidarLaserScanPub.inputs:renderProductPath"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("RTXLidarPointCloudPub.inputs:topicName", self.lidar_pc_topic),
                    ("RTXLidarPointCloudPub.inputs:frameId", self.lidar_frame_id),
                    ("RTXLidarPointCloudPub.inputs:fullScan", True),
                    ("RTXLidarPointCloudPub.inputs:nodeNamespace", self.lidar_pc_node_name),
                    ("RTXLidarPointCloudPub.inputs:type", "point_cloud"),
                    ("RTXLidarPointCloudPub.inputs:frameSkipCount", self.lidar_frameskip_cnt),

                    ("RTXLidarLaserScanPub.inputs:topicName", self.lidar_scan_topic),
                    ("RTXLidarLaserScanPub.inputs:frameId", self.lidar_frame_id),
                    ("RTXLidarLaserScanPub.inputs:nodeNamespace", self.lidar_scan_node_name),
                    ("RTXLidarLaserScanPub.inputs:type", "laser_scan"),
                    ("RTXLidarLaserScanPub.inputs:frameSkipCount", self.lidar_frameskip_cnt),
                ]
            }
        )
        set_target_prims(
            primPath=self.graph_prim_path+"/IsaacCreateRender",
            inputName="inputs:cameraPrim",
            targetPrimPaths=[self.lidar_prim_path],
        )

    # def ros2_imu(self):
    #     og.Controller.edit(
    #         self.graph_prim_path,
    #         {
    #             og.Controller.Keys.CREATE_NODES: [
    #                 ("Simgatestep1", "omni.isaac.core_nodes.IsaacSimulationGate"),
    #                 ("ReadImuNode", "omni.isaac.sensor.IsaacReadIMU"),
    #                 ("ImuPub", "omni.isaac.ros2_bridge.ROS2PublishImu"),
    #             ],
    #             og.Controller.Keys.CONNECT: [
    #                 (self.graph_prim_path+"/OnPlaybackTick.outputs:tick", "Simgatestep1.inputs:execIn"),
    #                 ("Simgatestep1.outputs:execOut", "ReadImuNode.inputs:execIn"),
    #                 (self.graph_prim_path+"/Context.outputs:context", "ImuPub.inputs:context"),
    #                 (self.graph_prim_path+"/ReadSimTime.outputs:simulationTime", "ImuPub.inputs:timeStamp"),
    #                 ("ReadImuNode.outputs:execOut", "ImuPub.inputs:execIn"),
    #                 ("ReadImuNode.outputs:angVel", "ImuPub.inputs:angularVelocity"),
    #                 ("ReadImuNode.outputs:linAcc", "ImuPub.inputs:linearAcceleration"),
    #                 ("ReadImuNode.outputs:orientation", "ImuPub.inputs:orientation"),
    #             ],
    #             og.Controller.Keys.SET_VALUES: [
    #                 ("Simgatestep1.inputs:step", self.imu_step),
    #                 ("ReadImuNode.inputs:readGravity", True),
    #                 ("ReadImuNode.inputs:imuPrim", self.imu_prim_path),
    #                 ("ImuPub.inputs:topicName", self.imu_topic_name),
    #                 ("ImuPub.inputs:frameId", self.imu_frame_id),
    #                 ("ImuPub.inputs:nodeNamespace", self.imu_node_name),
    #             ]
    #         }
    #     )

    def ros2_tf_joint(self):
        og.Controller.edit(
            self.graph_prim_path,
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.CONNECT: [
                    (self.graph_prim_path+"/OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    (self.graph_prim_path+"/OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    (self.graph_prim_path+"/SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    (self.graph_prim_path+"/OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    (self.graph_prim_path+"/Context.outputs:context", "PublishJointState.inputs:context"),
                    (self.graph_prim_path+"/Context.outputs:context", "SubscribeJointState.inputs:context"),
                    (self.graph_prim_path+"/ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    (
                        "SubscribeJointState.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:velocityCommand",
                        "ArticulationController.inputs:velocityCommand",
                    ),
                    ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    (self.graph_prim_path+"/OnPlaybackTick.outputs:tick",
                        "PublishTF.inputs:execIn"),
                    (self.graph_prim_path+"/ReadSimTime.outputs:simulationTime",
                        "PublishTF.inputs:timeStamp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ArticulationController.inputs:robotPath", self.robot_base_link_prim),
                    ("PublishJointState.inputs:topicName", self.joint_state_topic_name),
                    ("SubscribeJointState.inputs:topicName", self.joint_command_topic_name),
                    ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(self.robot_base_link_prim)]),
                    ("PublishTF.inputs:topicName", self.tf_topic),
                ]
            }
        )
        set_target_prims(
            primPath=self.graph_prim_path+"/PublishTF",
            inputName="inputs:targetPrims",
            targetPrimPaths=[self.robot_base_link_prim],
        )
        set_target_prims(
            primPath=self.graph_prim_path+"/PublishTF",
            inputName="inputs:parentPrim",
            targetPrimPaths=[self.robot_base_link_prim],
        )

    def setup_sensor(self):
        if self.robot_is_valid:
            # self.set_camera()
            self.set_lidar()
            self.set_imu()
            self.world_reset()

    def setup_ros(self):
        self.ros2_bridge()
        if self.robot_is_valid:
            # self.ros2_camera()
            self.ros2_lidar()
            self.ros2_tf_joint()
        self.world_reset()
            

    def world_reset(self):
        self.world.reset()
        if hasattr(self, 'robot') and self.robot is not None:
            print("Resetting robot position...")
            self.robot.set_world_pose(position=self.robot_spawn_position)

    def world_render(self):
        self.world.step(render=True)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        sim = StartSim()
        
        # 월드 설정 전에 잠시 대기
        import time
        time.sleep(5)
        
        sim.set_world()
        
        # 로봇 스폰 전에 추가 대기
        time.sleep(2)
        
        sim.spawn_robot()
        sim.world_reset()  # 로봇 스폰 후 즉시 리셋
        
        time.sleep(1)
        
        sim.setup_sensor()
        sim.setup_ros()
        
        while simulation_app.is_running():
            sim.world_render()
            time.sleep(0.01)  # 작은 딜레이 추가
            
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()