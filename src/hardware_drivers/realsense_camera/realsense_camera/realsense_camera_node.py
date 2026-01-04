#!/usr/bin/env python3
"""
RealSense Camera Node
Publishes color and depth images from Intel RealSense cameras
Supports multiple cameras with configurable namespaces
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header, String
from geometry_msgs.msg import TransformStamped
import numpy as np
from cv_bridge import CvBridge
import pyrealsense2 as rs
from typing import Optional, Dict, List
import threading
import time


class RealSenseCameraNode(Node):
    """ROS 2 node for Intel RealSense depth cameras"""

    def __init__(self):
        super().__init__('realsense_camera_node')

        # Parameters
        self.declare_parameter('camera_serial_numbers', [])
        self.declare_parameter('camera_names', ['camera_front', 'camera_rear'])
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_pointcloud', False)
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('depth_fps', 30)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('align_depth_to_color', True)

        # Get parameters
        self.camera_serials = self.get_parameter('camera_serial_numbers').value
        self.camera_names = self.get_parameter('camera_names').value
        self.enable_color = self.get_parameter('enable_color').value
        self.enable_depth = self.get_parameter('enable_depth').value
        self.enable_pointcloud = self.get_parameter('enable_pointcloud').value
        self.color_width = self.get_parameter('color_width').value
        self.color_height = self.get_parameter('color_height').value
        self.color_fps = self.get_parameter('color_fps').value
        self.depth_width = self.get_parameter('depth_width').value
        self.depth_height = self.get_parameter('depth_height').value
        self.depth_fps = self.get_parameter('depth_fps').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.align_depth_to_color = self.get_parameter('align_depth_to_color').value

        # Initialize
        self.bridge = CvBridge()
        self.pipelines: Dict[str, rs.pipeline] = {}
        self.configs: Dict[str, rs.config] = {}
        self.aligns: Dict[str, rs.align] = {}
        self.camera_infos: Dict[str, Dict] = {}
        self.running = True

        # Status publisher
        self.status_pub = self.create_publisher(String, 'realsense/status', 10)

        # Discover and initialize cameras
        self._discover_cameras()

        # Create publishers for each camera
        self._create_publishers()

        # Start camera threads
        self.camera_threads: List[threading.Thread] = []
        for camera_name in self.camera_names:
            if camera_name in self.pipelines:
                thread = threading.Thread(
                    target=self._camera_loop,
                    args=(camera_name,),
                    daemon=True
                )
                thread.start()
                self.camera_threads.append(thread)

        self.get_logger().info(f'RealSense Camera Node started with {len(self.pipelines)} camera(s)')
        self.publish_status('initialized', f'Started with {len(self.pipelines)} camera(s)')

    def _discover_cameras(self):
        """Discover available RealSense cameras"""
        ctx = rs.context()
        devices = ctx.query_devices()

        if len(devices) == 0:
            self.get_logger().error('No RealSense devices found!')
            self.publish_status('error', 'No cameras detected')
            return

        self.get_logger().info(f'Found {len(devices)} RealSense device(s)')

        # If serial numbers specified, use those; otherwise use all devices
        available_serials = [dev.get_info(rs.camera_info.serial_number) for dev in devices]

        if self.camera_serials and len(self.camera_serials) > 0:
            # Use specified serials
            target_serials = self.camera_serials
        else:
            # Use all available cameras
            target_serials = available_serials[:len(self.camera_names)]

        # Initialize cameras
        for i, serial in enumerate(target_serials):
            if serial not in available_serials:
                self.get_logger().warn(f'Camera with serial {serial} not found')
                continue

            if i >= len(self.camera_names):
                camera_name = f'camera_{i}'
            else:
                camera_name = self.camera_names[i]

            try:
                self._initialize_camera(camera_name, serial, devices[available_serials.index(serial)])
            except Exception as e:
                self.get_logger().error(f'Failed to initialize camera {camera_name} ({serial}): {e}')
                self.publish_status('error', f'Failed to initialize {camera_name}: {e}')

    def _initialize_camera(self, camera_name: str, serial: str, device: rs.device):
        """Initialize a single camera"""
        self.get_logger().info(f'Initializing camera: {camera_name} (serial: {serial})')

        # Create pipeline and config
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)

        # Configure streams
        if self.enable_color:
            config.enable_stream(
                rs.stream.color,
                self.color_width,
                self.color_height,
                rs.format.rgb8,
                self.color_fps
            )

        if self.enable_depth:
            config.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                rs.format.z16,
                self.depth_fps
            )

        # Start pipeline
        profile = pipeline.start(config)

        # Get camera intrinsics
        if self.enable_color:
            color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
            color_intrinsics = color_profile.get_intrinsics()
            self.camera_infos[camera_name] = {
                'color': self._intrinsics_to_camera_info(color_intrinsics, camera_name, 'color')
            }

        if self.enable_depth:
            depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
            depth_intrinsics = depth_profile.get_intrinsics()
            if 'depth' not in self.camera_infos.get(camera_name, {}):
                self.camera_infos[camera_name] = {}
            self.camera_infos[camera_name]['depth'] = self._intrinsics_to_camera_info(
                depth_intrinsics, camera_name, 'depth'
            )

        # Create align object if needed
        align = None
        if self.align_depth_to_color and self.enable_color and self.enable_depth:
            align = rs.align(rs.stream.color)

        self.pipelines[camera_name] = pipeline
        self.configs[camera_name] = config
        self.aligns[camera_name] = align

        self.get_logger().info(f'Camera {camera_name} initialized successfully')

    def _intrinsics_to_camera_info(self, intrinsics, camera_name: str, stream_type: str) -> CameraInfo:
        """Convert RealSense intrinsics to ROS CameraInfo"""
        info = CameraInfo()
        info.header.frame_id = f'{camera_name}_{stream_type}_optical_frame'
        info.width = intrinsics.width
        info.height = intrinsics.height
        info.distortion_model = 'plumb_bob'

        # Camera matrix (3x3 row-major)
        info.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        ]

        # Distortion coefficients
        info.d = list(intrinsics.coeffs[:5])

        # Rectification matrix (identity)
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection matrix
        info.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return info

    def _create_publishers(self):
        """Create ROS publishers for each camera"""
        self.publishers: Dict[str, Dict] = {}

        for camera_name in self.pipelines.keys():
            self.publishers[camera_name] = {}

            if self.enable_color:
                self.publishers[camera_name]['color_image'] = self.create_publisher(
                    Image, f'{camera_name}/color/image_raw', 10
                )
                self.publishers[camera_name]['color_info'] = self.create_publisher(
                    CameraInfo, f'{camera_name}/color/camera_info', 10
                )

            if self.enable_depth:
                self.publishers[camera_name]['depth_image'] = self.create_publisher(
                    Image, f'{camera_name}/depth/image_rect_raw', 10
                )
                self.publishers[camera_name]['depth_info'] = self.create_publisher(
                    CameraInfo, f'{camera_name}/depth/camera_info', 10
                )

            if self.enable_pointcloud and self.enable_depth:
                self.publishers[camera_name]['pointcloud'] = self.create_publisher(
                    PointCloud2, f'{camera_name}/points', 10
                )

    def _camera_loop(self, camera_name: str):
        """Main loop for a single camera"""
        pipeline = self.pipelines[camera_name]
        align = self.aligns.get(camera_name)

        frame_period = 1.0 / self.publish_rate
        last_frame_time = time.time()

        while rclpy.ok() and self.running:
            try:
                # Wait for frames
                frames = pipeline.wait_for_frames(timeout_ms=1000)

                # Align depth to color if requested
                if align:
                    frames = align.process(frames)

                current_time = time.time()
                if current_time - last_frame_time < frame_period:
                    continue
                last_frame_time = current_time

                # Get frames
                color_frame = None
                depth_frame = None

                if self.enable_color:
                    color_frame = frames.get_color_frame()
                if self.enable_depth:
                    depth_frame = frames.get_depth_frame()

                # Publish color image
                if color_frame and self.enable_color:
                    color_image = np.asanyarray(color_frame.get_data())
                    # Convert RGB to BGR for OpenCV
                    color_image = color_image[:, :, ::-1]
                    ros_image = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
                    ros_image.header.frame_id = f'{camera_name}_color_optical_frame'
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    self.publishers[camera_name]['color_image'].publish(ros_image)

                    # Publish camera info
                    info = self.camera_infos[camera_name]['color']
                    info.header.stamp = ros_image.header.stamp
                    self.publishers[camera_name]['color_info'].publish(info)

                # Publish depth image
                if depth_frame and self.enable_depth:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    ros_image = self.bridge.cv2_to_imgmsg(depth_image, '16UC1')
                    ros_image.header.frame_id = f'{camera_name}_depth_optical_frame'
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    self.publishers[camera_name]['depth_image'].publish(ros_image)

                    # Publish camera info
                    info = self.camera_infos[camera_name]['depth']
                    info.header.stamp = ros_image.header.stamp
                    self.publishers[camera_name]['depth_info'].publish(info)

                    # Publish pointcloud if enabled
                    if self.enable_pointcloud and 'pointcloud' in self.publishers[camera_name]:
                        pointcloud = self._depth_to_pointcloud(
                            depth_frame, color_frame, camera_name
                        )
                        if pointcloud:
                            self.publishers[camera_name]['pointcloud'].publish(pointcloud)

            except Exception as e:
                self.get_logger().error(f'Error in camera loop for {camera_name}: {e}')
                self.publish_status('error', f'{camera_name}: {e}')
                time.sleep(0.1)

        # Cleanup
        pipeline.stop()
        self.get_logger().info(f'Camera {camera_name} stopped')

    def _depth_to_pointcloud(self, depth_frame: rs.depth_frame, color_frame: Optional[rs.video_frame],
                            camera_name: str) -> Optional[PointCloud2]:
        """Convert depth frame to PointCloud2 message"""
        try:
            # Get depth intrinsics
            depth_intrinsics = rs.video_stream_profile(
                depth_frame.profile
            ).get_intrinsics()

            # Create pointcloud
            points = rs.pointcloud()
            if color_frame:
                points.map_to(color_frame)
            points = points.calculate(depth_frame)

            vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
            colors = None
            if color_frame:
                colors = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)

            # Create PointCloud2 message
            msg = PointCloud2()
            msg.header.frame_id = f'{camera_name}_depth_optical_frame'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height = 1
            msg.width = len(vertices)
            msg.is_dense = False

            # Point fields
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

            if colors is not None:
                msg.fields.append(
                    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
                )

            msg.point_step = 12 if colors is None else 16
            msg.row_step = msg.point_step * msg.width
            msg.data = vertices.tobytes()

            return msg

        except Exception as e:
            self.get_logger().debug(f'Error creating pointcloud: {e}')
            return None

    def publish_status(self, status: str, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[{status}] {message}'
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down RealSense cameras...')
        self.running = False
        time.sleep(0.5)  # Give threads time to stop

        for pipeline in self.pipelines.values():
            try:
                pipeline.stop()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
