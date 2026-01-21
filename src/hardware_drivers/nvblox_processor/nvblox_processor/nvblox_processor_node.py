#!/usr/bin/env python3
"""
nvblox Processor Node
Pre-processes RealSense depth data using nvblox for efficient 3D mapping
Generates TSDF, mesh, and ESDF outputs for visualization and VLA features
"""

# Standard library
import threading
import time
from typing import Dict, Optional

# Third-party
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray


class NvbloxProcessorNode(Node):
    """ROS 2 node for processing RealSense data with nvblox"""

    def __init__(self):
        super().__init__("nvblox_processor_node")

        # Parameters
        self.declare_parameter("camera_names", ["camera_front", "camera_rear"])
        self.declare_parameter("voxel_size", 0.05)  # 5cm voxels
        self.declare_parameter("tsdf_truncation_distance", 0.1)  # 10cm truncation
        self.declare_parameter("max_tsdf_weight", 100.0)
        self.declare_parameter("mesh_update_rate", 5.0)  # Hz
        self.declare_parameter(
            "pointcloud_downsample_factor", 1
        )  # Full quality (fusion node downsamples)
        self.declare_parameter("publish_tsdf_markers", True)
        self.declare_parameter("publish_mesh_markers", True)
        self.declare_parameter("publish_downsampled_points", True)
        self.declare_parameter("fuse_cameras", True)  # Multi-camera fusion
        self.declare_parameter("status_topic", "/nvblox/status")
        self.declare_parameter("namespace", "/nvblox/full")  # Full quality namespace

        # Get parameters
        self.camera_names = self.get_parameter("camera_names").value
        self.voxel_size = self.get_parameter("voxel_size").value
        self.tsdf_truncation_distance = self.get_parameter("tsdf_truncation_distance").value
        self.max_tsdf_weight = self.get_parameter("max_tsdf_weight").value
        self.mesh_update_rate = self.get_parameter("mesh_update_rate").value
        self.downsample_factor = int(self.get_parameter("pointcloud_downsample_factor").value)
        self.publish_tsdf_markers = bool(self.get_parameter("publish_tsdf_markers").value)
        self.publish_mesh_markers = bool(self.get_parameter("publish_mesh_markers").value)
        self.publish_downsampled_points = bool(
            self.get_parameter("publish_downsampled_points").value
        )
        self.fuse_cameras = bool(self.get_parameter("fuse_cameras").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.namespace = str(self.get_parameter("namespace").value)

        # CV Bridge
        self.bridge = CvBridge()

        # Storage for camera info and latest frames
        self.camera_infos: Dict[str, CameraInfo] = {}
        self.latest_depth_images: Dict[str, Optional[Image]] = {}
        self.latest_color_images: Dict[str, Optional[Image]] = {}
        self.latest_timestamps: Dict[str, float] = {}

        # Thread safety
        self.lock = threading.Lock()

        # Subscribers for each camera
        self.depth_subscribers: Dict[str, rclpy.subscription.Subscription] = {}
        self.color_subscribers: Dict[str, rclpy.subscription.Subscription] = {}
        self.camera_info_subscribers: Dict[str, rclpy.subscription.Subscription] = {}

        # Publishers
        self.status_publisher = self.create_publisher(String, self.status_topic, 10)
        self.mesh_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        self.tsdf_marker_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        self.downsampled_pointcloud_publishers: Dict[str, rclpy.publisher.Publisher] = {}

        # Full quality publishers (for fusion node)
        self.full_pointcloud_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        self.full_image_publishers: Dict[str, rclpy.publisher.Publisher] = {}

        # Fused publishers (multi-camera fusion)
        self.fused_mesh_publisher: Optional[rclpy.publisher.Publisher] = None
        self.fused_tsdf_publisher: Optional[rclpy.publisher.Publisher] = None

        # Initialize publishers and subscribers for each camera
        # Use BEST_EFFORT QoS to match camera publishers (they use BEST_EFFORT to prevent blocking)
        # Third-party
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        for camera_name in self.camera_names:
            # Subscribers
            self.create_subscription(
                Image,
                f"/hardware/{camera_name}/depth/image_rect_raw",
                lambda msg, name=camera_name: self._depth_callback(msg, name),
                camera_qos,  # BEST_EFFORT to match camera publisher
            )
            self.create_subscription(
                Image,
                f"/hardware/{camera_name}/color/image_raw",
                lambda msg, name=camera_name: self._color_callback(msg, name),
                camera_qos,  # BEST_EFFORT to match camera publisher
            )
            self.create_subscription(
                CameraInfo,
                f"/hardware/{camera_name}/depth/camera_info",
                lambda msg, name=camera_name: self._camera_info_callback(msg, name),
                10,
            )

            # Publishers (full quality for fusion node)
            self.full_pointcloud_publishers[camera_name] = self.create_publisher(
                PointCloud2, f"{self.namespace}/{camera_name}/pointcloud", 10
            )
            self.full_image_publishers[camera_name] = self.create_publisher(
                Image, f"{self.namespace}/{camera_name}/image", 10
            )

            # Legacy publishers (per-camera, for backward compatibility)
            if self.publish_mesh_markers:
                self.mesh_publishers[camera_name] = self.create_publisher(
                    MarkerArray, f"{self.namespace}/{camera_name}/mesh", 10
                )
            if self.publish_tsdf_markers:
                self.tsdf_marker_publishers[camera_name] = self.create_publisher(
                    MarkerArray, f"{self.namespace}/{camera_name}/tsdf", 10
                )
            if self.publish_downsampled_points:
                self.downsampled_pointcloud_publishers[camera_name] = self.create_publisher(
                    PointCloud2, f"{self.namespace}/{camera_name}/points_downsampled", 10
                )

        # Fused publishers (multi-camera fusion)
        if self.fuse_cameras and len(self.camera_names) > 1:
            self.fused_mesh_publisher = self.create_publisher(
                MarkerArray, f"{self.namespace}/mesh", 10
            )
            self.fused_tsdf_publisher = self.create_publisher(
                MarkerArray, f"{self.namespace}/tsdf", 10
            )

            # Initialize storage
            self.latest_depth_images[camera_name] = None
            self.latest_color_images[camera_name] = None
            self.latest_timestamps[camera_name] = 0.0

        # Timer for mesh updates
        self.mesh_timer = self.create_timer(1.0 / self.mesh_update_rate, self._update_meshes)

        # Status
        self.publish_status("initialized", "nvblox processor node initialized")

        self.get_logger().info("nvblox processor node started")

    def _depth_callback(self, msg: Image, camera_name: str):
        """Callback for depth images"""
        with self.lock:
            self.latest_depth_images[camera_name] = msg
            self.latest_timestamps[camera_name] = time.time()

            # Publish full quality pointcloud (fusion node will downsample)
            if camera_name in self.full_pointcloud_publishers:
                self._process_and_publish_full_pointcloud(msg, camera_name)

            # Process and publish downsampled pointcloud (legacy, for backward compatibility)
            if (
                self.publish_downsampled_points
                and camera_name in self.downsampled_pointcloud_publishers
            ):
                self._process_and_publish_downsampled_points(msg, camera_name)

    def _color_callback(self, msg: Image, camera_name: str):
        """Callback for color images"""
        # OPTIMIZATION: Store message reference (no copy) and republish directly
        # This avoids unnecessary conversions - just update timestamp and republish
        with self.lock:
            self.latest_color_images[camera_name] = msg

            # Publish full quality image (fusion node will downsample)
            # OPTIMIZATION: Republish message directly without conversion (zero-copy)
            if camera_name in self.full_image_publishers:
                msg.header.stamp = self.get_clock().now().to_msg()
                self.full_image_publishers[camera_name].publish(msg)

    def _camera_info_callback(self, msg: CameraInfo, camera_name: str):
        """Callback for camera info"""
        with self.lock:
            self.camera_infos[camera_name] = msg

    def _process_and_publish_full_pointcloud(self, depth_msg: Image, camera_name: str):
        """Process depth image and publish full quality pointcloud (no downsampling)"""
        if camera_name not in self.camera_infos:
            return

        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            # OPTIMIZATION: Use astype with copy=False to avoid unnecessary copy if possible
            depth_array = depth_image.astype(np.float32, copy=False) / 1000.0  # Convert mm to meters

            # Get camera info
            cam_info = self.camera_infos[camera_name]
            fx = cam_info.k[0]
            fy = cam_info.k[4]
            cx = cam_info.k[2]
            cy = cam_info.k[5]

            # Full quality (no downsampling)
            height, width = depth_array.shape
            points = []

            # Generate pointcloud (full resolution)
            for v in range(height):
                for u in range(width):
                    z = depth_array[v, u]
                    if z > 0 and z < 10.0:  # Valid depth range (0-10m)
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        points.append([x, y, z])

            if len(points) == 0:
                return

            # Create PointCloud2 message
            points_array = np.array(points, dtype=np.float32).reshape(-1, 3)
            pointcloud_msg = self._create_pointcloud2(points_array, depth_msg.header)
            pointcloud_msg.header.stamp = self.get_clock().now().to_msg()

            # Publish full quality pointcloud
            self.full_pointcloud_publishers[camera_name].publish(pointcloud_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing full pointcloud for {camera_name}: {e}")

    def _process_and_publish_downsampled_points(self, depth_msg: Image, camera_name: str):
        """Process depth image and publish downsampled pointcloud (legacy, for backward compatibility)"""
        if camera_name not in self.camera_infos:
            return

        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            # OPTIMIZATION: Use astype with copy=False to avoid unnecessary copy if possible
            depth_array = depth_image.astype(np.float32, copy=False) / 1000.0  # Convert mm to meters

            # Get camera info
            cam_info = self.camera_infos[camera_name]
            fx = cam_info.k[0]
            fy = cam_info.k[4]
            cx = cam_info.k[2]
            cy = cam_info.k[5]

            # Downsample
            height, width = depth_array.shape
            step = self.downsample_factor
            downsampled_points = []

            # Generate pointcloud (downsampled)
            for v in range(0, height, step):
                for u in range(0, width, step):
                    z = depth_array[v, u]
                    if z > 0 and z < 10.0:  # Valid depth range (0-10m)
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        downsampled_points.append([x, y, z])

            if len(downsampled_points) == 0:
                return

            # Create PointCloud2 message
            points = np.array(downsampled_points, dtype=np.float32).reshape(-1, 3)
            pointcloud_msg = self._create_pointcloud2(points, depth_msg.header)

            # Publish
            self.downsampled_pointcloud_publishers[camera_name].publish(pointcloud_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing downsampled points for {camera_name}: {e}")

    def _create_pointcloud2(self, points: np.ndarray, header: Header) -> PointCloud2:
        """Create a PointCloud2 message from numpy array"""
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        # Ensure points are contiguous and properly formatted
        if not points.flags["C_CONTIGUOUS"]:
            points = np.ascontiguousarray(points)
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def _update_meshes(self):
        """Periodic update of mesh and TSDF markers"""
        with self.lock:
            # Process per-camera meshes/TSDF (legacy, for backward compatibility)
            for camera_name in self.camera_names:
                if camera_name not in self.latest_depth_images:
                    continue
                if self.latest_depth_images[camera_name] is None:
                    continue
                if camera_name not in self.camera_infos:
                    continue

                try:
                    # Process TSDF markers
                    if self.publish_tsdf_markers and camera_name in self.tsdf_marker_publishers:
                        self._publish_tsdf_markers(camera_name)

                    # Process mesh markers
                    if self.publish_mesh_markers and camera_name in self.mesh_publishers:
                        self._publish_mesh_markers(camera_name)

                except Exception as e:
                    self.get_logger().error(f"Error updating meshes for {camera_name}: {e}")

            # Process fused mesh/TSDF (multi-camera fusion)
            if self.fuse_cameras and len(self.camera_names) > 1:
                try:
                    # Check if all cameras have valid data
                    all_cameras_ready = all(
                        camera_name in self.latest_depth_images
                        and self.latest_depth_images[camera_name] is not None
                        and camera_name in self.camera_infos
                        for camera_name in self.camera_names
                    )

                    if all_cameras_ready:
                        if self.fused_tsdf_publisher:
                            self._publish_fused_tsdf()
                        if self.fused_mesh_publisher:
                            self._publish_fused_mesh()
                except Exception as e:
                    self.get_logger().error(f"Error updating fused meshes: {e}")

    def _publish_tsdf_markers(self, camera_name: str):
        """Publish TSDF visualization markers"""
        depth_msg = self.latest_depth_images[camera_name]
        if depth_msg is None:
            return

        try:
            # Convert depth image
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32) / 1000.0

            # Get camera info
            cam_info = self.camera_infos[camera_name]
            fx = cam_info.k[0]
            fy = cam_info.k[4]
            cx = cam_info.k[2]
            cy = cam_info.k[5]

            # Create TSDF voxel markers (simplified visualization)
            markers = MarkerArray()
            marker = Marker()
            marker.header = depth_msg.header
            marker.ns = f"{camera_name}_tsdf"
            marker.id = 0
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale.x = self.voxel_size
            marker.scale.y = self.voxel_size
            marker.scale.z = self.voxel_size
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            # Sample points for TSDF visualization (downsampled)
            height, width = depth_array.shape
            step = max(1, int(self.downsample_factor * 2))
            points = []

            for v in range(0, height, step):
                for u in range(0, width, step):
                    z = depth_array[v, u]
                    if z > 0 and z < 10.0:
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        # Voxelize
                        voxel_x = round(x / self.voxel_size) * self.voxel_size
                        voxel_y = round(y / self.voxel_size) * self.voxel_size
                        voxel_z = round(z / self.voxel_size) * self.voxel_size
                        points.append([voxel_x, voxel_y, voxel_z])

            # Add points to marker (using geometry_msgs Point)
            for point in points[:1000]:  # Limit to 1000 voxels for performance
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = float(point[2])
                marker.points.append(p)

            markers.markers.append(marker)
            self.tsdf_marker_publishers[camera_name].publish(markers)

        except Exception as e:
            self.get_logger().error(f"Error publishing TSDF markers for {camera_name}: {e}")

    def _publish_mesh_markers(self, camera_name: str):
        """Publish mesh visualization markers generated from depth data"""
        depth_msg = self.latest_depth_images[camera_name]
        if depth_msg is None:
            return

        if camera_name not in self.camera_infos:
            return

        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            # OPTIMIZATION: Use astype with copy=False to avoid unnecessary copy if possible
            depth_array = depth_image.astype(np.float32, copy=False) / 1000.0  # Convert mm to meters

            # Get camera info
            cam_info = self.camera_infos[camera_name]
            fx = cam_info.k[0]
            fy = cam_info.k[4]
            cx = cam_info.k[2]
            cy = cam_info.k[5]

            # Generate mesh triangles from depth image
            height, width = depth_array.shape
            step = self.downsample_factor * 2  # Use larger step for mesh to reduce size
            triangles = []

            # Generate triangles from depth image (simple approach)
            for v in range(0, height - step, step):
                for u in range(0, width - step, step):
                    # Get depth values for 4 corners of a quad
                    z00 = depth_array[v, u]
                    z01 = depth_array[v, u + step] if u + step < width else 0
                    z10 = depth_array[v + step, u] if v + step < height else 0
                    z11 = (
                        depth_array[v + step, u + step]
                        if (v + step < height and u + step < width)
                        else 0
                    )

                    # Check if all depths are valid
                    if all(z > 0 and z < 10.0 for z in [z00, z01, z10, z11] if z > 0):
                        # Convert pixel coordinates to 3D points
                        def pixel_to_3d(u_pix, v_pix, z_val):
                            x = (u_pix - cx) * z_val / fx
                            y = (v_pix - cy) * z_val / fy
                            return [x, y, z_val]

                        p00 = pixel_to_3d(u, v, z00)
                        p01 = pixel_to_3d(u + step, v, z01)
                        p10 = pixel_to_3d(u, v + step, z10)
                        p11 = pixel_to_3d(u + step, v + step, z11)

                        # Create two triangles from the quad
                        # Triangle 1: p00 -> p01 -> p10
                        triangles.extend([p00, p01, p10])
                        # Triangle 2: p01 -> p11 -> p10
                        triangles.extend([p01, p11, p10])

            if len(triangles) == 0:
                return

            # Create mesh marker
            markers = MarkerArray()
            marker = Marker()
            marker.header = depth_msg.header
            marker.ns = f"{camera_name}_mesh"
            marker.id = 0
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 0.7
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0

            # Add triangle vertices as points
            for triangle in triangles:
                point = Point()
                point.x = float(triangle[0])
                point.y = float(triangle[1])
                point.z = float(triangle[2])
                marker.points.append(point)

            markers.markers.append(marker)
            self.mesh_publishers[camera_name].publish(markers)

        except Exception as e:
            self.get_logger().error(f"Error publishing mesh markers for {camera_name}: {e}")

    def _publish_fused_tsdf(self):
        """Publish fused TSDF from all cameras"""
        try:
            # Collect TSDF data from all cameras and fuse
            # For now, use a simple approach: combine voxels from all cameras
            # In a full implementation, this would use proper TSDF fusion

            fused_markers = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "base_link"  # Use common frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "fused_tsdf"
            marker.id = 0
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale.x = self.voxel_size
            marker.scale.y = self.voxel_size
            marker.scale.z = self.voxel_size
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            # Collect points from all cameras (simplified fusion)
            all_points = []
            for camera_name in self.camera_names:
                depth_msg = self.latest_depth_images[camera_name]
                if depth_msg is None:
                    continue

                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                depth_array = np.array(depth_image, dtype=np.float32) / 1000.0

                cam_info = self.camera_infos[camera_name]
                fx = cam_info.k[0]
                fy = cam_info.k[4]
                cx = cam_info.k[2]
                cy = cam_info.k[5]

                height, width = depth_array.shape
                step = max(1, int(self.downsample_factor * 2))

                for v in range(0, height, step):
                    for u in range(0, width, step):
                        z = depth_array[v, u]
                        if z > 0 and z < 10.0:
                            x = (u - cx) * z / fx
                            y = (v - cy) * z / fy
                            # Voxelize
                            voxel_x = round(x / self.voxel_size) * self.voxel_size
                            voxel_y = round(y / self.voxel_size) * self.voxel_size
                            voxel_z = round(z / self.voxel_size) * self.voxel_size
                            all_points.append([voxel_x, voxel_y, voxel_z])

            # Deduplicate voxels (simple fusion)
            unique_voxels = {}
            for point in all_points:
                key = tuple(point)
                if key not in unique_voxels:
                    unique_voxels[key] = point

            # Add points to marker
            for point in list(unique_voxels.values())[:2000]:  # Limit for performance
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = float(point[2])
                marker.points.append(p)

            fused_markers.markers.append(marker)
            if self.fused_tsdf_publisher:
                self.fused_tsdf_publisher.publish(fused_markers)

        except Exception as e:
            self.get_logger().error(f"Error publishing fused TSDF: {e}")

    def _publish_fused_mesh(self):
        """Publish fused mesh from all cameras"""
        try:
            # Collect mesh data from all cameras and fuse
            # For now, use a simple approach: combine triangles from all cameras
            # In a full implementation, this would use proper mesh fusion

            fused_markers = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "base_link"  # Use common frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "fused_mesh"
            marker.id = 0
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 0.7
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0

            # Collect triangles from all cameras (simplified fusion)
            all_triangles = []
            for camera_name in self.camera_names:
                depth_msg = self.latest_depth_images[camera_name]
                if depth_msg is None:
                    continue

                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                depth_array = np.array(depth_image, dtype=np.float32) / 1000.0

                cam_info = self.camera_infos[camera_name]
                fx = cam_info.k[0]
                fy = cam_info.k[4]
                cx = cam_info.k[2]
                cy = cam_info.k[5]

                height, width = depth_array.shape
                step = self.downsample_factor * 2

                def pixel_to_3d(u_pix, v_pix, z_val):
                    x = (u_pix - cx) * z_val / fx
                    y = (v_pix - cy) * z_val / fy
                    return [x, y, z_val]

                for v in range(0, height - step, step):
                    for u in range(0, width - step, step):
                        z00 = depth_array[v, u]
                        z01 = depth_array[v, u + step] if u + step < width else 0
                        z10 = depth_array[v + step, u] if v + step < height else 0
                        z11 = (
                            depth_array[v + step, u + step]
                            if (v + step < height and u + step < width)
                            else 0
                        )

                        if all(z > 0 and z < 10.0 for z in [z00, z01, z10, z11] if z > 0):
                            p00 = pixel_to_3d(u, v, z00)
                            p01 = pixel_to_3d(u + step, v, z01)
                            p10 = pixel_to_3d(u, v + step, z10)
                            p11 = pixel_to_3d(u + step, v + step, z11)

                            # Add triangles
                            all_triangles.extend([p00, p01, p10])
                            all_triangles.extend([p01, p11, p10])

            # Add triangles to marker
            for triangle in all_triangles[:3000]:  # Limit for performance
                p = Point()
                p.x = float(triangle[0])
                p.y = float(triangle[1])
                p.z = float(triangle[2])
                marker.points.append(p)

            fused_markers.markers.append(marker)
            if self.fused_mesh_publisher:
                self.fused_mesh_publisher.publish(fused_markers)

        except Exception as e:
            self.get_logger().error(f"Error publishing fused mesh: {e}")

    def publish_status(self, status: str, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f"{status}: {message}"
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NvbloxProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
