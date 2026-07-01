#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
from tf.transformations import quaternion_matrix

# Low-latency copy of zlab_robots_calibration/script/display.py.
# The original version performs one TF query per point. This version performs
# one TF query per target frame and transforms axis endpoints locally.

class FrameReprojector:
    def __init__(self):
        rospy.init_node('frame_reprojector_node', anonymous=True)
        
        # 参数获取
        self.image_topic = rospy.get_param('~image_topic', '/zed2i/zed_node/left/image_rect_gray')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/zed2i/zed_node/left/camera_info')
        # 默认使用相机的光学frame，如果传了参数则使用自定义的camera_frame
        self.camera_frame = rospy.get_param('~camera_frame', 'rig') 
        
        # target_frames 格式约定: [{'frame': 'target1', 'type': 'axes', 'length': 0.1}, 
        #                         {'frame': 'target2', 'type': 'point'},
        #                         {'frame': 'target3', 'type': 'z_axis', 'length': 0.1}]
        self.target_frames = rospy.get_param('~target_frames', [
            {'frame': 'diana7_em_tcp_filt', 'type': 'axes', 'length': 0.1},
            {'frame': 'arm1_em_tcp_filt', 'type': 'axes', 'length': 0.1},
            {'frame': 'arm2_em_tcp_filt', 'type': 'axes', 'length': 0.1},
            {'frame': 'sensor_array_filt', 'type': 'point'}
        ])
        
        self.show_window = rospy.get_param('~show_window', False)
        self.tf_timeout_s = float(rospy.get_param('~tf_timeout_s', 0.005))

        self.bridge = CvBridge()
        self.cam_model = image_geometry.PinholeCameraModel()
        self.has_cam_info = False

        # TF2 初始化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅与发布
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cam_info_cb)
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.image_pub = rospy.Publisher('~reprojected_image', Image, queue_size=1)

        rospy.loginfo("FrameReprojector initialized. Waiting for image and camera info...")

    def cam_info_cb(self, msg):
        if not self.has_cam_info:
            self.cam_model.fromCameraInfo(msg)
            self.has_cam_info = True
            if not self.camera_frame:
                self.camera_frame = msg.header.frame_id
            rospy.loginfo(f"Received CameraInfo. Using camera_frame: {self.camera_frame}")

    def lookup_frame_matrix(self, source_frame):
        """Return matrix that transforms points from source_frame to camera_frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout_s),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logdebug_throttle(
                1.0,
                f"TF lookup failed: {self.camera_frame} <- {source_frame}: {e}",
            )
            return None

        tr = transform.transform.translation
        rot = transform.transform.rotation
        mat = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        mat[0, 3] = tr.x
        mat[1, 3] = tr.y
        mat[2, 3] = tr.z
        return mat

    def transform_point(self, matrix, xyz):
        pt = matrix.dot([xyz[0], xyz[1], xyz[2], 1.0])
        return pt[0], pt[1], pt[2]

    def project_point(self, xyz):
        x, y, z = xyz
        if z <= 0:
            return None
        return tuple(map(int, self.cam_model.project3dToPixel((x, y, z))))

    def draw_axes(self, cv_image, frame_id, length=0.1):
        """绘制三轴 (RGB = XYZ)"""
        matrix = self.lookup_frame_matrix(frame_id)
        if matrix is None:
            return

        origin = self.transform_point(matrix, (0, 0, 0))
        pt_x = self.transform_point(matrix, (length, 0, 0))
        pt_y = self.transform_point(matrix, (0, length, 0))
        pt_z = self.transform_point(matrix, (0, 0, length))

        uv_origin = self.project_point(origin)
        uv_x = self.project_point(pt_x)
        uv_y = self.project_point(pt_y)
        uv_z = self.project_point(pt_z)
        if not all([uv_origin, uv_x, uv_y, uv_z]):
            return

        thickness = 2
        cv2.line(cv_image, uv_origin, uv_x, (0, 0, 255), thickness) # X轴红色
        cv2.line(cv_image, uv_origin, uv_y, (0, 255, 0), thickness) # Y轴绿色
        cv2.line(cv_image, uv_origin, uv_z, (255, 0, 0), thickness) # Z轴蓝色
        cv2.putText(cv_image, frame_id, uv_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def draw_z_axis(self, cv_image, frame_id, length=0.1):
        """只绘制单轴 (Z轴为例)"""
        matrix = self.lookup_frame_matrix(frame_id)
        if matrix is None:
            return

        origin = self.transform_point(matrix, (0, 0, 0))
        pt_z = self.transform_point(matrix, (0, 0, length))
        uv_origin = self.project_point(origin)
        uv_z = self.project_point(pt_z)
        if not all([uv_origin, uv_z]):
            return

        cv2.line(cv_image, uv_origin, uv_z, (255, 0, 0), 2)
        cv2.putText(cv_image, f"{frame_id}_Z", uv_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    def draw_point(self, cv_image, frame_id):
        """只绘制单点 (原点)"""
        matrix = self.lookup_frame_matrix(frame_id)
        if matrix is None:
            return

        origin = self.transform_point(matrix, (0, 0, 0))
        uv_origin = self.project_point(origin)
        if uv_origin is None:
            return

        cv2.circle(cv_image, uv_origin, 5, (0, 255, 255), -1)
        cv2.putText(cv_image, frame_id, uv_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    def image_cb(self, msg):
        if not self.has_cam_info or not self.camera_frame:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # 遍历要投射的坐标系配置进行渲染
        for target in self.target_frames:
            frame_id = target.get('frame')
            draw_type = target.get('type', 'axes')
            length = target.get('length', 0.1)

            if not frame_id:
                continue

            if draw_type == 'axes':
                self.draw_axes(cv_image, frame_id, length)
            elif draw_type == 'z_axis':
                self.draw_z_axis(cv_image, frame_id, length)
            elif draw_type == 'point':
                self.draw_point(cv_image, frame_id)

        # 发布画好的图像
        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)
            if self.show_window:
                cv2.imshow("Reprojected Image", cv_image)
                cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        node = FrameReprojector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
