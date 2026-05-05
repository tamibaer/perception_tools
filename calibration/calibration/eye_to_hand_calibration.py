import rclpy
import cv2
import numpy as np
import tf2_ros

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation


class CameraEyeToHandCalibration(Node):
    def __init__(self):
        super().__init__('eye_to_hand_calibration')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info', 
            self.camera_info_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Calibration Node started 📸")

        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        table_spacing = 0.09334 

        self.grid_rows = 18
        self.grid_cols = 7  
        #self.grid_cols = 4

        x_offset = 0.04667
        y_offset = 0.14001

        objp = np.zeros((self.grid_rows * self.grid_cols, 3), np.float32)

        for j in range(self.grid_cols):
            for i in range(self.grid_rows):
                objp[i + (j*self.grid_rows), 0] = (x_offset + (((self.grid_rows/2.0)-1) * table_spacing)) - ((i)*table_spacing)
                objp[i + (j*self.grid_rows), 1] =  y_offset + (j * table_spacing)

        objp = objp[::-1] # Flip point order (wenn kamera auf dem Kopf steht 🙃)

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
      
        params = cv2.SimpleBlobDetector_Params()
        params.minArea = 5
        params.minDistBetweenBlobs = 5

        detector = cv2.SimpleBlobDetector_create(params)
        ret, centers = cv2.findCirclesGrid(gray, (self.grid_rows,  self.grid_cols),None,flags=cv2.CALIB_CB_SYMMETRIC_GRID, blobDetector=detector)

        vis = frame.copy()

        if ret:
            cv2.drawChessboardCorners(vis, (self.grid_rows,  self.grid_cols), centers, ret)
            if self.camera_matrix is None:
                self.get_logger().warn("No camera matrix yet")
                return
            success, rvec, tvec = cv2.solvePnP(
                objp,
                centers,
                self.camera_matrix,
                self.dist_coeffs
            )

            if success:
                R_cv, _ = cv2.Rodrigues(rvec)
                t_cv = np.asarray(tvec).reshape(3,)

                trans = self.tf_buffer.lookup_transform(
                    "camera_link",
                    "camera_color_optical_frame",
                    rclpy.time.Time()
                )

                q = trans.transform.rotation
                R_link_opt = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

                t_link_opt = np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])

                R_obj_link = R_link_opt @ R_cv 
                t_obj_link = R_link_opt @ t_cv + t_link_opt

                cam_pos = -R_obj_link.T @ t_obj_link

                rot_ros = Rotation.from_matrix(R_obj_link.T)

                euler_ros_zyx = rot_ros.as_euler('xyz', degrees=False)
                quat_ros = rot_ros.as_quat() 

                print("Position:", np.round(cam_pos.flatten(), 3))
                print("Euler xyz (roll, pitch, yaw):", np.round(euler_ros_zyx, 3))
                print("Quaternion:", np.round(quat_ros, 3))

                cv2.drawFrameAxes(
                    vis,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    0.1
                )

        cv2.imshow('undistorted', vis)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraEyeToHandCalibration()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()