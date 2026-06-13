
"""
@作者: RZR
@说明: 单目平面测量模块
"""


import cv2
import numpy as np
import os
import math

class MonocularPlaneMeasurer:
    def __init__(self, camera_matrix, dist_coeffs):
        self.camera_matrix = np.array(camera_matrix)
        self.dist_coeffs = np.array(dist_coeffs)
        
        # 从相机矩阵提取参数
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        
        # 旋转解耦参数（根据相机内参和焦距计算，需要标定）
        # 每1角度对应的像素个数，与分辨率和焦距有关
        self.pixel_per_deg_x = 5.36  # 需要实际标定
        self.pixel_per_deg_y = 5.34  # 需要实际标定
        
        
        print(f"相机参数初始化:")
        print(f"焦距: fx={self.fx:.2f}, fy={self.fy:.2f}")
        print(f"主点: cx={self.cx:.2f}, cy={self.cy:.2f}")
        
    def undistort_image(self, image):
        """对图像进行畸变校正"""
        h, w = image.shape[:2]
        
        # 获取最优新相机矩阵
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        
        # 畸变校正
        undistorted = cv2.undistort(
            image, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix
        )
        
        # 裁剪ROI区域
        x, y, w, h = roi
        if w > 0 and h > 0:
            undistorted = undistorted[y:y+h, x:x+w]
            
        return undistorted, new_camera_matrix
    
    def undistort_points(self, points):
        """对点坐标进行畸变校正"""
        points = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
        
        # 校正点坐标
        undistorted_points = cv2.undistortPoints(
            points, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix
        )
        
        return undistorted_points.reshape(-1, 2)
    
    def pixel_to_world_xy(self, u, v, Z):
        """
        将像素坐标转换为世界XY坐标
        u, v: 像素坐标 (校正后)
        Z: 相机到平面的距离 (米)
        返回: (X, Y) 世界坐标 (米)
        """
        # 计算归一化坐标
        x_norm = (self.cx - u) / self.fx
        y_norm = (self.cy - v) / self.fy
        
        # 计算世界坐标
        X = x_norm * Z
        Y = y_norm * Z
        
        return X, Y
    
    def rotate_decoupling(self, pixel_points, roll_deg, pitch_deg, image_width=640, image_height=480):
        """
        旋转解耦：补偿因机体俯仰、横滚旋转而造成的目标像素坐标变化
        
        参数:
        - pixel_points: [(u1, v1), (u2, v2), ...] 原始像素坐标（图像坐标系）
        - roll_deg: 横滚角（度），右倾为正
        - pitch_deg: 俯仰角（度），前倾为负
        - image_width, image_height: 图像尺寸
        
        返回:
        - decoupled_points: 解耦后的像素坐标（飞机坐标系）
        """
        decoupled_points = []
        
        # 计算角度对应的像素偏移量（在图像坐标系中）
        # 由于飞机坐标系与图像坐标系的转换，这里需要调整符号
        pitch_offset_pixels = self.pixel_per_deg_y * pitch_deg  # 俯仰角影响y方向
        roll_offset_pixels = self.pixel_per_deg_x * roll_deg    # 横滚角影响x方向
        
        # 限幅，避免过大偏移
        pitch_offset_pixels = np.clip(pitch_offset_pixels, -image_height/2, image_height/2)
        roll_offset_pixels = np.clip(roll_offset_pixels, -image_width/2, image_width/2)
        
        for point in pixel_points:
            # 转换到以图像中心为原点的坐标系
            u_center = point[0] - self.cx
            v_center = point[1] - self.cy
            
            # 应用旋转补偿
            # 注意：根据飞机坐标系定义，需要交换x,y并改变符号
            # 飞机X = 图像Y，飞机Y = -图像X
            u_decoupled = u_center - roll_offset_pixels
            v_decoupled = v_center - pitch_offset_pixels
            
            # 转换回图像坐标系
            u_final = u_decoupled + self.cx
            v_final = v_decoupled + self.cy
            
            decoupled_points.append((u_final, v_final))
            
        return decoupled_points
    
    def pixel_to_world_with_decoupling(self, u, v, plane_distance, roll_deg=0.0, pitch_deg=0.0,use_undistort=False):
        """
        将单个像素坐标转换为世界坐标（带旋转解耦）
        
        参数:
        - u, v: 像素坐标
        - plane_distance: 相机到平面的距离 (米)
        - roll_deg: 横滚角（度）
        - pitch_deg: 俯仰角（度）
        - use_undistort: 是否对点进行畸变校正
        
        返回:
        - X, Y: 世界坐标 (米)
        """
        if use_undistort:
            # 1. 对点进行畸变校正
            undistorted_point = self.undistort_points([(u, v)])[0]
            # 2. 应用旋转解耦
            decoupled_point = self.rotate_decoupling(
                [undistorted_point], roll_deg, pitch_deg
            )[0]
        else:
            # 不进行畸变校正，直接使用原始点
            decoupled_point = self.rotate_decoupling(
                [(u, v)], roll_deg, pitch_deg
            )[0]

        # 3. 转换为世界坐标
        u_decoupled, v_decoupled = decoupled_point
        X, Y = self.pixel_to_world_xy(u_decoupled, v_decoupled, plane_distance)
        
        return X, Y
    
    def get_world_position(self, fc_pos_x, fc_pos_y, yaw_rad, target_x, target_y):
        """将获取的机体坐标转换为全局坐标
        参数:
        - fc_pos_x, fc_pos_y: 飞控当前全局坐标 (米)
        - yaw_rad: 飞机航向角（弧度）范围（-π, π），0度指向飞机坐标系x轴正方向
        - target_x, target_y: 目标相对于飞机的坐标 (米)（飞机坐标系：x前，y左）
        返回:
        - global_x, global_y: 目标全局坐标 (米)
        """
        dx = -target_y
        dy = -target_x

        ref_ax_x = math.cos(yaw_rad)  # 航向在世界坐标系x方向的分量
        ref_ax_y = math.sin(yaw_rad)  # 航向在世界坐标系y方向的分量
        
        dx_world = dx * ref_ax_x + dy * (-ref_ax_y)
        dy_world = dx * ref_ax_y + dy * ref_ax_x
        
        # 计算全局坐标
        global_x = fc_pos_x + dx_world
        global_y = fc_pos_y + dy_world
        
        return global_x, global_y
    
    def measure_points_with_decoupling(self, image, pixel_points, plane_distance, 
                                      roll_deg=0, pitch_deg=0, use_image_undistort=True):
        """
        带旋转解耦的测量多个点的世界坐标
        
        参数:
        - pixel_points: [(u1, v1), (u2, v2), ...] 像素坐标列表
        - plane_distance: 相机到平面的距离 (米)
        - roll_deg: 横滚角（度），右倾为正
        - pitch_deg: 俯仰角（度），前倾为负
        - use_image_undistort: 是否对整图进行畸变校正
        """
        if use_image_undistort:
            # 方法1: 对整个图像进行畸变校正
            undistorted_img, new_cam_matrix = self.undistort_image(image)
            
            # 更新内参 (由于裁剪，主点可能变化)
            if undistorted_img.shape != image.shape:
                print("警告: 图像尺寸因畸变校正而改变，测量精度可能受影响")
            
            # 应用旋转解耦
            decoupled_points = self.rotate_decoupling(
                pixel_points, roll_deg, pitch_deg, 
                undistorted_img.shape[1], undistorted_img.shape[0]
            )
            
            world_points = []
            for u, v in decoupled_points:
                X, Y = self.pixel_to_world_xy(u, v, plane_distance)
                world_points.append((X, Y))
                
            return world_points, undistorted_img
            
        else:
            # 方法2: 只对点坐标进行畸变校正
            undistorted_pixel_points = self.undistort_points(pixel_points)
            
            # 应用旋转解耦
            decoupled_points = self.rotate_decoupling(
                undistorted_pixel_points, roll_deg, pitch_deg,
                image.shape[1], image.shape[0]
            )
            
            world_points = []
            for (u, v) in decoupled_points:
                X, Y = self.pixel_to_world_xy(u, v, plane_distance)
                world_points.append((X, Y))
                
            return world_points, image
    
    def measure_points(self, image, pixel_points, plane_distance, use_image_undistort=True):
        """测量多个点的世界坐标（兼容原有接口）"""
        return self.measure_points_with_decoupling(
            image, pixel_points, plane_distance, 0, 0, use_image_undistort
        )
    
    def measure_distance_with_decoupling(self, image, point1, point2, plane_distance, 
                                        roll_deg=0, pitch_deg=0):
        """带旋转解耦的测量两个点之间的实际距离"""
        world_points, _ = self.measure_points_with_decoupling(
            image, [point1, point2], plane_distance, roll_deg, pitch_deg, False
        )
        
        X1, Y1 = world_points[0]
        X2, Y2 = world_points[1]
        
        distance = np.sqrt((X2 - X1)**2 + (Y2 - Y1)**2)
        return distance

    def measure_distance(self, image, point1, point2, plane_distance):
        """测量两个点之间的实际距离（兼容原有接口）"""
        return self.measure_distance_with_decoupling(
            image, point1, point2, plane_distance, 0, 0
        )

class CameraMeasurementApp:
    """
        测试模块
    """
    def __init__(self, camera_matrix, dist_coeffs):
        self.measurer = MonocularPlaneMeasurer(camera_matrix, dist_coeffs)
        self.selected_points = []
        self.plane_distance = 0.075  # 默认距离0.075米
        self.show_undistorted = False
        self.measurement_mode = False
        self.use_decoupling = False  # 是否启用旋转解耦
        self.roll_deg = 0.0  # 横滚角（度）
        self.pitch_deg = 0.0  # 俯仰角（度）
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if not self.cap.isOpened():
            print("无法打开摄像头")
            exit()
    
    def run(self):
        """运行主程序"""
        print("摄像头测量程序启动")
        print("使用说明:")
        print("- 按 'm' 键进入/退出测量模式")
        print("- 在测量模式下点击图像选择点")
        print("- 按 'd' 键设置平面距离")
        print("- 按 'c' 键清除所有点")
        print("- 按 'u' 键切换畸变校正显示")
        print("- 按 'r' 键设置横滚角 (roll)")
        print("- 按 'p' 键设置俯仰角 (pitch)")
        print("- 按 'e' 键启用/禁用旋转解耦")
        print("- 按 ESC 或 'q' 键退出")
        
        cv2.namedWindow("Camera Measurement")
        cv2.setMouseCallback("Camera Measurement", self.mouse_callback)
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("获取帧失败")
                break
            
            display_frame = frame.copy()
            
            # 如果选择显示校正图像且不在测量模式下
            if self.show_undistorted and not self.measurement_mode:
                display_frame, _ = self.measurer.undistort_image(frame)
            
            # 在测量模式下绘制点和测量信息
            if self.measurement_mode:
                self.draw_measurement_info(display_frame)
            
            # 显示模式信息
            mode_text = "Measure Mode" if self.measurement_mode else "Preview mode"
            cv2.putText(display_frame, f"Mode: {mode_text}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Dis: {self.plane_distance}m", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Point: {len(self.selected_points)}", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Decouple: {'ON' if self.use_decoupling else 'OFF'}", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if self.use_decoupling else (0, 0, 255), 2)
            cv2.putText(display_frame, f"Roll: {self.roll_deg:.1f} deg", (10, 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Pitch: {self.pitch_deg:.1f} deg", (10, 180), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow("Camera Measurement", display_frame)
            
            key = cv2.waitKey(30) & 0xFF
            if key == 27 or key == ord('q'):  # ESC或q退出
                break
            elif key == ord('m'):  # 切换测量模式
                self.measurement_mode = not self.measurement_mode
                print(f"{'进入' if self.measurement_mode else '退出'}测量模式")
            elif key == ord('d'):  # 设置距离
                self.set_plane_distance()
            elif key == ord('c'):  # 清除点
                self.selected_points = []
                print("已清除所有测量点")
            elif key == ord('u'):  # 切换畸变校正显示
                self.show_undistorted = not self.show_undistorted
                print(f"{'显示' if self.show_undistorted else '隐藏'}畸变校正图像")
            elif key == ord('r'):  # 设置横滚角
                self.set_roll_angle()
            elif key == ord('p'):  # 设置俯仰角
                self.set_pitch_angle()
            elif key == ord('e'):  # 切换旋转解耦
                self.use_decoupling = not self.use_decoupling
                print(f"{'启用' if self.use_decoupling else '禁用'}旋转解耦")
        
        self.cap.release()
        cv2.destroyAllWindows()
    
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数"""
        if event == cv2.EVENT_LBUTTONDOWN and self.measurement_mode:
            self.selected_points.append((x, y))
            print(f"添加点 {len(self.selected_points)}: ({x}, {y})")
            
            # 计算并显示世界坐标
            if self.use_decoupling:
                world_points, _ = self.measurer.measure_points_with_decoupling(
                    self.get_current_frame(), [(x, y)], self.plane_distance,
                    self.roll_deg, self.pitch_deg, False
                )
            else:
                world_points, _ = self.measurer.measure_points(
                    self.get_current_frame(), [(x, y)], self.plane_distance, False
                )
                
            X, Y = world_points[0]
            print(f"  世界坐标: X={X:.4f}m, Y={Y:.4f}m")
    
    def draw_measurement_info(self, frame):
        """在帧上绘制测量信息"""
        # 绘制所有点
        for i, (x, y) in enumerate(self.selected_points):
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"{i+1}", (x+10, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # 显示世界坐标
            if self.use_decoupling:
                world_points, _ = self.measurer.measure_points_with_decoupling(
                    self.get_current_frame(), [(x, y)], self.plane_distance,
                    self.roll_deg, self.pitch_deg, False
                )
            else:
                world_points, _ = self.measurer.measure_points(
                    self.get_current_frame(), [(x, y)], self.plane_distance, False
                )
                
            X, Y = world_points[0]
            cv2.putText(frame, f"({X:.3f},{Y:.3f})m", (x+10, y+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        # 绘制连线和距离
        if len(self.selected_points) >= 2:
            for i in range(len(self.selected_points)-1):
                pt1 = self.selected_points[i]
                pt2 = self.selected_points[i+1]
                
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
                
                # 计算并显示距离
                if self.use_decoupling:
                    distance = self.measurer.measure_distance_with_decoupling(
                        self.get_current_frame(), pt1, pt2, self.plane_distance,
                        self.roll_deg, self.pitch_deg
                    )
                else:
                    distance = self.measurer.measure_distance(
                        self.get_current_frame(), pt1, pt2, self.plane_distance
                    )
                    
                mid_x = (pt1[0] + pt2[0]) // 2
                mid_y = (pt1[1] + pt2[1]) // 2
                cv2.putText(frame, f"{distance:.3f}m", (mid_x, mid_y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    def get_current_frame(self):
        """获取当前帧（用于测量计算）"""
        ret, frame = self.cap.read()
        if ret:
            return frame
        return None
    
    def set_plane_distance(self):
        """设置平面距离"""
        try:
            print(f"当前平面距离: {self.plane_distance}米")
            new_distance = input("请输入新的平面距离 (米): ").strip()
            if new_distance:
                self.plane_distance = float(new_distance)
                print(f"平面距离已设置为: {self.plane_distance}米")
        except ValueError:
            print("输入无效，保持原距离")
    
    def set_roll_angle(self):
        """设置横滚角"""
        try:
            print(f"当前横滚角: {self.roll_deg}度 (右倾为正)")
            new_roll = input("请输入新的横滚角 (度): ").strip()
            if new_roll:
                self.roll_deg = float(new_roll)
                print(f"横滚角已设置为: {self.roll_deg}度")
        except ValueError:
            print("输入无效，保持原角度")
    
    def set_pitch_angle(self):
        """设置俯仰角"""
        try:
            print(f"当前俯仰角: {self.pitch_deg}度 (前倾为负)")
            new_pitch = input("请输入新的俯仰角 (度): ").strip()
            if new_pitch:
                self.pitch_deg = float(new_pitch)
                print(f"俯仰角已设置为: {self.pitch_deg}度")
        except ValueError:
            print("输入无效，保持原角度")

# 使用您的相机参数初始化
camera_matrix = np.array([
    [303.14839604, 0.0, 325.69011423],
    [0.0, 302.73501984, 246.41753334],
    [0.0, 0.0, 1.0]
])

dist_coeffs = np.array([0.07238952, -0.12088445, 0.00071579, 0.00371255, 0.04629666])

if __name__ == "__main__":
    app = CameraMeasurementApp(camera_matrix, dist_coeffs)
    app.run()