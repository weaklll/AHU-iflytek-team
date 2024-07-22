#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
from multiprocessing import Process, Value, Array

import rospy
import subprocess

from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Image
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image, ImageDraw, ImageFont
"""
基于3.0的基础
改进切线
"""


class PIDController:
    def __init__(self, Kp, Ki, Kd, min_output=None, max_output=None, integral_limit=None):
        # 初始化PID控制器的比例、积分和微分系数
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # 用于存储上一次的误差值
        self.previous_error = 0

        # 用于存储积分累加值
        self.integral = 0

        # 输出值的最小和最大限制，可选参数
        # self.min_output = min_output
        # self.max_output = max_output

        # 积分累加值的限制，可选参数
        self.integral_limit = integral_limit

    def compute(self, error, dt):
        # 积分部分：将当前误差乘以时间增量后累加到积分累加值中
        self.integral += error * dt

        # 如果设置了积分限制，对积分累加值进行限制
        # if self.integral_limit is not None:
        #     self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        
        # 微分部分：计算当前误差和上一次误差的差值，然后除以时间增量
        derivative = (error - self.previous_error) / dt
        
        # 更新上一次误差为当前误差
        self.previous_error = error
        
        # 计算PID控制器的输出值
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # 如果设置了最小输出限制，对输出值进行限制
        # if self.min_output is not None:
        #     output = max(output, self.min_output)
        
        # 如果设置了最大输出限制，对输出值进行限制
        # if self.max_output is not None:
        #     output = min(output, self.max_output)
        
        # 返回PID控制器的输出值
        return output


class CameraConfig():
    def __init__(self, name, camera_id, brightness, contrast, exposure, img_size=(320, 240)) -> None:
        self.name = name
        self.camera_id = camera_id
        self.brightness = brightness
        self.contrast = contrast
        self.exposure = exposure
        self.img_size = img_size


def my_print(head, head_type='info', content=None):
    if content is None:
        content = ''
    if head_type == 'info':
        bg = 42
        word = 38
    elif head_type == 'warn':
        bg = 43
        word = 31
    elif head_type == 'err':
        bg = 41
        word = 38
    elif head_type == 'data':
        bg = 47
        word = 30
    else:
        bg = 45
        word = 38
    print(f"\n\033[{bg};{word}m   {head}   \033[0m\n{content}\n")

camera_fixed_config = CameraConfig(name='Fixed Cam',   # 不修改 须与变量名对应
                                  camera_id=0,  # 相机id
                                  brightness=0,  # 亮度 -64~(0)~64
                                  contrast=4,  # 对比度 0~(4)~95
                                  exposure=0, # 0 自动曝光  1~10000 手动曝光值
                                  img_size=(320, 240)  # 画幅
                                 )


disarm    = 0  # 上锁   线速度为0
arm       = 1  # 解锁   线速度为1
vel_mode  = 2  # 速度模式   线速度为巡线速度
angular = 3 # 旋转模式
turn_left_l = 4
turn_right_l = 5
stop = 6 # 刹车


l_0x40 = 0.63  # 线位移 m
a_0x40 = 27.0  # 角位移 °
vel_2_0x40 = 1.0 / l_0x40  # 分母为 0x40/(+64)校准时 1s内行驶的距离 m
omega_2_0x40 = 0.465 / a_0x40  # 分母为 最大值0x40/(+64)校准时 1s内旋转的角度 °

lane_list = ["left", "right", " dual"]

control_pts_offset = 35

control_points = np.float32(np.load('/home/iflytek/move/src/send_goals/scripts/perspective.npy'))

transform_dst_size = (320, 240)
dst_pts = np.float32([(0,                     0),
                      (transform_dst_size[0], 0),
                      (transform_dst_size[0], transform_dst_size[1]),
                      (0,                     transform_dst_size[1])])

# canny
low_threshold = 50
high_threshold = 150
kernel_size = 7

binary_type = 1  # 0 cv2.THRESH_BINARY(黑底白线->黑底白线 / 白底黑线->白底黑线)   1 cv2.THRESH_BINARY_INV(白底黑线->黑底白线 / 黑底白线->白底黑线)


#二值化阈值
#vitual_line -> utils -> thresh_binary.py
# binary_l = 139 # 光线较亮


# binary_l = 180  #晚上开灯时
# binary_l = 153    #下午晴天

binary_l = 160    #黄昏

"""
LaneLines
"""

n_windows = 9

margin = 40

min_pix = 50

left_lane_thresh = 100
right_lane_thresh = 100

# 切换巡线
lane_change = 1
left_exit = 0
right_exit = 0

# 直角
right_angle_1 = 0
right_angle_2 = 0
turn_left = 0


# 终点
final_stop = 0

# 计时
pre_time = 0
count = 0


xl_b_pix = 49
xr_b_pix = 268

x_real = 0.39  # m  左右  (固定值 与地图尺寸相关)
y_real = 0.19  # m  前后  (测量值 与控制点选取相关)
x_pix2m = x_real / dst_pts[1][0] - dst_pts[0][0]
y_pix2m = y_real / dst_pts[2][1] - dst_pts[1][1]

straight_threshold = 0.04


max_vx = 0.2    # 表征最高限度 0.2 ，控制线速度  0.3 
kp_x = 0.1    # 表征弯道降速 0.001，
kp_y = 100        # 表征横向纠偏 3
kp_w = -800      # -800 表征弯道转速 120  1000

buffer_size = 1  # 视选取的控制点超前车身的情况 适当增加缓存的指令数量 以达到滞后控制的效果



class Thresholding:
    """
    Canny 边缘检测
    """
    def __init__(self, img_size=(640, 480)):
        self.resize = img_size[0] / 640
        self.low_threshold = low_threshold
        self.high_threshold = high_threshold
        self.kernel_size = kernel_size

    def forward(self, img, lane=2, image=False, debug=False):

        ##########################################################################################
        """
        Method 1  canny
        """
        
        # 转换为灰度图像
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 使用高斯模糊减少噪声
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)

        # Canny 边缘检测
        edges = cv2.Canny(blurred, self.low_threshold, self.high_threshold)

        # 创建一个闭运算的核
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # 在闭运算结果上应用dilate和erode操作
        dilated = cv2.dilate(closed, kernel, iterations=1)
        eroded = cv2.erode(dilated, kernel, iterations=1)

        # 创建掩码，黑色为线条，其它部分为白色
        dual_lane = np.where(eroded == 255, 0, 255).astype(np.uint8)
        dual_lane = 255 - dual_lane
        ##########################################################################################
        
        ##########################################################################################
        """
        Method 2  threshold
        """
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # # threshold
        # _, dual_lane = cv2.threshold(img, binary_l, 255, binary_type)

        # dual_lane = cv2.bitwise_not(dual_lane)

        # black_pixels = (dual_lane == 255).sum(axis=0)
        # threshold = 30

        # # 判断是否有反光
        # if np.any(black_pixels > threshold):
        #     print("反光发现")
        #     for i in range(len(black_pixels)):
        #         if black_pixels[i] > threshold:
        #             dual_lane[:, i] = 0  # 将mask中对应列的黑色像素变为白色
        # else:
            
            # print("无反光")   

        # ##########################################################################################
        
        
        # if lane == 0:  # 选择左线
        #     dual_lane[:, int(275 * self.resize):] = 0  # 将右半图像涂黑
        # elif lane == 1:  # 选择右线
        #     dual_lane[:, :int(375 * self.resize)] = 0  # 将左半图像涂黑
        
        
            
        return dual_lane

  

class PerspectiveTransformation:

    def __init__(self, img_size=None):
        if img_size is None:
            self.img_size = (320, 240)
        else:
            self.img_size = img_size
        self.src = control_points
        self.dst_size = transform_dst_size
        self.dst_pts = dst_pts
        self.M = cv2.getPerspectiveTransform(self.src, self.dst_pts)
        self.M_inv = cv2.getPerspectiveTransform(self.dst_pts, self.src)

    def forward(self, img, flags=cv2.INTER_LINEAR):
       

        return cv2.warpPerspective(img, self.M, self.dst_size, flags=flags)

        
class LaneLines:
    def __init__(self, dst_size=(160, 120)):

        self.img_size = dst_size
        self.midpoint = self.img_size[0] // 2  # 图像垂直中心
        self.n_windows = n_windows  # 滑动窗口数量
        self.window_height = np.int(self.img_size[1] // n_windows)  # 单个滑动窗口高度
        self.half_height = self.window_height // 2
        self.margin = margin  # 滑动窗口左右宽度
        self.min_pix = min_pix  # 认为滑动窗口中有车道线的最小像素值

        self.canvas = np.zeros((self.img_size[1], self.img_size[0], 3), np.uint8)

        self.left_fit = None
        self.right_fit = None

        self.binary = None
        self.nonzero = None
        self.nonzero_x = None
        self.nonzero_y = None
        self.clear_visibility = True
        self.dir = []

    def forward(self, img, lane=1):
        self.extract_features(img)
        img = self.fit_poly(img, lane=lane)
        gradiant, differ_pix = self.calculate(lane=lane)
        return img, gradiant, differ_pix

    def extract_features(self, img):
        self.nonzero = img.nonzero()
        self.nonzero_x = np.array(self.nonzero[1])
        self.nonzero_y = np.array(self.nonzero[0])

    def pixels_in_window(self, center):
        t_l = (center[0] - self.margin, center[1])
        b_r = (center[0] + self.margin, center[1] + self.window_height)

        coord_x = (t_l[0] <= self.nonzero_x) & (self.nonzero_x <= b_r[0])
        coord_y = (t_l[1] <= self.nonzero_y) & (self.nonzero_y <= b_r[1])
        return self.nonzero_x[coord_x & coord_y], self.nonzero_y[coord_x & coord_y]

    def find_lane_pixels(self, img, lane=1):

        assert (len(img.shape) == 2)

        left_ang = 0
        left_zero = 0
        left_line = 0
        left_line_posi = 0
        left_ang_sum = 0

        right_ang = 0
        right_zero = 0
        right_line = 0
        right_line_posi = 0
        right_ang_sum = 0

        out_img = np.dstack((img, img, img))
        histogram = np.sum(img[img.shape[0] // 4:, :], axis=0)  # img[h, w, channel]
        if lane == 0:
            # 找到直方图中左起第一个峰，认为是左线的起点
            left_x_base = np.argmax(histogram[:self.midpoint])
            # 当前左线x位置
            left_x_current = left_x_base

            y_current = self.img_size[1]

            # 创建空列表来接收左线像素点的坐标
            left_x, left_y = [], []

            # 遍历每个滑动窗口
            for i in range(self.n_windows):
                y_current -= self.window_height
                center_left = (left_x_current, y_current)
                # if image:
                cv2.rectangle(out_img,
                              (left_x_current - self.margin, y_current),
                              (left_x_current + self.margin, y_current + self.window_height),
                              (200, 200, 0), 2)
                # 找当前窗口中的非0像素点
                good_left_x, good_left_y = self.pixels_in_window(center_left)

                # 将当前窗口中的车道线像素点的坐标记录
                left_x.extend(good_left_x)
                left_y.extend(good_left_y)

                # print('left min_pix', len(good_left_x))
                if len(good_left_x) > self.min_pix:
                    left_x_current = np.int32(np.mean(good_left_x))  # 更新有效窗口的中心位置
                    left_line_posi = i

                else:
                    left_zero = left_zero + 1
                
                if len(good_left_x)>850:
                    left_ang = left_ang + 1
                # print(f"当前左线第{i}个窗口有{len(good_left_x)}个车道线点")
                left_ang_sum = left_ang_sum + len(good_left_x)

            if abs(left_x_current - left_x_base) <= 15:
                left_line = 1

          

            return left_x, left_y,left_ang,left_ang_sum,left_zero,left_line,left_line_posi

        elif lane == 1:
            right_x_base = np.argmax(histogram[self.midpoint+40:]) + self.midpoint+40
            right_x_current = right_x_base
            y_current = self.img_size[1]
            right_x, right_y = [], []
            for i in range(self.n_windows):
                y_current -= self.window_height
                center_right = (right_x_current, y_current)
                good_right_x, good_right_y = self.pixels_in_window(center_right)
                right_x.extend(good_right_x)
                right_y.extend(good_right_y)
                if len(good_right_x) > self.min_pix:
                    right_x_current = np.int32(np.mean(good_right_x))
                    right_x_current = np.int32(np.mean(good_right_x))
                    right_line_posi = i
                
                else:
                    right_zero = right_zero +1
                    
                    
                if len(good_right_x)>850:
                    right_ang = right_ang + 1
                right_ang_sum = right_ang_sum + len(good_right_x)
                # print(f"当前右线线第{i}个窗口有{len(good_right_x)}个车道线点")
            if abs(right_x_current - right_x_base) <= 15 :
                right_line = 1
            
           
            return right_x, right_y,right_ang,right_ang_sum,right_zero,right_line,right_line_posi
        else:
            pass


    def fit_poly(self, img, lane=2):

        global lane_change 
        global left_exit 
        global right_exit
        global right_angle_1
        global right_angle_2
        global final_stop
        
        global turn_left # 转向
        global pre_time  
        global count    # 计数器

        out_img = self.canvas.copy()

        lane = 1

        # 确定边线
        left_x, left_y,left_ang,left_ang_sum,left_zero,left_line,left_line_posi = self.find_lane_pixels(img, lane=0)
        right_x, right_y,right_ang,right_ang_sum,right_zero,right_line,right_line_posi = self.find_lane_pixels(img, lane=1)

        # 软件清零，防止误判
        if right_angle_2 == 1:
            right_angle_1 = 0
            right_angle_2 = 0

        # 选择巡线
        if len(left_y) >= left_lane_thresh:
            self.left_fit = np.polyfit(left_y, left_x, 1)
            left_exit = 1
        
        else:
            left_exit = 0

        if len(right_y) >= right_lane_thresh:
            self.right_fit = np.polyfit(right_y, right_x, 1)
            right_exit = 1
        
        else:
            right_exit = 0


        # 选择车道线
        if len(right_y) <= right_lane_thresh:
            
            lane_change = 0 
            lane = 0
           
        if len(left_y) <= left_lane_thresh :
            lane_change = 1
            lane = 1 

        
        
        # print("                              ")

        # if left_exit == 0 and right_exit == 0 :
        #     final_stop = 1
        #     print("已到达终点")

       
        # print(f"right_ang_sum = {right_ang_sum}")
        # print(f"left_ang_sum = {left_ang_sum}")
        # 到达左转第一标志位
        # if left_ang >= 1 and right_ang == 0 and right_angle_1 == 0 and right_ang_sum <2200 and right_ang_sum >1000 :
        # if left_ang >= 1 and right_ang == 0 and right_angle_1 == 0 and right_ang_sum > 0:
        if right_line == 1 and right_line_posi == 8 and (left_line_posi > 0 and left_line_posi <5) :
        
            turn_left = 1
            right_angle_1 = 1 
            
            print("到达左转第一标志位")

        # 到达右转第一标志位
        # elif left_ang == 0 and right_ang >= 1 and right_angle_1 == 0 and left_ang_sum <2200 and left_ang_sum > 1000:
        # elif left_ang == 0 and right_ang >= 1 and right_angle_1 == 0 and left_ang_sum>0 :
        elif left_line == 1 and left_line_posi == 8 and (right_line_posi > 0 and right_line_posi <5):
            turn_left = 0
            right_angle_1 = 1 
            
            print("到达右转第一标志位")
            

        

        

        # 强制切线
        if right_angle_1:
            if turn_left:
                lane_change = 1
                print("强制切换到右线")
                if len(right_y) <= right_lane_thresh:
                    my_print("强制切换右线错误,已修正",'warn')
                    lane_change = 0
                    turn_left = 0
               
            
            else:
                lane_change = 0
                print("强制切换到左线")
                
                if len(left_y) <= left_lane_thresh:
                    my_print("强制切换左线错误，已修正",'warn')
                    lane_change = 1
                    turn_left = 1
                
        print("                              ")
        
        # 到达第二标志位
        if right_angle_1 ==1 or right_angle_1 == 0:
            if left_ang >= 1 and right_ang >= 1 and left_line_posi != right_line_posi:
                right_angle_2 = 1
                my_print("到达第二标志位")
                # print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        
        print(f"right_angle_1 == {right_angle_1}")
        print(f"当前转向 = {turn_left}")
        print(f"当前巡线 = {lane_change}")
        print(f"right_angle_2 = {right_angle_2}")
        print(f"final_stop = {final_stop}")

        print("****************************")

        if count == 1:
            if time.time() - pre_time < 2.0 :
                if right_angle_2:

                    right_angle_2 = 0
                    my_print("第二标志位判断错误")
                

            else :
                count = 0 # 计数器清零

        # 检测到直角元素
        if right_angle_2 == 1:
            # 防止直角弯短时间误判
            if count == 0:
                pre_time = time.time()
                count = 1
                    
        
         
       
     
            
    def calculate(self, lane=1):
        global lane_change 
       

        differ_pix = self.measure(lane=lane_change)
        if lane_change :
            gradiant = self.right_fit[0]
        
        else :
            gradiant = self.left_fit[0]
            
        gradiant = round(gradiant, 5)
        if not gradiant:  # 防止 div 0
            gradiant = 0.00001
        if len(self.dir) > 5:
            self.dir.pop(0)

      
        return gradiant, differ_pix

    def measure(self, lane=1):
       
        if lane == 0:  # 左线
            xl = np.dot(self.left_fit, [transform_dst_size[1], 1])
            differ = -((xr_b_pix + xl_b_pix)/2 -xl - 119 - 1)
            print(differ)

        elif lane == 1:  # 右线
            xr = np.dot(self.right_fit, [transform_dst_size[1], 1])
            differ = xr - (xr_b_pix + xl_b_pix)/2 - 115.17 - 15
            # print(differ)
           
      
        return differ
class FindLaneLines:
    def __init__(self, img_size_=(320, 240)):
        """ Init Application"""
        self.img_size = img_size_
        self.thresholding = Thresholding(img_size=img_size_)  # 二值化
        self.transform = PerspectiveTransformation(img_size=img_size_)  # 透视变换
        self.lane_lines = LaneLines(dst_size=transform_dst_size) # 扫线

    
    def forward(self, img, lane):

        image = True
        debug = False
        # if image:
        #     cv2.imshow('undistort', img)
        #     cv2.waitKey(0)
                

        out_img = np.copy(img)
        img = self.thresholding.forward(img, lane=lane)
        

        
        
        img = self.transform.forward(img)
        # if image:
        #     cv2.imshow('top down view', img)
        #     if debug:
        #         cv2.waitKey(0)
        
        img, gradiant, differ_pix = self.lane_lines.forward(img, lane=lane)
        # if image:
        #     cv2.imshow('lane in top down view', img)
        #     if debug:
        #         cv2.waitKey(0)

        

        differ = differ_pix * x_pix2m   # 像素距离转换成实际距离
        return out_img, gradiant, differ  
    
    def process_frame(self, img, lane):
        out_img, gradiant, differ = self.forward(img, lane)

        return out_img, gradiant, differ


def lane_detection(flag_task_l, img_mem_front_l, flag_lane_l, number_lane_l, cmd_chassis_l):

    global lane_change  
    
    global right_angle_1
    global right_angle_2
    global turn_left

    findLaneLines_l = FindLaneLines(img_size_=camera_fixed_config.img_size)
    pid_1 = PIDController(Kp = -800, Ki = 0, Kd = 12)
    keep_lane = False
    
    my_print("这里是车道线检测进程")

    cmd_buffer = [[vel_mode, 0, 0, 0]] * buffer_size
    while flag_task_l.value:

        if flag_lane_l.value:
            keep_lane = True
            img = np.array(img_mem_front_l, np.uint8)

            try:
                
                
                out_img, gradiant, differ = findLaneLines_l.process_frame(img, number_lane_l.value)


                out_put = pid_1.compute(error = differ, dt = 0.01)

                cmd_future = [vel_mode,                                                   # 模式
                              min(round(max_vx - 0.2 + kp_x / abs(gradiant), 2), max_vx), # 前进速度  有弯则适当减速 <max_vx
                              round(kp_y * 0, 2),                                    # 横向对正车道中线
                              round(out_put, 2)]                                  # 转弯速度
                
                if right_angle_2 == 1:
                    if turn_left == 1:
                        cmd_future[0] = turn_left_l
                    
                    else :
                        cmd_future[0] = turn_right_l
            
                if flag_lane_l.value:
                    cmd_chassis_l[:] = cmd_buffer[0]
                    # print("cmd vel lane", cmd_chassis_l[:])
                    cmd_buffer.pop(0)
                    cmd_buffer.append(cmd_future)
                else:
                    cmd_chassis_l[:] = [vel_mode, 0, 0, 0]
            except (TypeError, ZeroDivisionError):
                pass

        else:
            if keep_lane:  # 确保退出车道线行驶后能停车
                cmd_chassis_l[:] = [vel_mode, 0, 0, 0]
                keep_lane = False

            time.sleep(0.1)

bridge = CvBridge()

def img_callback(msg,img_mem_fixed):
    """
    获取图像
    """
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    img = cv2.flip(img, 1)
    img = cv2.resize(img, (320, 240))
    img_mem_fixed[:] = img.copy()




def camera(flag_task_v, cmd_chassis_v,img_mem_fixed):
    """
    采集指定编号相机的帧

    Args:
        flag_task_v:        任务进行标志位 (共享)
        cmd_chassis_v:      底盘控制 (共享)
    """
    my_print("这里是读取固定位摄像头进程")
    rospy.init_node('camera_node', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", ROSImage, img_callback, callback_args=img_mem_fixed)
    rospy.spin()



def chassis_control(flag_task_c, cmd_chassis_c):

    my_print("这里是行驶进程")

    
    rospy.init_node('linetrack', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    msg = Twist()
    count_c = 0
    last_instruction = []

    while flag_task_c.value:
        if cmd_chassis_c[:] == last_instruction:
            continue

        if cmd_chassis_c[0] in [disarm, arm]:
            pass

         # velocity control
        elif cmd_chassis_c[0] == vel_mode:
            

            vel = [cmd_chassis_c[1] * vel_2_0x40, 
                   cmd_chassis_c[2] * vel_2_0x40, 
                   cmd_chassis_c[3] * omega_2_0x40]
            
            
            if vel[0] < 0:
                vel[2] = -vel[2]
            
            msg.linear.x = vel[0]
            msg.angular.z = vel[2]
            cmd_vel_pub.publish(msg)
            time.sleep(.001)
        
        elif cmd_chassis_c[0] == turn_left_l or cmd_chassis_c[0] == turn_right_l:
            
            rospy.set_param('num',1)
            if count_c >= 3:
                time.sleep(0.5)

                msg.linear.x = 0.2
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0; 
                
                for i in range(3):
                    cmd_vel_pub.publish(msg)
                    time.sleep(0.1)

                subprocess.run(["aplay","/home/iflytek/move/src/send_goals/voice/Finish.wav"])
                time.sleep(1)

                while True:
                    time.sleep(0.1)
            
            rate = rospy.Rate(10)
            msg.linear.x = 0.39
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0

            if cmd_chassis_c[0] == turn_left_l:
                msg.angular.z = 2.5
            
            else:
                msg.angular.z = -2.6


            rospy.loginfo("开始旋转")

            for i in range(5):
                cmd_vel_pub.publish(msg)
                time.sleep(0.1)

            rospy.loginfo("旋转结束")

            time.sleep(0.5)
            count_c = count_c + 1
            
            # while True:
            
                # msg.linear.x = 0.0
                # msg.linear.y = 0.0
                # msg.linear.z = 0.0
                # msg.angular.x = 0.0
                # msg.angular.y = 0.0
                # msg.angular.z = 0; 
                # cmd_vel_pub.publish(msg)
                
            # time.sleep(0.1)
            

        elif cmd_chassis_c[0] == stop:

            
            rospy.loginfo("刹车")

            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0; 

            while True:
                cmd_vel_pub.publish(msg)
                time.sleep(0.1)
            


        last_instruction = cmd_chassis_c[:]


    my_print("行驶进程结束")

def get_param():
    if_follow_line=0
    rospy.init_node('get_param_node', anonymous=True)
    while not if_follow_line:
        if_follow_line=rospy.get_param('/if_follow_line', 0)

    rospy.loginfo("到达巡线起点，即将开始巡线")


   

if __name__ == "__main__":


    # 
    from multiprocessing import Process, Value, Array, shared_memory
    import numpy as np
    import time
    img_shape = (camera_fixed_config.img_size[1], camera_fixed_config.img_size[0], 3)
    img_size = np.prod(img_shape)

    

    # 创建用于存储固定图像的共享内存
    shm_fixed = shared_memory.SharedMemory(create=True, size=img_size * np.uint8().itemsize)
    img_mem_fixed = np.ndarray(img_shape, dtype=np.uint8, buffer=shm_fixed.buf)
    img_mem_fixed.fill(0)  # 将图像初始化为全黑

    flag_task = Value('i', 1)  # 任务进行标志位(默认开启)

    flag_lane = Value('i', 0)  # 车道线辅助行驶标志(默认开启)
    number_lane = Value('i', 0)  # 车道线选择   0 左  1 右  2 双

    cmd_chassis = Array('d', 4)  # 底盘控制 [1 linear.x linear.y linear.z angular.z] v = ωr
    cmd_chassis[0] = arm  # 解锁

 
    p_video_fixed = Process(target=camera,
                            args=(flag_task, cmd_chassis, img_mem_fixed))
                                  
    # 车道线子进程
    p_lane = Process(target=lane_detection,
                     args=(flag_task, img_mem_fixed, flag_lane, number_lane, cmd_chassis))
    
    # 底盘控制子进程
    p_chassis = Process(target=chassis_control,
                        args=(flag_task, cmd_chassis))
    
    my_print("开启固定位摄像头读取帧进程")
    p_video_fixed.start()

    my_print("开启底盘行驶进程")
    p_chassis.start()
    time.sleep(1)

    get_param()
    # while True:
    #     if_follow_line = rospy.get_param('/if_follow_line',0)
    #     if if_follow_line == 1:
    #         print('----------开始巡线----------')
    #         break
        
    

    
    my_print("开启车道线检测进程")
    p_lane.start()
    
    flag_lane.value = 1
    while flag_task.value:
        time.sleep(1)
    
    
   

