# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
from multiprocessing import Process, Value, Array
import sharedmem
import rospy
from geometry_msgs.msg import Twist
from PIL import Image, ImageDraw, ImageFont
import subprocess
import csv

""""
改写第一标志位
实现低速直角弯全程
停车没写

"""



class CameraConfig():
    def __init__(self, name, camera_id, brightness, contrast, exposure, img_size=(320, 240)) -> None:
        self.name = name
        self.camera_id = camera_id
        self.brightness = brightness
        self.contrast = contrast
        self.exposure = exposure
        self.img_size = img_size


class PID():
    def __init__(self, kp, ki, kd) -> None:
        self.p = kp
        self.i = ki
        self.d = kd

def overlayPNG(imgBack, imgFront, pos=None):
    if pos is None:
        pos = [0, 0]

    hf, wf, cf = imgFront.shape
    hb, wb, cb = imgBack.shape
    *_, mask = cv2.split(imgFront)
    maskBGRA = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGRA)
    maskBGR = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    imgRGBA = cv2.bitwise_and(imgFront, maskBGRA)
    imgRGB = cv2.cvtColor(imgRGBA, cv2.COLOR_BGRA2BGR)

    imgMaskFull = np.zeros((hb, wb, cb), np.uint8)
    imgMaskFull[pos[1]:hf + pos[1], pos[0]:wf + pos[0], :] = imgRGB
    imgMaskFull2 = np.ones((hb, wb, cb), np.uint8) * 255
    maskBGRInv = cv2.bitwise_not(maskBGR)
    imgMaskFull2[pos[1]:hf + pos[1], pos[0]:wf + pos[0], :] = maskBGRInv

    imgBack = cv2.bitwise_and(imgBack, imgMaskFull2)
    imgBack = cv2.bitwise_or(imgBack, imgMaskFull)

    return imgBack

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


# #####################################################################
# camera
# ########
camera_fixed_config = CameraConfig(name='Fixed Cam',   # 不修改 须与变量名对应
                                  camera_id=0,  # 相机id
                                  brightness=0,  # 亮度 -64~(0)~64
                                  contrast=4,  # 对比度 0~(4)~95
                                  exposure=0, # 0 自动曝光  1~10000 手动曝光值
                                  img_size=(320, 240)  # 画幅
                                 )

"""
相机内参
可以使用ros中的标定工具camera_calibration来获取
"""
# camera_matrix
# 从  ~/.ros/camera_info/head_camera.yaml >> camera_matrix >> data
# [fx, 0, cx, 0, fy, cy, 0, 0, 1]
# 转为  np.array()
# [[fx, 0, cx],
#  [0, fy, cy],
#  [0, 0, 1]]
camera_matrix = np.array([[411.3261791696689, 0.0, 323.6570089289733],  # [fx, 0, cx]s
                          [0.0, 408.1251597540983, 238.7885974162409],  # [0, fy, cy]
                          [0.0, 0.0, 1.0]],
                         dtype=np.float64)
# distortion_coefficients
# 从  ~/.ros/camera_info/head_camera.yaml >> distortion_coefficients >> data
# [k1, k2, p1, p2, 0]
# 转为  np.array()
# [k1, k2, p1, p2]
distortion_coefficients = np.array([-0.3085708052694404, 0.07981579355064194, 0.0008839337992127878, -0.0009765228024578548],
                                   dtype=np.float64)
# projection_matrix
# 从  ~/.ros/camera_info/head_camera.yaml >> projection_matrix >> data
# [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
# 转为  np.array()
# [[fx, 0, cx],
#  [0, fy, cy],
#  [0, 0, 1]]
projection_matrix = np.array([[316.1893310546875, 0.0, 323.9889318368223],
                              [0.0,        358.1781311035156, 239.0412111827609],
                              [0.0,               0.0,        1.0]],
                             dtype=np.float64)
                             
# #####################################################################


# #####################################################################
# chassis
# ########
# 模式位
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

# #####################################################################


# #####################################################################
# lane_detection
# ######
lane_list = ["left", "right", " dual"]

"""
control points
"""
# 实际控制点离开车道线的像素距离 pix
control_pts_offset = 35

# 存储坐标点
control_points = np.float32(np.load('/home/ucar/visial_line/perspective.npy'))

transform_dst_size = (320, 240)
dst_pts = np.float32([(0,                     0),
                      (transform_dst_size[0], 0),
                      (transform_dst_size[0], transform_dst_size[1]),
                      (0,                     transform_dst_size[1])])

"""
thresholding
"""
low_threshold = 50
high_threshold = 150
kernel_size = 7

binary_type = 1  # 0 cv2.THRESH_BINARY(黑底白线->黑底白线 / 白底黑线->白底黑线)   1 cv2.THRESH_BINARY_INV(白底黑线->黑底白线 / 黑底白线->白底黑线)
# 以下 二值化 参数配合/utils/thresh_binary.py获取
# binary_l = 177
# binary_l = 176  # 光线较暗
binary_l = 177  # 光线较亮
# binary_type = 0  # 0 cv2.THRESH_BINARY(黑底白线->黑底白线 / 白底黑线->白底黑线)   1 cv2.THRESH_BINARY_INV(白底黑线->黑底白线 / 黑底白线->白底黑线)
# # 以下 二值化 参数配合/utils/thresh_binary.py获取
# binary_l = 137


"""
LaneLines
"""
# 滑动窗口数量
n_windows = 9
# 窗口左右覆盖范围 pix
margin = 40
# 单个滑动窗口内确认出现车道线所需的最少像素点数 pix
min_pix = 50

# 二值图中左/右线像素点数阈值 pix
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
"""
curvature
"""
# 以下 车道线 参数配合/utils/lane_pix.py获取
# 对正车道线时 俯视视角中 左/右车道线底部的x轴像素坐标
xl_b_pix = 56
xr_b_pix = 279

x_real = 0.40  # m  左右  (固定值 与地图尺寸相关)
y_real = 0.41  # m  前后  (测量值 与控制点选取相关)
# 像素距离到实际距离的转换系数
x_pix2m = x_real / dst_pts[1][0] - dst_pts[0][0]
y_pix2m = y_real / dst_pts[2][1] - dst_pts[1][1]

# 认为车道线为直线时 车道线一次拟合的斜率的绝对值 的阈值
straight_threshold = 0.04
# straight_threshold = 0.10


"""
Kp
"""
max_vx = 0.2    # 表征最高限度 0.2 ，控制线速度
kp_x = 0.1    # 表征弯道降速 0.001，
kp_y = 100        # 表征横向纠偏 3
kp_w = -800      # 表征弯道转速 120

buffer_size = 1  # 视选取的控制点超前车身的情况 适当增加缓存的指令数量 以达到滞后控制的效果

# #####################################################################

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
    """
    视角转换类，进行原始视角和俯视视角之间的仿射变换
    """

    def __init__(self, img_size=None):
        """
        初始化

        Parameters:
            img_size: 图片尺寸
        """
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
        """
        原始视角  -->  俯视视角

        Parameters:
            img: 原始视角图
            flags: 使用双线性插值

        Returns:
            Image: 俯视视角图
        """
        return cv2.warpPerspective(img, self.M, self.dst_size, flags=flags)

    def backward(self, img, flags=cv2.INTER_LINEAR):
        """
        俯视视角  -->  原始视角

        Parameters:
            img: 俯视视角图
            flags: 使用双线性插值

        Returns:
            Image: 原始视角图
        """
        return cv2.warpPerspective(img, self.M_inv, self.img_size, flags=flags)
        
class LaneLines:
    """
    提取车道线
    """
    def __init__(self, dst_size=(160, 120)):
        """
        初始化

        Parameters:
            img_size_: 图片尺寸
        """
        # np.set_printoptions(precision=6, suppress=True)
        # 超参数
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

    def forward(self, img, lane=2, image=False, debug=False):
        """
        检测图像中的车道线

        Parameters:
            img: 俯视图阈值化后的二值图
            lane: 车道线选择    0 left   1 right   2 dual
            image: 是否显示处理过程
            debug: 是否单步显示
        Returns:
            out_img: 包含车道线信息的RGB图
        """


        self.extract_features(img)
        img = self.fit_poly(img, image=image, debug=debug)
        gradiant, differ_pix, direction = self.calculate()
        return img, gradiant, differ_pix, direction

    def extract_features(self, img):
        """
        提取二值图中的非0像素点的坐标

        Parameters:
            img: 二值图
        """
        self.nonzero = img.nonzero()
        # self.nonzero = np.where(img == 0)
        self.nonzero_x = np.array(self.nonzero[1])
        self.nonzero_y = np.array(self.nonzero[0])

    def pixels_in_window(self, center):
        """
        返回特定窗口中的像素点

        Parameters:
            center: 窗口中线x坐标

        Returns:
            pixel_x: 窗口中像素点的x坐标
            pixel_y: 窗口中像素点的y坐标
        """
        t_l = (center[0] - self.margin, center[1])
        b_r = (center[0] + self.margin, center[1] + self.window_height)

        coord_x = (t_l[0] <= self.nonzero_x) & (self.nonzero_x <= b_r[0])
        coord_y = (t_l[1] <= self.nonzero_y) & (self.nonzero_y <= b_r[1])
        return self.nonzero_x[coord_x & coord_y], self.nonzero_y[coord_x & coord_y]

    def find_lane_pixels(self, img, lane=2, image=False, debug=False):
        """
        找到属于车道线的像素点

        Parameters:
            img: 俯视图阈值化后的二值图
            lane: 车道线选择    0 left   1 right   2 dual
            image: 是否显示处理过程
            debug: 是否单步显示
        Returns:
            left_x: 左线像素点的x坐标
            left_y: 左线像素点的y坐标
            right_x: 右线像素点的x坐标
            right_y: 右线像素点的y坐标
            out_img: 后续为车道线涂色用的RGB图
        """
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

        # 创建一张用来后续做可视化的三通道图片
        out_img = np.dstack((img, img, img))

        # 创建输入的二值图底部一个滑窗高度的区域的直方图
        histogram = np.sum(img[img.shape[0] // 4:, :], axis=0)  # img[h, w, channel]

        # 寻找左线
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

            print(f"left_ang = {left_ang}")
            print(f"left_zero = {left_zero}")
            print(f"left_x_base = {left_x_base}")
            print(f"left_x_current = {left_x_current}")
            print(f"left_line = {left_line}")
            print(f"left_line_posi = {left_line_posi}")
            
        

            # 可视化
            # if image:
            out_img[left_y, left_x] = [255, 0, 0]
            cv2.line(out_img, (xl_b_pix, self.img_size[1] - 10), (xl_b_pix, self.img_size[1]), (255, 255, 255), 5)
            cv2.line(out_img, (xr_b_pix, self.img_size[1] - 10), (xr_b_pix, self.img_size[1]), (255, 255, 255), 5)
            cv2.imshow('slide windows L', out_img)
            cv2.moveWindow("slide windows L", 25, 350)
            if debug:
                cv2.waitKey(0)
            return left_x, left_y,left_ang,left_ang_sum,left_zero,left_line,left_line_posi
        # 寻找右线
        elif lane == 1:
            # 找到直方图右半区域中的第一个峰
            right_x_base = np.argmax(histogram[self.midpoint+40:]) + self.midpoint+40
            right_x_current = right_x_base
            y_current = self.img_size[1]

            right_x, right_y = [], []

            

            for i in range(self.n_windows):
                y_current -= self.window_height
                center_right = (right_x_current, y_current)
                # if image:
                cv2.rectangle(out_img,
                              (right_x_current - self.margin, y_current),
                              (right_x_current + self.margin, y_current + self.window_height),
                              (255, 200, 0), 2)

                good_right_x, good_right_y = self.pixels_in_window(center_right)
                

                right_x.extend(good_right_x)
                right_y.extend(good_right_y)

                # print('right min_pix', len(good_right_x))
                if len(good_right_x) > self.min_pix:
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
            
            print(f"right_ang = {right_ang}")
            print(f"right_zero = {right_zero}")
            print(f"right_x_base = {right_x_base}")
            print(f"right_x_current = {right_x_current}")
            print(f"right_line = {right_line}")
            print(f"right_line_posi = {right_line_posi}")
            
            # if image:
            out_img[right_y, right_x] = [0, 0, 255]
            cv2.line(out_img, (xl_b_pix, self.img_size[1] - 10), (xl_b_pix, self.img_size[1]), (255, 255, 255), 5)
            cv2.line(out_img, (xr_b_pix, self.img_size[1] - 10), (xr_b_pix, self.img_size[1]), (255, 255, 255), 5)
            cv2.imshow('slide windows R', out_img)
            cv2.moveWindow("slide windows R", 25, 350)
            if debug:
                cv2.waitKey(0)
            return right_x, right_y,right_ang,right_ang_sum,right_zero,right_line,right_line_posi
        # 两条线都寻找
        else:
            left_x_base = np.argmax(histogram[:self.midpoint])
            right_x_base = np.argmax(histogram[self.midpoint:]) + self.midpoint
            left_x_current = left_x_base
            right_x_current = right_x_base
            y_current = self.img_size[1]

            left_x, left_y, right_x, right_y = [], [], [], []

          
            for i in range(self.n_windows):
                y_current -= self.window_height
                center_left = (left_x_current, y_current)
                center_right = (right_x_current, y_current)
                # if image:
                cv2.rectangle(out_img,
                              (left_x_current - self.margin, y_current),
                              (left_x_current + self.margin, y_current + self.window_height),
                              (150, 200, 0), 2)
                cv2.rectangle(out_img,
                              (right_x_current - self.margin, y_current),
                              (right_x_current + self.margin, y_current + self.window_height),
                              (150, 200, 0), 2)

                good_left_x, good_left_y = self.pixels_in_window(center_left)
                good_right_x, good_right_y = self.pixels_in_window(center_right)

                left_x.extend(good_left_x)
                left_y.extend(good_left_y)
                right_x.extend(good_right_x)
                right_y.extend(good_right_y)

               
                

                if len(good_left_x) > self.min_pix:
                    left_x_current = np.int32(np.mean(good_left_x))
                if len(good_right_x) > self.min_pix:
                    right_x_current = np.int32(np.mean(good_right_x))
                
                
                if len(good_left_x)>750:
                    left_ang = left_ang + 1

                if len(good_right_x)>750:
                    right_ang = right_ang + 1
                
                left_ang_sum = left_ang_sum + len(good_left_x)
                right_ang_sum = right_ang_sum + len(good_right_x)

        
            print("****************************")
            print(f"left_ang_sum = {left_ang_sum}")
            print(f"right_ang_sum = {right_ang_sum}")
            # print("***************************")
            # if image:
            out_img[left_y, left_x] = [255, 0, 0]
            out_img[right_y, right_x] = [0, 0, 255]
            cv2.line(out_img, (xl_b_pix, self.img_size[1] - 10), (xl_b_pix, self.img_size[1]), (255, 255, 255), 5)
            cv2.line(out_img, (xr_b_pix, self.img_size[1] - 10), (xr_b_pix, self.img_size[1]), (255, 255, 255), 5)
            cv2.imshow('slide windows', out_img)
            cv2.moveWindow("slide windows", 25, 350)
            if debug:
                cv2.waitKey(0)

            return left_x, left_y, right_x, right_y,left_ang,right_ang,left_ang_sum,right_ang_sum




    def fit_poly(self, img, image=False, debug=False):
        """
        找到二值图中的车道线并画出

        Parameters:
            img: 俯视图阈值化后的二值图
            lane: 车道线选择    0 left   1 right   2 dual
            image: 是否显示处理过程
            debug: 是否单步显示
        Returns:
            out_img: 画了车道线的RGB图
        """

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

        # print("确定边线")

        # # 确定边线
        # left_x, left_y, right_x, right_y, left_ang, right_ang ,left_ang_sum,right_ang_sum= self.find_lane_pixels(img, lane=2, image=image, debug=debug)


        left_x, left_y,left_ang,left_ang_sum,left_zero,left_line,left_line_posi = self.find_lane_pixels(img, lane=0, image=image, debug=debug)
        right_x, right_y,right_ang,right_ang_sum,right_zero,right_line,right_line_posi = self.find_lane_pixels(img, lane=1, image=image)
        


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

        
        
        print("                              ")

        # if left_exit == 0 and right_exit == 0 :
        #     final_stop = 1
        #     print("已到达终点")

       
        print(f"right_ang_sum = {right_ang_sum}")
        print(f"left_ang_sum = {left_ang_sum}")
        # 到达左转第一标志位
        # if left_ang >= 1 and right_ang == 0 and right_angle_1 == 0 and right_ang_sum <2200 and right_ang_sum >1000 :
        # if left_ang >= 1 and right_ang == 0 and right_angle_1 == 0 and right_ang_sum > 0:
        if right_line == 1 and right_line_posi == 8 and (left_line_posi > 0 and left_line_posi <5) :
        
            turn_left = 1
            right_angle_1 = 1 
            my_print("到达左转第一标志位")

        # 到达右转第一标志位
        # elif left_ang == 0 and right_ang >= 1 and right_angle_1 == 0 and left_ang_sum <2200 and left_ang_sum > 1000:
        # elif left_ang == 0 and right_ang >= 1 and right_angle_1 == 0 and left_ang_sum>0 :
        elif left_line == 1 and left_line_posi == 8 and (right_line_posi > 0 and right_line_posi <5):
            turn_left = 0
            right_angle_1 = 1 
            my_print("到达右转第一标志位")
            

        if count == 1:
            if time.time() - pre_time < 2.0 :
                if right_angle_1:

                    right_angle_1 = 0
                    print("第一标志位判断错误")
                

            else :
                count = 0 # 计数器清零

        

        # 强制切线
        if right_angle_1:
            if turn_left:
                lane_change = 1
                print("强制切换到右线")
            
            else:
                lane_change = 0
                print("强制切换到左线")
        print("                              ")
        
        # 到达第二标志位
        if right_angle_1 ==1:
            if left_ang >= 1 and right_ang >= 1 and left_line_posi != right_line_posi:
                right_angle_2 = 1
                my_print("到达第二标志位")
                print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        
        print(f"right_angle_1 == {right_angle_1}")
        print(f"当前转向 = {turn_left}")
        print(f"当前巡线 = {lane_change}")
        print(f"right_angle_2 = {right_angle_2}")
        print(f"final_stop = {final_stop}")

        print("****************************")

        # 检测到直角元素
        if right_angle_2 == 1:
            # 防止直角弯短时间误判
            if count == 0:
                pre_time = time.time()
                count = 1
                    
         

        if lane == 0:
            
            # 生成左线的坐标来画图
            if len(left_y):
                plot_y = np.linspace(np.min(left_y), np.max(left_y), self.n_windows)

                # left_fit_x = self.left_fit[0] * plot_y ** 2 + self.left_fit[1] * plot_y + self.left_fit[2]  # 2nd
                left_fit_x = self.left_fit[0] * plot_y + self.left_fit[1]

                # 可视化
                for i in range(self.n_windows):
                    cv2.circle(out_img,
                               (int(left_fit_x[i]), int(plot_y[i] - self.half_height)), 10,
                               (0, 255, 0), -1)

            return out_img
        elif lane == 1:
            
            
            if len(right_y):
                plot_y = np.linspace(np.min(right_y), np.max(right_y), self.n_windows)

                # right_fit_x = self.right_fit[0] * plot_y ** 2 + self.right_fit[1] * plot_y + self.right_fit[2]
                right_fit_x = self.right_fit[0] * plot_y + self.right_fit[1]

                for i in range(self.n_windows):
                    cv2.circle(out_img,
                               (int(right_fit_x[i]), int(plot_y[i] - self.half_height)), 10,
                               (0, 0, 255), -1)
            
            return out_img
        
        else:

            if len(left_y) and len(right_y):
                plot_y = np.linspace(np.min(right_y), np.max(right_y), self.n_windows)

                # left_fit_x = self.left_fit[0] * plot_y ** 2 + self.left_fit[1] * plot_y + self.left_fit[2]
                # right_fit_x = self.right_fit[0] * plot_y ** 2 + self.right_fit[1] * plot_y + self.right_fit[2]
                left_fit_x = self.left_fit[0] * plot_y + self.left_fit[1]
                right_fit_x = self.right_fit[0] * plot_y + self.right_fit[1]

                for i in range(self.n_windows):
                    tl = (int(left_fit_x[i]), int(plot_y[i]) - 5)
                    br = (int(right_fit_x[i]), int(plot_y[i]) + self.window_height + 5)
                    cv2.rectangle(out_img, tl, br, (255, 0, 255), -1)

            return out_img

        

        

    def calculate(self):

        global lane_change

        # 计算曲率半径及中线像素偏移
        differ_pix = self.measure(lane=lane_change)
        
        if lane_change :
            gradiant = self.right_fit[0]
        
        else :
            gradiant = self.left_fit[0]
        
        
        # 确定车道线弯曲方向
        if abs(gradiant) <= straight_threshold:
            self.dir.append('F')
        elif gradiant > 0:
            self.dir.append('L')
        else:
            self.dir.append('R')

        # 平缓化
        if len(self.dir) > 5:
            self.dir.pop(0)

        # 找到出现最多次的方向信息
        direction = max(set(self.dir), key=self.dir.count)

        
     

        return gradiant, differ_pix, direction

    def measure(self, lane=2):
        """
        计算俯视视角下的曲率及中心偏差

        Args:
            lane: 车道线选择    0 left   1 right   2 dual

        Returns:
            中心偏差(pix)
        """
    
        if lane == 0:  # 左线
        
            xl = np.dot(self.left_fit, [transform_dst_size[1], 1])
            differ = -((xr_b_pix + xl_b_pix)/2 -xl - 119)
            
        elif lane == 1:  # 右线

            xr = np.dot(self.right_fit, [transform_dst_size[1], 1])
            differ = xr - (xr_b_pix + xl_b_pix)/2 - 108
            
        else:  
            pass

        return differ

"""
进行车辆道路线的检测和跟踪。
"""
class FindLaneLines:
    """
    寻找车道线
    """
    def __init__(self, img_size_=(320, 240)):
        """ Init Application"""
        self.img_size = img_size_
        self.thresholding = Thresholding(img_size=img_size_)  # 二值化
        self.transform = PerspectiveTransformation(img_size=img_size_)  # 透视变换
        self.lane_lines = LaneLines(dst_size=transform_dst_size) # 未知，猜测传入的参数为去畸变参数

        # 读取了左转，右转，直走的图片
        self.left_curve_img = cv2.imread('/home/ucar/visial_line/utils/left_turn.png',
                                         cv2.IMREAD_UNCHANGED)
        self.right_curve_img = cv2.imread('/home/ucar/visial_line/utils/right_turn.png',
                                          cv2.IMREAD_UNCHANGED)
        self.keep_straight_img = cv2.imread('/home/ucar/visial_line/utils/straight.png',
                                            cv2.IMREAD_UNCHANGED)
        
        # 缩放图像，使其新的 width（宽度）和 height（高度）分别等于原图像尺寸与640和480的比例。
        self.left_curve_img = cv2.resize(self.left_curve_img, (0, 0), fx=self.img_size[0] / 640, fy=self.img_size[1] / 480)
        self.right_curve_img = cv2.resize(self.right_curve_img, (0, 0), fx=self.img_size[0] / 640, fy=self.img_size[1] / 480)
        self.keep_straight_img = cv2.resize(self.keep_straight_img, (0, 0), fx=self.img_size[0] / 640, fy=self.img_size[1] / 480)

        # 获取调整大小后的左转图像的尺寸（宽度、高度和通道数）
        self.sign_size = self.left_curve_img.shape

        # 定义新的宽度 W 和高度 H 的值
        self.W = 125 + self.sign_size[0]
        self.H = 150 + self.sign_size[1]


    def forward(self, img, lane, image=True, debug=False):

        image = True
        if image:
            cv2.imshow('undistort', img)
            if debug:
                cv2.waitKey(0)

        # 图像去畸变
        # 因畸变主要在固定位相机中出现，而由于包括 插usb的先后顺序在内 的各种原因 可能导致相机id顺序并不为本代码约定的顺序
        # 故在此处确定固定位相机视频帧时进行去畸变
        # 使用undistort方法去畸变
        # img = cv2.undistort(src=img, cameraMatrix=camera_matrix, distCoeffs=distortion_coefficients,
        #                     dst=None, newCameraMatrix=projection_matrix)
        # if image:
        #     cv2.imshow('distort', img)
        #     if debug:
        #         cv2.waitKey(0)
        
        # 复制一张原图
        out_img = np.copy(img)

        # 把指定的控制点在图像上标记出来
        # 将 debug 设置为真，图像会停留在屏幕上，等待任意键的输入来关闭图像窗口，
        # 否则，图像窗口会在短暂的时间后自动关闭。
        if image:
            # 将四个预设点添加到图像中
            # 并将图像打印到窗口
            with_dot = np.copy(img)
            for c in range(4):
                cv2.circle(with_dot, control_points[c].astype(int), 5, (255, 0, 255), -1)
                cv2.line(with_dot,
                         control_points[c].astype(int),
                         control_points[c+1 if c+1 < 4 else 0].astype(int),
                         (255, 255, 0), 1)
            cv2.imshow('Ctrl Pts', with_dot)
            if debug:
                cv2.waitKey(0)
 
        # 因为是白底，故先进行阈值过滤再转换视角，防止视角转换时出现黑边影响后续
        img = self.thresholding.forward(img, lane=lane, image=image, debug=debug)
        if image:
            cv2.imshow(f'threshold {lane_list[lane]} lane', img)
            if debug:
                cv2.waitKey(0)

        # 视角转换
        img = self.transform.forward(img)
        if image:
            cv2.imshow('top down view', img)
            if debug:
                cv2.waitKey(0)

        # 车道检测
        img, gradiant, differ_pix, direction = self.lane_lines.forward(img, lane=lane, image=image, debug=debug)
        if image:
            cv2.imshow('lane in top down view', img)
            if debug:
                cv2.waitKey(0)

        # 视角转换回来
        img = self.transform.backward(img)
        if image:
            cv2.imshow('lane in original view', img)
            if debug:
                cv2.waitKey(0)

        # 车道分割 叠加至原图
        out_img = cv2.addWeighted(out_img, 1, img, 0.8, 0)

        # # 添加车道信息
        differ = differ_pix * x_pix2m   # 像素距离转换成实际距离
        out_img = self.draw_osd(out_img, differ, direction, lane=lane)
        return out_img, gradiant, differ
    
    def draw_osd(self, out_img, differ, direction, lane):
        # 画车道线中线,红线
        cv2.line(out_img,
                 (int((control_points[3][0] + control_points[2][0])//2 - differ/x_pix2m), self.img_size[1] - 40),
                 (int((control_points[3][0] + control_points[2][0])//2 - differ/x_pix2m), self.img_size[1]),
                 (0, 0, 255), 5)

        # 画视野中线，蓝线
        cv2.line(out_img,
                 (int(control_points[3][0] + control_points[2][0]) // 2 - 40, self.img_size[1] - 20),
                 (int(control_points[3][0] + control_points[2][0]) // 2 - 40, self.img_size[1]),
                 (255, 0, 0), 5)
        
        # 添加相关信息
        widget = np.copy(out_img[30:self.H, :self.W])
        widget = widget // 1.2
        widget[0, :] = [0, 0, 255]
        widget[-1, :] = [0, 0, 255]
        widget[:, 0] = [0, 0, 255]
        widget[:, -1] = [0, 0, 255]
        out_img[30:self.H, :self.W] = widget

        # 添加可视化箭头
        if direction == 'L':
            msg = "Left Curve Ahead"
            out_img = overlayPNG(out_img, self.left_curve_img,
                                       [self.W // 2 - self.sign_size[0] // 2, 35])

        elif direction == 'R':
            msg = "Right Curve Ahead"
            out_img = overlayPNG(out_img, self.right_curve_img,
                                       [self.W // 2 - self.sign_size[0] // 2, 35])

        else:
            msg = "Keep Straight Ahead"
            out_img = overlayPNG(out_img, self.keep_straight_img,
                                       [self.W // 2 - self.sign_size[0] // 2, 35])

        cv2.putText(out_img, msg, org=(5, 50 + self.sign_size[1]),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5,
                    color=(255, 0, 0), thickness=2)

        cv2.putText(out_img, f"{lane_list[lane]} Lane Keeping",
                    org=(5, 70 + self.sign_size[1]),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5,
                    color=(0, 255, 0), thickness=2)

        cv2.putText(out_img, "{:.2f} m off center".format(-differ),
                    org=(5, 90 + self.sign_size[1]),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5,
                    color=(255, 255, 255), thickness=2)
        
        return out_img
    
    def process_frame(self, img, lane, image=False):
        """
        处理视频帧    配合lane_detection用

        Args:
            img:            视频帧
            lane:           车道线选择   0 左  1 右  2 双

        Returns:
            处理后图像, 曲率半径, 中心偏差
        """
        out_img, gradiant, differ = self.forward(img, lane, image=image, debug=False)

        return out_img, gradiant, differ


def lane_detection(flag_task_l, img_mem_front_l, flag_lane_l, number_lane_l, cmd_chassis_l, image=False):
    """"
    车道检测线主函数
    参数：
    flag_task_l： 任务进行标志位
    img_mem_front_l： 接受从摄像头获取的图像
    flag_lane_l：未知，初始值为0，进程全部打开后为1
    number_lane_l：选择车道线
    cmd_chassis_l：小车速度状态，多进程共享变量。这个变量可以在多个进程之间共享并更新，使得不同的进程能够读取到同一变量的最新值。
    image：用于判断图像是否为空，在FindLaneLines.forward方法中使用
    处理摄像头获取的图像，并将处理后的图像实时显示，
    计算曲率半径, 中心偏差
    并通过二者计算小车的前进速度，横向修正速度,旋转速度
    并将速度存储至cmd_chassis_l
    """
            
    global lane_change  
    
    global right_angle_1
    global right_angle_2
    global turn_left
    # 创建巡线实例
    findLaneLines_l = FindLaneLines(img_size_=camera_fixed_config.img_size)
    keep_lane = False

    

    data_list = [None] * 5

    # 对列表每个元素进行赋值
    data_list[0] = 0
    
    my_print("这里是车道线检测进程")
    cv2.namedWindow("Lane")
    cv2.moveWindow("lane", 425, 350)

    # 作为缓冲队列使用。它存储了未来要送入底盘控制的控制命令
    # [vel_mode, 0, 0, 0]
    # [车辆的速度模式, 前进速度，横向修正速度,旋转速度]
    # buffer_size = 1 缓冲区大小

    cmd_buffer = [[vel_mode, 0, 0, 0]] * buffer_size

    # flag_lane_l.value 用于判断车辆此时是否需要保持在车道上行驶。
    # keep_lane 用于控制车辆在退出车道保持后是否需要停车，将速度改为0，然后延时，实现停车
    while flag_task_l.value:

        if flag_lane_l.value:
            keep_lane = True

            # 将存储在内存中的图像img_mem_front_l提取出来，并将其转化为8位无符号整数的NumPy数组格式。
            img = np.array(img_mem_front_l, np.uint8)

            try:
                # 记录当前的时间，用于后续计算函数的运行时间
                t_l_0 = time.time()

                # 传入图像，车道线选择，以及输入图像不为空
                # 返回 处理后图像, 曲率半径, 中心偏差
                out_img, gradiant, differ = findLaneLines_l.process_frame(img, number_lane_l.value, image=image)
           
                # 计算帧率
                fps_lane = round(1 / (time.time() - t_l_0))
                # 将帧率添加在图像上
                cv2.putText(out_img, f'FPS:{fps_lane}', (7, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (125, 0, 255), 2)
                # 显示处理后的图像和帧率
                cv2.moveWindow("Lane", 425, 350)
                cv2.imshow('Lane', out_img)

                cmd_future = [vel_mode,                                                   # 模式
                            min(round(max_vx - 0.2 + kp_x / abs(gradiant), 2), max_vx), # 前进速度  有弯则适当减速 <max_vx
                            round(kp_y * 0, 2),                                    # 横向对正车道中线
                            round(kp_w * differ, 2)]  
                
                # if final_stop == 1:
                #     cmd_future[0] = stop

                # if right_angle_1 == 1:
                #     cmd_future[0] = stop
                
                if right_angle_2 == 1:
                    if turn_left == 1:
                        cmd_future[0] = turn_left_l
                    
                    else :
                        cmd_future[0] = turn_right_l

                
                
                    
                
                # 更新指令的缓存cmd_buffer，并将新的命令更新到cmd_chassis_l
                if flag_lane_l.value:
                    cmd_chassis_l[:] = cmd_buffer[0]
                    # print("cmd vel lane", cmd_chassis_l[:])
                    cmd_buffer.pop(0)
                    cmd_buffer.append(cmd_future)
                else:
                    cmd_chassis_l[:] = [vel_mode, 0, 0, 0]
            except (TypeError, ZeroDivisionError):
                pass
            key = cv2.waitKey(1) & 0xff

            if lane_change == 1:
                number_lane_l.value = 1
            
            else:
                number_lane_l.value = 0

            # 按下Esc，强制停车，退出巡线模式
            if key == 27:
                cmd_chassis_l[:] = [vel_mode, 0, 0, 0]
                time.sleep(1)
                flag_lane_l.value = 0
                flag_task_l.value = 0
                break
            # 按下空格，强制停车，等待下一个CV事件
            elif key == ord(' '):
                cmd_chassis_l[:] = [vel_mode, 0, 0, 0]
                cv2.waitKey(0)

            # 手动切换车道线选择
            elif key == ord('a'):
                number_lane_l.value = 0
            elif key == ord('d'):
                number_lane_l.value = 1
            elif key in (ord('w'), ord('s')):
                number_lane_l.value = 2

        else:
            if keep_lane:  # 确保退出车道线行驶后能停车
                cmd_chassis_l[:] = [vel_mode, 0, 0, 0]
                keep_lane = False

            time.sleep(0.1)

def camera(flag_task_v, img_mem_v,
           flag_lane_v, cmd_chassis_v,
           camera_config_v, record_v=False):
    """
    采集指定编号相机的帧

    Args:
        flag_task_v:        任务进行标志位 (共享)
        img_mem_v:          相机捕获的帧 (共享)
        flag_lane_v:        车道线行驶外部中断 (共享)
        cmd_chassis_v:      底盘控制 (共享)
        camera_config_v:    相机配置 (包含相机编号 画幅 亮度 对比度 曝光度)  相机编号* (ls -l /dev/video*  约定 0定 1动)
        record_v:           是否录像 (默认不录)
    """
    my_print("这里是读取机动位摄像头进程" if camera_config_v.camera_id else "这里是读取固定位摄像头进程")

    cap = cv2.VideoCapture(camera_config_v.camera_id)
    resize = False
    if camera_config_v.img_size[0] != 320 or camera_config_v.img_size[1] != 240:
        resize = True
    if isinstance(camera_config_v.camera_id, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_config_v.img_size[0])  # 设置宽度
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_config_v.img_size[1])  # 设置高度
        cap.set(cv2.CAP_PROP_BRIGHTNESS, camera_config_v.brightness)  # 设置亮度
        cap.set(cv2.CAP_PROP_CONTRAST, camera_config_v.contrast)  # 设置对比度
        if camera_config_v.exposure:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 0~2.6手动曝光  2.6~4自动曝光
            cap.set(cv2.CAP_PROP_EXPOSURE, camera_config_v.exposure)
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))  # 设置视频流格式
        my_print(f"Setting {camera_config_v.name} to "
                 f"Width {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}  "
                 f"Height {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}  "
                 f"Brightness {cap.get(cv2.CAP_PROP_BRIGHTNESS)}  "
                 f"Contrast {cap.get(cv2.CAP_PROP_CONTRAST)} "
                 f"Exposure {'Manual ' + str(cap.get(cv2.CAP_PROP_EXPOSURE)) if camera_config_v.exposure else 'Auto'} ")

    else:
        my_print("Wrong Camera ID", 'err')
        return
    
    out = None

    if record_v and isinstance(camera_config_v.camera_id, int):
        # # 录制视频
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 视频格式
        fps = cap.get(cv2.CAP_PROP_FPS)  # 视频帧率
        size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))  # 视频宽高
        out = cv2.VideoWriter(f'video_{int(time.time())}.mp4', fourcc, fps, size)

    cv2.namedWindow(camera_config_v.name)

    while flag_task_v.value and cap.isOpened():
        ret, img = cap.read()
        img = cv2.flip(img, 1)

        if not ret:
            my_print(f'Fail to Load Stream of Camera {camera_config_v.camera_id}', 'err')
            break

        if record_v:
            out.write(img)  # 录制视频

        img_mem_v[:] = img.copy()

        if resize:
            img = cv2.resize(img, (320, 240))

        cv2.imshow(camera_config_v.name, img)
        cv2.moveWindow(camera_config_v.name, 800, 5 if camera_config_v.name == "Mobile Cam" else 350)

        key = cv2.waitKey(1) & 0xff
        if key == ord(' '):  # 暂停 中止检测
            flag_lane_v.value = 0  # 停止车道线检测
            cmd_chassis_v[:] = [vel_mode, 0, 0, 0]  # 停车
            img = np.zeros((camera_config_v.img_size[1], camera_config_v.img_size[0], 3))  # 清空画面，防止误识别
            img_mem_v[:] = img.copy()
            if resize:
                img = cv2.resize(img, (320, 240))
            cv2.imshow(camera_config_v.name, img)
            cv2.waitKey(0)
            flag_lane_v.value = 1  # 恢复车道线检测
        elif key == 27:
            cmd_chassis_v[:] = [vel_mode, 0, 0, 0]  # 停车
            break
  
    cap.release()
    cv2.destroyAllWindows()
    if record_v:
        out.release()
    cmd_chassis_v[:] = [disarm, 0, 0, 0]  # 停车
    time.sleep(.5)
    my_print("读取机动位摄像头进程结束" if camera_config_v.camera_id else "读取固定位摄像头进程结束")
    flag_task_v.value = 0


def chassis_control(flag_task_c, cmd_chassis_c):
    """
        底盘控制进程

        Args:
            flag_task_c:        巡线任务开启标志位 (共享)  1 进行巡线 0 结束巡线
            cmd_chassis_c:      底盘控制 (共享) 四元数，控制底盘速度
            with_chassis_c:     开启底盘

        Returns:
            None
    """
    my_print("这里是行驶进程")  
    
    rospy.init_node('linetrack', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    msg = Twist()
    last_instruction = []
    while flag_task_c.value:
        if cmd_chassis_c[:] == last_instruction:
            continue

        if cmd_chassis_c[0] in [disarm, arm]:
            msg.linear.x = cmd_chassis_c[0]
            my_print("Engine Shutdown" if cmd_chassis_c[0] == disarm else "Engine Start", "warn")
            cmd_chassis_c[:] = [8, 0, 0, 0]

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


            
            
            rate = rospy.Rate(10)
            msg.linear.x = 0.4
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

            
            # while True:
            
            #     msg.linear.x = 0.0
            #     msg.linear.y = 0.0
            #     msg.linear.z = 0.0
            #     msg.angular.x = 0.0
            #     msg.angular.y = 0.0
            #     msg.angular.z = 0; 
            #     cmd_vel_pub.publish(msg)
                
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


if __name__ == "__main__":
    pass
    """
    有底盘测试
    初始化进程变量
    开启巡线进程
    同时设置标志位结束巡线进程
    """
    # 创建和管理在不同进程之间共享的NumPy数组
    import sharedmem

    # Process：多进程
    # Value，Array:共享内存
    from multiprocessing import Process, Value, Array
    import sys
    
 
    



    # 创建进程间通信变量并初始化
    #  创建一个空的在进程间共享的多维数组，用于存储从摄像头获取的图像。
    img_mem_fixed = sharedmem.empty((camera_fixed_config.img_size[1], camera_fixed_config.img_size[0], 3), np.uint8)  # 固定位图像帧 初始化为全黑
    
    # 始终为1，若 flag_task 改为0，杀死巡线所有进程，巡线结束
    flag_task = Value('i', 1)  # 任务进行标志位(默认开启)
    
    # 初始值为0，进程全部打开后为1
    flag_lane = Value('i', 0)  # 车道线辅助行驶标志(默认开启)
    number_lane = Value('i', 1)  # 车道线选择   0 左  1 右  2 双
    
    # 四元数，控制底盘速度
    cmd_chassis = Array('d', 4)  # 底盘控制 [1 linear.x linear.y linear.z angular.z] v = ωr
    cmd_chassis[0] = arm  # 解锁
    
    # 固定位视频帧采集子进程
    # 创建一个进程，其目标函数是camera，参数是一些共享变量和配置文件。
    p_video_fixed = Process(target=camera,
                            args=(flag_task, img_mem_fixed,
                                  flag_lane, cmd_chassis,
                                  camera_fixed_config, False))
    
    # 车道线子进程
    p_lane = Process(target=lane_detection,
                     args=(flag_task, img_mem_fixed, flag_lane, number_lane, cmd_chassis, False))
    
    # 底盘控制子进程
    p_chassis = Process(target=chassis_control,
                        args=(flag_task, cmd_chassis))
    
    my_print("开启固定位摄像头读取帧进程")
    p_video_fixed.start()
    
   
    my_print("开启底盘行驶进程")
    p_chassis.start()
    time.sleep(1)
    
    my_print("开启车道线检测进程")
    p_lane.start()
    
    flag_lane.value = 1
    while flag_task.value:
        time.sleep(1)
    
    # 捕获可能的异常
    try:
        p_video_fixed.join()
    except (AssertionError, NameError):
        pass
    try:
        p_lane.join()
    except (AssertionError, NameError):
        pass
    try:
        p_chassis.join()
    except (AssertionError, NameError):
        pass