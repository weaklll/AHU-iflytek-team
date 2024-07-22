#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
from multiprocessing import Process, Value, Array
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import ctypes
from functools import partial

"""
5.0版本，最简单版本
删除所有可视化操作
删除底盘调用模块
没有调试部分，保证小车正常巡线
同时删除了sharedmem包，使用multiprocessing 重构
使用usb_cam获取图像,将img_mem_v改为全局变量
"""

class CameraConfig():
    def __init__(self, name, camera_id, brightness, contrast, exposure, img_size=(320, 240)) -> None:
        self.name = name
        self.camera_id = camera_id
        self.brightness = brightness
        self.contrast = contrast
        self.exposure = exposure
        self.img_size = img_size

def create_shared_memory(camera_config):
    width, height = camera_config.img_size
    # 创建一个在进程间共享的多维数组，用于存储从摄像头获取的图像
    shared_array_base = Array(ctypes.c_uint8, width * height * 3, lock=False)
    shared_array = np.frombuffer(shared_array_base, dtype=np.uint8)
    img_mem_fixed = shared_array.reshape((height, width, 3))
    
    # 将数组初始化为全黑
    img_mem_fixed[:] = 0
    return img_mem_fixed

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

img_mem_fixed = np.zeros((camera_fixed_config.img_size[1], camera_fixed_config.img_size[0], 3), np.uint8)
# #####################################################################
# chassis
# ########
# 模式位
disarm    = 0  # 上锁   线速度为0
arm       = 1  # 解锁   线速度为1
vel_mode  = 2  # 速度模式   线速度为巡线速度



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
control_points = np.float32(np.load('/home/iflytek/move/src/send_goals/scripts/perspective.npy'))

transform_dst_size = (320, 240)
dst_pts = np.float32([(0,                     0),
                      (transform_dst_size[0], 0),
                      (transform_dst_size[0], transform_dst_size[1]),
                      (0,                     transform_dst_size[1])])

"""
thresholding
"""

binary_type = 1  # 0 cv2.THRESH_BINARY(黑底白线->黑底白线 / 白底黑线->白底黑线)   1 cv2.THRESH_BINARY_INV(白底黑线->黑底白线 / 黑底白线->白底黑线)
# 以下 二值化 参数配合/utils/thresh_binary.py获取
# binary_l = 177
# binary_l = 176  # 光线较暗
binary_l = 189  # 光线较亮
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
        
    def forward(self, img, lane=2):

        
        """
        Method 2  threshold
        """
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # threshold
        _, dual_lane = cv2.threshold(img, binary_l, 255, binary_type)
        dual_lane = cv2.bitwise_not(dual_lane)

        if lane == 0:  # 选择左线
            dual_lane[:, int(275 * self.resize):] = 0  # 将右半图像涂黑
        elif lane == 1:  # 选择右线
            dual_lane[:, :int(375 * self.resize)] = 0  # 将左半图像涂黑
            
        cv2.imshow("Camera Feed", img)
        cv2.waitKey(1)
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

    def forward(self, img, lane=1):
        """
        检测图像中的车道线

        Parameters:
            img: 俯视图阈值化后的二值图
            lane: 车道线选择    0 left   1 right   2 dual
        Returns:
            out_img: 包含车道线信息的RGB图
        """

        self.extract_features(img)
        img = self.fit_poly(img, lane=lane)
        gradiant, differ_pix = self.calculate(lane=lane)

        return img, gradiant, differ_pix

    def extract_features(self, img):
        """
        提取二值图中的非0像素点的坐标

        Parameters:
            img: 二值图
        """
        self.nonzero = img.nonzero()
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

    def find_lane_pixels(self, img, lane=1):
        """
        找到属于车道线的像素点

        Parameters:
            img: 俯视图阈值化后的二值图
            lane: 车道线选择    0 left   1 right   2 dual
           
        Returns:
            left_x: 左线像素点的x坐标
            left_y: 左线像素点的y坐标
            right_x: 右线像素点的x坐标
            right_y: 右线像素点的y坐标
            out_img: 后续为车道线涂色用的RGB图
        """
        assert (len(img.shape) == 2)

        # 创建一张用来后续做可视化的三通道图片
        out_img = np.dstack((img, img, img))

        # 创建输入的二值图底部一个滑窗高度的区域的直方图
        histogram = np.sum(img[img.shape[0] // 4:, :], axis=0)  # img[h, w, channel]

        # 寻找左线
        if lane == 0:
           pass

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

                good_right_x, good_right_y = self.pixels_in_window(center_right)

                right_x.extend(good_right_x)
                right_y.extend(good_right_y)

                
                if len(good_right_x) > self.min_pix:
                    right_x_current = np.int32(np.mean(good_right_x))
           
            return right_x, right_y

        # 两条线都寻找
        else:
            pass


    def fit_poly(self, img, lane=1):
        """
        找到二值图中的车道线并画出

        Parameters:
            img: 俯视图阈值化后的二值图
            lane: 车道线选择    0 left   1 right   2 dual

        Returns:
            out_img: 画了车道线的RGB图
        """
        cv2.imshow("Camera Feed", img)
        cv2.waitKey(1)
        out_img = self.canvas.copy()

        if lane == 0:
            pass

        elif lane == 1:
            right_x, right_y = self.find_lane_pixels(img, lane=lane)

            if len(right_y) > right_lane_thresh:
                self.right_fit = np.polyfit(right_y, right_x, 1)

            if len(right_y):
                plot_y = np.linspace(np.min(right_y), np.max(right_y), self.n_windows)

                right_fit_x = self.right_fit[0] * plot_y + self.right_fit[1]

                for i in range(self.n_windows):
                    cv2.circle(out_img,
                               (int(right_fit_x[i]), int(plot_y[i] - self.half_height)), 10,
                               (0, 0, 255), -1)
            return out_img

        else:
            pass
            

    def calculate(self, lane=1):
        # 计算曲率半径及中线像素偏移
        differ_pix = self.measure(lane=lane)

        
        if lane == 0:
            pass
        elif lane == 1:
            gradiant = self.right_fit[0]
        else:
            pass
            
        gradiant = round(gradiant, 5)
        if not gradiant:  # 防止 div 0
            gradiant = 0.00001
        
        # print(gradiant)

        

        # 平缓化
        if len(self.dir) > 5:
            self.dir.pop(0)

      
        return gradiant, differ_pix

    def measure(self, lane=1):
        """
        计算俯视视角下的曲率及中心偏差

        Args:
            lane: 车道线选择    0 left   1 right   2 dual

        Returns:
            中心偏差(pix)
        """
        # 计算视野中心距道路中心的偏移(选取底部为参考)
        if lane == 0:  # 左线
            pass

        elif lane == 1:  # 右线
            xr = np.dot(self.right_fit, [transform_dst_size[1], 1])
            differ = xr - (xr_b_pix + xl_b_pix)/2 - 108

        else:  # 双线
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
        self.lane_lines = LaneLines(dst_size=transform_dst_size) # 扫线

    
    def forward(self, img, lane):

        # 复制一张原图
        out_img = np.copy(img)

        # 因为是白底，故先进行阈值过滤再转换视角，防止视角转换时出现黑边影响后续
        img = self.thresholding.forward(img, lane=lane)

        # 视角转换
        img = self.transform.forward(img)

        # 车道检测
        img, gradiant, differ_pix = self.lane_lines.forward(img, lane=lane)
  
        # # 添加车道信息
        differ = differ_pix * x_pix2m   # 像素距离转换成实际距离
        return out_img, gradiant, differ  
    
    def process_frame(self, img, lane):
        """
        处理视频帧    配合lane_detection用

        Args:
            img:            视频帧
            lane:           车道线选择   0 左  1 右  2 双

        Returns:
            处理后图像, 曲率半径, 中心偏差
        """
        out_img, gradiant, differ = self.forward(img, lane)

        return out_img, gradiant, differ


def lane_detection(flag_task_l, img_mem_front_l, flag_lane_l, number_lane_l, cmd_chassis_l):
    """"
    车道检测线主函数
    参数：
    flag_task_l： 任务进行标志位
    img_mem_front_l： 接受从摄像头获取的图像
    flag_lane_l：未知，初始值为0，进程全部打开后为1
    number_lane_l：选择车道线
    cmd_chassis_l：小车速度状态，多进程共享变量。这个变量可以在多个进程之间共享并更新，使得不同的进程能够读取到同一变量的最新值。
    """
    # 创建巡线实例
    findLaneLines_l = FindLaneLines(img_size_=camera_fixed_config.img_size)
    keep_lane = False
    
    my_print("这里是车道线检测进程")
   
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
                # 传入图像，车道线选择，以及输入图像不为空
                # 返回 处理后图像, 曲率半径, 中心偏差
                out_img, gradiant, differ = findLaneLines_l.process_frame(img, number_lane_l.value)

                cmd_future = [vel_mode,                                                   # 模式
                              min(round(max_vx - 0.2 + kp_x / abs(gradiant), 2), max_vx), # 前进速度  有弯则适当减速 <max_vx
                              round(kp_y * 0, 2),                                    # 横向对正车道中线
                              round(kp_w * differ, 2)]                                  # 转弯速度
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

        else:
            if keep_lane:  # 确保退出车道线行驶后能停车
                cmd_chassis_l[:] = [vel_mode, 0, 0, 0]
                keep_lane = False

            time.sleep(0.1)

def img_callback(msg,img_mem_fixed):
    """
    获取图像
    """
    bridge = CvBridge()
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
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback, callback_args=img_mem_fixed)
    # rospy.Subscriber("/usb_cam/image_raw", Image,  callback_args=(img_mem_fixed,))
    rospy.spin()
    
    # cmd_chassis_v[:] = [disarm, 0, 0, 0]  # 停车
    # time.sleep(.5)
    # flag_task_v.value = 0



def chassis_control(flag_task_c, cmd_chassis_c):
    """
        底盘控制进程

        Args:
            flag_task_c:        巡线任务开启标志位 (共享)  1 进行巡线 0 结束巡线
            cmd_chassis_c:      底盘控制 (共享) 四元数，控制底盘速度
           
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
    
    # Process：多进程
    # Value，Array:共享内存
    from multiprocessing import Process, Value, Array
    
    img_mem_fixed = create_shared_memory(camera_fixed_config)
  
    # 创建进程间通信变量并初始化
    
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
                            args=(flag_task, cmd_chassis,img_mem_fixed))

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
    
    my_print("开启车道线检测进程")
    p_lane.start()
    
    flag_lane.value = 1
    while flag_task.value:
        time.sleep(1)
