# 2024 第十九届讯飞组智能车代码
## AHU：二代车跑不对
- 视觉推理部署代码和导航部分如若需要，点亮star，私信邮箱3475395361@qq.com（现已更新至其它仓库中，见其他仓库）
- 巡线部分如果有疑问，点亮star，私信邮箱3042898802@qq.com or 3376864955@qq.com
- 雷达数据处理如果有疑问，点亮star，私信邮箱luanwz424@163.com
-----------------------------------------------------------------------
导航、视觉以及雷达数据处理均采用C++编写，并集成，相对于python而言无需考虑复杂的系统环境问题（ubuntu）。
-----------------------------------------------------------------------
视觉针对rk3588的yolov5模型（转rknn格式）进行了npu加速，通过获取usb-cam发布的话题信息，转为opencv处理格式进行推理，同时将检测结果通过话题发出，实时检测，同步接收，50fps+(单核npu)
同时采用异步线程池及调用三核npu，npu定频，可达120fps+
