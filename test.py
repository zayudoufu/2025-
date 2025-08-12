from machine import Timer
import time, os, sys
import math
from media.sensor import *
from media.display import *
from media.media import *
from machine import Pin
from machine import PWM, FPIOA
# ---------- 摄像头编号 ----------
sensor_id = 2
sensor = None

A4_W_MM = 210  # A4纸宽度 mm
A4_H_MM = 297  # A4纸高度 mm
FX = 700       # 摄像头焦距像素（根据实际标定）
FY = 700

# ---------- 显示模式 ----------
DISPLAY_MODE = "VIRT"  # "VIRT" | "LCD" | "HDMI"
if DISPLAY_MODE == "VIRT":
    DISPLAY_WIDTH = 800
    DISPLAY_HEIGHT = 480
elif DISPLAY_MODE == "LCD":
    DISPLAY_WIDTH = 800
    DISPLAY_HEIGHT = 480
elif DISPLAY_MODE == "HDMI":
    DISPLAY_WIDTH = 1920
    DISPLAY_HEIGHT = 1080
else:
    raise ValueError("请选择 'VIRT', 'LCD' 或 'HDMI'")
# ---------- FPIOA & 引脚 ----------
fpioa = FPIOA()
fpioa.set_function(62, FPIOA.GPIO62)  # 红灯
fpioa.set_function(20, FPIOA.GPIO20)  # 绿灯
fpioa.set_function(63, FPIOA.GPIO63)  # 蓝灯
fpioa.set_function(53, FPIOA.GPIO53)  # 按键
fpioa.set_function(1, FPIOA.GPIO1)    # 备用
fpioa.set_function(42, FPIOA.PWM0)    # Pitch 舵机
fpioa.set_function(52, FPIOA.PWM4)    # Yaw   舵机
# ---------- LED / 激光 / 按键 ----------
LED_R = Pin(62, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
LED_G = Pin(20, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
LED_B = Pin(63, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
button = Pin(53, Pin.IN, Pin.PULL_DOWN)
blue_light = Pin(14, Pin.OUT, pull=Pin.PULL_DOWN, drive=7)

# 共阳 LED：高电平灭，低电平亮
LED_R.high()
LED_G.high()
LED_B.high()
blue_light.low()
# ---------- 默认中心 / PID ----------
mindle_x = 572/2
mindle_y = 433/2
cx_rect = mindle_x   # 矩形中心 X
cy_rect = mindle_y   # 矩形中心 Y
cx_blob = mindle_x   # 色块中心 X
cy_blob = mindle_y   # 色块中心 Y

dt = 0.02                         # 控制周期 20 ms
kp_yaw = 0.05                   # Yaw 轴 PID 参数
ki_yaw = 0.000
kd_yaw = 0.05

kp_pitch = 0.030                 # Pitch 轴 PID 参数
ki_pitch = 0.000
kd_pitch = 0.005

prev_err_yaw = 0.0
integral_yaw = 0.0
prev_err_pitch = 0.0
integral_pitch = 0.0

yaw_center = 1000   #  # 偏航角，减小逆时针
pitch_center = 550 # 默认频率50Hz,占空比75%，减小向上，
yaw_duty = yaw_center
pitch_duty = pitch_center
# 初始化 PWM：50 Hz，占空比 = value/100
Pitch_pwm = PWM(0, 50, pitch_center / 100, enable=True)
Yaw_pwm = PWM(4, 50, yaw_center / 100, enable=True)
# ---------- 小灯闪烁 ----------
ight_value = 0
yes_times=0
no_rect_cnt = 0
pid_freeze = False         # 冻结 PID 标志
def pid_con(timer):
    global yaw_duty, pitch_duty, prev_err_yaw, prev_err_pitch
    global integral_yaw, integral_pitch, cx_rect, cy_rect, yes_times
    # 红色心跳灯
    LED_R.value(1 - LED_R.value())

    if pid_freeze:#180舵机舵机不输出，360舵机保持不动
        Yaw_pwm.duty(yaw_center / 100)
        Pitch_pwm.duty(pitch_center / 100)
        print("pid_freeze")
        return

    # ---------- Yaw ----------
    err_yaw = cx_rect - mindle_x
    integral_yaw += err_yaw * dt
    integral_yaw = max(-300, min(300, integral_yaw))
    derivative_yaw = (err_yaw - prev_err_yaw) / dt
    delta_yaw = kp_yaw * err_yaw + ki_yaw * integral_yaw + kd_yaw * derivative_yaw
    yaw_duty += delta_yaw
    yaw_duty = max(690, min(740, yaw_duty))
    Yaw_pwm.duty(yaw_duty / 100)

    # ---------- Pitch ----------
    err_pitch = cy_rect - mindle_y
    integral_pitch += err_pitch * dt
    integral_pitch = max(-300, min(300, integral_pitch))
    derivative_pitch = (err_pitch - prev_err_pitch) / dt
    delta_pitch = kp_pitch * err_pitch + ki_pitch * integral_pitch + kd_pitch * derivative_pitch
    pitch_duty += delta_pitch
    #pitch_duty = max(1050, min(1220, pitch_duty))
   # Pitch_pwm.duty(pitch_duty / 100)
    # 命中判定
    if abs(err_pitch) < 10 and abs(err_yaw) < 10:
        yes_times += 1
        if yes_times > 20:
            blue_light.high()
            LED_B.low()
    else:
        yes_times = 0
        blue_light.low()
        LED_B.high()
    prev_err_yaw   = err_yaw
    prev_err_pitch = err_pitch

def pid_control(cx_rect, cy_rect, mindle_x, mindle_y, kp_yaw, ki_yaw, kd_yaw, kp_pitch, ki_pitch, kd_pitch, prev_err_yaw, prev_err_pitch, integral_yaw, integral_pitch):
    global yaw_duty, pitch_duty
    dt = 0.02  # 控制周期 20 ms
    # Yaw 控制
    err_yaw = cx_rect - mindle_x
    integral_yaw += err_yaw * dt
    integral_yaw = max(-300, min(300, integral_yaw))
    derivative_yaw = (err_yaw - prev_err_yaw) / dt
    delta_yaw = kp_yaw * err_yaw + ki_yaw * integral_yaw + kd_yaw * derivative_yaw
    yaw_duty += delta_yaw
    yaw_duty = max(0, min(6000, yaw_duty))  # 限制占空比范围
    pwm1.duty(int(yaw_duty / 10000 * 255))  # 确保占空比是整数

    # Pitch 控制
    err_pitch = cy_rect - mindle_y
    integral_pitch += err_pitch * dt
    integral_pitch = max(-300, min(300, integral_pitch))
    derivative_pitch = (err_pitch - prev_err_pitch) / dt
    delta_pitch = kp_pitch * err_pitch + ki_pitch * integral_pitch + kd_pitch * derivative_pitch
    pitch_duty += delta_pitch
    pitch_duty = max(0, min(6000, pitch_duty))  # 限制占空比范围
    pwm2.duty(int(pitch_duty / 10000 * 255))  # 确保占空比是整数

    return err_yaw, err_pitch, integral_yaw, integral_pitch
# 形状过滤#寻找矩形的代码，要求a4矩形检测得到的边缘是宽度大于黑线
def is_a4(r):
   x, y, w, h = r.rect()
   if w < 140 or h < 100:
       return False
   ratio = w /h
   return abs(ratio - 1.414) <= 0.15   # 容差 ±0.15
def is_valid_a4_rect(r, img):
   x, y, w, h = r.rect()
   border_thickness = 2  # 边框厚度

   # 检查边框是否至少两层黑色像素
   for i in range(border_thickness):
       for j in range(w):
           if img.get_pixel(x + j, y + i)[0] > 128 or img.get_pixel(x + j, y + h - i - 1)[0] > 128:
               return False
       for j in range(h):
           if img.get_pixel(x + i, y + j)[0] > 128 or img.get_pixel(x + w - i - 1, y + j)[0] > 128:
               return False

   # 检查内部是否主要是白色像素
   white_pixel_count = 0
   total_pixel_count = (w - 2* border_thickness) * (h - 2 * border_thickness)
   for i in range(border_thickness, w - border_thickness):
       for j in range(border_thickness, h - border_thickness):
           if img.get_pixel(x + i, y + j)[0] > 135:
               white_pixel_count += 1

   if white_pixel_count / total_pixel_count > 0.8:
       return True
   else:
       return False
# 启动定时器
tim = Timer(-1)
tim.init(period=20, mode=Timer.PERIODIC, callback=pid_con)
# 主程序
try:
    # 初始化摄像头
    sensor = Sensor(id=sensor_id)
    sensor.reset()
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
    # 初始化显示
    if DISPLAY_MODE == "VIRT":
        Display.init(Display.VIRT, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, fps=60)
        print("Display initialized in VIRT mode")
    elif DISPLAY_MODE == "LCD":
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
        print("Display initialized in LCD mode")
    elif DISPLAY_MODE == "HDMI":
        Display.init(Display.LT9611, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
        print("Display initialized in HDMI mode")

    MediaManager.init()
    sensor.run()
    # 颜色阈值
    rect_thre =(43, 100, -54, 43, -55, 65) # 矩形二值化阈值
    while True:
        os.exitpoint()
        # 1. 采集原图
        img = sensor.snapshot(chn=CAM_CHN_ID_0)
        # 2. 复制并二值化 → 找矩形
        img_rect = img.copy()
        img_rect.binary([rect_thre])
        rects = img_rect.find_rects(threshold=14000, pixels_threshold=0)
        # 3. 过滤 A4 矩形
        rects_a4 = list(filter(is_a4, rects))
        if rects_a4:
            pid_freeze = False
            no_rect_cnt=0
            # 3-1 取面积最大的 A4 矩形
            r = max(rects_a4, key=lambda x: x.rect()[2] * x.rect()[3])
            roi = r.rect()
            x, y, w, h = roi
            cx_rect = x + w // 2
            cy_rect = y + h // 2
            print("A4 rectangle center: X={}, Y={}".format(cx_rect, cy_rect))
            # --------- 距离估算 ----------
            if w == 0 or h == 0:
                print("Width or height is zero, skipping distance calculation")
            else:
                D_w = (A4_W_MM * FX) / w  # 用宽度算
                D_h = (A4_H_MM * FY) / h  # 用高度算
                D = (D_w + D_h) / 2      # 取平均
                #print(f"距离 ≈ {D:.1f} mm ({D/10:.1f} cm)")
                img_rect.draw_rectangle(roi, color=(255, 0, 0), thickness=2)
                img_rect.draw_cross(cx_rect, cy_rect, color=(0, 0, 255))
        else:
            # 没检测到 A4
           no_rect_cnt += 1
           cx_rect = mindle_x # cx_rect/cy_rect 不再改，舵机保持上一次角度
           cy_rect = mindle_y
           cx_blob = mindle_x
           cy_blob = mindle_y
           pid_freeze = True
           blue_light.toggle()
           if no_rect_cnt >= 30:
               pid_freeze = False #180舵机回到pitch_center.360舵机回到yaw_center,保持旋转寻找矩形
               LED_G.low()      #bug
               # 连续 N 帧没目标 → 冻结 PID，关闭激光，红灯亮
        # 6. 显示结果
        Display.show_image(img)
        time.sleep_ms(1)
except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    # 停止传感器运行
    if isinstance(sensor, Sensor):
        sensor.stop()
    # 反初始化显示模块
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    # 释放媒体缓冲区
    MediaManager.deinit()
