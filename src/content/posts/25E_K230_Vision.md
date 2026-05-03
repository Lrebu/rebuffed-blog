---
title: 电赛25E题：基于01科技K230的视觉方案
published: 2026-03-01
description: "分享一下基于k230的视觉方案"
encrypted: false
pinned: true

alias: "25e-k230-vision"
tags: ["视觉","K230"]
category: "视觉"
---

这是一篇分享K230视觉方案的文章，如有错误指正或优化建议请发送至1599161405@qq.com。
（完整代码在文末）

## 设计思路
下面是本方案的设计思路，接下来分模块地对各个部分进行介绍。

```
摄像头采集 → 矩形检测 → 几何验证 → 透视校正中心 → EMA 平滑追踪 → UART 发送偏移量
```

## 全局参数
参数这一块先不太深入讲东西，主要目的是让大家有个印象。

### 1.分辨率参数
```py
IMG_W, IMG_H = 320, 240        # 图像分辨率
IDE_w, IDE_h = 800, 480        # IDE窗口分辨率
```

### 2.画面中心参数
```py
IMG_CX, IMG_CY = IMG_W // 2, IMG_H // 2   # 画面中心坐标 (160, 120)
```

### 3.串口通信参数

设置发送时间间隔为30ms（为了和云台主控上串口接收的频率一致，如有其他要求，请自行修改）

```py
SEND_INTERVAL_MS = 30           # 串口发送最小间隔（毫秒）
last_send_time = ticks_ms()     # 上次发送的时间戳
```

### 4.Canny边缘检测与轮廓逼近参数

这些参数大家先有个印象即可，后面会着重讲，尽量给大家讲明白。

```py
canny_thresh1      = 50         # Canny 低阈值
canny_thresh2      = 150        # Canny 高阈值
approx_epsilon     = 0.04       # 轮廓逼近精度（周长的比例系数）
area_min_ratio     = 0.001      # 最小面积占图像面积的比例
max_angle_cos      = 0.3        # 轮廓逼近后角点的最大余弦值
gaussian_blur_size = 5          # 高斯模糊核大小（5×5）
```

### 5.矩形验证阈值
```py
RECT_MIN_SIDE   = 20       # 最短边最小像素长度
RECT_ASPECT_MIN = 0.45     # 最短边与最长边之比的下限
RECT_SIDE_RATIO = 0.3      # 对边长度差异容忍比例
RECT_ANGLE_TOL  = 25       # 单角偏离 90° 的最大容差（度）
RECT_DIAG_RATIO = 0.15     # 两条对角线长度差异容忍比例
RECT_AREA_MIN   = 800      # 最小面积（最短边 × 最长边）
```

| 参数      | 几何含义                                                                                                                                                                                                 |
|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `RECT_MIN_SIDE`       | 防止检测到过小的噪声矩形                                                                                                                                                                                      |
| `RECT_ASPECT_MIN`   | 防止长条形四边形                                                                                                                                                                            |
| `RECT_SIDE_RATIO`      | 要求对边近似等长                                                                                                                                                   |
| `RECT_ANGLE_TOL`    | 每个内角与 90° 的偏差不超过 25°                                                                                                                          |
| `RECT_DIAG_RATIO` | 	矩形对角线应近似等长                                                                                                                                                   |                                     

## 核心函数讲解
接下来为大家讲解几个比较重要的函数

###  validate_rect(corners) — 矩形验证
这个函数作用是根据接收到的4个角点坐标，来验证四个角点链接形成的四边形是否为矩形。

#### 1.计算边向量和边长
```py
for i in range(4):
    dx = corners[(i+1)%4][0] - corners[i][0]
    dy = corners[(i+1)%4][1] - corners[i][1]
    vectors.append((dx, dy))
    sides.append(math.sqrt(dx*dx + dy*dy))
```
在接收到4个角点坐标后，计算出四边形四条边的方向向量的坐标

```py
dx = corners[(i+1)%4][0] - corners[i][0]
dy = corners[(i+1)%4][1] - corners[i][1]
```
其中orners是角点列表，vectors是边向量列表，sides是存储边长的列表

```py
corners = [(x0, y0), (x1, y1), (x2, y2), (x3, y3)] # 索引:分别是0，1，2，3，4
vectors和sides同理
```
#### 2.四重检查
这一步的目的是通过多重限制，筛选掉不符合要求的矩形

1.最短边长度
```py
if min_s < RECT_MIN_SIDE:
    return False, 0
```
这一步的目的是筛选掉太小的矩形。

2.最小面积
```py
if max_s * min_s < RECT_AREA_MIN:
    return False, 0
```
这一步的目的也是筛选掉太小的矩形，但是第一步是从边长长度来筛选，第二步是从面积大小来筛选。

3.宽高比
```py
if min_s / max_s < RECT_ASPECT_MIN:
    return False, 0
```
这一步目的是筛选掉太细长的矩形，短边/长边 ≥ 0.45，至于为什么不是短边/长边=1，因为在透视下四边形可能是梯形等，所以要留够误差。

4.对边等长
```py
if abs(sides[0] - sides[2]) / max_s > RECT_SIDE_RATIO:
    return False, 0
if abs(sides[1] - sides[3]) / max_s > RECT_SIDE_RATIO:
    return False, 0
```
这一步目的是限制四边形对边大致相等，同时留够误差，原因和上一步一样

#### 3.内角验证
```py
dot = -(v1[0] * v2[0] + v1[1] * v2[1])
cos_a = dot / (m1 * m2)
angle = math.degrees(math.acos(cos_a))
```
这一步是计算相邻内角。
注意：代码中取了负号 dot = -(v1·v2)，因为实际上相邻的两个方向向量之间的角是四边形的外角，故需要加负号将向量反向。
然后，判断∣90°−α∣≤25°，是为了限制65°≤α≤115°，在保证基本矩形形状的前提下，允许合理的透视变形。

#### 4.对角线等长验证
```py
d1 = sqrt((corners[2][0] - corners[0][0])**2 + (corners[2][1] - corners[0][1])**2)
d2 = sqrt((corners[3][0] - corners[1][0])**2 + (corners[3][1] - corners[1][1])**2)
```
因为矩形的对角线等长，所以检测到的四边形的对角线也应该大致等长。

#### 5.评分函数
```py
score = max(0, 100 - angle_dev_sum * 4) * 0.5 + \
        (min(sides[0], sides[2]) / max(sides[0], sides[2]) + \
         min(sides[1], sides[3]) / max(sides[1], sides[3])) * 25
```
这个评分函数的计算公式是角度得分和对边比例得分各占50%，当检测到多个矩形时，这个评分函数帮助选择最优的矩形。
角度得分：angle_dev_sum是四个角的偏差总和，下面将结合例子来辅助理解
```
如果四个角都是90°：
angle_dev_sum = 0 + 0 + 0 + 0 = 0
angle_score = 100 - 0 = 100

如果四个角平均偏差5°（85°,95°,85°,95°）：
angle_dev_sum = 5+5+5+5 = 20
angle_score = 100 - 20*4 = 100 - 80 = 20
```
对边比例得分：这一部分同样配上例子来辅助理解
```
如果对边完全相等：
s0=s2, s1=s3 → 两个比例都是1 → (1+1)×25 = 50

如果对边差异10%：
s0=100, s2=90 → 90/100=0.9
s1=100, s3=90 → 90/100=0.9
→ (0.9+0.9)×25 = 45
```

### 透视校正中心
```py
def perspective_center(corners):
    """计算四边形两条对角线的交点，即透视下的真实矩形中心"""
    x1, y1 = corners[0]
    x2, y2 = corners[2]
    x3, y3 = corners[1]
    x4, y4 = corners[3]
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-6:
        return (sum(c[0] for c in corners) / 4.0,
                sum(c[1] for c in corners) / 4.0)
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    cx = x1 + t * (x2 - x1)
    cy = y1 + t * (y2 - y1)
    return (cx, cy)
```
那么在这里我们就需要知道透视的相关知识（在2.四重检查中的宽高比大概提了一句），我们先从一个问题切入：求检测到的矩形正中心坐标，为什么不能直接取四个角点坐标的平均值？答：当摄像头与矩形存在透视关系时（非正俯视），矩形在图像中呈梯形变形。此时四点质心（X平均,Y平均）不等于矩形的真实几何中心的投影点。
那么我们应该获得实际矩形正中心在图像的投影坐标呢？答：两条对角线 AC 和 BD 的交点才是透视不变的中心。
这个结论是有数学证明的（有误差，但是误差几乎可以忽略不计），但是由于输入法等因素，就不列出来了。

### 防坐标抖动\丢帧\误检跳变
这一部分是本方案最核心的部分，在K230稳定，丝滑地运行有很大的作用。
这一部分代码主要有三个功能，分别是防坐标抖动，丢帧，误检跳变。

#### 创建变量
```py
def __init__(self, alpha=0.85, max_miss=8):
    self.alpha = alpha          # EMA 平滑系数
    self.max_miss = max_miss    # 允许连续丢帧的最大帧数
    self.filtered_center = None # EMA 平滑后的中心坐标
    self.last_corners = None    # 上一帧有效的排序角点
    self.miss_count = 0         # 当前连续丢帧计数
    self.velocity = (0.0, 0.0)  # 速度估计 (px/s)
    self.last_time = ticks_ms() # 上帧时间戳
```

| 变量      | 含义                                                                                                                                                                                                 |
|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `alpha`       | EMA 权重                                                                                                                                                                                      |
| `max_miss`   | 最多 8 帧未检测到目标，超过则判定丢失                                                                                                                                                                            |
| `filtered_center`      | 经过 EMA 平滑后的目标中心(x,y)                                                                                                                                                   |
| `miss_count`    | 连续未检测到目标的帧数                                                                                                                          |
| `velocity` | 	目标运动速度                                                                                                                                                   |    

#### 角点排序
```py
def _sort_corners(self, corners):
        """排序为 [左上, 右上, 右下, 左下]"""
        cx = sum(c[0] for c in corners) / 4
        cy = sum(c[1] for c in corners) / 4
        top = sorted([c for c in corners if c[1] < cy], key=lambda c: c[0])
        bot = sorted([c for c in corners if c[1] >= cy], key=lambda c: c[0], reverse=True)
        if len(top) < 2:
            s = sorted(corners, key=lambda c: c[1])
            top = sorted(s[:2], key=lambda c: c[0])
            bot = sorted(s[2:], key=lambda c: c[0], reverse=True)
        return top + bot
```
计算过程：首先计算所有x坐标的平均值（cx）,所有y坐标的平均值(cy)，就可以得到四边形的几何中心坐标。然后分成上下两排，y坐标小于中心 → 上面的点，y坐标大于等于中心 → 下面的点。（这点大家可能对排序顺序有疑问，别急，后面会提到）然后，再分成左右两排，上面一排的点按x从小到大（从左到右），下面的点按x从大到小（从右到左），最后将两个列表拼起来return。
排序顺序问题：K230的Y轴正方向是 ↓ ，X轴正方向是 → ，故排序顺序如上。

#### EMA平滑
其实这一部分我感觉没什么好讲的，大家可以把它当成低通滤波器（EMA平滑是低通滤波器的一种数字实现方式）,算法如下
```
x=α*x(i)+(1-α)*x(i-1)
```
这一步作用就是让代码更平滑。

#### 串口偏移量发送
这一部分也没什么好讲的，就介绍一下吧。
```py
def send_offset(uart, ox, oy):
    ox = max(-9.999, min(9.999, ox))
    oy = max(-9.999, min(9.999, oy))
    msg = "%+.3f,%+.3f" % (ox, oy)
    uart.write(msg)
```

```
ox：水平归一化偏移量
oy：垂直归一化偏移量
发送格式示例：+0.312,-0.156，带符号、3 位小数。
```

## 主循环
这一部分相比较上一部分轻松许多。
### 图像采集
这一部分更没什么好说的了，直接看官方文档就行了。
```py
sensor = Sensor(id=2, width=1280, height=960, fps=90)
sensor.set_framesize(width=320, height=240)
sensor.set_pixformat(Sensor.RGB888)
```
不过要注意这里输出的格式是RGB888

### 矩形检测
```py
rects = cv_lite.rgb888_find_rectangles_with_corners(
    image_shape, img_np,
    canny_thresh1, canny_thresh2,
    approx_epsilon, area_min_ratio,
    max_angle_cos, gaussian_blur_size
)
```
这里要注意的是我使用的是cv_lite，cv_lite是K230平台优化的轻量级CV库，使用这个库，可以减轻硬件压力，这也是本方案可以跑到90+fps的重要原因。（关于矩形检测还是去看官方文档就行了）

### 最优矩形选取
```py
for r in rects:
    corners = [(r[4], r[5]), (r[6], r[7]), (r[8], r[9]), (r[10], r[11])]
    valid, score = validate_rect(corners)
    if valid and score > best_score:
        best_score = score
        best_corners = corners
        best_center = perspective_center(corners)
```
遍历所有检测到的矩形，经过validate_rect验证后取评分最高的作为目标矩形。

### 偏移量归一化
```py
offset_x = (target_pt[0] - IMG_CX) / IMG_CX
offset_y = (target_pt[1] - IMG_CY) / IMG_CY
```
(0,0)目标在画面正中心,(-1,-1)目标在左上角,(+1,+1)目标在右下角
（排序问题上面有解释）

### 可视化
| 绘制元素      | 含义                                                                                                                                                                                                 |
|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `青色十字`       | 画面中心基准                                                                                                                                                                                      |
| `绿色矩形框`   | 检测到的矩形轮廓                                                                                                                                                                            |
| `蓝色角点圆`      | 四个角点位置                                                                                                                                                   |
| `橙色对角线`    | 调试用，可视化交点                                                                                                                          |
| `红色十字` | 	追踪器输出的目标中心                                                                                                                                                   |
| `状态文字`       | OK=正常检测，PRED=预测补帧，LOST=目标丢失                                                                                                                                                    |

### 内存管理
```py
if frame_count % 50 == 0:
    gc.collect()
```
K230 运行 MicroPython，每 50 帧手动触发垃圾回收，防止卡顿。

## 总结
本视觉方案的技术栈完整覆盖了从 图像采集 → 特征提取 → 几何验证 → 追踪 → 通信输出 的全流程。
整套方案在 K230 上实测可达 90+ fps 的有效处理帧率。

## 代码
```py
#基于01科技K230的25电赛E题的视觉方案

import time, os, sys, math, gc
from media.sensor import *
from media.display import *
from media.media import *
from time import ticks_ms, ticks_diff
from machine import UART
from machine import FPIOA
import struct

###全局参数
IMG_W, IMG_H = 320, 240
IMG_CX, IMG_CY = IMG_W // 2, IMG_H // 2
IDE_w, IDE_h = 800, 480

SEND_INTERVAL_MS = 30
last_send_time = ticks_ms()

canny_thresh1      = 50
canny_thresh2      = 150
approx_epsilon     = 0.04
area_min_ratio     = 0.001
max_angle_cos      = 0.3
gaussian_blur_size = 5

RECT_MIN_SIDE   = 20
RECT_ASPECT_MIN = 0.45
RECT_SIDE_RATIO = 0.3
RECT_ANGLE_TOL  = 25
RECT_DIAG_RATIO = 0.15
RECT_AREA_MIN   = 800

#-------矩形验证
def validate_rect(corners):
    sides = []
    vectors = []
    for i in range(4):
        dx = corners[(i+1)%4][0] - corners[i][0]
        dy = corners[(i+1)%4][1] - corners[i][1]
        vectors.append((dx, dy))
        sides.append(math.sqrt(dx*dx + dy*dy))

    max_s = max(sides)
    min_s = min(sides)

    if min_s < RECT_MIN_SIDE:
        return False, 0
    if max_s * min_s < RECT_AREA_MIN:
        return False, 0
    if min_s / max_s < RECT_ASPECT_MIN:
        return False, 0
    if abs(sides[0] - sides[2]) / max_s > RECT_SIDE_RATIO:
        return False, 0
    if abs(sides[1] - sides[3]) / max_s > RECT_SIDE_RATIO:
        return False, 0

    angle_dev_sum = 0
    for i in range(4):
        v1 = vectors[i]
        v2 = vectors[(i+1)%4]
        dot = -(v1[0] * v2[0] + v1[1] * v2[1])
        m1, m2 = sides[i], sides[(i+1)%4]
        if m1 < 0.01 or m2 < 0.01:
            return False, 0
        cos_a = max(-1.0, min(1.0, dot / (m1 * m2)))
        angle = math.degrees(math.acos(cos_a))
        dev = abs(90 - angle)
        if dev > RECT_ANGLE_TOL:
            return False, 0
        angle_dev_sum += dev

    if angle_dev_sum > RECT_ANGLE_TOL * 2.5:
        return False, 0

    d1 = math.sqrt((corners[2][0] - corners[0][0])**2 + (corners[2][1] - corners[0][1])**2)
    d2 = math.sqrt((corners[3][0] - corners[1][0])**2 + (corners[3][1] - corners[1][1])**2)
    md = max(d1, d2)
    if md < 1 or abs(d1 - d2) / md > RECT_DIAG_RATIO:
        return False, 0

    score = max(0, 100 - angle_dev_sum * 4) * 0.5 + \
            (min(sides[0], sides[2]) / max(sides[0], sides[2]) + \
             min(sides[1], sides[3]) / max(sides[1], sides[3])) * 25
    return True, score

#-------透视校正中心（对角线交点）
def perspective_center(corners):
    """计算四边形两条对角线的交点，即透视下的真实矩形中心"""
    x1, y1 = corners[0]
    x2, y2 = corners[2]
    x3, y3 = corners[1]
    x4, y4 = corners[3]
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-6:
        return (sum(c[0] for c in corners) / 4.0,
                sum(c[1] for c in corners) / 4.0)
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    cx = x1 + t * (x2 - x1)
    cy = y1 + t * (y2 - y1)
    return (cx, cy)

#-------防坐标抖动\丢帧\误检跳变
class RectTracker:
    def __init__(self, alpha=0.85, max_miss=8):
        self.alpha = alpha          # 中心点平滑系数（越大越跟随新值）
        self.max_miss = max_miss
        self.filtered_center = None
        self.last_corners = None    # 保存排序后的角点（仅用于绘制）
        self.miss_count = 0
        self.velocity = (0.0, 0.0)
        self.last_time = ticks_ms()

    def _sort_corners(self, corners):
        """排序为 [左上, 右上, 右下, 左下]"""
        cx = sum(c[0] for c in corners) / 4
        cy = sum(c[1] for c in corners) / 4
        top = sorted([c for c in corners if c[1] < cy], key=lambda c: c[0])
        bot = sorted([c for c in corners if c[1] >= cy], key=lambda c: c[0], reverse=True)
        if len(top) < 2:
            s = sorted(corners, key=lambda c: c[1])
            top = sorted(s[:2], key=lambda c: c[0])
            bot = sorted(s[2:], key=lambda c: c[0], reverse=True)
        return top + bot

    def update(self, new_corners, new_center):
        now = ticks_ms()
        dt = ticks_diff(now, self.last_time) / 1000.0
        self.last_time = now

        if new_center is not None:
            # 用排序后的原始角点（不做EMA平滑！）
            sc = self._sort_corners(new_corners)
            self.last_corners = sc

            # 在原始角点上计算对角线交点
            raw_cx, raw_cy = perspective_center(sc)

            # 速度估计
            if self.filtered_center:
                self.velocity = (
                    (raw_cx - self.filtered_center[0]) / max(0.01, dt),
                    (raw_cy - self.filtered_center[1]) / max(0.01, dt)
                )

            # 只对中心点做一次EMA平滑
            if self.filtered_center:
                fcx = self.alpha * raw_cx + (1 - self.alpha) * self.filtered_center[0]
                fcy = self.alpha * raw_cy + (1 - self.alpha) * self.filtered_center[1]
            else:
                fcx = raw_cx
                fcy = raw_cy

            self.filtered_center = (round(fcx), round(fcy))
            self.miss_count = 0
            return self.last_corners, self.filtered_center, False
        else:
            if self.filtered_center and self.miss_count < self.max_miss:
                px = self.filtered_center[0] + self.velocity[0] * dt
                py = self.filtered_center[1] + self.velocity[1] * dt
                self.filtered_center = (round(px), round(py))
                self.miss_count += 1
                return self.last_corners, self.filtered_center, True
            else:
                self.velocity = (0.0, 0.0)
                return None, None, False

    def reset(self):
        self.filtered_center = None
        self.last_corners = None
        self.miss_count = 0
        self.velocity = (0.0, 0.0)

# ---------- 串口发送函数
def send_offset(uart, ox, oy):
    try:
        ox = max(-9.999, min(9.999, ox))
        oy = max(-9.999, min(9.999, oy))
        msg = "%+.3f,%+.3f" % (ox, oy)
        uart.write(msg)
    except Exception:
        pass

#---------- 摄像头初始化
sensor = Sensor(id=2, width=1280, height=960, fps=90)
sensor.reset()
sensor.set_framesize(width=IMG_W, height=IMG_H)
sensor.set_pixformat(Sensor.RGB888)
import cv_lite
import ulab.numpy as np

Display.init(Display.ST7701, width=IDE_w, height=IDE_h, to_ide=True, quality=50)

# ---------- 串口初始化（UART1，与STM32通信）
fpioa = FPIOA()
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)
uart = UART(UART.UART1, 115200)

MediaManager.init()
sensor.run()

clock = time.clock()
tracker = RectTracker()
frame_count = 0

image_shape = [IMG_H, IMG_W]

while True:
    clock.tick()

    img = sensor.snapshot()

    best_corners = None
    best_center = None

    img_np = img.to_numpy_ref()

    rects = cv_lite.rgb888_find_rectangles_with_corners(
        image_shape, img_np,
        canny_thresh1, canny_thresh2,
        approx_epsilon, area_min_ratio,
        max_angle_cos, gaussian_blur_size
    )

    best_score = -1
    for r in rects:
        corners = [(r[4], r[5]), (r[6], r[7]), (r[8], r[9]), (r[10], r[11])]
        valid, score = validate_rect(corners)
        if valid and score > best_score:
            best_score = score
            best_corners = corners
            best_center = perspective_center(corners)

    corners, center, is_pred = tracker.update(best_corners, best_center)
    target_pt = center if center else None

    # ---------- 计算偏移量
    if target_pt:
        offset_x = (target_pt[0] - IMG_CX) / IMG_CX
        offset_y = (target_pt[1] - IMG_CY) / IMG_CY
    else:
        offset_x, offset_y = 0.0, 0.0

    # ---------- 串口发送偏移量给STM32
    now = ticks_ms()
    if ticks_diff(now, last_send_time) >= SEND_INTERVAL_MS:
        send_offset(uart, offset_x, offset_y)
        last_send_time = now

    # ---------- 绘制调试信息
    img.draw_cross(IMG_CX, IMG_CY, size=5, color=(0, 255, 255), thickness=1)

    if corners:
        for i in range(4):
            c1 = corners[i]
            c2 = corners[(i + 1) % 4]
            img.draw_line(int(c1[0]), int(c1[1]), int(c2[0]), int(c2[1]),
                         color=(0, 255, 0), thickness=2)
        for c in corners:
            img.draw_circle(int(c[0]), int(c[1]), 3, color=(0, 0, 255), fill=True)
        # 画对角线用于调试
        img.draw_line(int(corners[0][0]), int(corners[0][1]),
                      int(corners[2][0]), int(corners[2][1]),
                      color=(255, 128, 0), thickness=1)
        img.draw_line(int(corners[1][0]), int(corners[1][1]),
                      int(corners[3][0]), int(corners[3][1]),
                      color=(255, 128, 0), thickness=1)

    if target_pt:
        img.draw_cross(target_pt[0], target_pt[1], size=6, color=(255, 0, 0), thickness=2)
        img.draw_string_advanced(5, 30, 10,
            "Off:%.3f,%.3f" % (offset_x, offset_y), color=(255, 255, 0))
        print(f"offset_x:{offset_x:.3f},offset_y:{offset_y:.3f}")
    status = "PRED" if is_pred else ("OK" if center else "LOST")
    img.draw_string_advanced(5, 5, 10,
        "%.1ffps %s" % (clock.fps(), status), color=(255, 255, 255))

    Display.show_image(img,
        x=(IDE_w - IMG_W) // 2,
        y=(IDE_h - IMG_H) // 2)

    frame_count += 1
    if frame_count % 50 == 0:
        gc.collect()
```