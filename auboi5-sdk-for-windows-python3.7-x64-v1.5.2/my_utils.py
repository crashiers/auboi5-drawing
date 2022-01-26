from random import gammavariate
from time import sleep
import cv2
import numpy as np
import pandas as pd
from icecream import ic
from matplotlib import pyplot as plt
from math import pi, degrees, radians
import time
from utils import *
from PIL import Image
# 循环引用了
# from drawing import window_every_brush

# 识别颜色不对就要把相机拔下来重新插电

# 全局变量
check_every_num_physical_stroke = 10
max_mix_count = 4
min_mix_interval = 7
NUM_BRUSH = 2
init_two_saves = False

ion = True

L_th = 0.25 # 相似容忍度，0 to 1
# L_th_got_pos = 0.35 # 相似容忍度，0 to 1
L_th_got_pos = L_th # 相似容忍度，0 to 1
# L_th = 0.38 # 0 to 1
# L_th = 0.38 # 0 to 1
# L_th = 0.15 # 0 to 1

# robot要多用力画
# original_draw_depth = 0.000
# original_draw_depth = 0.001
original_draw_depth =  0.005
speed_divisor = 5 # 5是极限了，再快固定不住

# original_mix_depth = 0.010 # 1
original_mix_depth = 0.000 # 2
# original_mix_depth = - 0.010
# mix_depth_adj = 0.007

emotion = 'sad'

# original_dip_depth = 0.035 # 1
original_dip_depth = 0.015 # 2
# original_dip_depth = 0.003

# canvas_height = - 0.01 # 画画的位置跟调色的z坐标本就不同
canvas_height = + 0.042 - 0.004 - 0.008 # 画画的位置跟调色的z坐标本就不同
canvas_offset_x = 0.205
# CANVAS_X = 295 - 10 * 2
# CANVAS_Y = 205

# CANVAS_X = 526
# CANVAS_Y = 373

CANVAS_X = 600 -  10 * 2
CANVAS_Y = 500 - 5

pigments_height_adj = 0.00
# height_in_the_air = 0.20
height_in_the_air = 0.15
mix_place_xy = {1:
            [(-0.17 + 0.05 * j, 0.50 - i * 0.05) 
            for j in range(8) for i in range(10) ],
            # for j in range(4) for i in range(9) ],
            2:
            [(-0.17 + 0.05 * j, 0.50 - i * 0.05) 
            for j in range(8) for i in range(10) ],
            # for j in range(4) for i in range(9) ]
            3:
            [(-0.17 + 0.05 * j, 0.50 - i * 0.05) 
            for j in range(8) for i in range(10) ],
            # for j in range(4) for i in range(9) ]
            4:
            [(-0.17 + 0.05 * j, 0.50 - i * 0.05) 
            for j in range(8) for i in range(10) ],
            # for j in range(4) for i in range(9) ]
            5:
            [(-0.17 + 0.05 * j, 0.50 - i * 0.05) 
            for j in range(8) for i in range(10) ],
            # for j in range(4) for i in range(9) ]
            }
mix_array_last_time = {1:
            {(-0.17 + 0.05 * j, 0.50 - i * 0.05) : 0
            for j in range(8) for i in range(10) },
            # for j in range(4) for i in range(9) ],
            2:
            {(-0.17 + 0.05 * j, 0.50 - i * 0.05) : 0
            for j in range(8) for i in range(10) },
            # for j in range(4) for i in range(9) ]
            3:
            {(-0.17 + 0.05 * j, 0.50 - i * 0.05) : 0
            for j in range(8) for i in range(10) },
            # for j in range(4) for i in range(9) ]
            4:
            {(-0.17 + 0.05 * j, 0.50 - i * 0.05) : 0
            for j in range(8) for i in range(10) },
            # for j in range(4) for i in range(9) ]
            5:
            {(-0.17 + 0.05 * j, 0.50 - i * 0.05) : 0
            for j in range(8) for i in range(10) }
            # for j in range(4) for i in range(9) ]
            }

# mix_last_time = 0

# 颜料高度，单位米
pigments_height = {
    'C': 0.055 - 0.004, 'M': 0.010 - 0.004, 'Y': 0.055 - 0.004, 'K': 0.058 - 0.004, 'W': 0.053 - 0.004
    }

# 大窗口
window_every_brush = {1:
np.array([(339 - 5, 329), (353 - 5, 350)]),
2:
np.array([(341, 326), (356, 349)]),
3:
np.array([(339, 337), (349, 347)]),
4:
np.array([(346, 342), (362, 358)]),
5:
np.array([(359, 329), (383, 348)])
}

# # 用9*9的中间的小区域做均值
# mid = {}
# for k, i in window_every_brush.items():
#     window = i
#     mid[k] = (window[0]+window[1])//2

# win_new = {}
# # mid = [int(i) for i in mid]
# for k, i in mid.items():
#     win_new[k] = np.array([i-1,i+1])

# window_every_brush = win_new
# # ic(win_new)

calibrate_coord_origin = [0.2683583776067739, -0.6221894891561226, 0.457813437164197]
calibrate_T_xy = {'C':(-0.285, 0.43), 
            'M':(-0.285, 0.35),
            'Y':(-0.285, 0.27),
            'K':(-0.285, 0.19),
            'W':(-0.285, 0.09)}

mix_movement_length = 0.010
# check_every_num_physical_stroke = 5


from_color_to_int = {'C':0 , 'M':1 , 'Y':2 , 'K':3 , 'W': 4}

SERVER_IP = '192.168.11.127' # 机械臂 ip，机械臂作为服务器

# SERVER_IP = '192.168.11.129'
# SERVER_IP = '192.168.11.128'

CMYKW_str = ['C', 'M', 'Y', 'K', 'W']

def normalize(a, min, max, to_min, to_max):
    return to_min + (a - min) / (max - min) * (to_max - to_min)

# 沿Ｚ轴运动
def up_to_height_in_the_air(robot, height): 
    current_pos = robot.get_current_waypoint()
    
    # O = (0.2587857035467645, -0.6103817043849445, 0.46096235061927077) # 因此单位是米
    
    current_pos['pos'][2] = height + calibrate_coord_origin[2] # 运动至这么高处
    
    ik_result = robot.inverse_kin(current_pos['joint'], current_pos['pos'], current_pos['ori'])
    # logger.info(ik_result)
    
    robot.move_line(ik_result['joint'])

def judge_similar( i, j):
    return np.max(np.abs(np.array(i) - np.array(j))) < L_th

def judge_pos_similar( i, j):
    return np.max(np.abs(np.array(i) - np.array(j))) < 0.001

def mix_movement(robot, current):
    delta = np.array((0, -1, 0, 1, 0)) * mix_movement_length
    current = np.array(current)
    # for i in range(3):
    #     pos = ( current[0] + mix_movement_length, current[1], current[2]) 
    # current_xy = np.array(( current[0], current[1] ) )
    
    for i in range(4):
        
        pos = current + np.array((delta[i], delta[i+1], 0))
        # pos = ( current[0] + mix_movement_length, current[1], current[2]) 
        mere_move_cartesian(robot, pos)

    mere_move_cartesian(robot, current)

# 移动到某个点，用笛卡尔坐标
def move_cartesian_in_up_way(robot, to_):
    # 注意安全，在这个函数里首先，抬起，到0.15m的高度，在这个高度下水平移动，再，落下，到目标点位
    # 抬起，移动，落下

    # 抬起
    # 用户坐标转换成基坐标
    # 在这设置目标坐标
    up_to_height_in_the_air(robot, height_in_the_air)

    pos_to = ( to_[0], to_[1], height_in_the_air) 
    
    # 移动，
    mere_move_cartesian(robot, pos_to)
    
    # 落下
    mere_move_cartesian(robot, to_)

def tupleChanger(tup, divisor):
    return tuple((i / divisor for i in tup))    

def speed_changer(robot, speed_divisor):
    joint_maxvelc = (2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177)
    joint_maxacc = (17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5)
    joint_maxvelc_1 = tupleChanger(joint_maxvelc, speed_divisor) # 调节速度
    joint_maxacc_1 = tupleChanger(joint_maxacc, speed_divisor)            
    robot.set_joint_maxacc(joint_maxacc_1)
    robot.set_joint_maxvelc(joint_maxvelc_1)
    robot.set_arrival_ahead_blend(0.05)
    robot.set_collision_class(7) # 设置碰撞等级

def read_one_frame(cap):
    ret, frame = cap.read()
    if ret:
        if frame is not None:
            cv2.imshow('camera', frame)
            cv2.waitKey(1)
        else:
            print("无画面")
    else:
        print("无法读取摄像头！")

def RGB2CMYK(rgb):
    # 先归一化
    r, g, b = rgb[0] / 255, rgb[1] / 255, rgb[2] / 255
    K = 1-max(r, g, b)
    C = (1 - r - K) / (1 - K)
    M = (1 - g - K) / (1 - K)
    Y = (1 - b - K) / (1 - K)
    return C, M, Y, K

def cal_mean_color_for_window_in_cmyk(img, window):
    # 一定要先转化成RGB再送进来
    # 还是直接送进来好了
    # 完完整整的按照window来检测

    img = img[window[0][1]:window[1][1], window[0][0]:window[1][0] ]
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # avg_color_per_row = np.average(img, axis=0)
    # avg_color = np.average(avg_color_per_row, axis=0)
    # ic(img.shape)
    l = []
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            C, M, Y, K = RGB2CMYK(img[i, j])
            l.append([C, M, Y, K])
    # g = np.mean(img, 0)
    # g = np.mean(g, 0)
    # ic(g.shape)
    l = np.array(l)
    
    # ic(g)
    # ic(np.mean(l, 0))
    ret = np.mean(l, 0)
    # ic(ret.shape)
    return ret

def cal_mean_color_for_window_in_rgb(img, window):
    # 一定要先转化成RGB再送进来
    # 还是直接送进来好了
    # 完完整整的按照window来检测

    img = img[window[0][1]:window[1][1], window[0][0]:window[1][0] ]
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # avg_color_per_row = np.average(img, axis=0)
    # avg_color = np.average(avg_color_per_row, axis=0)
    # ic(img.shape)
    l = []
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            R, G, B = (img[i, j])
            l.append([R, G, B])
    # g = np.mean(img, 0)
    # g = np.mean(g, 0)
    # ic(g.shape)
    l = np.array(l)
    
    # ic(g)
    # ic(np.mean(l, 0))
    ret = np.mean(l, 0)
    # ic(ret.shape)
    return ret

def get_current_robo_state(robot):
    # 获取当前位置, robot的
    current_pos = robot.get_current_waypoint() # 法兰中心相对于基坐标

    ic(current_pos['joint'])
    ic(current_pos['pos']) # 法兰中心相对于基坐标的位置
    ic(current_pos['ori']) # 法兰中心相对于基坐标的姿态

def robot_init():
    
    logger_init() # 初始化logger
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time())) # 启动测试
    Auboi5Robot.initialize() # 系统初始化
    robot = Auboi5Robot() # 创建机械臂控制类
    handle = robot.create_context() # 创建上下文
    logger.info("robot.rshd={0}".format(handle)) # 打印上下文

    return robot, handle

# 移动到某个点，用笛卡尔坐标，传入想象中的用户坐标
def mere_move_cartesian(robot, to_):
    
    O = calibrate_coord_origin # 用户坐标转换成基坐标
    # O = (0.252664215711952, -0.6055484403729522, 0.4622574282366732) # 用户坐标转换成基坐标
    T = to_ # 在这设置目标坐标
    
    pos = ( O[0] - T[0], O[1] - T[1], O[2] + T[2]) # 法兰
    ori = [7.900953430891173e-07,0.7071120073763648,-0.7071015549415427,-4.774523074271425e-06] # 先摆正

    robot.move_to_target_in_cartesian(pos, list([degrees(i) for i in robot.quaternion_to_rpy(ori)]))

def calc_mid_points(a_x, a_y, b_x, b_y):
    return [ [ ((1-lamb) * a_x + lamb * b_x) , ((1-lamb) * a_y + lamb * b_y) ] for lamb in np.arange(0, 1, 0.2) ]

# 移动到某个点，用笛卡尔坐标，传入想象中的用户坐标
def convert_imaginary_user_coord_to_base(coord):
    
    O = calibrate_coord_origin # 用户坐标转换成基坐标
    
    pos = ( O[0] - coord[0], O[1] - coord[1], O[2] + coord[2]) # 法兰
    
    return pos

def whole_mere_move_cartesian():
    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))

    cap = cv2.VideoCapture(0)
    cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)

    ret, frame = cap.read()
    if ret:
        if frame is not None:
            cv2.imshow('camera', frame)
            cv2.waitKey(1)
        else:
            print("无画面")
    else:
        print("无法读取摄像头！")


    try:

        # 链接服务器
        ip = SERVER_IP
        # ip = 'localhost'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:

            joint_maxvelc = (2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177)
            joint_maxacc = (17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5)
            
            joint_maxvelc = tupleChanger(joint_maxvelc, 20)
            joint_maxacc = tupleChanger(joint_maxacc, 20)
            # print(f"{type(joint_maxvelc)}, {joint_maxvelc}")
            # print(f"{type(joint_maxacc)}, {joint_maxacc}")
            
            robot.set_joint_maxacc(joint_maxacc)
            robot.set_joint_maxvelc(joint_maxvelc)
            robot.set_arrival_ahead_blend(0.05)

            # 重新上电
            # robot.robot_shutdown()

            # 上电
            # robot.robot_startup()

            # 设置碰撞等级
            robot.set_collision_class(7)

            # 开始

            # 获取当前位置, robot的
            """ 返回：
            六个关节角 {'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            *位置 'pos': [-0.06403157614989634, -0.4185973810159096, 0.816883228463401],
            *姿态 'ori': [-0.11863209307193756, 0.3820514380931854, 0.0, 0.9164950251579285]}
            """

            # 获得当前状态
            current_pos = robot.get_current_waypoint() # 法兰中心相对于基坐标

            ic(current_pos['joint'])
            ic(current_pos['pos']) # 法兰中心相对于基坐标的位置
            ic(current_pos['ori']) # 法兰中心相对于基坐标的姿态

            mere_move_cartesian(robot, (0, 0, 0))

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))

    finally:
        # 断开服务器链接
        if robot.connected:
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()

def recognize_color():
    # gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)                     # 高斯模糊
    # hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)                 # 转化成HSV图像
    # erode_hsv = cv2.erode(hsv, None, iterations=2)                   # 腐蚀 粗的变细
    # inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
    # cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    # c = max(cnts, key=cv2.contourArea)
    # rect = cv2.minAreaRect(c)
    # box = cv2.boxPoints(rect)
    # cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)
    pass

def test_camera():
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            if frame is not None:
                # gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)                     # 高斯模糊
                # hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)                 # 转化成HSV图像
                # erode_hsv = cv2.erode(hsv, None, iterations=2)                   # 腐蚀 粗的变细
                # inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
                # cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

                # c = max(cnts, key=cv2.contourArea)
                # rect = cv2.minAreaRect(c)
                # box = cv2.boxPoints(rect)
                # cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)

                k = cv2.waitKey(1)
                if k == 27 :
                    print("1")
                    filename = 'test.png'
                    cv2.imwrite(filename, frame, params=None)
                    # cv2.destroyAllWindows()

                cv2.imshow('camera', frame)
                cv2.waitKey(1)
            else:
                print("无画面")
        else:
            print("无法读取摄像头！")

    cap.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def test_cal_mean_color_for_window():
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            if frame is not None:
                # ic(frame.type)
                window_every_brush = {1:
[(333 - 2,337 - 2 - 16), (350 - 2,353 - 10 - 8)]}
                ret = cal_mean_color_for_window(frame, window_every_brush[1])
                ic(ret)
            else:
                print("无画面")
        else:
            print("无法读取摄像头！")

    cap.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def test_show_that_window():
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)

    # time.sleep(5)
    # 试后面几次
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            if frame is not None:
                
                window_every_brush = {1:
[(333 - 2,337 - 2 - 16), (350 - 2,353 - 10 - 8)]}
                window = np.array(window_every_brush[1])
                
                show_that_window(frame, window)
            else:
                print("无画面")
        else:
            print("无法读取摄像头！")

    cap.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def test_track_move(robot):
    #添加3个全局路点
    import math
    wp1 = (math.radians(-17.682977) , math. radians(27.785112), math. radians(-138.615629),
    math. radians( - 76.400734) , math. radians( -90) , math. radians(-107.682980))
    wp2 = ( math. radians(-17.682979) , math. radians(-0.337404) , math. radians(-131.976125),
    math. radians ( -41.638725) , math. radians( -90) , math.radians(-107.682980))
    wp3 = (math. radians( -34.376667) , math. radians(2.484861) , math. radians( -129.077134) ,
    math. radians( -41.562015) , math. radians(-90), math. radians(- 124.376707))
    robot .add_waypoint (wp1 )
    robot . add_waypoint (wp2)
    robot. add_waypoint (wp3)
    robot.move_track(RobotMoveTrackType.CARTESIAN_MOVEP)

if __name__ == '__main__':
    # test_show_that_window()
    ic(1)


    # 测试识别颜色的window
    # for ind, i in enumerate([f'uppermost-{j}.png' for j in range(9)]):
    #     image = cv2.imread(i)
    #     draw_1=cv2.rectangle(image, (333,337), (350,353), (0,255,0), 2)
    #     # cv2.imshow("draw_0", image)#显示画过矩形框的图片
    #     cv2.imwrite(i[:-4] + f'-rect.png', draw_1)
    #     cv2.waitKey(0)


    # data = pd.read_csv('happy_data_transfer_df_brush_1.csv')
    # ic(data[data['num_stroke'] == 3])


    # image = cv2.imread(f'uppermost-{5}-rect.png')
    # crop=image[337+2:353 -2, 333 +2:350 -2] 
    # # cv2.imshow("draw_0", image)#显示画过矩形框的图片
    # cv2.imwrite(f'crop.png', crop)
    # cv2.waitKey(0)


    # img = cv2.imread('uppermost-5-rect.png',cv2.COLOR_BGR2RGB)
    # # (333,337) 这是点，不是x和y的范围！
    # i = 337 +2
    # j = 333 +2
    # k = img[i:353 -2,j:350 -2] # 行，列，其中行为竖着数的，列为横向运动着数的
    # avg_color_per_row = np.average(k, axis=0)
    # avg_color = np.average(avg_color_per_row, axis=0)
    # # g = np.mean(k, 0)
    # # g = np.mean(g, 0)
    # # ic(g.shape)
    # ic(avg_color)



    # test_camera()
    # test_cal_mean_color_for_window()

# test_img = Image.open('D:\BaiduNetdiskWorkspace\绘画作业\E43843D1-96F0-4A0D-B987-82FD69B3DD3B.png')
# plt.imshow(test_img)
# plt.show()
