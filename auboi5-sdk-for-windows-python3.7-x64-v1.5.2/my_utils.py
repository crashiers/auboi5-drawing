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
# 循环引用了
# from drawing import window_every_brush


# 全局变量
original_draw_depth = 0.001
# original_draw_depth = 0.002


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

def cal_mean_color_for_window(img, window):
    # 一定要先转化成RGB再送进来
    # 还是直接送进来好了

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

def get_current_state(robot):
    # 获取当前位置, robot的
    current_pos = robot.get_current_waypoint() # 法兰中心相对于基坐标

    ic(current_pos['joint'])
    ic(current_pos['pos']) # 法兰中心相对于基坐标的位置
    ic(current_pos['ori']) # 法兰中心相对于基坐标的姿态

def robot_init():
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

    return robot, handle

# 移动到某个点，用笛卡尔坐标
def mere_move_cartesian(robot, to_):
    # 用户坐标转换成基坐标
    # 在这设置目标坐标
    O = (0.2587857035467645, -0.6103817043849445, 0.46096235061927077)
    
    T = to_
    
    pos = ( O[0] - T[0], O[1] - T[1], O[2] + T[2]) # 法兰
    # pos_user = (0.1, 0.05, 0.1) # 法兰
    # 先摆正
    # rpy_user = tuple((i for i in robot.quaternion_to_rpy()))
    ori = [7.900953430891173e-07,0.7071120073763648,-0.7071015549415427,-4.774523074271425e-06]

    robot.move_to_target_in_cartesian(pos, list([degrees(i) for i in robot.quaternion_to_rpy(ori)]))

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

def show_that_window(frame, window):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    draw_1=cv2.rectangle(frame, window[0], window[1], (255,0,0), 1)
    draw_1_rgb = cv2.cvtColor(draw_1, cv2.COLOR_BGR2RGB)
    
    # 原frame
    plt.subplot(221)
    plt.imshow(frame_rgb)

    # 框出大window
    plt.subplot(222)
    plt.imshow(draw_1_rgb)

    # 用9*9的中间的小区域做均值
    mid = (window[0]+window[1])//2
    # mid = [int(i) for i in mid]
    win_new = [mid-1,mid+1]
    ic(win_new)

    # 原frame的大window
    plt.subplot(223)
    plt.imshow(frame_rgb[window[0][1]:window[1][1],window[0][0]:window[1][0]])

    # 原frame的大window中的小区域
    # plt.subplot(224)
    # plt.imshow(frame_rgb[win_new[0][1]:win_new[1][1],win_new[0][0]:win_new[1][0]])

    ret = cal_mean_color_for_window(frame, window)
    # ret = cal_mean_color_for_window(frame, win_new)
    ic(ret)
    
    plt.show()

def calibration_include_camera_for_each_brush(num_brush):
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

            # 在这里出现的都是current_imaginary_user_pos
            # 及时更新
            # current_imaginary_user_pos = (-0.19, 0.425, 0.15)
               
               
            draw_depth = original_draw_depth
            mix_place = {1:list(
                [(-0.17 + 0.05 * j, 0.50 - i * 0.05, -draw_depth) 
                for j in range(4) for i in range(9) ]
                ),
                2:list(
                [(-0.17 + 0.05 * j, 0.50 - i * 0.05, -(0.315-0.313) -draw_depth) 
                for j in range(4) for i in range(9) ]
                )
                }
            mix_place_count = 0

            # 首先读出需要的颜色等等
            
            data_transfer_df = pd.read_csv('happy_data_transfer_df_brush_1.csv')
            for index, row in data_transfer_df.iterrows():
                num_physical_stroke = int(row['num_physical_stroke'])
                num_stroke = int(row['num_stroke'])
                assert num_brush == int(row['num_brush'])
                start_point_x = row['start_point_x']
                start_point_y = row['start_point_y']
                end_point_x = row['end_point_x']
                end_point_y = row['end_point_y']
            
                theta_deg = row['theta_deg']
                theta_rad = row['theta_rad']
                r_t = int(row['r'])
                g_t = int(row['g'])
                b_t = int(row['b'])


                # 得到目标颜色
                rgb_T = (r_t, g_t, b_t)
                # rgb_T = (199, 172, 221)
                CMYK_T = RGB2CMYK(rgb_T)
                C_t = CMYK_T
                ic(C_t)
                C_d = np.array([0, 0, 0, 0]) # 初始化
                check_every_num_physical_stroke = 5
                CMYKW_str = ['C', 'M', 'Y', 'K', 'W']


                # 读取表格
                df_read = pd.read_csv(f'save_ok_color_pos_{num_brush}.csv')
                got_pos = None
                for ind, i in df_read.iterrows():
                    if np.max(np.abs(np.array((i['C'], i['M'], i['Y'], i['K'])) - C_t)) < 0.15:
                        got_pos = (i['x'], i['y'], i['z'])
                
                if got_pos:
                    move_cartesian_in_up_way(robot, got_pos)
                    mix_movement(robot, got_pos)
                    up_to_15mm(robot)

                # 混合颜色
                while not got_pos and True:
                
                    C_m = np.argmax(np.abs(C_t - C_d))
                    if C_d[C_m] > C_t[C_m]:
                        C_m = 4 # 白色
                    # 这里有一个循环需要一直调色
                    
                    #计算需要的颜色C_m


                    # 取得这种颜色
                    L = np.max(np.abs(C_t - C_d)) # 0 to 1
                    ic(L)

                    dip_depth = 0.005 * (L)
                    # 第1个笔刷, Target
                    T={1:
                    {'C':(-0.25, 0.425, 0.076 - dip_depth), 'M':(-0.25, 0.34, 0.073 - dip_depth),'Y':(-0.25, 0.26, 0.083 - dip_depth),
                    'K':(-0.25, 0.17, 0.054 - dip_depth),'W':(-0.25, 0.07, 0.074 - dip_depth)},
                    2:
                    {'C':(-0.25, 0.425, 0.076 -(0.315-0.313) - dip_depth), 'M':(-0.25, 0.34, 0.073 -(0.315-0.313) - dip_depth),'Y':(-0.25, 0.26, 0.083 -(0.315-0.313) - dip_depth),
                    'K':(-0.25, 0.17, 0.054 -(0.315-0.313) - dip_depth),'W':(-0.25, 0.07, 0.074 -(0.315-0.313) - dip_depth)}
                    }
                    # # W 
                    # # T = (-0.25, 0.07, 0.15) # 或许可以用比-0.01小的值
                    # # C
                    # T = (-0.25, 0.425, 0.35) # 或许可以用比-0.01小的值
                    
                    # 这里要优化成字典
                    # mix_place_upper_most = (-0.17, 0.50, -draw_depth)
                    # mix_place = list([(-0.17, 0.50 - i * 0.05, -draw_depth) for i in range(9)])
                    # mix_place = list([(-0.17 + 0.05, 0.50 - i * 0.05, -draw_depth) for i in range(9)])
                    # mix_place = [(-0.19, 0.58, -draw_depth)]

                    
                    # T = (0., 0, -0.01) # 想象中的用户坐标系
                    ic(C_m)
                    move_cartesian_in_up_way(robot, T[num_brush][CMYKW_str[C_m]])
                    # 先在这里mix看看
                    move_cartesian_in_up_way(robot, mix_place[mix_place_count])
                    mix_movement(robot, mix_place[mix_place_count])
                    up_to_15mm(robot)

                    # mere_move_cartesian(robot, (-0.25, 0.425, 0.15))

                    # print('hello')
                    time.sleep(1)
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
                            # if k == 27 :
                                # print("1")
                            # filename = f'uppermost-{ind}.png'
                            # cv2.imwrite(filename, frame, params=None)
                                # cv2.destroyAllWindows()
                            draw_1=cv2.rectangle(frame, window_every_brush[num_brush][0], window_every_brush[num_brush][1], (0,255,0), 2)

                            cv2.imshow('camera', draw_1)
                            cv2.waitKey(1)
                        else:
                            print("无画面")
                    else:
                        print("无法读取摄像头！")
                    
                    # 识别结果
                    C_d = cal_mean_color_for_window(frame,window_every_brush[num_brush])
                    L_new = np.max(np.abs(C_t - C_d)) # 0 to 1
                    if L_new <= L_th:
                        df_update = pd.read_csv(f'save_ok_color_pos_{num_brush}.csv') 
                        df_update = df_update.append({
                            'x': mix_place[num_brush][mix_place_count][0], 
                        'y': mix_place[num_brush][mix_place_count][1],
                        'z': mix_place[num_brush][mix_place_count][2], 
                        'C': C_d[0], 'M': C_d[1], 'Y': C_d[2], 'K': C_d[3]
                        })
                        df_update.to_csv(f'save_ok_color_pos_{num_brush}.csv')
                        mix_place_count += 1
                        break

                # 真正开始绘画阶段
                mere_move_cartesian((start_point_x, start_point_y, mix_place[num_brush][2]))
                mere_move_cartesian((end_point_x, end_point_y, mix_place[num_brush][2]))


            # 断开服务器链接
            # robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))

    finally:

        cap.release()
        cv2.waitKey(0)
        cv2.destroyAllWindows()
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

if __name__ == '__main__':
    test_show_that_window()

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



