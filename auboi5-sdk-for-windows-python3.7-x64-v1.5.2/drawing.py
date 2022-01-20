#! /usr/bin/env python
# coding=utf-8

# De'en 版本
import _thread
import cv2
import numpy as np
import time
from multiprocessing import Process, Queue
from math import pi, degrees, radians
from utils import *

# 机械臂 ip，机械臂作为服务器
SERVER_IP = '192.168.11.127'

# SERVER_IP = '192.168.11.129'
# SERVER_IP = '192.168.11.128'

def draw():
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
            current_imaginary_user_pos = (-0.19, 0.425, 0.15)

            dip_depth = 0.005
            # 第1个笔刷, Target
            T={1:
            {'C':(-0.25, 0.425, 0.061 - dip_depth), 'M':(-0.25, 0.34, 0.056 - dip_depth),'Y':(-0.25, 0.26, 0.070 - dip_depth),
            'K':(-0.25, 0.17, 0.041 - dip_depth),'W':(-0.25, 0.07, 0.060 - dip_depth)}
            }
            # # W 
            # # T = (-0.25, 0.07, 0.15) # 或许可以用比-0.01小的值
            # # C
            # T = (-0.25, 0.425, 0.35) # 或许可以用比-0.01小的值
            # 这里要优化成字典
            draw_depth = 0.003
            mix_place_upper_most = (-0.17, 0.50, -draw_depth)
            mix_place = list([(-0.17, 0.50 - i * 0.05, -draw_depth) for i in range(9)])
            # mix_place = [(-0.19, 0.58, -draw_depth)]

            # T = (0., 0, -0.01) # 想象中的用户坐标系
            for ind, i in enumerate(mix_place):
                if ind in list(range(4)):
                    continue
                # move_cartesian_in_up_way(robot, T[1]['C'])
                move_cartesian_in_up_way(robot, i)
                mix_movement(robot, i)
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
                        filename = f'uppermost-{ind}.png'
                        cv2.imwrite(filename, frame, params=None)
                            # cv2.destroyAllWindows()

                        cv2.imshow('camera', frame)
                        cv2.waitKey(1)
                    else:
                        print("无画面")
                else:
                    print("无法读取摄像头！")

            


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


def up_to_15mm(robot): 
    # 做个判断，如果已经那么高了就不用再加了
    # 沿Ｚ轴运动0.1毫米
    current_pos = robot.get_current_waypoint()

    # if current_pos['pos'][2] <= 0.46096235061927077 + 0.14:
    
    # 下降了10cm，因此单位是米
    O = (0.2587857035467645, -0.6103817043849445, 0.46096235061927077)
    
    current_pos['pos'][2] = 0.15 + O[2]
    # current_pos['pos'][2] += 0.15
    # current_pos['pos'][2] -= 0.001
    
    ik_result = robot.inverse_kin(current_pos['joint'], current_pos['pos'], current_pos['ori'])
    # logger.info(ik_result)
    
    robot.move_line(ik_result['joint'])

def mix_movement(robot, current):
    for i in range(3):
        pos = ( current[0] + 0.01, current[1], current[2]) # 法兰
    
        # pos_user = (0.1, 0.05, 0.1) # 法兰
        
        mere_move_cartesian(robot, pos)

        mere_move_cartesian(robot, current)

# 移动到某个点，用笛卡尔坐标
def move_cartesian_in_up_way(robot, to_):
            # 在这里出现的都是current_imaginary_user_pos
    # 注意安全，在这个函数里首先，抬起，到0.15m的高度，在这个高度下水平移动，再，落下，到目标点位
    # 因而需要from的点
    # 抬起，移动，落下

    # 抬起

    # 用户坐标转换成基坐标
    # 在这设置目标坐标
    # pos = ( from_[0], from_[1], 0.15) # 法兰
    
    # # pos_user = (0.1, 0.05, 0.1) # 法兰
    
    # mere_move_cartesian(robot, pos)
    up_to_15mm(robot)

    pos_to = ( to_[0], to_[1], 0.15) # 法兰
    
    # 移动，
    mere_move_cartesian(robot, pos_to)
    
    # 落下
    mere_move_cartesian(robot, to_)

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

def tupleChanger(tup, divisor):
    return tuple((i / divisor for i in tup))

# _thread.start_new_thread( color,  ())
draw()
