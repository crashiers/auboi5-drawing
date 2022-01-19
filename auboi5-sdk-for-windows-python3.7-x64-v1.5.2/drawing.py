#! /usr/bin/env python
# coding=utf-8

# De'en 版本

import time
from multiprocessing import Process, Queue
from math import pi, degrees, radians
from utils import *

# 机械臂 ip，机械臂作为服务器
SERVER_IP = '192.168.11.127'

# SERVER_IP = '192.168.11.129'
# SERVER_IP = '192.168.11.128'

# 移动到某个点
def move_cartesian_test():
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
            
            joint_maxvelc = tupleChanger(joint_maxvelc, 10)
            joint_maxacc = tupleChanger(joint_maxacc, 10)
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
            # robot.set_collision_class(7)

            # joint_radian = (1, 0, 0, 0, 0, 0)
            # # 轴动到初始位置
            # robot.move_joint(joint_radian)

            # joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
            # logger.info("move joint to {0}".format(joint_radian))
            # robot.move_joint(joint_radian)

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

            # ic(robot.rpy_to_quaternion((92.04/180*pi, -46.20/180*pi, -1.81/180*pi)))

            #弧度
            # ic(robot.quaternion_to_rpy(current_pos['ori'])) 
            # import math
            # ic(math.degrees(1.5688836445000016), math.degrees(-0.8054248684787352), math.degrees( -0.004295073905417389))
            
            # current_tool = robot.get_tool_kinematics_param()
            # ic(current_tool['pos'])
            # ic(current_tool['ori'])

            # 用户坐标转换成基坐标
            # 在这设置目标坐标
            pos_user = ( -0. , 0 - 0., 0.2) # 法兰
            # pos_user = (0.1, 0.05, 0.1) # 法兰
            # 先摆正
            # rpy_user = tuple((i for i in robot.quaternion_to_rpy()))
            ori_user = current_pos['ori']

            userCoord = {'coord_type': RobotCoordType.Robot_World_Coordinate,
            'calibrate_method': RobotCoordCalMethod.CoordCalMethod_xOxy,
            'calibrate_points': 
            {
                'point1': (1.7801673412322998,
                           -0.22873090207576752,
                           -2.24575138092041,
                           -0.30449768900871277,
                           -1.5856152772903442,
                           -0.030411619693040848),
                'point2': (1.482286810874939,  
                           -0.3715052008628845,
                           -2.3423125743865967,  
                           -0.2687637507915497,  
                           -1.6264402866363525,  
                           0.02880869247019291),
                'point3': (1.4943876266479492,
                           -0.2409527599811554,
                           -2.2585678100585938,
                           -0.3149295449256897,
                           -1.6248520612716675,
                           0.04082148149609566)
            },
            'tool_desc':{
                'pos':(-0.023457, -0.000971, 0.274371),
                'ori':tuple(robot.rpy_to_quaternion((3.134034/180*pi, 0.044473/180*pi, 3.071751/180*pi)))
            }}
            userTool = {
                'pos':(-0.023457, -0.000971, 0.274371),
                'ori':tuple(robot.rpy_to_quaternion((3.134034/180*pi, 0.044473/180*pi, 3.071751/180*pi)))
            }
            ret_base = robot.user_to_base(pos_user, ori_user, userCoord, userTool)
            ic(ret_base['pos'], ret_base['ori'])





            # 逆解
            # 法兰与工具末端z值相差0.1
            # 把位姿转化成轴动
            joint_radian = current_pos['joint']
            
            dst = {'pos': tuple(ret_base['pos']), 'ori': tuple(ret_base['ori'])} 

            # ik_result = robot.inverse_kin(joint_radian, dst['pos'], current_pos['ori'])
            ik_result = robot.inverse_kin(joint_radian, dst['pos'], dst['ori'])
            ic(ik_result)

            robot.move_joint(ik_result['joint'])
            # robot.move_line(ik_result['joint'])


            # 工具的位置和姿态
            # 工具转轴的向量（相对于法兰盘，这样需要测量得到x,y,z本测试样例默认以x=0,y=0,ｚ轴为0.1米）
            # tool_pos_on_end = (0, 0, 0.10)

            # # 工具姿态（w,x,y,z 相对于法兰盘，不知道的情况下，默认填写如下信息）
            # tool_ori_on_end = (1, 0, 0, 0)

            # # 描述字典
            # tool_desc = {"pos": tool_pos_on_end, "ori": tool_ori_on_end}

            # # 得到法兰盘工具末端点相对于基座坐标系中的位置
            # tool_pos_on_base = robot.base_to_base_additional_tool(current_pos['pos'],
            #                                                       current_pos['ori'],
            #                                                       tool_desc)

            # logger.info("current_pos={0}".format(current_pos['pos'][0]))

            # logger.info("tool_pos_on_base={0}".format(tool_pos_on_base['pos'][0]))

            # # 讲工具转轴向量平移到基座坐标系下(旋转方向符合右手准则)
            # rotate_axis = map(lambda a, b: a - b, tool_pos_on_base['pos'], current_pos['pos'])

            # logger.info("rotate_axis={0}".format(rotate_axis))

            # # 坐标系默认使用基座坐标系（默认填写下面的值就可以了）
            # user_coord = {'coord_type': RobotCoordType.Robot_Base_Coordinate,
            #               'calibrate_method': 0,
            #               'calibrate_points':
            #                   {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            #                    "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            #                    "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
            #               'tool_desc':
            #                   {"pos": (0.0, 0.0, 0.0),
            #                    "ori": (1.0, 0.0, 0.0, 0.0)}
            #               }

            # 调用转轴旋转接口，最后一个参数为旋转角度（弧度）
            # robot.move_rotate(user_coord, rotate_axis, 1)
            # robot.move_rotate(user_coord, rotate_axis, -1)

            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))

    finally:
        # 断开服务器链接
        if robot.connected:
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()


move_cartesian_test()