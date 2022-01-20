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

# 测试函数
# 测试了轴动、正解、逆解、轨迹运动（圆）
def test(test_count):
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
        # ip = 'localhost'
        # ip = '192.168.11.128'
        ip = SERVER_IP

        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # 重新上电
            #robot.robot_shutdown()
            #
            # # 上电
            robot.robot_startup()
            #
            # # 设置碰撞等级
            robot.set_collision_class(7)

            # 设置工具端电源为１２ｖ
            # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

            # 设置工具端ＩＯ_0为输出
            #robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

            # 获取工具端ＩＯ_0当前状态
            #tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
            #logger.info("tool_io_0={0}".format(tool_io_status))

            # 设置工具端ＩＯ_0状态
            #robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)


            # 获取控制柜用户DO
            #io_config = robot.get_board_io_config(RobotIOType.User_DO)

            # 输出DO配置
            #logger.info(io_config)

            # 当前机械臂是否运行在联机模式
            #logger.info("robot online mode is {0}".format(robot.is_online_mode()))

            # 循环测试
            while test_count > 0:
                test_count -= 1

                joint_status = robot.get_joint_status()
                logger.info("joint_status={0}".format(joint_status))

                # 初始化全局配置文件
                robot.init_profile()

                # 设置关节最大加速度
                robot.set_joint_maxacc(tupleChanger((1.5, 1.5, 1.5, 1.5, 1.5, 1.5), 10))

                # 设置关节最大加速度
                robot.set_joint_maxvelc(tupleChanger((1.5, 1.5, 1.5, 1.5, 1.5, 1.5), 10) )

                # joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                # logger.info("move joint to {0}".format(joint_radian))

                # robot.move_joint(joint_radian)

                current_pos = robot.get_current_waypoint() # 法兰中心相对于基坐标

                ic(current_pos['joint'])
                ic(current_pos['pos']) # 法兰中心相对于基坐标的位置
                ic(current_pos['ori']) # 法兰中心相对于基坐标的姿态

                # 试试轴动到黑色上方
                joint_radian = (1.946600, 0.771331, -0.592744, 0.238943, -1.620678, 1.952583)
                logger.info("move joint to {0}".format(joint_radian))

                robot.move_joint(joint_radian)

                # 获取关节最大加速度
                logger.info(robot.get_joint_maxacc())

        # #         # 正解测试
        # #         fk_ret = robot.forward_kin((-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008))
        # #         """
        # # * NOTES:       六个关节角 {'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        # # *              位置 'pos': [-0.06403157614989634, -0.4185973810159096, 0.816883228463401],
        # # *              姿态 'ori': [-0.11863209307193756, 0.3820514380931854, 0.0, 0.9164950251579285]}
        # # """
        # #         logger.info(fk_ret)

        #         fk_ret = robot.forward_kin(joint_radian)
        #         """
        # * NOTES:       六个关节角 {'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        # *              位置 'pos': [-0.06403157614989634, -0.4185973810159096, 0.816883228463401],
        # *              姿态 'ori': [-0.11863209307193756, 0.3820514380931854, 0.0, 0.9164950251579285]}
        # """
        #         logger.info(fk_ret)
        #         ic([degrees(i) for i in fk_ret['joint']], robot.quaternion_to_rpy(fk_ret['ori']), robot.rpy_to_quaternion((radians(179.9),radians(0),radians(-89.9))))


                # 逆解\
                # 把位姿转化成轴动
                # 原来这个要设置成0，原来弄错了，但是在这试了好像又并没有什么影响
                joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                ik_result = robot.inverse_kin(joint_radian, fk_ret['pos'], fk_ret['ori'])
                logger.info(ik_result)

                # # 轴动1
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                # logger.info("move joint to {0}".format(joint_radian))
                # robot.move_joint(joint_radian)

                # # # 轴动2
                # # joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                # # logger.info("move joint to {0}".format(joint_radian))
                # # robot.move_joint(joint_radian)

                # 首先一定要移动到全局路点
                # 轴动3
                joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

                # 设置机械臂末端最大线加速度(m/s)
                robot.set_end_max_line_acc(0.5)

                # 获取机械臂末端最大线加速度(m/s)
                robot.set_end_max_line_velc(0.2)

                # 清除所有已经设置的全局路点
                robot.remove_all_waypoint()

                # 添加全局路点1,用于轨迹运动
                joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                robot.add_waypoint(joint_radian)

                # 添加全局路点2,用于轨迹运动
                joint_radian = (-0.211675, -0.325189, -1.466753, 0.429232, -1.570794, -0.211680)
                robot.add_waypoint(joint_radian)

                # 添加全局路点3,用于轨迹运动
                joint_radian = (-0.037186, -0.224307, -1.398285, 0.396819, -1.570796, -0.037191)
                robot.add_waypoint(joint_radian)

                # 设置圆运动圈数
                robot.set_circular_loop_times(3)

                # 圆弧运动
                logger.info("move_track ARC_CIR")
                robot.move_track(RobotMoveTrackType.ARC_CIR)

                # 清除所有已经设置的全局路点
                robot.remove_all_waypoint()

        #         # 机械臂轴动 回到0位
        #         joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
        #         logger.info("move joint to {0}".format(joint_radian))
        #         robot.move_joint(joint_radian)

            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))


    finally:
        # 断开服务器链接
        if robot.connected:
            # 关闭机械臂
            robot.robot_shutdown()
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

# 沿Ｚ轴运动0.1毫米，直接修改current pos
def step_test():
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
            # 重新上电
            robot.robot_shutdown()

            # 上电
            robot.robot_startup()

            # 设置碰撞等级
            robot.set_collision_class(7)

            # 初始化全局配置文件
            robot.init_profile()
            
            # logger.info(robot.get_board_io_config(RobotIOType.User_DI))
            
            # 获取当前位置
            logger.info(robot.get_current_waypoint())
            
            joint_radian = (0, 0, 0, 0, 0, 0)
            # 轴动到初始位置
            joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
            robot.move_joint(joint_radian)
            
            # 沿Ｚ轴运动0.1毫米
            current_pos = robot.get_current_waypoint()
            
            # 下降了10cm，因此单位是米
            current_pos['pos'][2] -= 0.1
            # current_pos['pos'][2] -= 0.001
            
            ik_result = robot.inverse_kin(current_pos['joint'], current_pos['pos'], current_pos['ori'])
            logger.info(ik_result)
            
            # joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
            # logger.info("move joint to {0}".format(joint_radian))
            # robot.move_joint(joint_radian)
            
            robot.move_line(ik_result['joint'])

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
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

# 没看懂辨识轨迹是干嘛的
def excit_traj_track_test():
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
        # ip = 'localhost'
        ip = SERVER_IP
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:

            # 重新上电
            # robot.robot_shutdown()

            # 上电
            # robot.robot_startup()

            # 设置碰撞等级
            # robot.set_collision_class(7)

            joint_radian = (0, 0, 0, 0, 0, 0)
            # 轴动到初始位置
            robot.move_joint(joint_radian)

            logger.info("starup excit traj track....")

            # 启动辨识轨迹
            robot.startup_excit_traj_track("dynamics_exciting_trajectories/excitTraj1.offt", 1, 0)

            # joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
            # robot.move_joint(joint_radian)
            
            # 延时两秒等待辨识结果
            time.sleep(5)

            # 获取辨识结果
            dynidentify_ret = robot.get_dynidentify_results()
            logger.info("dynidentify result={0}".format(dynidentify_ret))
            for i in range(0,54):
	            dynidentify_ret[i] = dynidentify_ret[i]/1024.0
            logger.info("dynidentify result={0}".format(dynidentify_ret))

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

# 没看懂辨识轨迹是干嘛的
def get_current_pos_test():
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
        # ip = 'localhost'
        ip = SERVER_IP
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:

            # 重新上电
            # robot.robot_shutdown()

            # 上电
            # robot.robot_startup()

            # 设置碰撞等级
            # robot.set_collision_class(7)

            # 获得当前状态
            current_pos = robot.get_current_waypoint() # 法兰中心相对于基坐标

            ic(current_pos['joint'])
            ic(current_pos['pos']) # 法兰中心相对于基坐标的位置
            ic(current_pos['ori']) # 法兰中心相对于基坐标的姿态

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

# 旋转，旋转接口，直接调用
# 工具端与用户坐标系用默认参数即可
def move_rotate_test():
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

            # 重新上电
            # robot.robot_shutdown()

            # 上电
            # robot.robot_startup()

            # 设置碰撞等级
            # robot.set_collision_class(7)

            # joint_radian = (1, 0, 0, 0, 0, 0)
            # # 轴动到初始位置
            # robot.move_joint(joint_radian)

            joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
            logger.info("move joint to {0}".format(joint_radian))
            robot.move_joint(joint_radian)

            # 开始

            # 获取当前位置, robot的
            """ 返回：
            六个关节角 {'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            *位置 'pos': [-0.06403157614989634, -0.4185973810159096, 0.816883228463401],
            *姿态 'ori': [-0.11863209307193756, 0.3820514380931854, 0.0, 0.9164950251579285]}
            """
            current_pos = robot.get_current_waypoint()

            # 工具的位置和姿态
            # 工具转轴的向量（相对于法兰盘，这样需要测量得到x,y,z本测试样例默认以x=0,y=0,ｚ轴为0.1米）
            tool_pos_on_end = (0, 0, 0.10)

            # 工具姿态（w,x,y,z 相对于法兰盘，不知道的情况下，默认填写如下信息）
            tool_ori_on_end = (1, 0, 0, 0)

            # 描述字典
            tool_desc = {"pos": tool_pos_on_end, "ori": tool_ori_on_end}

            # 得到法兰盘工具末端点相对于基座坐标系中的位置
            tool_pos_on_base = robot.base_to_base_additional_tool(current_pos['pos'],
                                                                  current_pos['ori'],
                                                                  tool_desc)

            logger.info("current_pos={0}".format(current_pos['pos'][0]))

            logger.info("tool_pos_on_base={0}".format(tool_pos_on_base['pos'][0]))

            # 讲工具转轴向量平移到基座坐标系下(旋转方向符合右手准则)
            rotate_axis = map(lambda a, b: a - b, tool_pos_on_base['pos'], current_pos['pos'])

            logger.info("rotate_axis={0}".format(rotate_axis))

            # 坐标系默认使用基座坐标系（默认填写下面的值就可以了）
            user_coord = {'coord_type': RobotCoordType.Robot_Base_Coordinate,
                          'calibrate_method': 0,
                          'calibrate_points':
                              {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                               "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                               "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
                          'tool_desc':
                              {"pos": (0.0, 0.0, 0.0),
                               "ori": (1.0, 0.0, 0.0, 0.0)}
                          }

            # 调用转轴旋转接口，最后一个参数为旋转角度（弧度）
            # robot.move_rotate(user_coord, rotate_axis, 1)
            robot.move_rotate(user_coord, rotate_axis, -1)

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

# 移动到某个点，用笛卡尔坐标
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
            
            joint_maxvelc = tupleChanger(joint_maxvelc, 15)
            joint_maxacc = tupleChanger(joint_maxacc, 15)
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
            O = (0.2587857035467645, -0.6103817043849445, 0.46096235061927077)
            
            T = (-0.25, 0.07, 0.15) # 或许可以用比-0.01小的值
            # T = (0., 0, -0.01) # 想象中的用户坐标系
            
            pos = ( O[0] - T[0], O[1] - T[1], O[2] + T[2]) # 法兰
            # pos_user = (0.1, 0.05, 0.1) # 法兰
            # 先摆正
            # rpy_user = tuple((i for i in robot.quaternion_to_rpy()))
            ori = [7.900953430891173e-07,0.7071120073763648,-0.7071015549415427,-4.774523074271425e-06]

            robot.move_to_target_in_cartesian(pos, list([degrees(i) for i in robot.quaternion_to_rpy(ori)]))
            # userCoord = {'coord_type': RobotCoordType.Robot_World_Coordinate,
            # 'calibrate_method': RobotCoordCalMethod.CoordCalMethod_xOxy,
            # 'calibrate_points': 
            # {
            #     'point1': (1.7801673412322998,
            #                -0.22873090207576752,
            #                -2.24575138092041,
            #                -0.30449768900871277,
            #                -1.5856152772903442,
            #                -0.030411619693040848),
            #     'point2': (1.482286810874939,  
            #                -0.3715052008628845,
            #                -2.3423125743865967,  
            #                -0.2687637507915497,  
            #                -1.6264402866363525,  
            #                0.02880869247019291),
            #     'point3': (1.4943876266479492,
            #                -0.2409527599811554,
            #                -2.2585678100585938,
            #                -0.3149295449256897,
            #                -1.6248520612716675,
            #                0.04082148149609566)
            # },
            # 'tool_desc':{
            #     'pos':(-0.023457, -0.000971, 0.274371),
            #     'ori':tuple(robot.rpy_to_quaternion((3.134034/180*pi, 0.044473/180*pi, 3.071751/180*pi)))
            # }}
            # userTool = {
            #     'pos':(-0.023457, -0.000971, 0.274371),
            #     'ori':tuple(robot.rpy_to_quaternion((3.134034/180*pi, 0.044473/180*pi, 3.071751/180*pi)))
            # }
            # ret_base = robot.user_to_base(pos_user, ori_user, userCoord, userTool)
            # ic(ret_base['pos'], ret_base['ori'])





            # # 逆解
            # # 法兰与工具末端z值相差0.1
            # # 把位姿转化成轴动
            # joint_radian = current_pos['joint']
            
            # dst = {'pos': tuple(ret_base['pos']), 'ori': tuple(ret_base['ori'])} 

            # # ik_result = robot.inverse_kin(joint_radian, dst['pos'], current_pos['ori'])
            # ik_result = robot.inverse_kin(joint_radian, dst['pos'], dst['ori'])
            # ic(ik_result)

            # robot.move_joint(ik_result['joint'])
            # # robot.move_line(ik_result['joint'])


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

# 控制柜I/O
def test_rsm():
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
        #ip = 'localhost'
        ip = '192.168.10.88'
        port = 8899
        result = robot.connect(ip, port)
        
        #robot.enable_robot_event()

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:

            # robot.move_pause()

            #joint_radian = (0, 0, 0, 0, 0, 0)
            # 轴动到初始位置
            #robot.move_joint(joint_radian)

            while True:
                time.sleep(0.05)

                rel = robot.set_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_02, 0)
                print(rel)
                print("++++++++++++++++++++++++")
                #result = robot.get_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_02)
                #print(result)
                # print("*********************************")

                time.sleep(2)
                # rel1 = robot.set_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_02, 0)
                # print(rel1)
                # print("++++++++++++++++++++++++")

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

# 获取当前Waypoint的类
class GetRobotWaypointProcess(Process):
    def __init__(self):
        Process.__init__(self)
        self.isRunWaypoint = False
        self._waypoints = None


    def startMoveList(self, waypoints):
        if self.isRunWaypoint == True:
            return False
        else:
            self._waypoints = waypoints

    def run(self):
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
            #ip = 'localhost'
            # ip = '192.168.65.131'
            ip = SERVER_IP
            port = 8899
            result = robot.connect(ip, port)

            if result != RobotErrorType.RobotError_SUCC:
                logger.info("connect server{0}:{1} failed.".format(ip, port))
            else:

                # 连接成功
                while True:
                    time.sleep(2)
                    
                    # 获取当前Waypoint
                    waypoint = robot.get_current_waypoint()
                    print(waypoint)
                    print("----------------------------------------------")


                    # 断开服务器链接
                robot.disconnect()

        except RobotError as e:
            logger.error("robot Event:{0}".format(e))

        except KeyboardInterrupt:
            # 断开服务器链接
            if robot.connected:
                # 断开机械臂链接
                robot.disconnect()
            # 释放库资源
            Auboi5Robot.uninitialize()
            print("get  waypoint run end-------------------------")

# 打印queue中数据
def runWaypoint(queue):
    while True:
        # while not queue.empty():
        # ic(queue)
        # ic(queue.get(True))
        print(queue.get(True))


# 在三点之间运动的例子
def test_process_demo():
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

        # time.sleep(0.2)
        # process_get_robot_current_status = GetRobotWaypointProcess()
        # process_get_robot_current_status.daemon = True
        # process_get_robot_current_status.start()
        # time.sleep(0.2)

        # 进程间数据通信
        queue = Queue()

        # 创建进程
        p = Process(target=runWaypoint, args=(queue,))
        p.start()
        time.sleep(5)
        print("process started.")

        # 链接服务器
        #ip = 'localhost'
        # ip = '192.168.65.131'
        ip = SERVER_IP
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # 连接成功

            robot.enable_robot_event()
            robot.init_profile()
            joint_maxvelc = (2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177)
            joint_maxacc = (17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5)
            
            joint_maxvelc = tupleChanger(joint_maxvelc, 10)
            joint_maxacc = tupleChanger(joint_maxacc, 10)
            # print(f"{type(joint_maxvelc)}, {joint_maxvelc}")
            # print(f"{type(joint_maxacc)}, {joint_maxacc}")
            
            robot.set_joint_maxacc(joint_maxacc)
            robot.set_joint_maxvelc(joint_maxvelc)
            robot.set_arrival_ahead_blend(0.05)
            
            # 初始化设置完毕

            while True:
                time.sleep(1)

                # 移动到六个关节角的位置
                joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                robot.move_joint(joint_radian, True)

                time.sleep(4)
                
                # 移动到六个关节角的位置
                joint_radian = (55.5/180.0*pi, -20.5/180.0*pi, -72.5/180.0*pi, 38.5/180.0*pi, -90.5/180.0*pi, 55.5/180.0*pi)
                robot.move_joint(joint_radian, True)

                time.sleep(4)

                # 移动到六个关节角的位置
                joint_radian = (0, 0, 0, 0, 0, 0)
                robot.move_joint(joint_radian, True)

                time.sleep(4)

                print("-----------------------------")

                # 将(0, 0, 0, 0, 0, 0)放入队列中
                queue.put(joint_radian)

                # time.sleep(5)

                # process_get_robot_current_status.test()

                # print("-----------------------------")

                # 断开服务器链接
            robot.disconnect()

    # 异常处理
    except KeyboardInterrupt:
        robot.move_stop()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))



    finally:
        # 断开服务器链接
        if robot.connected:
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        print("run end-------------------------")

# 
def tupleChanger(tup, divisor):
    return tuple((i / divisor for i in tup))

# # 移动到某个点
# def move_cartesian_test():
#     # 初始化logger
#     logger_init()

#     # 启动测试
#     logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

#     # 系统初始化
#     Auboi5Robot.initialize()

#     # 创建机械臂控制类
#     robot = Auboi5Robot()

#     # 创建上下文
#     handle = robot.create_context()

#     # 打印上下文
#     logger.info("robot.rshd={0}".format(handle))

#     try:

#         # 链接服务器
#         ip = SERVER_IP
#         # ip = 'localhost'
#         port = 8899
#         result = robot.connect(ip, port)

#         if result != RobotErrorType.RobotError_SUCC:
#             logger.info("connect server{0}:{1} failed.".format(ip, port))
#         else:

#             joint_maxvelc = (2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177)
#             joint_maxacc = (17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5)
            
#             joint_maxvelc = tupleChanger(joint_maxvelc, 10)
#             joint_maxacc = tupleChanger(joint_maxacc, 10)
#             # print(f"{type(joint_maxvelc)}, {joint_maxvelc}")
#             # print(f"{type(joint_maxacc)}, {joint_maxacc}")
            
#             robot.set_joint_maxacc(joint_maxacc)
#             robot.set_joint_maxvelc(joint_maxvelc)
#             robot.set_arrival_ahead_blend(0.05)

#             # 重新上电
#             # robot.robot_shutdown()

#             # 上电
#             # robot.robot_startup()

#             # 设置碰撞等级
#             robot.set_collision_class(7)

#             # joint_radian = (1, 0, 0, 0, 0, 0)
#             # # 轴动到初始位置
#             # robot.move_joint(joint_radian)

#             # joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
#             # logger.info("move joint to {0}".format(joint_radian))
#             # robot.move_joint(joint_radian)

#             # 开始

#             # 获取当前位置, robot的
#             """ 返回：
#             六个关节角 {'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
#             *位置 'pos': [-0.06403157614989634, -0.4185973810159096, 0.816883228463401],
#             *姿态 'ori': [-0.11863209307193756, 0.3820514380931854, 0.0, 0.9164950251579285]}
#             """

#             # 获得当前状态
#             current_pos = robot.get_current_waypoint() # 法兰中心相对于基坐标

#             ic(current_pos['joint'])
#             ic(current_pos['pos']) # 法兰中心相对于基坐标的位置
#             ic(current_pos['ori']) # 法兰中心相对于基坐标的姿态

#             # ic(robot.rpy_to_quaternion((92.04/180*pi, -46.20/180*pi, -1.81/180*pi)))

#             #弧度
#             # ic(robot.quaternion_to_rpy(current_pos['ori'])) 
#             # import math
#             # ic(math.degrees(1.5688836445000016), math.degrees(-0.8054248684787352), math.degrees( -0.004295073905417389))
            
#             # current_tool = robot.get_tool_kinematics_param()
#             # ic(current_tool['pos'])
#             # ic(current_tool['ori'])

#             # 用户坐标转换成基坐标
#             # 在这设置目标坐标
#             pos_user = ( -0.26 , 0 + 0.42, 0.2) # 法兰
#             # pos_user = (0., 0.0, 0.2) # 法兰
#             # 先摆正
#             # rpy_user = tuple((i for i in robot.quaternion_to_rpy()))
#             ori_user = current_pos['ori']

#             userCoord = {'coord_type': RobotCoordType.Robot_World_Coordinate,
#             'calibrate_method': RobotCoordCalMethod.CoordCalMethod_xOxy,
#             'calibrate_points': 
#             {
#                 'point1': (1.7801673412322998,
#                            -0.22873090207576752,
#                            -2.24575138092041,
#                            -0.30449768900871277,
#                            -1.5856152772903442,
#                            -0.030411619693040848),
#                 'point2': (1.482286810874939,  
#                            -0.3715052008628845,
#                            -2.3423125743865967,  
#                            -0.2687637507915497,  
#                            -1.6264402866363525,  
#                            0.02880869247019291),
#                 'point3': (1.4943876266479492,
#                            -0.2409527599811554,
#                            -2.2585678100585938,
#                            -0.3149295449256897,
#                            -1.6248520612716675,
#                            0.04082148149609566)
#             },
#             'tool_desc':{
#                 'pos':(-0.023457, -0.000971, 0.274371),
#                 'ori':tuple(robot.rpy_to_quaternion((3.134034/180*pi, 0.044473/180*pi, 3.071751/180*pi)))
#             }}
#             userTool = {
#                 'pos':(-0.023457, -0.000971, 0.274371),
#                 'ori':tuple(robot.rpy_to_quaternion((3.134034/180*pi, 0.044473/180*pi, 3.071751/180*pi)))
#             }
#             ret_base = robot.user_to_base(pos_user, ori_user, userCoord, userTool)
#             ic(ret_base['pos'], ret_base['ori'])





#             # 逆解
#             # 法兰与工具末端z值相差0.1
#             # 把位姿转化成轴动
#             joint_radian = current_pos['joint']
            
#             dst = {'pos': tuple(ret_base['pos']), 'ori': tuple(ret_base['ori'])} 

#             # ik_result = robot.inverse_kin(joint_radian, dst['pos'], current_pos['ori'])
#             ik_result = robot.inverse_kin(joint_radian, dst['pos'], dst['ori'])
#             ic(ik_result)

#             robot.move_joint(ik_result['joint'])

#             # robot.move_rotate(userCoord,)
#             # robot.move_line(ik_result['joint'])
#             # robot.move_to_target_in_cartesian(pos_user, robot.quaternion_to_rpy(dst['ori']))
#             # robot.move_to_target_in_cartesian(ret_base['pos'], robot.quaternion_to_rpy(dst['ori']))
#             # robot.move_to_target_in_cartesian(ret_base['pos'], robot.quaternion_to_rpy(current_pos['ori']))


#             # 工具的位置和姿态
#             # 工具转轴的向量（相对于法兰盘，这样需要测量得到x,y,z本测试样例默认以x=0,y=0,ｚ轴为0.1米）
#             # tool_pos_on_end = (0, 0, 0.10)

#             # # 工具姿态（w,x,y,z 相对于法兰盘，不知道的情况下，默认填写如下信息）
#             # tool_ori_on_end = (1, 0, 0, 0)

#             # # 描述字典
#             # tool_desc = {"pos": tool_pos_on_end, "ori": tool_ori_on_end}

#             # # 得到法兰盘工具末端点相对于基座坐标系中的位置
#             # tool_pos_on_base = robot.base_to_base_additional_tool(current_pos['pos'],
#             #                                                       current_pos['ori'],
#             #                                                       tool_desc)

#             # logger.info("current_pos={0}".format(current_pos['pos'][0]))

#             # logger.info("tool_pos_on_base={0}".format(tool_pos_on_base['pos'][0]))

#             # # 讲工具转轴向量平移到基座坐标系下(旋转方向符合右手准则)
#             # rotate_axis = map(lambda a, b: a - b, tool_pos_on_base['pos'], current_pos['pos'])

#             # logger.info("rotate_axis={0}".format(rotate_axis))

#             # # 坐标系默认使用基座坐标系（默认填写下面的值就可以了）
#             # user_coord = {'coord_type': RobotCoordType.Robot_Base_Coordinate,
#             #               'calibrate_method': 0,
#             #               'calibrate_points':
#             #                   {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
#             #                    "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
#             #                    "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
#             #               'tool_desc':
#             #                   {"pos": (0.0, 0.0, 0.0),
#             #                    "ori": (1.0, 0.0, 0.0, 0.0)}
#             #               }

#             # 调用转轴旋转接口，最后一个参数为旋转角度（弧度）
#             # robot.move_rotate(user_coord, rotate_axis, 1)
#             # robot.move_rotate(user_coord, rotate_axis, -1)

#             # 断开服务器链接
#             robot.disconnect()

#     except RobotError as e:
#         logger.error("robot Event:{0}".format(e))

#     finally:
#         # 断开服务器链接
#         if robot.connected:
#             # 断开机械臂链接
#             robot.disconnect()
#         # 释放库资源
#         Auboi5Robot.uninitialize()

if __name__ == '__main__':
    # test_process_demo()
    # move_rotate_test()
    # test(1)
    # step_test()
    # excit_traj_track_test()
    # move_rotate_test()
    # get_current_pos_test()
    move_cartesian_test()
    logger.info("test completed")

    
