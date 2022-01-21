#! /usr/bin/env python
# coding=utf-8

# De'en 版本
import _thread
from tabnanny import check
import cv2
import numpy as np
import time
from multiprocessing import Process, Queue
import pandas as pd
from my_utils import *
# from my_utils import cal_mean_color_for_window, show_that_window, RGB2CMYK, robot_init, read_one_frame

# 全局变量
# 机械臂 ip，机械臂作为服务器
SERVER_IP = '192.168.11.127'

# SERVER_IP = '192.168.11.129'
# SERVER_IP = '192.168.11.128'

window_every_brush = {1:
np.array([(333 - 2,337 - 2 - 16), (350 - 2,353 - 10 - 8)])}
L_th = 0.38 # 0 to 1
# L_th = 0.15 # 0 to 1
original_dip_depth = - 0.002


def draw(num_brush):
    robot, handle = robot_init()

    # 显示都用plt试试
    cap = cv2.VideoCapture(0)
    # cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)

    # 先不读试试，遇到问题了再想如何解决
    # read_one_frame(cap) 

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
            
            # 调节速度
            joint_maxvelc_1 = tupleChanger(joint_maxvelc, 20)
            joint_maxacc_1 = tupleChanger(joint_maxacc, 20)
            
            robot.set_joint_maxacc(joint_maxacc_1)
            robot.set_joint_maxvelc(joint_maxvelc_1)
            robot.set_arrival_ahead_blend(0.05)

            # 设置碰撞等级
            robot.set_collision_class(7)

            # 开始

            # 在这里出现的都是current_imaginary_user_pos
            # 及时更新
            # current_imaginary_user_pos = (-0.19, 0.425, 0.15)
               
               
            draw_depth = original_draw_depth
            # draw_depth = 0.002
            
            mix_place = {1:list(
                [(-0.17 + 0.05 * j, 0.50 - i * 0.05, -draw_depth) 
                for j in range(4) for i in range(9) ]
                ),
                2:list(
                [(-0.17 + 0.05 * j, 0.50 - i * 0.05, -(0.315-0.313) -draw_depth) 
                for j in range(4) for i in range(9) ]
                )
                }
            # 需要写文件的是这个和画到第几笔了
            state_df = pd.read_csv('save_state.csv') 
            
            # 这个是将要去mix的place
            mix_place_count = int(state_df[state_df['num_brush']==num_brush].iloc[0]['mix_place_count'])
            ic(mix_place_count)

            # 首先读出需要的颜色等等
            data_transfer_df = pd.read_csv('happy_data_transfer_df_brush_1.csv')
            
            for index, row in data_transfer_df.iterrows():
                num_physical_stroke = int(row['num_physical_stroke'])
                ic(num_physical_stroke)
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
                ic('得到目标颜色', C_t)
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
                else:
                    ic('未找到相似颜色')

                # 混合颜色
                # 这里有一个循环需要一直调色
                while not got_pos and True:
                
                    C_m = np.argmax(np.abs(C_t - C_d))
                    if C_d[C_m] > C_t[C_m]:
                        C_m = 4 # 白色
                    
                    #计算需要的颜色C_m

                    # 取得这种颜色
                    L = np.max(np.abs(C_t - C_d)) # 0 to 1
                    ic('最大的差的通道', L)
                    ic('检测到的颜色', C_d)

                    dip_depth = original_dip_depth * (L)
                    # dip_depth = 0.005 * (L)
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
                    move_cartesian_in_up_way(robot, mix_place[num_brush][mix_place_count])
                    mix_movement(robot, mix_place[num_brush][mix_place_count])
                    up_to_15mm(robot)

                    # mere_move_cartesian(robot, (-0.25, 0.425, 0.15))

                    # print('hello')
                    # time.sleep(2)
                    ret, frame = cap.read()
                    if ret:
                        if frame is not None:
                            k = cv2.waitKey(1)
                            # if k == 27 :
                                # print("1")
                            # filename = f'uppermost-{ind}.png'
                            # cv2.imwrite(filename, frame, params=None)
                                # cv2.destroyAllWindows()
                            cv2.imshow('show', frame)

                            show_that_window(frame, window_every_brush[num_brush])

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
                        }, ignore_index=True)
                        df_update.to_csv(f'save_ok_color_pos_{num_brush}.csv')

                        mix_place_count += 1
                        state_df = pd.read_csv('save_state.csv')
                        state_df[state_df['num_brush']==num_brush].iloc[0]['mix_place_count'] = mix_place_count
                        state_df.to_csv('save_state.csv')

                        break

                # 真正开始绘画阶段
                mere_move_cartesian(robot, (start_point_x, start_point_y, mix_place[num_brush][0][2]))
                mere_move_cartesian(robot, (end_point_x, end_point_y, mix_place[num_brush][0][2]))


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
        pos = ( current[0] + 0.015, current[1], current[2]) # 法兰
    
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

def tupleChanger(tup, divisor):
    return tuple((i / divisor for i in tup))

def init_save_ok_color_pos(num_brush):
    # 都是虚拟的用户坐标系坐标
    df = pd.DataFrame(columns=['x', 'y', 'C', 'M', 'Y', 'K'])
    df.to_csv(f'save_ok_color_pos_{num_brush}.csv')

def init_save_state():
    # 都是虚拟的用户坐标系坐标
    df = pd.DataFrame(columns=['num_brush', 'mix_place_count', 'num_physical_stroke'])
    df.to_csv(f'save_state.csv')




if __name__ == '__main__':

    # _thread.start_new_thread( color,  ())
    # whole_mere_move_cartesian()

    # init_save_ok_color_pos(1)
    # init_save_state()

    draw(1)
