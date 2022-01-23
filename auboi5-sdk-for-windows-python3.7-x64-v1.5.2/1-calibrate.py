from my_utils import *
from drawing_robot import drawing_robot

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

            # ret, frame = cap.read()
            # if ret:
            #     if frame is not None:
            #         cv2.imshow('camera', frame)
            #         cv2.waitKey(1)
            #     else:
            #         print("无画面")
            # else:
            #     print("无法读取摄像头！")
            
            # get_current_robo_state(robot) # 1
            # mere_move_cartesian(robot, (0.0, 0, 0.0))

            # up_to_height_in_the_air(robot, height_in_the_air) # 2 

            # 在这里出现的都是current_imaginary_user_pos
            # 及时更新
            # current_imaginary_user_pos = (-0.19, 0.425, 0.15)
            
            # mere_move_cartesian(robot, (calibrate_T_xy['C'][0], calibrate_T_xy['C'][1], 0.15)) # 3
            # mere_move_cartesian(robot, (calibrate_T_xy['M'][0], calibrate_T_xy['M'][1], 0.15))
            # mere_move_cartesian(robot, (calibrate_T_xy['Y'][0], calibrate_T_xy['Y'][1], 0.15))
            # mere_move_cartesian(robot, (calibrate_T_xy['K'][0], calibrate_T_xy['K'][1], height_in_the_air))
            # mere_move_cartesian(robot, (calibrate_T_xy['W'][0], calibrate_T_xy['W'][1], 0.15))
            
            drawing_robo = drawing_robot(num_brush, 'happy') # 4
            
            # drawing_robo.recall_state()
            # drawing_robo.calc_pigments_color_target_coord()
            
            # drawing_robo.get_what_pigment(robot, 'K') # 取得这种颜色，每句话的开始执行时和结束执行时都是低于15cm的
            # dest = drawing_robo.mix_pigment(robot) # 每句话的开始执行时和结束执行时都是低于15cm的

            # cap = cv2.VideoCapture(0) # 5 
            # cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)
            # drawing_robo.capture_color(robot, cap) 
                    
            # drawing_robo.recog_mean_color_for_window(window_every_brush)

            # drawing_robo.all_colors_and_show()

            # 就用这个函数最好！
            # drawing_robo.draw_with_end_dot_by_mid_point( robot, 0.25, 0.25, 0.25, 0.) # 7
            # drawing_robo.draw_with_end_dot( robot, 0, 0.25, 0.25, 0.25) # 6
            # drawing_robo.draw_with_end_dot( robot, 0.25, 0.25, 0.25, 0.) # 7

            # drawing_robo.draw_with_end_dot( robot, 0, 0.25, 0.25, 0.25) # 6
            
            # 废弃
            # drawing_robo.draw_with_end_dot_in_track( robot, 0.25, 0.25, 0.25, 0.) # 7
            # test_track_move(robot)
            
            
            
            # mere_move_cartesian(robot, 
            #     ( 0.25, 0, height_in_the_air))
            # drawing_robo.capture_color(robot, cap)  # 它可能是你在运行的时候机械臂还没运行到终点就运行了下面那句话
            # # 多补点颜料也不是坏事，只要调色调对就可以了
            # drawing_robo.recog_mean_color_for_window(window_every_brush)

            # drawing_robo.all_colors_and_show()



            # cap.release()
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            return

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

        
        # 断开服务器链接
        if robot.connected:
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()

if __name__ == '__main__':
    calibration_include_camera_for_each_brush(1)
