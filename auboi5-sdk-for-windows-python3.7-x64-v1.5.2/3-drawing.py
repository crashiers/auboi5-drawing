from enum import Flag
from drawing_robot import drawing_robot
from my_utils import *

# 重要的全局变量在my_utils中

def draw(num_brush):
    # global mix_last_time
    
    robot, handle = robot_init()
    cap = cv2.VideoCapture(0) # 打开相机

    try:
        result = robot.connect(SERVER_IP, port = 8899) # 链接服务器
        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server failed.")
        else:
            speed_changer(robot, speed_divisor) # 以什么速度画

            # 开始
            # 在这里出现的都是current_imaginary_user_pos
            drawing_robo = drawing_robot(num_brush, emotion) # 就画一笔的robo，谁来画
            
            if init_two_saves:
                drawing_robo.init_save_ok_color_pos() # 需要时调用
                drawing_robo.init_save_state() # 这两个表格自己看着调整

            drawing_robo.recall_state() # 暂定为1行：画到哪儿了，颜色调到哪了
            # ic('这个是将要去mix的place', drawing_robot.mix_place_count)
            
            # 首先读出需要的颜色等等
            data_transfer_df = pd.read_csv('data/'+f'After-grouping-{emotion}_data_transfer_df_brush_{NUM_BRUSH}.csv') # 画什么
            max_s_x = np.max(data_transfer_df['start_point_x']) # 标准化
            min_s_x = np.min(data_transfer_df['start_point_x'])
            
            max_s_y = np.max(data_transfer_df['start_point_y'])
            min_s_y = np.min(data_transfer_df['start_point_y'])
            
            max_e_x = np.max(data_transfer_df['end_point_x'])
            min_e_x = np.min(data_transfer_df['end_point_x'])
            
            max_e_y = np.max(data_transfer_df['end_point_y'])
            min_e_y = np.min(data_transfer_df['end_point_y'])

            tmp = drawing_robo.index #首先确定要画的，从robo记忆中的这幅画要去画的笔画开始
            ic('从robo记忆中的这幅画要去画的笔画：')
            ic(tmp)
            data_transfer_df = data_transfer_df.iloc[int(tmp):] # 从哪里开始画
            # data_transfer_df = data_transfer_df.iloc[int(tmp) - 1:] # 从哪里开始画
            # data_transfer_df = data_transfer_df.loc[str(int(tmp) - 1):] # 从哪里开始画
            # data_transfer_df = data_transfer_df.iloc[int(tmp):]

            for _, row in data_transfer_df.iterrows(): # 一笔一笔画
                # index = int(row[''])
                
                # ic('从文件中读取出的，记录下画到哪儿了：')
                # ic(drawing_robo.index)
                
                # num_stroke = int(row['num_stroke'])
                assert num_brush == int(row['num_brush'])

                s_x = row['start_point_x'] # 记得单位转换，将像素（毫米）转化成米
                s_y = row['start_point_y'] 
                e_x = row['end_point_x'] 
                e_y = row['end_point_y'] 

                s_x = normalize(s_x, min_s_x, max_s_x, 0, CANVAS_X)
                s_y = CANVAS_Y - normalize(s_y, min_s_y, max_s_y, 0, CANVAS_Y)
                e_x = normalize(e_x, min_e_x, max_e_x, 0, CANVAS_X)
                e_y = CANVAS_Y - normalize(e_y, min_e_y, max_e_y, 0, CANVAS_Y)
                
                start_point_x = s_x / 1000
                start_point_y = s_y / 1000
                end_point_x = e_x / 1000
                end_point_y = e_y / 1000
            

                # start_point_x = row['start_point_x'] / 1000
                #     start_point_y = row['start_point_y'] / 1000
                #     end_point_x = row['end_point_x'] / 1000
                #     end_point_y = row['end_point_y'] / 1000
            
                # theta_deg = row['theta_deg'] # 暂时不用
                # theta_rad = row['theta_rad']

                r_t = int(row['r']) # 画什么颜色
                g_t = int(row['g'])
                b_t = int(row['b'])
                drawing_robo.set_target_color(r_t, g_t, b_t)
                
                # 换颜色了
                if not judge_similar(RGB2CMYK(drawing_robo.current_C_in_rgb), RGB2CMYK((r_t, g_t, b_t))) :
                    ic('找找有没有相似的颜色')
                    drawing_robo.look_color_t_up_in_table()
                    
                    drawing_robo.enought_pigments = False
                    
                    if drawing_robo.got_pos:
                        ic('已经调过这种颜色，获取到该mix的位置')
                        # ic('直接用已经调好的颜色，将要移动到mix的位置')
                        ic(drawing_robo.got_pos)
                        
                        # move_cartesian_in_up_way(robot, got_pos)
                        # mix_movement(robot, got_pos)
                        # up_to_15mm(robot)
                
                    else:
                        drawing_robo.got_pos = None # 因为这个变量的存在与否会影响到mix_pigment
                        ic('未找到相似颜色，则去下一个调色的位置进行调色')

                # 混合颜色，这里有一个循环需要一直调色
                # while True:
                flag = False
                mix_count = 0
                while not drawing_robo.enought_pigments:
                # while not got_pos and not drawing_robo.enought_pigments and True:
                
                    if not drawing_robo.got_pos or flag:
                        drawing_robo.calc_color_m()
                        # ic('需要的颜色', drawing_robo.C_m)
                        
                        drawing_robo.calc_L() 
                        
                        # ic('检测到的颜色', drawing_robo.C_d)
                        # ic('目标颜色', drawing_robo.C_t)
                        # ic('最大的差的通道', drawing_robo.L)

                        drawing_robo.calc_dip_depth_accor_to_L()    
                        drawing_robo.calc_pigments_color_target_coord()                             
                        
                        drawing_robo.get_pigment(robot) # 取得这种颜色，每句话的开始执行时和结束执行时都是低于15cm的
                        mix_count += 1
                    
                    # 先在这里mix看看
                    dest = drawing_robo.mix_pigment(robot) # 每句话的开始执行时和结束执行时都是低于15cm的

                    # ic('将要移动到mix的位置', dest)
                    
                    drawing_robo.capture_color(robot, cap)
                    
                    drawing_robo.recog_mean_color_for_window(window_every_brush)

                    drawing_robo.all_colors_and_show()

                    drawing_robo.calc_L()
                    ic('识别结果', drawing_robo.C_t, drawing_robo.C_d, drawing_robo.L)

                    if drawing_robo.got_pos:
                        this_L_th = L_th_got_pos
                    else:
                        this_L_th = L_th
                    
                    if drawing_robo.L <= this_L_th or mix_count == max_mix_count:
                        ic('颜色已经足够相似')
                        if not drawing_robo.got_pos:
                            drawing_robo.save_ok_color_pos()
                        
                        if not drawing_robo.got_pos:
                            drawing_robo.state_mix_place_count_incre()
                            # ic('这个是将要去mix的place', self.mix_place_count)
                        
                        drawing_robo.enought_pigments = True
                        drawing_robo.current_C_in_rgb = (r_t, g_t, b_t)

                        flag = False
                        mix_count = 0

                        break
                    else:
                        # mix_xy = mix_place_xy[drawing_robo.num_brush][CMYKW_str[drawing_robo.C_m]]
                        # mix_count[num_brush][(mix_xy[0], mix_xy[1])] += 1
                        if drawing_robo.got_pos:
                            # pos1 = (drawing_robo.got_pos[0], drawing_robo.got_pos[1])
                            # for pos2 in mix_array_last_time[num_brush]. keys():
                            #     if judge_pos_similar(pos1, pos2):
                            #         break
                            # mix_array_last_time[num_brush][pos2] += 1
                            # mix_array_last_time[num_brush][pos2] %= min_mix_interval
                            # tmp = mix_array_last_time[num_brush][pos2]
                            
                            # mix_array_last_time[num_brush][(drawing_robo.got_pos[0], drawing_robo.got_pos[1])] %= min_mix_interval
                            # tmp = mix_array_last_time[num_brush][(drawing_robo.got_pos[0], drawing_robo.got_pos[1])]
                            
                            # if not tmp: 
                            # # if not mix_last_time: 
                            #     flag = True # 当调色盘有那种颜色，但是调色盘上颜料不够时
                            #     drawing_robo.enought_pigments = False
                            # else:
                            #     drawing_robo.enought_pigments = True
                            
                            # if not mix_last_time: 
                            flag = True # 当调色盘有那种颜色，但是调色盘上颜料不够时
                            drawing_robo.enought_pigments = False

                            

                            # mix_last_time += 1
                            # mix_last_time %= min_mix_interval
                            

                # 真正开始绘画阶段
                # ic('将要移去的起始点')
                # ic((start_point_x, start_point_y, drawing_robo.mix_place[num_brush][0][2]))
                # ic('将要移去的终点')
                # ic((end_point_x, end_point_y, drawing_robo.mix_place[num_brush][0][2]))
                # drawing_robo.draw(robot, start_point_x, start_point_y, end_point_x, end_point_y)
                drawing_robo.draw_with_end_dot_by_mid_point(robot, start_point_x, start_point_y, end_point_x, end_point_y)
                
                # ic('记录一下要去画哪一笔了', drawing_robo.num_physical_stroke)
                drawing_robo.state_index_incre()
                # ic('这个是将要去画的物理笔画', self.num_physical_stroke)

                drawing_robo.check_every_num_physical_stroke_count = (drawing_robo.check_every_num_physical_stroke_count + 1) % check_every_num_physical_stroke
                if drawing_robo.check_every_num_physical_stroke_count == 0:
                    # draw_with_end_dot
                    
                    ic('开始例行检查颜料是否充足')
                    drawing_robo.capture_color(robot, cap)
                    drawing_robo.recog_mean_color_for_window(window_every_brush)
                    drawing_robo.all_colors_and_show()
                    drawing_robo.calc_L()
                    ic('识别结果', drawing_robo.C_d, drawing_robo.L)
                    if drawing_robo.L <= L_th:
                        ic('颜色已经充足')
                        drawing_robo.enought_pigments = True
                    else:
                        drawing_robo.enought_pigments = False
                                
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

if __name__ == '__main__':

    # _thread.start_new_thread( color,  ())
    # whole_mere_move_cartesian()
    # calibration_include_camera_for_each_brush(1)


    draw(NUM_BRUSH)
