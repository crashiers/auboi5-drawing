from my_utils import *

class drawing_robot():
    def __init__(self, num_brush, drawing_name) -> None:
        
        # mix_place和真正画的时候的第三个坐标都应该是-draw_depth 
        self.draw_depth = original_draw_depth # robot要多用力画
        # robot要调色的地方的坐标
        self.mix_place = {1:list(
            [(mix_place_xy[1][i][0], mix_place_xy[1][i][1], -self.draw_depth) for i in range(len(mix_place_xy[1])) ]
            ),
            2:list(
            [(mix_place_xy[2][i][0], mix_place_xy[2][i][1], -(0.315-0.313) -self.draw_depth) for i in range(len(mix_place_xy[2])) ]
            )
            }

        self.num_brush = num_brush # robot当前拿的画笔
        self.drawing_name = drawing_name # 一次运行，还能画两幅画不成

        self.C_d = np.array([0, 0, 0, 0]) # 初始化
        self.enought_pigments = False
        self.check_every_num_physical_stroke_count = 0

        self.dip_depth = original_dip_depth # 初始值
        self.got_pos = False
        self.C_t_in_rgb = (255, 255, 255)
        self.current_C_in_rgb = (255, 255, 255)
        
    
    def recall_state(self):
        # 需要写文件的是，mix_place_count和画某幅画 要去画的笔画
        self.state_df = pd.read_csv('save_state.csv') 

        # 这个是将要去mix的place
        df = self.state_df
        self.mix_place_count = int(df.iloc[0]['mix_place_count'])
        self.num_physical_stroke = int(df.iloc[0]['num_physical_stroke'])

        # df = self.state_df[self.state_df['drawing_name']==self.drawing_name]
        # self.mix_place_count = int(df.iloc[0]['mix_place_count'])
        # ic('这个是将要去mix的place', self.mix_place_count)
        # self.num_physical_stroke = int(df.iloc[0]['num_physical_stroke'])
    
    def set_target_color(self, r_t, g_t, b_t):
        # 得到目标颜色
        rgb_T = (r_t, g_t, b_t)
        self.C_t_in_rgb = rgb_T
        # rgb_T = (199, 172, 221)
        CMYK_T = RGB2CMYK(rgb_T)
        self.C_t = CMYK_T
        ic('得到目标颜色in CMYK', self.C_t)

    def calc_color_m(self):
        #计算需要的颜色C_m，实际为CMYK中的通道之一，对应的颜色
        self.C_m = np.argmax(np.abs(self.C_t - self.C_d))
        if self.C_d[self.C_m] > self.C_t[self.C_m]:
            self.C_m = 4 # 白色
    
    def calc_L(self):
        self.L = np.max(np.abs(self.C_t - self.C_d)) # 0 to 1

    def calc_dip_depth_accor_to_L(self):
        self.dip_depth = original_dip_depth * self.L   

    def calc_pigments_color_target_coord(self):
        # 蘸颜料的地方, Target，想象中的用户坐标系
        self.T = {1:
        {'C':(calibrate_T_xy['C'][0], calibrate_T_xy['C'][1], pigments_height['C'] - self.dip_depth + pigments_height_adj), 
        'M':(calibrate_T_xy['M'][0], calibrate_T_xy['M'][1], pigments_height['M'] - self.dip_depth + pigments_height_adj),
        'Y':(calibrate_T_xy['Y'][0], calibrate_T_xy['Y'][1], pigments_height['Y'] - self.dip_depth + pigments_height_adj),
        'K':(calibrate_T_xy['K'][0], calibrate_T_xy['K'][1], pigments_height['K'] - self.dip_depth + pigments_height_adj),
        'W':(calibrate_T_xy['W'][0], calibrate_T_xy['W'][1], pigments_height['W'] - self.dip_depth + pigments_height_adj)},
        2:
        {'C':(calibrate_T_xy['C'][0], calibrate_T_xy['C'][1], pigments_height['C'] -(0.315-0.313) - self.dip_depth + pigments_height_adj), 
        'M':(calibrate_T_xy['M'][0], calibrate_T_xy['M'][1], pigments_height['M'] -(0.315-0.313) - self.dip_depth + pigments_height_adj),
        'Y':(calibrate_T_xy['Y'][0], calibrate_T_xy['Y'][1], pigments_height['Y'] -(0.315-0.313) - self.dip_depth + pigments_height_adj),
        'K':(calibrate_T_xy['K'][0], calibrate_T_xy['K'][1], pigments_height['K'] -(0.315-0.313) - self.dip_depth + pigments_height_adj),
        'W':(calibrate_T_xy['W'][0], calibrate_T_xy['W'][1], pigments_height['W'] -(0.315-0.313) - self.dip_depth + pigments_height_adj)}
        }

    def state_mix_place_count_incre(self):
        state_df = pd.read_csv('save_state.csv') 
        assert int(state_df.iloc[0]['mix_place_count']) == self.mix_place_count
        
        self.mix_place_count += 1

        # 这个是将要去mix的place
        state_df.iloc[0, 0] = self.mix_place_count
        # state_df.iloc[0]['mix_place_count'] = self.mix_place_count
        # state_df[state_df['drawing_name']==self.drawing_name].iloc[0]['mix_place_count'] = self.mix_place_count
        state_df.to_csv('save_state.csv', index=False)
    
    def state_num_physical_stroke_incre(self):
        state_df = pd.read_csv('save_state.csv') 
        assert int(state_df.iloc[0]['num_physical_stroke']) == self.num_physical_stroke
        
        self.num_physical_stroke += 1

        state_df.iloc[0, 2] = self.num_physical_stroke
        # state_df.iloc[0]['num_physical_stroke'] = self.num_physical_stroke
        # state_df[state_df['drawing_name']==self.drawing_name].iloc[0]['num_physical_stroke'] = self.num_physical_stroke
        state_df.to_csv('save_state.csv', index=False)
    
    def judge_similar(self, i, j):
        return np.max(np.abs(np.array(i) - np.array(j))) < L_th
    
    def look_color_t_up_in_table(self):
        df_read = pd.read_csv(f'save_ok_color_pos.csv') # 读取表格
        self.got_pos = None
        for ind, i in df_read.iterrows():
            if self.judge_similar((i['C'], i['M'], i['Y'], i['K']), self.C_t):
                self.got_pos = (i['x'], i['y'], i['z'])
    
    def save_ok_color_pos(self):
        df_update = pd.read_csv(f'save_ok_color_pos.csv') 
        df_update = df_update.append({
            'x': self.mix_place[self.num_brush][self.mix_place_count][0], 
            'y': self.mix_place[self.num_brush][self.mix_place_count][1],
            'z': self.mix_place[self.num_brush][self.mix_place_count][2], 
            'C': self.C_t[0], 'M': self.C_t[1], 'Y': self.C_t[2], 'K': self.C_t[3]
            # 'C': self.C_d[0], 'M': self.C_d[1], 'Y': self.C_d[2], 'K': self.C_d[3]
        }, ignore_index=True)
        df_update.to_csv(f'save_ok_color_pos.csv', index=False)

    def init_save_ok_color_pos(self):
        # 都是虚拟的用户坐标系坐标
        df = pd.DataFrame(columns=['x', 'y', 'z', 'C', 'M', 'Y', 'K'])
        df.to_csv(f'save_ok_color_pos.csv', index=False)
    
    def init_save_state(self):
        drawing_name = self.drawing_name
        # 都是虚拟的用户坐标系坐标
        df = pd.DataFrame(columns=['mix_place_count', 'drawing_name', 'num_physical_stroke'])
        df = df.append( {'mix_place_count': 0, 'drawing_name': drawing_name, 'num_physical_stroke': 0}, ignore_index= True)
        df.to_csv(f'save_state.csv', index=False)

    def get_what_pigment(self, robot, what_you_want):
        move_cartesian_in_up_way(robot, self.T[self.num_brush][CMYKW_str[from_color_to_int[what_you_want]]])
    
    def get_pigment(self, robot):
        move_cartesian_in_up_way(robot, self.T[self.num_brush][CMYKW_str[self.C_m]])
    
    # def move_just_above_what_pigment(self, robot, what_you_want):
    #     move_cartesian_in_up_way(robot, self.T[self.num_brush][CMYKW_str[what_you_want]])
    
    def mix_pigment(self, robot):
        if self.got_pos:
            dest = self.got_pos
        else:
            dest = self.mix_place[self.num_brush][self.mix_place_count]
            
        move_cartesian_in_up_way(robot, dest)
        mix_movement(robot, dest)

        return dest
    
    def capture_color(self, robot, cap):
        up_to_height_in_the_air(robot, height_in_the_air)
        ret, self.frame = cap.read()
        if ret:
            if self.frame is not None:
                # cv2.imshow('show', frame)
                cv2.waitKey(1)
            else:
                print("无画面")
        else:
            print("无法读取摄像头！")

    def all_colors_and_show(self):
        window = window_every_brush[self.num_brush]
        frame = self.frame

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        draw_1=cv2.rectangle(frame, window[0], window[1], (0,255,0), 1)
        draw_1_rgb = cv2.cvtColor(draw_1, cv2.COLOR_BGR2RGB)
        
        if ion:
            plt.ion()
        
        fig = plt.figure('Figure 1')
        ax1 = fig.add_subplot(221)
        ax2 = fig.add_subplot(222)
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224)
        ax1.title.set_text('Target Color')
        ax2.title.set_text('Window')
        ax3.title.set_text('Zoom in to Window')
        ax4.title.set_text('Detected Color')
        
        # 目标颜色
        plt.subplot(221)
        # plt.imshow(frame_rgb)
        img = Image.new("RGB",(15,15))###创建一个5*5的图片
        pixTuple = (int(self.C_t_in_rgb[0]),int(self.C_t_in_rgb[1]),int(self.C_t_in_rgb[2]),0)###三个参数依次为R,G,B,A   R：红 G:绿 B:蓝 A:透明度
        # 还是就用 RGB0 吧
        for i in range(15):
            for j in range(15):
                img.putpixel((i,j),pixTuple)
        plt.imshow(img)

        # 框出大window
        plt.subplot(222)
        plt.imshow(draw_1_rgb)

        # 用9*9的中间的小区域做均值
        # mid = (window[0]+window[1])//2
        # # mid = [int(i) for i in mid]
        # win_new = [mid-1,mid+1]
        # ic(win_new)

        # 原frame的大window
        plt.subplot(223)
        plt.imshow(frame_rgb[window[0][1]:window[1][1],window[0][0]:window[1][0]])

        # 原frame的大window中的小区域
        # plt.subplot(224)
        # plt.imshow(frame_rgb[win_new[0][1]:win_new[1][1],win_new[0][0]:win_new[1][0]])

        # 显示检测到的结果颜色 
        plt.subplot(224)
        img = Image.new("RGB",(15,15))###创建一个5*5的图片
        # stroke_num = 1
        # # 为较淡，不一定
        # pixTuple = (int(255*data['x_color'][0,stroke_num,0]),int(255*data['x_color'][0,stroke_num,1]),int(255*data['x_color'][0,stroke_num,2]),0)###三个参数依次为R,G,B,A   R：红 G:绿 B:蓝 A:透明度
        # # 为较浓
        pixTuple = (int(self.C_d_in_rgb[0]),int(self.C_d_in_rgb[1]),int(self.C_d_in_rgb[2]),0)###三个参数依次为R,G,B,A   R：红 G:绿 B:蓝 A:透明度
        # 还是就用 RGB0 吧
        for i in range(15):
            for j in range(15):
                img.putpixel((i,j),pixTuple)
        plt.imshow(img)




        # ret = cal_mean_color_for_window(frame, window)
        # ret = cal_mean_color_for_window(frame, win_new)
        # ic(ret)
        
        plt.show()

        # time.sleep(5)
        # plt.close()


    def recog_mean_color_for_window(self, window):
        # 识别结果
        self.C_d_in_rgb = cal_mean_color_for_window_in_rgb(self.frame, window[self.num_brush])
        self.C_d = cal_mean_color_for_window_in_cmyk(self.frame, window[self.num_brush])
                    
    
    # def calibrate(self, robot):
    #     move_cartesian_in_up_way(robot, (0, 0, 0.15))
    
    # 就画一笔
    def draw(self, robot, start_point_x, start_point_y, end_point_x, end_point_y):
        mere_move_cartesian(robot, 
        (start_point_x, start_point_y, canvas_height + self.mix_place[self.num_brush][0][2] + 0.01))
        
        mere_move_cartesian(robot, 
        (start_point_x, start_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))
        mere_move_cartesian(robot, 
        (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))

    # 就画一笔
    def draw_with_end_dot(self, robot, start_point_x, start_point_y, end_point_x, end_point_y):
        mere_move_cartesian(robot, 
        (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2] + 0.01))
        mere_move_cartesian(robot, 
        (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))
        mere_move_cartesian(robot, 
        (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2] + 0.01))
        
        
        mere_move_cartesian(robot, 
        (start_point_x, start_point_y, canvas_height + self.mix_place[self.num_brush][0][2] + 0.01))
        
        mere_move_cartesian(robot, 
        (start_point_x, start_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))
        mere_move_cartesian(robot, 
        (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))

    # 就画一笔
    def draw_with_end_dot_by_mid_point(self, robot, start_point_x, start_point_y, end_point_x, end_point_y):
        # mere_move_cartesian(robot, 
        # (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2] + 0.01))
        # mere_move_cartesian(robot, 
        # (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))
        # mere_move_cartesian(robot, 
        # (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2] + 0.01))
        
        
        mere_move_cartesian(robot, 
        (canvas_offset_x + start_point_x, start_point_y, canvas_height + self.mix_place[self.num_brush][0][2] + 0.02))
        mere_move_cartesian(robot, 
        (canvas_offset_x + start_point_x, start_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))

        mid_points = calc_mid_points(canvas_offset_x + start_point_x, start_point_y, canvas_offset_x + end_point_x, end_point_y)
        for i in mid_points:
            mere_move_cartesian(robot, 
        (i[0], i[1], canvas_height + self.mix_place[self.num_brush][0][2]))

        mere_move_cartesian(robot, 
        (canvas_offset_x + end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))

    # 就画一笔，轨迹运动
    # 废弃
    def draw_with_end_dot_in_track(self, robot, start_point_x, start_point_y, end_point_x, end_point_y):
        mere_move_cartesian(robot, 
        (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2] + 0.01))
        mere_move_cartesian(robot, 
        (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))
        mere_move_cartesian(robot, 
        (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2] + 0.01))
        
        current_pos = robot.get_current_waypoint()
    
        current_pos['pos'] = convert_imaginary_user_coord_to_base(
            [start_point_x, start_point_y, canvas_height + self.mix_place[self.num_brush][0][2]])
        
        ik_result = robot.inverse_kin(current_pos['joint'], pos = current_pos['pos'], ori = current_pos['ori'])
        # # ik_result = robot.inverse_kin(current_pos['joint'], current_pos['pos'], current_pos['ori'])
        # # 清除所有已经设置的全局路点
        robot.remove_all_waypoint()

        # # 添加全局路点1,用于轨迹运动
        joint_radian = ik_result['joint']
        robot.add_waypoint(joint_radian)


        robot.move_joint(joint_radian)

        current_pos['pos'] = [end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2]]
        
        ik_result = robot.inverse_kin(current_pos['joint'], pos = current_pos['pos'], ori = current_pos['ori'])
        
        # 添加全局路点2,用于轨迹运动
        joint_radian = ik_result['joint']
        robot.add_waypoint(joint_radian)

        # # # 添加全局路点3,用于轨迹运动
        joint_radian = (-0.037186, -0.224307, -1.398285, 0.396819, -1.570796, -0.037191)
        robot.add_waypoint(joint_radian)

        # # # 设置圆运动圈数
        # # robot.set_circular_loop_times(3)

        # # # 圆弧运动
        # # logger.info("move_track ARC_CIR")
        robot.move_track(RobotMoveTrackType.CARTESIAN_MOVEP)

        
        # mere_move_cartesian(robot, 
        # (start_point_x, start_point_y, canvas_height + self.mix_place[self.num_brush][0][2] + 0.01))
        
        # mere_move_cartesian(robot, 
        # (start_point_x, start_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))
        # mere_move_cartesian(robot, 
        # (end_point_x, end_point_y, canvas_height + self.mix_place[self.num_brush][0][2]))

    

       