# from curses import window
import cv2
import numpy as np

# cap = cv2.VideoCapture(0)
# cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)

# while cap.isOpened():
#     ret, frame = cap.read()
#     if ret:
#         if frame is not None:
#             # gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)                     # 高斯模糊
#             # hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)                 # 转化成HSV图像
#             # erode_hsv = cv2.erode(hsv, None, iterations=2)                   # 腐蚀 粗的变细
#             # inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
#             # cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

#             # c = max(cnts, key=cv2.contourArea)
#             # rect = cv2.minAreaRect(c)
#             # box = cv2.boxPoints(rect)
#             # cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)

#             k = cv2.waitKey(1)
#             if k == 27 :
#                 print("1")
#                 filename = 'test.png'
#                 cv2.imwrite(filename, frame, params=None)
#                 # cv2.destroyAllWindows()

#             cv2.imshow('camera', frame)
#             cv2.waitKey(1)
#         else:
#             print("无画面")
#     else:
#         print("无法读取摄像头！")

# cap.release()
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# 测试识别颜色的window
for ind, i in enumerate([f'uppermost-{j}.png' for j in range(9)]):
    image = cv2.imread(i)
    draw_1=cv2.rectangle(image, (333,337), (350,353), (0,255,0), 2)
    # cv2.imshow("draw_0", image)#显示画过矩形框的图片
    cv2.imwrite(i[:-4] + f'-rect.png', draw_1)
    cv2.waitKey(0)



