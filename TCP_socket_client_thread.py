import socket
import numpy
import cv2
import threading
import time
# import Joystick function
import joy_subscription
import numpy as np

# TCP Socket Server(RaspberryP)iのIPアドレス
HOST = "192.168.11.4"
# 各TCP Socket通信用ポート設定
PORT_VS = 5569
PORT_SD = 5570
PORT_DS = 5571

# 本アプリの終了を設定
stop_application = False

# 魚眼レンス補正用係数
DIM = (640, 480)
K = np.array([[347.38111748075727, 0.0, 335.47829046944076], [0.0, 348.442407098082, 218.15058926268196], [0.0, 0.0, 1.0]])
D = np.array([[-0.01261564586101049], [0.028995626694767518], [-0.15921668298758498], [0.14226434190749948]])

# Raspberry Piカメラ画像を受信
def getimage():
    global stop_application
    # TCP Socket生成
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # TCP Socket接続
    sock.connect((HOST, PORT_VS))

    buf = []
    recvLen = 100

    # 画像データ受信処理
    while (recvLen > 0) and (stop_application == False):
        receivedStr = sock.recv(1024*8)
        recvLen = len(receivedStr)
        buf += receivedStr
    # TCP Socket切断
    sock.close()
    # 受信画像データをnumpyアレイに変換
    recData = numpy.array(buf, dtype='uint8')

    # 画像表示
    return cv2.imdecode(recData, 1)

# 9軸センサーデータを受信
def getSensor():
    global stop_application
    # TCP Socket生成
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # TCP Socket接続
    sock.connect((HOST, PORT_SD))

    recvLen = 256
    # センサーバイトデータ受信
    receivedByte = sock.recv(recvLen).strip()
    # TCP Socket切断
    sock.close()

    return receivedByte

# Raspberry Piカメラ画像取得
def th1_func():
        global stop_application

        while True:
            try:
                if stop_application == False:
                    # 画像データ受信
                    img = getimage()
                    #h,w = img.shape[:2]
                    nK = K.copy()
                    nK[0,0] = K[0,0] / 1.2
                    nK[1,1] = K[1,1] / 1.2
                    # 切り捨てる周辺画像を増やす
                    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), nK, DIM, cv2.CV_16SC2)
                    #map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
                    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                    cv2.waitKey(5)  # 少し待ってやらないと映像が生成される前に次の処理が来て映像が映りませんでした。
                    # img_90deg = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE) # 映像が時計方向90度に曲がっていた場合
                    cv2.imshow('Capture', undistorted_img)
                    #cv2.imshow('Capture', img)
                else:
                    break
            except:
                break

# 9軸センサーデータを取得
def th2_func():
        global stop_application

        while True:
            try:
                if stop_application == False:
                    # センサーデータ受信
                    buf = getSensor()
                    # 受信バイトコードを文字列に変換
                    leng = len(buf)
                    numStr = ''
                    for i in range(0, leng, 1):
                         numChr = chr(buf[i])
                         numStr += numChr
                    #print(numStr)
                    # 受信文字列を各軸センサーデータ（浮動小数点）に変換
                    pos_kanmas = []
                    pos_current = 1
                    keyword = ','
                    # データ区切り文字（カンマ)の位置を取得
                    while True:
                         index = numStr.find(keyword, pos_current)
                         if index == -1:
                              break
                         pos_kanmas.append(index)
                         pos_current = index + 1
                    # 各軸区切り文字（y/e/g）位置取得
                    pos_gyro = 1
                    pos_euler = str(numStr).find('e')
                    pos_grav = str(numStr).find('g')
                    # gyroの各軸データ取得
                    gyro_x_val = float(numStr[pos_gyro:pos_kanmas[0]])
                    gyro_y_val = float(numStr[pos_kanmas[0]+1:pos_kanmas[1]])
                    gyro_z_val = float(numStr[pos_kanmas[1]+1:pos_kanmas[2]])
                    print('gyro:['+str(gyro_x_val)+','+str(gyro_y_val)+','+str(gyro_z_val)+']')
                    # eulerの各軸データ取得
                    euler_x_val = float(numStr[pos_euler+1:pos_kanmas[3]])
                    euler_y_val = float(numStr[pos_kanmas[3]+1:pos_kanmas[4]])
                    euler_z_val = float(numStr[pos_kanmas[4]+1:pos_kanmas[5]])
                    print('euler:['+str(euler_x_val)+','+str(euler_y_val)+','+str(euler_z_val)+']')
                    # gravityの各軸データ取得
                    grav_x_val = float(numStr[pos_grav+1:pos_kanmas[6]])
                    grav_y_val = float(numStr[pos_kanmas[6]+1:pos_kanmas[7]])
                    grav_z_val = float(numStr[pos_kanmas[7]+1:pos_kanmas[8]])
                    print('gravity:['+str(grav_x_val)+','+str(grav_y_val)+','+str(grav_z_val)+']')
                else:
                    break
            except:
                break

# DS4ジョイスティック変位量を送信
def th3_func():
        global stop_application

        while True:
            try:
                if stop_application == False:
                    # TCP Socket生成
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    # TCP Socket接続
                    sock.connect((HOST, PORT_DS))

                    # 左右ジョイスティックの変位量取得
                    dataL = "{:.3f}".format(joy_subscription.ls_data)
                    dataR = "{:.3f}".format(joy_subscription.rs_data)

                    #print("L:"+str(data_l)+" R:"+str(data_r))
                    # 送信用文字列データ作成
                    sdata = 'l' + str(dataL) + 'r' + str(dataR)
                    # TCP Socket送信
                    sock.send(sdata.encode())
                    # TCP Socket切断
                    sock.close()
                    # 0.1秒待つ
                    time.sleep(0.1)
                else:
                    break
            except:
                break

# メイン処理関数
def main_func():
    global stop_application

    # Start Joystick function
    joy_subscription.main()
    # 各処理をスレッド化し開始
    thread1 = threading.Thread(target=th1_func)
    thread1.start()

    thread2 = threading.Thread(target=th2_func)
    thread2.start()

    thread3 = threading.Thread(target=th3_func)
    thread3.start()

    while True:
        try:
            stop_application = False
            time.sleep(0.1)

        except KeyboardInterrupt:
            # コントロールCキー終了処理
            print("stop")
            stop_application = True

            joy_subscription.rclpy_shutdown()
            
            thread1.join()
            thread2.join()
            thread3.join()
            break
    
if __name__ == '__main__':
	result = main_func()