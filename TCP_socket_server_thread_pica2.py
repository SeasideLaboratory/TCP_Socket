# -*- coding: utf-8 -*-

import socketserver
import cv2
import sys
import time
import threading
import serial
from picamera2 import Picamera2
import bno055

# TCP Socket ServerのIPアドレス
HOST = "192.168.11.4"
# 各TCP Socket通信用ポート設定
PORT = 5569             # Video stream port
PORT2 = 5570            # Sensor data port
PORT_DS = 5571          #Controller data port

# 本アプリの終了を設定
stop_application = False

"""
シリアル通信クラス
"""
class SerialComm:
    # 初期化
    def __init__(self):
        # オープンフラグ
        self.isPortOpen = False
        # 受信データ
        self.recvData = bytearray()
        # イベント生成
        self.event = threading.Event()

    # データ受信待ち(タイムアウト付き[sec])
    def recv(self, timeout=3):
        # タイムアウト用時間取得
        timeStart = time.time()
        timeEnd = timeStart
        # スレッド待ちイベントクリア
        self.event.clear()
        # 受信データクリア
        self.recvData.clear()
        # 受信結果 True:成功 False:失敗(タイムアウト)
        result = False

        # データ受信待ち
        while not self.event.is_set():
            # タイムアウトチェック
            timeEnd = time.time()
            if (timeEnd - timeStart > timeout):
                # データ送受信停止して失敗(タイムアウト)とする
                result = False
                self.stop()
                print("timeout:{0}sec".format(timeout))
                break

            # 受信データ読み取り
            buff = self.comm.read()

            # 受信データ判定
            if len(buff) > 0:
                # 受信データ追加
                self.recvData.extend(buff)
                # (仮)¥nを受信済なら成功とする
                if (self.recvData.find(b'\n')) >= 0:
                    # データ送受信停止して成功とする
                    result = True
                    self.stop()
                    break

        # 結果を返す
        return (result, self.recvData)

    # データ送信
    def send(self, data):
        self.comm.write(data)

    # データ送受信停止
    def stop(self):
        self.event.set()

    # シリルポートオープン
    def open(self, tty, baud='115200'):
        try:
            self.comm = serial.Serial(tty, baud, timeout=0.1)
            self.isPortOpen = True
        except Exception as e:
            self.isPortOpen = False

        return self.isPortOpen

    # シリアルポートクローズ(明示的に閉じる)
    def close(self):
        self.stop()
        if (self.isPortOpen):
            self.comm.close()
        self.isPortOpen = False

"""
TCP Socketハンドラークラス（カメラ画像送信用）
"""
class TCPHandler(socketserver.BaseRequestHandler):
    videoCap = ''

    def handle(self):
        if stop_application == False:
            # picamera2で画像をキャプチャ
            frame = picam2.capture_array()
            # 映像の品質を70/100に設定
            encodeParam = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
            # JPEGで画像を圧縮
            jpegs_byte = cv2.imencode('.jpeg', frame, encodeParam)[1]
            # 画像データをTCP Socketで送信
            self.request.send(jpegs_byte)
            
"""
TCP Socketハンドラークラス（9軸センサー送信用）
"""
class TCPHandler2(socketserver.BaseRequestHandler):

    def handle(self):
        # gyroscope値をBNO055から取得する
        vgyro = bno055.vgyro
        # 文字"y"をキーワードとしてgyroscope値をカンマで区切った文字列を作成
        # y[gyroscopeのX値],[gyroscopeのY値],[gyroscopeのX値],
        gyroStr = 'y'
        for i in range(0, 3, 1):
            gyroStr += str("{:.4f}".format(vgyro[i])) + ','

        # euler値をBNO055から取得する
        veuler = bno055.veuler
        # 文字"e"をキーワードとしてeuler値をカンマで区切った文字列を作成
        # e[eulerのX値],[eulerのY値],[eulerのX値],
        eulerStr = 'e'
        for i in range(0, 3, 1):
            eulerStr += str("{:.4f}".format(veuler[i])) + ','

        # gravity値をBNO055から取得する
        vgrav = bno055.vgrav
        # 文字"g"をキーワードとしてeuler値をカンマで区切った文字列を作成
        # g[gravityのX値],[gravityのY値],[gravityのX値],
        gravStr = 'g'
        for i in range(0, 3, 1):
            gravStr += str("{:.4f}".format(vgrav[i])) + ','

        # 各センター値の文字列を結合
        sendStr = gyroStr + eulerStr + gravStr
        # 9軸センサー値をTCP Socketで送信
        self.request.send(sendStr.encode())

        #print('send data : ' + str(send_str))
        
        # 0.1秒間隔で送信
        time.sleep(0.1)

"""
TCP Socketハンドラークラス（DS4ジョイスティック値受信用）
"""
class TCPHandler3(socketserver.BaseRequestHandler):
    
    def handle(self):
        recvLen = 16
        numStr = ''
        # ジョイスティック値を受信
        receivedStr = self.request.recv(recvLen).strip()
        # 受信文字数を取得
        leng = len(receivedStr)
    
        for i in range(0, leng, 1):
            # 受信文字コードを文字に変換
            numChr = chr(receivedStr[i])
            numStr = numStr + numChr

        # 受信文字列から左右ジョイスティック値（浮動小数点）を取得
        #pos_l = str(num).find('l')
        posL = 0
        posR = str(numStr).find('r')
                
        dataL = float(numStr[posL+1:posR-1])
        dataR = float(numStr[posR+1:])
                               
        # 左右ジョイスティック値（0.0 ～ ±1.0）を±0 ～ 255にマッピング
        speedL = int(255.0 * dataL)
        speedR = int(255.0 * dataR)
                
        print('left speed: ' + str(speedL))
                
        # Tiny:bitに左車輪駆動値データ送信           
        serialData = 'l' + str(speedL) + ','
        comm.send(serialData.encode())
        # 0.1秒待つ
        time.sleep(0.1)
        print('right speed: ' + str(speedR))
        # Tiny:bitに右車輪駆動値データ送信   
        serialData = 'r' + str(speedR) + ','
        comm.send(serialData.encode())
                      

# パイカメラの初期化
picam2 = Picamera2()
# 画像フォマットと解像度設定
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
# 動作開始
picam2.start()

# TCP Socketポートの再利用を許可する(コレがないとしばらくの間は同じポートを利用できない)
socketserver.TCPServer.allow_reuse_address = True
# 各TCPサーバー（ハンドラー）生成
server = socketserver.TCPServer((HOST, PORT), TCPHandler)
server2 = socketserver.TCPServer((HOST, PORT2), TCPHandler2)
server3 = socketserver.TCPServer((HOST, PORT_DS), TCPHandler3)

# Start a thread with the server -- that thread will then start one
# more thread for each request
server_thread = threading.Thread(target=server.serve_forever)
# Exit the server thread when the main thread terminates
server_thread.daemon = True
server_thread.start()
print("Server loop running in thread:", server_thread.name)

# Start a thread with the server -- that thread will then start one
# more thread for each request
server2_thread = threading.Thread(target=server2.serve_forever)
# Exit the server thread when the main thread terminates
server2_thread.daemon = True
server2_thread.start()
print("Server loop running in thread:", server2_thread.name)

# Start a thread with the server -- that thread will then start one
# more thread for each request
server3_thread = threading.Thread(target=server3.serve_forever)
# Exit the server thread when the main thread terminates
server3_thread.daemon = True
server3_thread.start()
print("Server loop running in thread:", server3_thread.name)

# USBシリアルを開く
comm = SerialComm()
# Raspberry PiのUSBシリアルデバイスとTiny:bitのボーレートである 115200 を設定
comm.open('/dev/ttyACM0', '115200')

# 9-axis sensor開始
bno055.main()

# 無限ループ
while True:
    try:
        stop_application = False
        time.sleep(0.1)

    except KeyboardInterrupt:
        # コントロールCキーで終了処理
        stop_application = True
        server.shutdown()
        server2.shutdown()
        server3.shutdown()

        sys.exit()
