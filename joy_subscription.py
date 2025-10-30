import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import subprocess
import time
import threading

ls_data = 0
rs_data = 0

class JoyTranslate(Node):
    def __init__(self):
        super().__init__('joy_translate_node') 
        #self.publisher = self.create_publisher(Twist,'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy,'joy', self.listener_callback, 10)
        #self.vel = Twist()

    def listener_callback(self, Joy):
        global ls_data
        global rs_data

        #print('LS:'+str(Joy.axes[1])+' - RS:'+str(Joy.axes[4]))

        ls_data = Joy.axes[1]
        rs_data = Joy.axes[4]

        #self.get_logger().info("LS:%f" % (Joy.axes[1])) 

def main(args=None):
    print('start ds4drv')
    subprocess.Popen('ds4drv')
    time.sleep(12)
    
    print('start ros2 run joy joy_node')
    subprocess.Popen(['ros2', 'run', 'joy', 'joy_node'])
    time.sleep(1)
    
    rclpy.init(args=args)
    joy_translate = JoyTranslate()

    # excute spin by another thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(joy_translate,))
    spin_thread.start()

def rclpy_shutdown():
    rclpy.shutdown()

if __name__ == '__main__':
    main()