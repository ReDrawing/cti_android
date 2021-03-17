import urllib.request
from urllib.error import URLError
import json
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class CameraReceiver(Node):
    '''!
        Receptor e publicador de imagens recebidas via rede de uma camera de celular
    '''

    def __init__(self):
        super().__init__('camera_receiver')
        self.pubImg = self.create_publisher(Image, "camera/image", 10)
        self.pubImgCompress = self.create_publisher(CompressedImage, "camera/compressed", 10)

        self.declare_parameter("ip","192.168.2.101")
        self.declare_parameter("frame_id","cellphone_camera")

        ip = self.get_parameter("ip").get_parameter_value().string_value

        self.cap = cv.VideoCapture("http://"+ip+":8080/video")
        self.cap.set(cv.CAP_PROP_BUFFERSIZE, 0)

        self.callbackGroup = ReentrantCallbackGroup()

        timer_period = 0.016666/2.0  # segundos
        self.timer = self.create_timer(timer_period, self.timerCallback)#, callback_group=self.callbackGroup)
        self.timer2 = self.create_timer(timer_period, self.timer2Callback)#, callback_group=self.callbackGroup)

        self.frame = None
        self.ret = False

        self.height = 0
        self.width = 0

    def timer2Callback(self):
        '''!
            Recebe a imagem
        '''

        self.ret, self.frame = self.cap.read()

        if(self.ret):

            self.height = self.frame.shape[0]
            self.width = self.frame.shape[1]

            self.flattened = self.frame.flatten().tolist()

    def timerCallback(self):
        '''!
            Callback do timer que recebe a publica a imagem
        '''
        frameId = self.get_parameter("frame_id").get_parameter_value().string_value

        tempo1 = cv.getTickCount()

        if(self.ret):
            self.ret = False
            imgMsg = Image()

            imgMsg._data = self.flattened 

            imgMsg.height = self.height
            imgMsg.width = self.width

            imgMsg.encoding = "8UC3"
            imgMsg.is_bigendian = 0
            imgMsg.step = imgMsg.width*3

            time = self.get_clock().now().to_msg()
            imgMsg.header.stamp = time
            
            imgMsg.header.frame_id = frameId
            
            self.pubImg.publish(imgMsg)

            #compImgMsg = bridge.cv2_to_compressed_imgmsg(frame)
            #compImgMsg.header.stamp = time
            #compImgMsg.header.frame_id = frameId
            #self.pubImgCompress.publish(compImgMsg)

            self.publishInfo(imgMsg.header)

        tempo2 = cv.getTickCount()

    def publishInfo(self, header):
        '''!
            @todo Implementar publicacao de CameraInfo
        '''
        pass
        

def main(args=None):
    rclpy.init(args=args)

    node = CameraReceiver()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()

    #rclpy.spin(node)

    node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
                
