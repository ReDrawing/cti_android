import urllib.request
from urllib.error import URLError
import json
import rclpy
from rclpy.node import Node
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
        self.pubImg = self.create_publisher(Image, "image_raw/image", 10)
        self.pubImgCompress = self.create_publisher(CompressedImage, "image_raw/compressed", 10)

        self.declare_parameter("ip","192.168.2.101")
        self.declare_parameter("frame_id","cellphone_camera")

        timer_period = 0.016666  # segundos
        self.timer = self.create_timer(timer_period, self.timerCallback)

    def timerCallback(self):
        '''!
            Callback do timer que recebe a imagem e publica
        '''

        ip = self.get_parameter("ip").get_parameter_value().string_value
        frameId = self.get_parameter("frame_id").get_parameter_value().string_value

        url = "http://"+ip+":8080/shot.jpg"

        with urllib.request.urlopen(url) as req:
            frame = np.array(bytearray(req.read()), dtype = np.uint8)
            frame = cv.imdecode(frame, -1)

            frameRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        
            bridge = CvBridge()
            imgMsg = bridge.cv2_to_imgmsg(frameRGB)
            compImgMsg = bridge.cv2_to_compressed_imgmsg(frameRGB)

            time = self.get_clock().now().to_msg()
            imgMsg.header.stamp = time
            compImgMsg.header.stamp = time
            
            imgMsg.header.frame_id = frameId
            imgMsg.header.frame_id = frameId

            self.pubImg.publish(imgMsg)
            self.pubImgCompress.publish(compImgMsg)

            publishInfo(imgMsg.header)

    def publishInfo(self, header):
        '''!
            @todo Implementar publicacao de CameraInfo
        '''
        pass
        

def main(args=None):
    rclpy.init(args=args)

    node = CameraReceiver()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
                
