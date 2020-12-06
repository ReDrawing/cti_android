import urllib.request
from urllib.error import URLError
import json
import rclpy
from rclpy.node import Node
import sys
import numpy as np

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

class IMUReceiver(Node):
    '''!
        Receptor e publicador de dados recebidos via rede da IMU de um celular

        @todo Permitir ler as covariancias de arquivos, usando o modelo do celular de parametro
    '''

    def __init__(self):
        super().__init__('imu_receiver')
        self.publisher = self.create_publisher(Imu, "imu", 10)
        self.publisherMag = self.create_publisher(MagneticField, "mag", 10)

        self.accelMeasureTime = 0
        self.gyroMeasureTime = 0
        self.magMeasureTime = 0

        self.declare_parameter("ip","192.168.2.101")
        self.declare_parameter("frame_id","cellphone_imu")

    def receiveAndPublish(self):
        '''!
            Recebe os dados e publica
        '''

        ip = self.get_parameter("ip").get_parameter_value().string_value

        url = "http://"+ip+":8080/sensors.json"

        try:

            with urllib.request.urlopen(url) as req:
                data = req.read()
                dataDict = json.loads(data)

                #print(len(dataDict["accel"]["data"]), len(dataDict["gyro"]["data"]))

                accelIndex = 0
                gyroIndex = 0

                while(accelIndex <len(dataDict["accel"]["data"]) and gyroIndex<len(dataDict["gyro"]["data"]) ):
                    if(dataDict["accel"]["data"][accelIndex][0]> self.accelMeasureTime):
                            self.accelMeasureTime = dataDict["accel"]["data"][accelIndex][0]
                            self.gyroMeasureTime = dataDict["gyro"]["data"][gyroIndex][0]

                            self.publish(dataDict["accel"]["data"][accelIndex][1], dataDict["gyro"]["data"][gyroIndex][1], self.accelMeasureTime)
                    
                    if(dataDict["accel"]["data"][accelIndex][0] == dataDict["gyro"]["data"][gyroIndex][0]):
                        accelIndex += 1
                        gyroIndex += 1
                    elif(dataDict["accel"]["data"][accelIndex][0] < dataDict["gyro"]["data"][gyroIndex][0]):
                        accelIndex += 1
                    else:
                        gyroIndex += 1

                for j in range(len( dataDict["mag"]["data"])):
                    if(dataDict["mag"]["data"][j][0]>self.magMeasureTime):
                        self.magMeasureTime = dataDict["mag"]["data"][j][0]

                        self.publishMag(dataDict["mag"]["data"][j][1], self.magMeasureTime)

        except URLError:
            self.get_logger().warn("Nao foi possivel conectar. O endereco ip foi definido corretamente?")
        except KeyError:
            self.get_logger().warn("O celular esta conectado?")
        except ConnectionResetError:
            self.get_logger().warn("Conexao cancelada. O celular foi desconectado?")

    def publish(self, accel, gyro, time):
        '''!
            Publica os dados de aceleracao e giro

            @param accel: float[3] - leituras de aceleracao linear
            @param gyro: float[3] - leituras de velocidade angular
            @param time: float - momento em que a leitura foi realizada
        '''

        msg = Imu()

        covariance = np.zeros((3,3), dtype=np.float64)

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.linear_acceleration_covariance = covariance

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.angular_velocity_covariance = covariance

        covariance[0][0] = -1

        msg.orientation_covariance = covariance

        frameId = self.get_parameter("frame_id").get_parameter_value().string_value
        msg.header.frame_id = frameId

        timeSec = float(time)/1000.0
        msg.header.stamp.sec = int(timeSec)
        msg.header.stamp.nanosec = int((timeSec*1000000000)%1000000000)

        self.publisher.publish(msg)
        

    def publishMag(self, mag, time):
        '''!
            Publica os dados de campo magnetico

            @param accel: float[3] - leituras do magnetometro
            @param time: float - momento em que a leitura foi realizada
        '''

        msg = MagneticField()

        msg.magnetic_field.x = mag[0]*0.000001
        msg.magnetic_field.y = mag[1]*0.000001
        msg.magnetic_field.z = mag[2]*0.000001 

        msg.magnetic_field_covariance = np.zeros((3,3), dtype=np.float64)

        frameId = self.get_parameter("frame_id").get_parameter_value().string_value
        msg.header.frame_id = frameId

        timeSec = float(time)/1000.0
        msg.header.stamp.sec = int(timeSec)
        msg.header.stamp.nanosec = int((timeSec*1000000000)%1000000000)

        self.publisherMag.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    imuReceiver = IMUReceiver()

    while rclpy.ok():
        imuReceiver.receiveAndPublish()
        rclpy.spin_once(imuReceiver, timeout_sec=0)



if __name__ == '__main__':
    main()