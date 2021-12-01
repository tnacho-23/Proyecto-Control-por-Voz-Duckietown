
"""
v.1.0.R
Este programa permite mover al Duckiebot dentro del simulador
usando control por voz.
Versión básica, adaptado para ROS

"""

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge, CvBridgeError # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy
import speech_recognition as sr #Importar Librería Speech Recognition
from duckietown_msgs.msg import Twist2DStamped 


class Template(object):
    def __init__(self, args):
        #publisher
        super(Template, self).__init__()
        self.publisher = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=10)
        self.v = 0
        self.omega = 0
       
        # subscriptor
        
    #loop
    def loop(self):
        r = sr.Recognizer()
        while not rospy.is_shutdown():
            key = cv2.waitKey(100)
            if key == ord('m'):
                print('Habla ahora...')
                with sr.Microphone() as source:
                    audio = r.listen(source)
                    try:
                        text = r.recognize_google(audio, language='es-ES')
                        print("Lo que escuché fue:",str(text))
                        if str(text)==('adelante'):
                            print('Moviendose hacia adelante')
                            self.v=0.44
                        elif str(text)==('derecha'):
                            print('Girando hacia la derecha')
                            self.omega = -0.44
                        elif str(text)==('izquierda'):
                            print('Girando hacia la izquierda')
                            self.omega=0.44
                        elif str(text)==('detente'):
                            print('stop')
                            self.omega = 0
                            self.v = 0
                        elif str(text)==('acelera'):
                            print('Acelerando vehículo')
                            self.v = self.v*1.2
                        elif str(text)==('frena'):
                            print('Frenando vehículo')
                            self.v = self.v*0.8
                        elif str(text)==('centro'):
                            print('Centrando vehículo')
                            self.omega = 0.0
                        elif str(text)==('atrás') or str(text)==('reversa'):
                            print('Retrocediendo')
                            self.v = -0.44
                        else:
                            self.v = 0
                            self.omega = 0
                        twist = Twist2DStamped()
                        twist.v = self.v
                        twist.omega = self.omega    
                        self.publisher.publish(twist)
                    except:
                        print("No te he entendido")

def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba
    obj.loop()

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

if __name__ =='__main__':
    main()
