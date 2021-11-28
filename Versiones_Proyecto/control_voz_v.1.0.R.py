
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


class Template(object):
    def __init__(self, args):
        #publisher
        super(Template, self).__init__()
        self.publisher = rospy.Publisher( )#Acá enviar la velocidad lineal
        self.publisher = rospy.Publisher() #Acá enviar la velocidad angular
       
        # subscriptor
        
    #Configuración del micrófono
        r = sr.Recognizer()
        if key == ord('m'):
            print('Habla ahora...')
            with sr.Microphone() as source:
                audio = r.listen(source)
                try:
                    text = r.recognize_google(audio, language='es-ES')
                    print("Lo que escuché fue:",str(text))
                    if str(text)==('adelante'):
                        print('Moviendose hacia adelante')
                        action[0]=0.44
                    elif str(text)==('derecha'):
                        print('Girando hacia la derecha')
                        action[1]=-0.44
                    elif str(text)==('izquierda'):
                        print('Girando hacia la izquierda')
                        action[1]=0.44
                    elif str(text)==('detente'):
                        print('stop')
                        action = np.array([0.0, 0.0])
                    elif str(text)==('acelera'):
                        print('Acelerando vehículo')
                        action[0]=action[0]*1.2
                    elif str(text)==('frena'):
                        print('Frenando vehículo')
                        action[0]=action[0]*0.8
                    elif str(text)==('centro'):
                        print('Centrando vehículo')
                        action[1]=0.0
                    elif str(text)==('atrás') or str(text)==('reversa'):
                        print('Retrocediendo')
                        action[0]=-0.44
                except:
                    print("No te he entendido")

def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
