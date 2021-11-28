#!/usr/bin/env python3

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge, CvBridgeError # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.publisher = rospy.Publisher("/image/yellow_detection", Image, queue_size=10)
        self.mask_publisher = rospy.Publisher("/image/yellow_mask", Image, queue_size=10)

        # subscriptor
        self.subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        
        self.lower_yellow = np.array([45,50,50])
        self.upper_yellow = np.array([100,255,255])
        self.min_area = 250

    def callback(self,msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        # Filtrar colores de la imagen en el rango utilizando
        mask = cv2.inRange(image_hsv, self.lower_yellow, self.upper_yellow)
        kernel = np.ones((5,5),np.uint8)
        image_erode = cv2.erode(mask, kernel, iterations = 1)
        #Operacion morfologica dilate
        image_dilate = cv2.dilate(image_erode, kernel, iterations = 1)

        # Bitwise-AND entre máscara (mask) y original (obs) para visualizar lo filtrado
        image_masked = cv2.bitwise_and(image_hsv, image_hsv, mask=image_dilate)

        # Se define kernel para operaciones morfológicas
        
        contours, _ = cv2.findContours(image_dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Iterar sobre contornos y dibujar bounding box de los patos
        for cnt in contours:
            # Obtener rectangulo que bordea un contorno
            x, y, w, h = cv2.boundingRect(cnt)
            AREA = cv2.contourArea(cnt)
            #Filtrar por area minima
            if AREA > self.min_area: # DEFINIR AREA
                #Dibujar rectangulo en el frame original
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0,0,255), 3)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            mask_msg = self.bridge.cv2_to_imgmsg(image_masked, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.mask_publisher.publish(mask_msg)
        self.publisher.publish(img_msg)

    #def procesar_img(self, img):
        # Cambiar espacio de color

        # Filtrar rango util

        # Aplicar mascara

        # Aplicar transformaciones morfologicas

        # Definir blobs

        # Dibujar rectangulos de cada blob

        # Publicar imagen final


def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
