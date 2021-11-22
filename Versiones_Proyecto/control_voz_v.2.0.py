# -*- coding: utf-8 -*-
"""
Created on Wed Oct 27 22:24:56 2021
@author: Nacho
"""
"""
Este programa permite mover al Duckiebot dentro del simulador
usando control por voz.
"""

import sys
import argparse
import gym
import gym_duckietown
from gym_duckietown.envs import DuckietownEnv
import numpy as np
import cv2
import speech_recognition as sr
import time
r = sr.Recognizer()

# Se leen los argumentos de entrada
parser = argparse.ArgumentParser()
parser.add_argument('--env-name', default="Duckietown-udem1-v1")
parser.add_argument('--map-name', default='udem1')
parser.add_argument('--distortion', default=False, action='store_true')
parser.add_argument('--draw-curve', action='store_true', help='draw the lane following curve')
parser.add_argument('--draw-bbox', action='store_true', help='draw collision detection bounding boxes')
parser.add_argument('--domain-rand', action='store_true', help='enable domain randomization')
parser.add_argument('--frame-skip', default=1, type=int, help='number of frames to skip')
parser.add_argument('--seed', default=1, type=int, help='seed')
args = parser.parse_args()

# Definición del environment
if args.env_name and args.env_name.find('Duckietown') != -1:
    env = DuckietownEnv(
        seed = args.seed,
        map_name = args.map_name,
        draw_curve = args.draw_curve,
        draw_bbox = args.draw_bbox,
        domain_rand = args.domain_rand,
        frame_skip = args.frame_skip,
        distortion = args.distortion,
    )
else:
    env = gym.make(args.env_name)
    
def red_alert(obs):
    red_img = np.zeros((480, 640, 3), dtype = np.uint8)
    red_img[:,:,0] = 255
    blend = cv2.addWeighted(obs, 0.5, red_img, 0.5, 0)

    return blend

# Se reinicia el environment
env.reset()
lower_yellow = np.array([140, 140, 0])
upper_yellow = np.array([255, 255, 5])
min_area = 25000
action = np.array([0.0, 0.0])
while True:

    # Captura la tecla que está siendo apretada y almacena su valor en key
    key = cv2.waitKey(100)
    #Si la tecla que se está presionando es esc, se termina el programa
    if key == 27:
        break


    # La acción de Duckiebot consiste en dos valores:
    # velocidad lineal y velocidad de giro
    # En este caso, ambas velocidades son 0 (acción por defecto)

    

    # Definir acción en base a la tecla apretada
    if key == ord('w'):
        action[0]=0.44
        print(str(key))
        
    if key == ord('s'):
        action[0]=-0.44
        print(str(key))
        
    if key == ord('a'):
        action[1]=1
        print(str(key))

    if key == ord('d'):
        action[1]=-1
        print(str(key))

    if key == ord('q'):
        action[0]=0.44
        action[1]=1
        print(str(key))

    if key == ord('e'):
        action[0]=0.44
        action[1]=-1
        print(str(key))

    if key == ord('z'):
        action[0]=-0.44
        action[1]=-1
        print(str(key))

    if key == ord('x'):
        action[0]=-0.44
        action[1]=1
        print(str(key))
    if key == ord('p'):
        action=np.array([0.0,0.0])

        
    #Configuración del micrófono
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
                    action[0]=action[0]*1.5
                elif str(text)==('frena'):
                    print('Frenando vehículo')
                    action[0]=action[0]*0.5
                elif str(text)==('centro'):
                    print('Centrando vehículo')
                    action[1]=0.0
                elif str(text)==('atrás') or str(text)==('reversa'):
                    print('Retrocediendo')
                    action[0]=-0.44

        
                
            except:
                print("No te he entendido")



    # Se ejecuta la acción definida anteriormente y se retorna la observación (obs),
    # la evaluación (reward), etc
    obs, reward, done, info = env.step(action)
    # obs consiste en un imagen de 640 x 480 x 3

    # done significa que el Duckiebot chocó con un objeto o se salió del camino
    if done:
        print('done!')
        # En ese caso se reinicia el simulador
        env.reset()
        ### CÓDIGO DE DETECCIÓN POR COLOR ###

        #Transformar imagen a espacio HSV
    img = cv2.cvtColor(obs,cv2.COLOR_RGB2HSV)

        # Filtrar colores de la imagen en el rango utilizando
    mask = cv2.inRange(obs,lower_yellow,upper_yellow)

        # Bitwise-AND entre máscara (mask) y original (obs) para visualizar lo filtrado
    fimg = cv2.bitwise_and(img, img,mask = mask)

        # Se define kernel para operaciones morfológicas
    kernel = np.ones((5,5),np.uint8)

        # Aplicar operaciones morfológicas para eliminar ruido
        # Esto corresponde a hacer un Opening
        # https://docs.opencv.org/trunk/d9/d61/tutorial_py_morphological_ops.html
        #Operacion morfologica erode
    fimg1= cv2.erode(fimg, kernel, iterations = 1)
        #Operacion morfologica dilate
    fimg2= cv2.dilate(fimg1, kernel, iterations = 1)

        # Busca contornos de blobs
        # https://docs.opencv.org/trunk/d3/d05/tutorial_py_table_of_contents_contours.html
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Iterar sobre contornos y dibujar bounding box de los patos
    for cnt in contours:
            # Obtener rectangulo que bordea un contorno
        x,y,w,h=cv2.boundingRect(cnt)
            #Filtrar por area minima
        AREA=w*h
        if AREA > min_area: # DEFINIR AREA
                #Dibujar rectangulo en el frame original
            cv2.rectangle(obs,(x,y),(x+w,y+h),(255,0,0),2)
            alert = True
            obs=red_alert(obs)
            action[0]=-0.5
            action[1]=0.0

    # Se muestra en una ventana llamada "patos" la observación del simulador
    cv2.imshow("patos", cv2.cvtColor(obs, cv2.COLOR_RGB2BGR))

# Se cierra el environment y termina el programa
env.close()
