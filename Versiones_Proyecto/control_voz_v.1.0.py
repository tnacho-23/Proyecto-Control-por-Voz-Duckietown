
"""
v.1.0
Este programa permite mover al Duckiebot dentro del simulador
usando control por voz.
Versión básica

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

# Se reinicia el environment
env.reset()

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



    # Se ejecuta la acción definida anteriormente y se retorna la observación (obs),
    # la evaluación (reward), etc
    obs, reward, done, info = env.step(action)
    # obs consiste en un imagen de 640 x 480 x 3

    # done significa que el Duckiebot chocó con un objeto o se salió del camino
    if done:
        print('done!')
        # En ese caso se reinicia el simulador
        env.reset()

    # Se muestra en una ventana llamada "patos" la observación del simulador
    cv2.imshow("patos", cv2.cvtColor(obs, cv2.COLOR_RGB2BGR))


# Se cierra el environment y termina el programa
env.close()
