#! /usr/bin/env python3
# -*- coding:utf-8 -*-

# Rodar com 
# roslaunch my_simulation rampa.launch


from __future__ import print_function, division


import rospy
import numpy as np
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from nav_msgs.msg import Odometry

bridge = CvBridge()

cv_image = None
media = []
centro = []

area = 0.0 # Variavel com a area do maior contorno

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

topico_odom = "/odom"

# Apenas valores para inicializar
x = -1000
y = -1000
z = -1000
distancia = 10

# Classes da MobileNet
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
        "ball", "bus", "car", "cat", "chair", "cow", "diningtable",\
        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
        "sofa", "train", "tvmonitor"]


# Detectar
CONFIDENCE = 0.4
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
    
def load_mobilenet():
    """Não mude ou renomeie esta função
        Carrega o modelo e os parametros da MobileNet. 
        Retorna a rede carregada.
    """

    proto = "MobileNetSSD_deploy.prototxt.txt" # descreve a arquitetura da rede
    model = "MobileNetSSD_deploy.caffemodel" # contém os pesos da rede em si
    net = cv2.dnn.readNetFromCaffe(proto, model)
    return net

def detect(net, frame, CONFIDENCE, COLORS, CLASSES):
    """
        Recebe - uma imagem colorida BGR
        Devolve: objeto encontrado
    """
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with the
        # prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence


        if confidence > CONFIDENCE:
            # extract the index of the class label from the `detections`,
            # then compute the (x, y)-coordinates of the bounding box for
            # the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # display the prediction
            label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
            print("[INFO] {}".format(label))
            cv2.rectangle(image, (startX, startY), (endX, endY),
                COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(image, label, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))

    # show the output image
    return image, results

def identifica_bicicleta(frame, results):
    global area
    global media
    global centro
    global id

    centro = (frame.shape[1]//2, frame.shape[0]//2)

    for (i, (label, conf, box, box2)) in enumerate(results):
        if label == "bicycle":
            x0 = box[0]
            y0 = box[1]
            x1 = box2[0]
            y1 = box2[1]
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
            media = (int((x0+x1)/2), int((y0+y1)/2))
            area = (x1-x0)*(y1-y0)
            id = i
            break

    return frame, centro, media, area

def recebeu_leitura(dado):
    """
        Grava nas variáveis x,y,z a posição extraída da odometria
        Atenção: *não coincidem* com o x,y,z locais do drone
    """
    global x
    global y 
    global z 

    x = dado.pose.pose.position.x
    y = dado.pose.pose.position.y
    z = dado.pose.pose.position.z

def scaneou(dado):
    global distancia
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    print("Leituras:")
    print(np.array(dado.ranges).round(decimals=2))
    distancia = dado.ranges[0]
    
# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global area
    global resultados

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs

    try:
        net = load_mobilenet()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        cv_image = temp_image.copy()
        saida, resultados = detect(net, cv_image, CONFIDENCE, COLORS, CLASSES)
        cv_image, centro, media, area = identifica_bicicleta(saida, resultados)
        # ATENÇÃO: ao mostrar a imagem aqui, não podemos usar cv2.imshow() dentro do while principal!! 
        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)


if __name__=="__main__":
    rospy.init_node("Q5")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    # Cria um subscriber que chama recebeu_leitura sempre que houver nova odometria
    recebe_odo = rospy.Subscriber(topico_odom, Odometry , recebeu_leitura)
    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    try:
        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
        
        parado = False
        while not rospy.is_shutdown():

            print (area)
            while parado == False:
                if area <  50:
                    # Nao encontrei o objeto, entao, gira
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,math.radians(10)))
                elif centro[0] - 20 < media[0] < centro[0] + 20:
                    # Esta centralizado, vai para frente
                    vel = Twist(Vector3(0.07,0,0), Vector3(0,0,0))
                    # Leitura do lidar
                    if distancia <= .3:
                        parado = True
                else:
                    # Encontrei, mas nao estou centralizado

                    if media[0] > centro[0]:

                        # Vai para a direita
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.07))
                    if media[0] < centro[0]:
                        # Vai para a esquerda
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.07))
                    
                velocidade_saida.publish(vel)
                rospy.sleep(0.1)
            while parado == True:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                velocidade_saida.publish(vel)
                rospy.sleep(2)
                delta_theta = math.radians(182.5)
                t_rot = delta_theta / 0.1
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                velocidade_saida.publish(vel)
                rospy.sleep(t_rot)
                while distancia >0.3:
                    vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    rospy.sleep(0.1)
                break

            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


