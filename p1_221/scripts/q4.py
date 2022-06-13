#! /usr/bin/env python3
# -*- coding:utf-8 -*-

# Rodar com 
# roslaunch my_simulation ???.launch
 

from __future__ import print_function, division
from dis import dis
import rospy
import numpy as np
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3, Pose
from tf import transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


bridge = CvBridge()

# Imagem vinda da câmera do robô
cv_image = None
# Ponto de referência usado para o controle do robô
media = []
# Centro da imagem
centro = []
# Distância entre o robô e a origem
distancia = 0.0

area = 0.0 

angulos = None # Variável que guarda os angulos da IMU

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

def crosshair(img, point, color, width=3,length=7):
    cv2.line(img, (point[0] - length//2, point[1]),  (point[0] + length//2, point[1]), color ,width, length)
    cv2.line(img, (point[0], point[1] - length//2), (point[0], point[1] + length//2),color ,width, length)


def leu_imu(dado):
    global angulos
    quat = dado.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))
    mensagem = """
	Tempo: {:}
	Orientação: {:.2f}, {:.2f}, {:.2f}
	Vel. angular: x {:.2f}, y {:.2f}, z {:.2f}\
	Aceleração linear:
	x: {:.2f}
	y: {:.2f}
	z: {:.2f}
    """.format(dado.header.stamp, angulos[0], angulos[1], angulos[2], dado.angular_velocity.x, dado.angular_velocity.y, dado.angular_velocity.z, dado.linear_acceleration.x, dado.linear_acceleration.y, dado.linear_acceleration.z)
    #print(mensagem)

def gerar_ponto_referencia(frame):
    """ 
    Deve gerar o ponto de referência para o controle do robô.
    OBS: Para mostrar imagens de evidência do processamento realizado use a função cv2.imshow()
    Entrada:
        - bgr: imagem colorida do OpenCV no formato BGR
    Saídas:
        - centro: a posição do pixel do centro da imagem
        - media: a posição do ponto de referência encontrado na imagem
        - img: a imagem com o ponto de referência desenhado
    """

    centro = []
    media = [0 ,0]
    img = frame.copy()    
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cor_menor = np.array([40/2, 200, 200])
    cor_maior = np.array([80/2, 255, 255])
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    centro = (frame.shape[1]//2, frame.shape[0]//2)


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - length//2, point[1]),  (point[0] + length//2, point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - length//2), (point[0], point[1] + length//2),color ,width, length)

    # A operação MORPH_CLOSE fecha todos os buracos na máscara menores
    # que um quadrado 7x7. É muito útil para juntar vários
    # pequenos contornos muito próximos em um só.
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))
    
    # Encontramos os contornos na máscara e selecionamos o de maior área
    #contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    maior_contorno = None
    maior_contorno_area = 0

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area > maior_contorno_area:
            maior_contorno = cnt
            maior_contorno_area = area

    # Encontramos o centro do contorno fazendo a média de todos seus pontos.
    if not maior_contorno is None :
        cv2.drawContours(img, [maior_contorno], -1, [0, 0, 255], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(img, (media[0], media[1]), 5, [0, 255, 0])
        cross(img, media, [255,0,0], 1, 17)
    else:
        media = (0, 0)

    # Representa a area e o centro do maior contorno no frame
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    cv2.putText(img,"{:d} {:d}".format(*media),(20,100), 1, 4,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(img,"{:0.1f}".format(maior_contorno_area),(20,50), 1, 4,(255,255,255),2,cv2.LINE_AA)
    
    crosshair(img, centro, (0,0,255), 7, 7)

    return centro, media, maior_contorno_area, img, segmentado_cor
    


def distancia_origem (dado):
    """
        Grava na 'distancia' a distância entre o robô e a origem através da posição extraída da odometria
    """
    global distancia
    global x
    global y 
    global z 

    x = dado.pose.pose.position.x
    y = dado.pose.pose.position.y
    z = dado.pose.pose.position.z
    distancia = math.sqrt(x**2 + y**2 + z**2)
    print("Distância: {:.2f}".format(distancia))



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, media, maior_contorno_area, img, segmentado_cor = gerar_ponto_referencia(cv_image)
    
        # ATENÇÃO: ao mostrar a imagem aqui, não podemos usar cv2.imshow() dentro do while principal!! 
        cv2.imshow("Referencia", img)
        cv2.imshow("mask", segmentado_cor)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)



if __name__=="__main__":
    rospy.init_node("Q4")

    topico_imagem = "/camera/image/compressed"
    topico_odom = "/odom"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebe_scan = rospy.Subscriber(topico_odom, Odometry , distancia_origem)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    recebe_imu = rospy.Subscriber("/imu", Imu, leu_imu)
    
    tolerancia = 10

    try:
        v_cruzeiro = 0.2
        # Insight do Adney:
        # Mesmo em manobra de desvio, o robô não pode parar repentinamente pois deslizaria 
        v_desvio = 0.07
        w_desvio = 0.15
        lista_posicoes = []
        while not rospy.is_shutdown():
            lista_posicoes.append((x,y,z))
            # Inicializando - por default anda em frente com uma velocidade não muito alta
            vel = Twist(Vector3(v_cruzeiro, 0, 0), Vector3(0, 0, 0))

            # Sensor de menor prioridade
            # Usando a imagem para saber se tenho que alinhar o robô
            # É útil se o desvio não é muito grande e não estou muito próximo à parede
            # Proporciona um alinhamento fino
            if media is not None and len(media) > 0 and media[0] > centro[0] + 50 :
                vel = Twist(Vector3(v_desvio,0,0), Vector3(0,0,-w_desvio))
            elif media is not None and len(media) > 0 and media[0] < centro[0] - 50:
                vel = Twist(Vector3(v_desvio,0,0), Vector3(0,0,w_desvio))


            # Verifica a orientação absoluta do robô
            # Caso o desvio seja muito grande, tenta girar no sentido contrário até se recuperar
            if angulos is not None and angulos[2] > 30:
                vel = Twist(Vector3(v_desvio,0,0), Vector3(0,0,-w_desvio))
            elif angulos is not None and angulos[2] < -30:
                vel = Twist(Vector3(v_desvio,0,0), Vector3(0,0,w_desvio))
            if len(lista_posicoes)>1000:
                posicao_inicial = lista_posicoes[1]
                distancia_inicial = math.sqrt((posicao_inicial[0])**2 + (posicao_inicial[1])**2 + (posicao_inicial[2])**2)
                if (distancia-distancia_inicial) < 0.02:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                print(distancia)
                print(distancia_inicial)
            # Publica a velocidade resultante
            velocidade_saida.publish(vel)

            rospy.sleep(0.1)
            

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")