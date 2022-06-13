#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
video = "quadrado.mp4"

def segmenta_azul(img_bgr):
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    menor = (int(150/2), 240, 240)
    maior = (int(190/2), 255, 255)
    mask = cv2.inRange(img_hsv, menor, maior)
    return mask

def pontos_fuga(img_bgr):
    """
    Cria e retorna uma nova imagem BGR com os
    pontos de fuga desenhados.

    Entrada:
    - img_bgr: imagem original no formato BGR

    Saída:
    - resultado: imagem BGR com os pontos de fuga desenhados 
    """
    mask_blue = segmenta_azul(img_bgr)
    lines = cv2.HoughLinesP(mask_blue, 1, np.pi/180, 100, minLineLength=50, maxLineGap=10)
    dic_m_positivos = {}
    dic_m_negativos = {}
    
    for line in lines:
        x1, y1, x2, y2 = line[0]
        m = (y2-y1)/(x2-x1)
        if m > 0:
            dic_m_positivos[m] = [x1, y1, x2, y2]
        else: 
            dic_m_negativos[m] = [x1, y1, x2, y2]
    
    #Identifica e desenha arestas
    #L1
    maior_m_negativo = min(dic_m_negativos.keys())
    L1 = dic_m_negativos[maior_m_negativo]
    cv2.line(img_bgr, (L1[0], L1[1]), (L1[2], L1[3]), (255, 0, 255), 6)
    hL1 = (L1[1] - maior_m_negativo*L1[0])
    #L2
    menor_m_positivo = min(dic_m_positivos.keys())
    L2 = dic_m_positivos[menor_m_positivo]
    cv2.line(img_bgr, (L2[0], L2[1]), (L2[2], L2[3]), (5, 115, 34), 6)
    hL2 = (L2[1] - menor_m_positivo*L2[0])

    #L3
    menor_m_negativo = max(dic_m_negativos.keys())
    L3 = dic_m_negativos[menor_m_negativo]
    cv2.line(img_bgr, (L3[0], L3[1]), (L3[2], L3[3]), (0, 0, 255), 6)
    hL3 = (L3[1] - menor_m_negativo*L3[0])

    #L4
    maior_m_positivo = max(dic_m_positivos.keys())
    L4 = dic_m_positivos[maior_m_positivo]
    cv2.line(img_bgr, (L4[0], L4[1]), (L4[2], L4[3]), (0, 255, 255), 6)
    hL4 = (L4[1] - maior_m_positivo*L4[0])

    #Identifica e desenha pontos de fuga (vértices)

    xiL1andL2 = (hL2- hL1)/(maior_m_negativo-menor_m_positivo)
    yiL1andL2 = maior_m_negativo*xiL1andL2 + hL1
    
    cv2.circle(img_bgr, (int(xiL1andL2),int(yiL1andL2)), radius=9, color=(0, 0, 255), thickness=-1)
    
    xiL2andL3 = (hL3- hL2)/(menor_m_positivo-menor_m_negativo)
    yiL2andL3 = menor_m_positivo*xiL2andL3 + hL2
    
    cv2.circle(img_bgr, (int(xiL2andL3),int(yiL2andL3)), radius=9, color=(0, 255, 0), thickness=-1)

    xiL3andL4 = (hL4- hL3)/(menor_m_negativo-maior_m_positivo)
    yiL3andL4 = menor_m_negativo*xiL3andL4 + hL3
    
    cv2.circle(img_bgr, (int(xiL3andL4),int(yiL3andL4)), radius=9, color=(255, 0, 0), thickness=-1)

    xiL4andL1 = (hL1- hL4)/(maior_m_positivo-menor_m_negativo)
    yiL4andL1 = maior_m_positivo*xiL4andL1 + hL4

    cv2.circle(img_bgr, (int(xiL4andL1),int(yiL4andL1)), radius=9, color=(255, 255, 255), thickness=-1)

    resultado = img_bgr.copy()
    return resultado

if __name__ == "__main__":

    # Inicializa a aquisição da webcam
    cap = cv2.VideoCapture(video)

    print("Se a janela com a imagem não aparecer em primeiro plano dê Alt-Tab")

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret == False:
            #print("Codigo de retorno FALSO - problema para capturar o frame")
            #cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            break

        # Our operations on the frame come here
        img = frame.copy()
        try:
            img = pontos_fuga(img)
        except:
            pass

        # NOTE que em testes a OpenCV 4.0 requereu frames em BGR para o cv2.imshow
        cv2.imshow('Input', frame)
        cv2.imshow('Output', img)

        # Pressione 'q' para interromper o video
        if cv2.waitKey(1000//30) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

