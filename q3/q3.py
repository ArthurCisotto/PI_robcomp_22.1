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
imgname = "bandeiras.png"

def segmenta_verde(img_bgr):
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    menor = (int(110/2), 120, 100)
    maior = (int(170/2), 255, 170)
    mask = cv2.inRange(img_hsv, menor, maior)
    return mask

def segmenta_amarelo(img_bgr):
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    menor = (int(50/2), 150, 200)
    maior = (int(100/2), 220, 255)
    mask = cv2.inRange(img_hsv, menor, maior)
    return mask

def segmenta_azul(img_bgr):
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    menor = (int(180/2), 150, 110)
    maior = (int(255/2), 220, 230)
    mask = cv2.inRange(img_hsv, menor, maior)
    return mask

def segmenta_branco(img_bgr):
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    menor = (int(230/2), 0, 200)
    maior = (int(255/2), 50, 255)
    mask = cv2.inRange(img_hsv, menor, maior)
    return mask

def junta_masks(mask_verde, mask_amarelo, mask_azul, mask_branco):
    mask = mask_verde + mask_amarelo + mask_azul + mask_branco
    return mask

def encontrar_contornos(mask):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca os contornos encontrados
    """
    # RETR_EXTERNAL: Apenas Contornos Externos
    contornos, arvore = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    return contornos, arvore

def encontrar_e_desenha_caixa_do_contorno(frame, contorno, mask0, mask1, mask2, mask3):
    for c in contorno:
        # desenha um retângulo em volta do contorno
        x, y, w, h = cv2.boundingRect(c)
        pixels_brancos_mask0 = 0
        for i in range(x, x+w):
            for j in range(y, y+h):
                if mask0[j,i] == 255:
                    pixels_brancos_mask0 += 1
        pixels_brancos_mask1 = 0
        for i in range(x, x+w):
            for j in range(y, y+h):
                if mask1[j,i] == 255:
                    pixels_brancos_mask1 += 1
        pixels_brancos_mask2 = 0
        for i in range(x, x+w):
            for j in range(y, y+h):
                if mask2[j,i] == 255:
                    pixels_brancos_mask2 += 1
        pixels_brancos_mask3 = 0
        for i in range(x, x+w):
            for j in range(y, y+h):
                if mask3[j,i] == 255:
                    pixels_brancos_mask3 += 1
        
        if (pixels_brancos_mask0 > 0) and (pixels_brancos_mask1 > 0) and (pixels_brancos_mask2 > 0) and (pixels_brancos_mask3 > 0):
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 5)

    return frame

    

def roda_todo_frame(frame):
    mask_verde = segmenta_verde(frame)
    output_verde = cv2.morphologyEx(mask_verde, cv2.MORPH_CLOSE, np.ones((10,10),np.uint8))
    mask_amarelo = segmenta_amarelo(frame)
    output_amarelo = cv2.morphologyEx(mask_amarelo, cv2.MORPH_CLOSE, np.ones((10,10),np.uint8))
    mask_azul = segmenta_azul(frame)
    output_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, np.ones((10,10),np.uint8))
    mask_branco = segmenta_branco(frame)
    output_branco = cv2.morphologyEx(mask_branco, cv2.MORPH_CLOSE, np.ones((10,10),np.uint8))
    contornos, arvore = encontrar_contornos(junta_masks(mask_verde, mask_amarelo, mask_azul, mask_branco))
    mask = junta_masks(mask_verde, mask_amarelo, mask_azul, mask_branco)
    output = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10,10),np.uint8))
    contornos, _ = encontrar_contornos(output)
    contornos = sorted(contornos, key=cv2.contourArea, reverse=True)
    output = encontrar_e_desenha_caixa_do_contorno(frame, contornos,output_verde, output_amarelo, output_azul, output_branco)
    return output

if __name__ == "__main__":

    frame = cv2.imread(imgname)
    
    # Our operations on the frame come here
    img = frame.copy()
    mask = roda_todo_frame(img)

    # NOTE que em testes a OpenCV 4.0 requereu frames em BGR para o cv2.imshow
    cv2.imshow('Input', frame)
    cv2.imshow('Output', mask)

    cv2.waitKey()
    cv2.destroyAllWindows()

