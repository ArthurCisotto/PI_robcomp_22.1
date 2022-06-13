#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math

def segmenta_linha_branca(bgr):
    """Não mude ou renomeie esta função
        deve receber uma imagem e segmentar as faixas brancas
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mini = (0, 0, 200)
    maxi = (180, 10, 255)
    mask = cv2.inRange(hsv, mini, maxi)
    mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN, np.ones((3,3),dtype=np.uint8))
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE, np.ones((3,3),dtype=np.uint8))
    
    return mask

def estimar_linha_nas_faixas(img, mask):
    """Não mude ou renomeie esta função
        deve receber uma imagem preta e branca e retorna dois pontos que formen APENAS uma linha em cada faixa. Desenhe cada uma dessas linhas na iamgem.
         formato: [[(x1,y1),(x2,y2)], [(x1,y1),(x2,y2)]]
    """

    edges = cv2.Canny(mask,50,100)
    mask[:] = edges[:]
    lines = cv2.HoughLinesP(edges,1,math.radians(1),30,minLineLength=50,maxLineGap=10)

    if lines is None or len(lines) < 1:
        return None

    len_dir = 0
    len_esq = 0
    x1d, y1d, x2d, y2d = 0, 0, 0, 0
    x1e, y1e, x2e, y2e = 0, 0, 0, 0
    
    for ((x1,y1,x2,y2),) in lines:
        m = (y2-y1)/(x2-x1)
        len2 = (y2-y1)**2 + (x2-x1)**2

        if m > 0 and len2 > len_dir:
            len_dir = len2
            x1d = x1
            y1d = y1
            x2d = x2
            y2d = y2
        elif m < 0 and len2 > len_esq :
            len_esq = len2
            x1e = x1
            y1e = y1
            x2e = x2
            y2e = y2

    if len_dir > 0:
        cv2.line(img, (int(x1d), int(y1d)), (int(x2d), int(y2d)),(0,128,0),5)
        
    if len_esq > 0:
        cv2.line(img, (int(x1e), int(y1e)), (int(x2e), int(y2e)),(0,128,0),5)
        
    return [[(x1d, y1d),(x2d, y2d)],[(x1e, y1e),(x2e, y2e)]]


def calcular_equacao_das_retas(linhas):
    """Não mude ou renomeie esta função
        deve receber dois pontos que estejam em cada uma das faixas e retornar a equacao das duas retas. Onde y = h + m * x. Formato: [(m1,h1), (m2,h2)]
    """

    (linha_d, linha_e) = linhas
    ((x1d, y1d),(x2d, y2d)) = linha_d
    ((x1e, y1e),(x2e, y2e)) = linha_e

    try: 
        m1 = (y2d - y1d)/(x2d - x1d)
        h1 = y1d - m1 * x1d

        m2 = (y2e - y1e)/(x2e - x1e)
        h2 = y1e - m2 * x1e

        return [(m1,h1), (m2,h2)]
    except: return None

def calcular_ponto_de_fuga(img, equacoes):
    """Não mude ou renomeie esta função
        deve receber duas equacoes de retas e retornar o ponto de encontro entre elas. Desenhe esse ponto na imagem.
    """
    if equacoes is not None:
        ((m1,h1), (m2,h2)) = equacoes

        xf = (h2-h1)/(m1-m2)
        yf = m1*xf + h1

        if img is not None:
            img = cv2.circle(img,(int(xf),int(yf)),5,(0,200,0),-1)

        return img, (xf,yf)
    else: return img, None


if __name__ == "__main__":
    linhas = [[(3,2.5),(4,0.6)],[(1,2.4),(0.6,1.1)]]
    equa = calcular_equacao_das_retas(linhas)
    print(equa)
    _, pontof = calcular_ponto_de_fuga(None, equa)
    print(pontof)