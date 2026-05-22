#!/usr/bin/env python3
import numpy as np
import cv2

width, height = 826, 638 # Definir as dimensões do mapa (gazebo = 523, 418) 

empty_map = np.full((height, width), 254, dtype=np.uint8) # Criar um mapa com todos os valores de 205 (áreas livres)

cv2.imwrite("empty_map.pgm", empty_map) # Salvar o mapa como arquivo .pgm
