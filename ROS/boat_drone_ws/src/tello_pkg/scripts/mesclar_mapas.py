#!/usr/bin/env python3
import cv2
import numpy as np

# Carregar os mapas
empty_map = cv2.imread("empty_map_real.pgm", cv2.IMREAD_GRAYSCALE)
existing_map = cv2.imread("mapa_real.pgm", cv2.IMREAD_GRAYSCALE)

# Certifique-se de que os mapas tenham o mesmo tamanho
if empty_map.shape != existing_map.shape:
    print("Os mapas precisam ter o mesmo tamanho.")
    exit()

# Mesclar os mapas (substituir áreas desconhecidas e ocupadas por áreas livres)
merged_map = np.where(existing_map == 0, 0, existing_map)  # Áreas ocupadas se tornam livres
merged_map = np.where(existing_map == 205, 254, merged_map)  # Áreas desconhecidas se tornam livres

# Salvar o mapa mesclado
cv2.imwrite("merged_navigable_map.pgm", merged_map)

# Atualizar o arquivo YAML, se necessário
