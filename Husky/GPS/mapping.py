from matplotlib import image, scale
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

mun_of_coordinates = 100

mapdata = open('maps/mapdata.txt', 'r')
map_corners = mapdata.read()
mapdata.close()
# Longetude range from mapdata.txt
lon_range = map_corners.split(' ')[0:2]

#open csv writer
#csv_writer = pd.read_csv('data.csv', names=['LATITUDE', 'LONGITUDE'], sep=',')

#for i in range(mun_of_coordinates):



print(lon_range)
# wait for user input
input()