import sys
import os
import matplotlib.pyplot as plt
import math

farr = [12, 30, 40, 50, 60, 70, 100]

for lim in [1200, 1800, 2400]:
    meanlat = [0.0 for x in farr]
    medlat = [0.0 for x in farr]
    taillat = [0.0 for x in farr]
    for f in farr:
        with open('Tsub_%d%d.out'%(lim, f), 'r') as fil:
            for l in fil.readlines():
