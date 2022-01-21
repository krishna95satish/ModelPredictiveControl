from tkinter import N
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

def recta(x1, y1, x2, y2):
    a = (y1 - y2) / (x1 - x2)
    b = y1 - a * x1
    return (a, b)

def curva_b(xa, ya, xb, yb, xc, yc):
    (x1, y1, x2, y2) = (xa, ya, xb, yb)
    (a1, b1) = recta(xa, ya, xb, yb)
    (a2, b2) = recta(xb, yb, xc, yc)
    puntos = []

    for i in range(0, 1000):
        if x1 == x2:
            continue
        else:
            (a, b) = recta(x1, y1, x2, y2)
        x = i*(x2 - x1)/1000 + x1
        y = a*x + b
        puntos.append((x,y))
        x1 += (xb - xa)/1000
        y1 = a1*x1 + b1
        x2 += (xc - xb)/1000
        y2 = a2*x2 + b2
    return puntos

def main():
    lista1 = curva_b(1, 2, 2, 1, 3, 2.5)
    lista2 = curva_b(1, 2, 2.5, 1.5, 3, 2.5)
    lista3 = curva_b(1, 2, 2.5, 2, 3, 2.5)
    lista4 = curva_b(1, 2, 1.5, 3, 3, 2.5)

    fig, ax = plt.subplots()
    ax.scatter(*zip(*lista1), s=1, c='b')
    ax.scatter(*zip(*lista2), s=1, c='r')
    ax.scatter(*zip(*lista3), s=1, c='g')
    ax.scatter(*zip(*lista4), s=1, c='k')
    #plt.show()
    for i in range(-100, 200, 4):
        print(i)    
if __name__ == "__main__":
    main()