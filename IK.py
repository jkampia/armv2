import numpy as np
import math

def return_xrot(phi):
    phi = math.radians(phi)
    res = np.array([[1, 0, 0], 
                    [0, math.cos(phi), math.sin(phi)], 
                    [0, math.sin(phi), math.cos(phi)]])
    #print(res)
    return res
def return_yrot(phi):
    phi = math.radians(phi)
    res = np.array([[math.cos(phi), 0, math.sin(phi)], 
                    [0, 1, 0], 
                    [-math.sin(phi), 0, math.cos(phi)]])
    #print(res)
    return res
def return_zrot(phi):
    phi = math.radians(phi)
    res = np.array([[math.cos(phi), -math.sin(phi), 0], 
                    [math.sin(phi), math.cos(phi), 0], 
                    [0, 0, 1]])
    #print(res)
    return res
def transform(x, y, z, x_phi, y_phi, z_phi):
    print(np.array([[x], [y], [z]])) # must be vertical array
    step1 = np.matmul(np.array([[x], [y], [z]]), return_zrot(z_phi))
    step2 = np.matmul(step1, return_xrot(x_phi))
    step3 = np.matmul(step2, return_yrot(y_phi))
    return step3

def main():
    x, y, z = 1, 2, 3
    xphi, yphi, zphi = 90, 45, 45
    x, y, z = transform(x,y,z,xphi,yphi,zphi)
    print(x,y,z)

main()
