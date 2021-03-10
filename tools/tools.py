import numpy as np

def Rxa(theta):
    R = np.array([[1, 0, 0],
                  [0, np.cos(theta), -np.sin(theta)],
                  [0, np.sin(theta), np.cos(theta)]])
    return R

def Rxp(theta):
    return Rxa(theta).transpose()

def Rya(theta):
    R = np.array([[np.cos(theta), 0, np.sin(theta)],
                  [0, 1, 0],
                  [-np.sin(theta), 0, np.cos(theta)]])
    return R

def Ryp(theta):
    return Rya(theta).transpose()

def Rza(theta):
    R = np.array([[np.cos(theta), -np.sin(theta), 0],
                  [ np.sin(theta), np.cos(theta), 0],
                  [0, 0, 1]])
    return R

def Rzp(theta):
    return Rza(theta).transpose()

def RotationVehicle2Body(phi, theta, psi):
    R = Rxp(phi) @ Ryp(theta) @ Rzp(psi)
    return R

def RotationBody2Vehicle(phi, theta, psi):
    return RotationVehicle2Body(phi, theta, psi).transpose()
