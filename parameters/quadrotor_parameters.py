import sys
sys.path.append('..')
import numpy as np

######################################################################################
                #   Initial Conditions
######################################################################################
#   Initial conditions for MAV
pn0 = 0.  # initial north position
pe0 = 0.  # initial east position
pd0 = 0.  # initial down position
u0 = 0.  # initial velocity along body x-axis
v0 = 0.  # initial velocity along body y-axis
w0 = 0.  # initial velocity along body z-axis
phi0 = 0.  # initial roll angle
theta0 = 0.  # initial pitch angle
psi0 = 0.  # initial yaw angle
p0 = 0  # initial roll rate
q0 = 0  # initial pitch rate
r0 = 0  # initial yaw rate

######################################################################################
                #   Physical Parameters
######################################################################################
mass = 10.0 #kg
Jx = 1.5 #kg m^2
Jy = 1.5
Jz = 2.0
gravity = 9.81