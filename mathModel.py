# copyright 2020 Armin Straller
# modelling the kinematic attributes of the robotArm
# currently working with torque commands. assuming an ideal toruqe behaviour.
# TODO: adding a small first order lag could make sense. 
# robot kinematics based on http://scholar.ppu.edu/bitstream/handle/123456789/884/SCARA%20Robot%20pdf.pdf?sequence=1&isAllowed=y

import math
import numpy as np

class DirectKinematicsModel(object):
    def __init__(self, lengthA1, lengthA2, offsetZ):
        self.a1 = lengthA1
        self.a2 = lengthA2
        self.d4 = offsetZ
        self.theta1 = 0
        self.theta2 = 0
        self.d3 = 0
        self.theta4 = 0
        self.T04 = np.zeros((4,4))
    def update(self, q):
        self.theta1 = q[0]
        self.theta2 = q[1]
        self.d3 = q[2]
        self.theta4 = q[3]
        s1 = math.sin(self.theta1)
        c1 = math.cos(self.theta1)
        s12 = math.sin(self.theta1 + self.theta2)
        c12 = math.cos(self.theta1 + self.theta2)
        s12m4 = math.sin(self.theta1 + self.theta2 - self.theta4)
        c12m4 = math.cos(self.theta1 + self.theta2 - self.theta4)
        
        self.T04 = np.array([   [c12m4,     s12m4,      0,  self.a2 * c12 + self.a1 * c1],
                                [s12m4,     -c12m4,     0,  self.a2 * s12 + self.a1 * s1],
                                [0,         0,          -1, -self.d3 + self.d4],
                                [0,         0,          0,  1]])

# The inverse kinematics model determines the individual joint angles and extensions 
# based on a given end effector position and rotation.
class InverseKinematicsModel(object):
    def __init__(self, lengthA1, lengthA2, offsetZ):
        self.a1 = lengthA1
        self.a2 = lengthA2
        self.d4 = offsetZ
        self.P = np.zeros((3,1)) #position vector relative to base
        self.R = np.zeros((3,3)) #rotational matrix relative to base
        self.T04 = np.zeros((4,4))
        self.theta1 = 0
        self.theta2 = 0
        self.d3 = 0
        self.beta = 0
        self.theta4 = 0
        self.q = np.zeros((4,1))
        pass
    def update(self, p, r, beta):
        self.P = p
        self.R = r
        self.T04 = np.array([   [self.R[1][1],  self.R[1][2],   self.R[1][3],   self.P[1]],
                                [self.R[2][1],  self.R[2][2],   self.R[2][3],   self.P[2]],
                                [self.R[3][1],  self.R[3][2],   self.R[3][3],   self.P[3]],
                                [0,             0,              0,              1]])
        
        # z distance
        self.d3 = self.d4 - self.P[3]

        # joint 2 
        c2 = (self.P[1] ** 2 + self.P[2] ** 2 - (self.a1 ** 2 + self.a2 ** 2)) / (2 * self.a1 * self.a2)
        s2 = math.sqrt(1 - c2 ** 2)
        self.theta2 = math.atan2(s2, c2)

        # joint 1
        s1 = ((self.a1 + self.a2 * c2) * self.P[2] - self.a2 * s2 * self.P[1]) / (self.P[1] ** 2 + self.P[2] ** 2)
        self.theta1 = math.asin(s1)
        
        self.beta = beta
        self.theta4 = self.theta1 + self.theta2 - self.beta

        self.q = np.array([self.theta1, self.theta2, self.d3, self.theta4])

# The differential kinematics model gives the relationship between the joint velocities 
# and the corresponding linear and angular velocities
class DifferentialKinematicsModel(object):
    def __init__(self, lengthA1, lengthA2, offsetZ):
        self.a1 = lengthA1
        self.a2 = lengthA2
        self.theta1 = 0
        self.theta2 = 0
        self.Pdot_e = np.zeros((4,1))
        self.omega_e = np.zeros((4,1))
        self.J = np.zeros((6,4))
    def update(self, q, qdot):
        self.theta1 = q[0]
        self.theta2 = q[1]
        s1 = math.sin(self.theta1)
        c1 = math.cos(self.theta1)
        s12 = math.sin(self.theta1 + self.theta2)
        c12 = math.cos(self.theta1 + self.theta2)
        self.J = np.array([ [- (self.a2 * s12 + self.a1 * s1),  self.a2 * s12,      0,  0],
                            [self.a2 * c12 + self.a1 * c1,      - self.a2 * c12,    0,  0],
                            [0,                                 0,                  -1, 0],
                            [0,                                 0,                  0,  0],
                            [0,                                 0,                  0,  0],
                            [1,                                 1,                  0,  -1]])
        
        JtimesQdot = self.J * qdot
        self.Pdot_e = JtimesQdot[0:3, 0:4]
        self.omega_e = JtimesQdot[3:6, 0:4]

# The dynamic model provides a description of the relationship between the joint actuator torques and the motion of the structure
# and the motion of the structure
# Frictions are neglected
class DynamicModel(object):
    def __init__(self, lengthA1, lengthA2, offsetZ, lengthCOGA1, lengthCOGA2, massLink1, massLink2, massLink34, intertiaL1, intertiaL2, intertiaL4):
        self.g0 = 9.81
        self.mL1 = massLink1
        self.mL2 = massLink2
        self.mL34 = massLink34
        self.a1 = lengthA1
        self.a2 = lengthA2
        self.theta1 = 0
        self.theta2 = 0
        self.l1 = lengthCOGA1
        self.l2 = lengthCOGA2
        self.JL1 = intertiaL1
        self.JL2 = intertiaL2
        self.JL4 = intertiaL4
        # gravitational matrix
        self.g = np.array([0,   0,  - self.mL34 * self.g0,  0])
        self.C = np.zeros((4,4))
        self.M = np.zeros((4,4))
        self.tau = np.zeros((4,1))
        
    def update(self, q, qdot, qdotdot):
        self.theta1 = q[0]
        self.theta2 = q[1]
        s2 = math.sin(self.theta2)
        c2 = math.cos(self.theta2)
        A = - self.mL2 * self.l2 * self.a1 * s2 - self.mL34 * self.a1 * self.a2 * s2
        # coriolis and centrifugal matrix
        self.C = np.array([ [A * qdot[1],   A * (qdot[0] + qdot[1]),    0,  0],
                            [- A * qdot[0], 0,                          0,  0],
                            [0,             0,                      0,  0],
                            [0,             0,                      0,  0]])
        M11 =   self.mL1 * self.l1 ** 2 + self.mL2 * (self.l2 ** 2 + self.a1 ** 2 + 2 * self.l2 * self.a1 * c2) \
                + self.mL34 * (self.l2 ** 2 + self.a1 ** 2 + 2 * self.a1 * self.a2 * c2) \
                + self.JL1 + self.JL2 + self.JL4
        M12 =   self.mL2 * (self.l2 ** 2 + self.l2 * self.a1 * c2) \
                + self.mL34 * (self.a2 ** 2 + self.a2 * self.a1 * c2) \
                + self.JL2 + self.JL4
        M21 = M12
        M22 = self.mL2 * self.l2 ** 2 + self.mL34 * self.a2 ** 2 + self.JL2 + self.JL4
        M14 = - self.JL4
        M41 = M14
        M24 = - self.JL4
        M42 = M24
        M33 = self.mL34
        M44 = self.JL4
        # mass matrix 
        self.M = np.array([ [M11,   M12,    0,      M14],
                            [M21,   M22,    0,      M24],
                            [0,     0,      M33,    0],
                            [M41,   M42,    0,      M44]])

        self.tau = self.M * qdotdot + self.C * qdot + self.g


class Model(object):
    def __init__(self, attachedLoad, uI):
        self.posAxis = np.zeros((4, 1))
        self.speedMotors = np.zeros((4, 1))
        self.torqueMotors = np.zeros((4, 1))
        self.alphaMotors = np.zeros((3, 1))
        self.omegaMotors = np.zeros((3, 1))
        self.inertiaAxis = np.zeros((3, 1))
        self.accelZ = 0
        self.speedZ = 0
        self.maxOmegaMotors = np.array([20, 20, 20, 20]) #rotations per second
        self.maxTorqueMotors = np.array([1.9, 1.9, 0.5, 0.5]) #Nm
        self.transmissionMotors = np.array([0.5, 0.5, 0.5, 0.125]) #Nm
        self.forceZ = 1.0 #N
        self.maxSpeedZ = 0.01 #m/s
        self.armLength = np.array([0.25, 0.25, 0.01, 0.20]) #meters
        self.load = attachedLoad
        self.armLoads =  np.array([2, 3, self.load]) #kg
        self.updateInterval = uI    

    def update(self):
        self.calculateInertiaAxis()
        for i in range(3):
            self.motorConversion(i)
        self.linearConversion(3)
        self.speedMotors[0] = self.omegaMotors[0]
        self.speedMotors[1] = self.omegaMotors[1]
        self.speedMotors[2] = self.omegaMotors[2]
    
    # Inertia calculation assumes that the loads of each individual arm act as point load at the center of the arm. 
    # This way the intertia can be easily calculated per simulation cycle depending on the joint angles.
    def calculateInertiaAxis(self):
        self.inertiaAxis[2] = (self.armLength[2] / 2) ** 2 * self.armLoads[2]
        self.inertiaAxis[1] = (self.armLength[1] / 2) ** 2 * (self.armLoads[1] + self.armLength[1] ** 2 * self.armLoads[2])
        
        lengthLoad1 = math.sqrt(    (math.cos(self.posAxis[0]) * self.armLength[0] + math.cos(self.posAxis[1]) * (self.armLength[1]/2))**2 + 
                                    (math.sin(self.posAxis[0]) * self.armLength[0] + math.sin(self.posAxis[1]) * (self.armLength[1]/2))**2 )
        lengthLoad2 = math.sqrt(    (math.cos(self.posAxis[0]) * self.armLength[0] + math.cos(self.posAxis[1]) * (self.armLength[1]))**2 + 
                                    (math.cos(self.posAxis[0]) * self.armLength[0] + math.cos(self.posAxis[1]) * (self.armLength[1]))**2 )
        
        self.inertiaAxis[0] = (self.armLength[0] / 2) ** 2 * self.armLoads[0] + lengthLoad1 ** 2 * self.armLoads[1] + lengthLoad2 ** 2 * self.armLoads[2]

    def motorConversion(self, id):
        self.alphaMotors[id] = (self.torqueMotors[id] / self.transmissionMotors[id]) / self.inertiaAxis[id] # rad / (s**2)
        self.omegaMotors[id] = self.omegaMotors[id] + self.alphaMotors[id] * self.updateInterval
        #limiting speed to maximum Motor angular speed
        if(self.omegaMotors[id] > self.maxOmegaMotors[id]):
            self.omegaMotors[id] = self.maxOmegaMotors[id]
        if(self.omegaMotors[id] < -self.maxOmegaMotors[id]):
            self.omegaMotors[id] = -self.maxOmegaMotors[id]
        self.posAxis[id] = self.posAxis[id] + (self.omegaMotors[id] * self.updateInterval) * self.transmissionMotors[id] # adjust for transmission
        #limiting position to maximum Motor position


    def linearConversion(self, id):
        pass

    def setMotorTorques(self, torques):
        for i in range(3):
            if (torques[i] > self.maxTorqueMotors[i]):
                self.torqueMotors[i] = self.maxTorqueMotors[i]
            elif (torques[i] < - self.maxTorqueMotors[i]):
                self.torqueMotors[i] = - self.maxTorqueMotors[i]
            else: 
                self.torqueMotors[i] = torques[i]
