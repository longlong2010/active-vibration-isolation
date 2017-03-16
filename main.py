from numpy import *;
import scipy.linalg;
import serial;
import time;

from positioning import *;
from controller import *;


x1 = 0.1509;
y1 = 0.0709;

x2 = 0.1530;
y2 = -0.1766;

x3 = -0.0526;
y3 = -0.1766;

x4 = 0.1380;
z4 = 0.0060;

x5 = -0.0376;
z5 = 0.0060;

y6 = 0.0597;
z6 = 0.0060;

param = PositionParam([[x1, y1], [x2, y2], [x3, y3], [x4, z4], [x5, z5], [y6, z6]]);
solver = PositionSolver(param);
reader = SensorReader('/dev/ttyUSB0', 9600);

I = array([11581946.33, 18047496.42, 29576720.71]) * 1e-9;
K = array([0.1, 0.1, 0.1]);
Kd = array([-0.1, -0.1, -0.1]);
ac = AttitudeController(AttitudeControllerParam(I, K, Kd));

m = 2.1;
K = array([-5, -5, -5]);
Kd = array([-3, -3, -3]);
pc = PositionController(PositionControllerParam(m, K, Kd));

r1 = 0.0667;
r2 = 0.0667;
d1 = 0.1100;
d2 = 0.1334;
d3 = 0.2;


recoder = PositionRecoder();
while True:
    t = time.time();
    v = reader.read();
    result = solver.solve(v);
    c = result.getPosition();
    Q = result.getDcm();
    recoder.add(t, c, Q);

    v = recoder.getVelocity();
    omega = recoder.getAngularVelocity();


    if v != None:
        pc.getControlForce(c, v);
        print(v);
    #ac.getControlTorque(Q, omega);
