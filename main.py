from numpy import *;
import scipy.linalg;
import serial;
import time;

from positioning import *;
from controller import *;
from actuator import *;

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
K = array([0.1, 0.1, 0.5]);
Kd = array([-0.1, -0.1, -0.1]);
Ki = array([0.01, 0.01, 0.02]);
ac = AttitudeController(AttitudeControllerParam(I, K, Kd, Ki));

m = 2.1;
K = array([-10, -10, -10]);
Kd = array([-5, -5, -5]);
Ki = array([0, 0, 0]);
pc = PositionController(PositionControllerParam(m, K, Kd, Ki));

r1 = 0.0667;
r2 = 0.0667;
d1 = 0.1100;
d2 = 0.1334;
d3 = 0.2;

c0 = array([0.031,  0.032,  0.031]);
r = array([0.022, -0.089, 0.06]);
xt = c0 + r;

recoder = PositionRecoder();

Fc = array([1, 1, 1, 1, 1, 1]);
actuator = Actuator(0x6f, 0x61, ActuatorParam(Fc));

r1 = 0.0667;
r2 = 0.0667;
d1 = 0.1100;
d2 = 0.1334;
d3 = 0.05;

converter = IOConverter(IOConverterParam(r1, r2, d1, d2, d3));

v0 = [0.030, 0.030, 0.030, 0.030, 0.030, 0.030];
while True:
    t = time.time();
    v = reader.read();
    for i in range(0, len(v)):
        if v[i] > 0.129:
            v[i] = v0[i];
    v0 = v;
    result = solver.solve(v);

    c = result.getPosition();
    Q = result.getDcm();
    
    xc = c + Q.T.dot(r);
    recoder.add(t, xc, Q);
    vc = recoder.getVelocity();
    omega = recoder.getAngularVelocity();
    
    if v != None and omega != None:
        F = pc.getControlForce(xt, xc, vc);
        T = ac.getControlTorque(Q, omega, t);
        #T[2] += 0.016;
        print(T);
        Fc = converter.getInput(F, T);
        actuator.execute(Fc);
