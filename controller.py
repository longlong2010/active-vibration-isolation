from numpy import *;
import scipy.integrate;
import math;

class PositionControllerParam:
    def __init__(self, m, K, Kd, Ki):
        self.m = m;
        self.K = K;
        self.Kd = Kd;
        self.Ki = Ki;
    def getMass(self):
        return self.m;
    def getK(self):
        return self.K;
    def getKd(self):
        return self.Kd;
    def getKi(self):
        return self.Ki;

class AttitudeControllerParam:
    def __init__(self, I, K, Kd, Ki):
        self.I = I;
        self.K = K;
        self.Kd = Kd;
        self.Ki = Ki;
    def getInertia(self):
        return self.I;
    def getK(self):
        return self.K;
    def getKd(self):
        return self.Kd;
    def getKi(self):
        return self.Ki;

class PositionController:
    def __init__(self, param):
        self.param = param;
        self.integrator = Integrator(array([0.0, 0.0, 0.0]));
    def getControlForce(self, xt, xc, vc, t):
        K = self.param.getK();
        Kd = self.param.getKd();
        Ki = self.param.getKi();
        xe = xc - xt;
        Ie = self.integrator.integrate(xe, t);
        F = xe * K + Ie * Ki + vc * Kd;
        return F;

class AttitudeController:
    def __init__(self, param):
        self.param = param;
        self.integrator = Integrator(array([0.0, 0.0, 0.0]));
    def getControlTorque(self, dcm, omega, t):
        K = self.param.getK();
        Kd = self.param.getKd();
        Ki = self.param.getKi();
        e = array([dcm[2][1] - dcm[1][2], dcm[0][2] - dcm[2][1], dcm[1][0] - dcm[0][1]]);
        Ie = self.integrator.integrate(e, t);
        T = 0.5 * (e * K + Ie * Ki) + omega * Kd;
        return T;

class IOConverterParam:
    def __init__(self, r1, r2, d1, d2, d3):
        Tm = zeros((6, 6));
        rt3 = math.sqrt(3);
        
        Tm[0] = array([(rt3 * r2 - d1 / 2) / (rt3 * d2 - d1 / 2), 0, 0, 0, 0, -rt3 / (rt3 * d2 - d1 / 2)]);
        Tm[1] = array([0, -d3 / d2, r2 / d2, 1 / d2, 0, 0]);
        Tm[2] = array([r1 / (rt3 * d2 - d1 / 2), 1, 0, 0, 0, 1 / (rt3 * d2 - d1 / 2)]);
        Tm[3] = array([d3 / d1, d3 / (2 * d2), r1 / (2 * d2), -1 / (2 * d2), 1 / d1, 0]);
        Tm[4] = array([r1 / (rt3 * d2 - d1 / 2), -1, 0, 0, 0, 1 / (rt3 * d2 - d1 / 2)]);
        Tm[5] = array([-d3 / d1, d3 / (2 * d2), r1 / (2 * d2), -1 / (2 * d2), -1 / d1, 0]);

        self.Tm = Tm;

    def getTransitionMatrix(self):
        return self.Tm;

class Integrator:
    def __init__(self, u = 0, v = None, t = None):
        self.v = v;
        self.t = t;
        self.u = u;
    def integrate(self, v, t):
        if self.t != None:
            dt = t - self.t;
            self.u += (v + self.v) / 2 * dt;
        self.t = t;
        self.v = v;
        return self.u;

class IOConverter:
    def __init__(self, param):
        self.param = param;

    def getInput(self, F, T):
        O = zeros(6);
        O[0:3] = F;
        O[3:6] = T;
        I = self.param.getTransitionMatrix().dot(O);
        return I;

if __name__ == '__main__':
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

    xc0 = array([0.01, 0.02, -0.03]);
    vc0 = array([0, 0, 0]);
    dcm0 = array([[0.9980, -0.0517, -0.0358], [0.0523, 0.9985, 0.0156], [0.0349, -0.0174, 0.9992]]);
    omega0 = array([0, 0, 0]);

    print(ac.getControlTorque(dcm0, omega0));
    print(pc.getControlForce(xc0, vc0));
    
    y0 = zeros(6);
    y0[0:3] = xc0;
    y0[3:6] = vc0;

    def f1(y, t):
        ydot = zeros(6);
        ydot[0:3] = y[3:6];
        ydot[3:6] = pc.getControlForce(y[0:3], y[3:6]) / m;
        return ydot;
    print(scipy.integrate.odeint(f1, y0, linspace(0, 15, 101)));

    y0 = zeros(12);
    for i in range(0, 3):
        y0[(i * 3):(i * 3 + 3)] = dcm0[i];
    y0[9:12] = omega0;

    def f2(y, t):
        ydot = zeros(12);
        dcm = zeros((3, 3));
        omegadot = zeros(3);

        for i in range(0, 3):
            dcm[i] = y[(i * 3):(i * 3 + 3)];
        omega = y[9:12];

        dcmdot = array([[0, omega[2], -omega[1]], [-omega[2], 0, omega[0]], [omega[1], -omega[0], 0]]).dot(dcm);
        
        T = ac.getControlTorque(dcm, omega);

        omegadot[0] = (T[0] + (I[1] - I[2]) * omega[1] * omega[2]) / I[0];
        omegadot[1] = (T[1] + (I[2] - I[0]) * omega[0] * omega[2]) / I[1];
        omegadot[2] = (T[2] + (I[0] - I[1]) * omega[0] * omega[1]) / I[2];

        for i in range(0, 3):
            ydot[(i * 3):(i * 3 + 3)] = dcmdot[i];

        ydot[9:12] = omegadot;
        return ydot;
    print(scipy.integrate.odeint(f2, y0, linspace(0, 15, 101)));

    converter = IOConverter(IOConverterParam(r1, r2, d1, d2, d3));
    #print(converter.getInput(K, Kd));
