from numpy import *;
import scipy.integrate;

class PositionControllerParam:
    def __init__(self, m, K, Kd):
        self.m = m;
        self.K = K;
        self.Kd = Kd;
    def getMass(self):
        return self.m;
    def getK(self):
        return self.K;
    def getKd(self):
        return self.Kd;

class AttitudeControllerParam:
    def __init__(self, I, K, Kd):
        self.I = I;
        self.K = K;
        self.Kd = Kd;
    def getInertia(self):
        return self.I;
    def getK(self):
        return self.K;
    def getKd(self):
        return self.Kd;

class PositionController:
    def __init__(self, param):
        self.param = param;
    def getControlForce(self, xc, vc):
        K = self.param.getK();
        Kd = self.param.getKd();
        F = xc * K + vc * Kd;
        return F;

class AttitudeController:
    def __init__(self, param):
        self.param = param;
    def getControlTorque(self, dcm, omega):
        K = self.param.getK();
        Kd = self.param.getKd();
        e = array([dcm[2][1] - dcm[1][2], dcm[0][2] - dcm[2][1], dcm[1][0] - dcm[0][1]]);
        T = 0.5 * e * K + omega * Kd;
        return T;

class IOConverterParam:
    def __init__(self, r1, r2, d1, d2, d3):
        self.r1 = r1;
        self.r2 = r2;
        self.d1 = d1;
        self.d2 = d2;
        self.d3 = d3;

class IOConverter:
    def __init__(self, param):
        self.param = param;

    def getInput(self, F, T):
        O = zeros(6);
        O[0:3] = F;
        O[3:6] = T;
        I = eye(6).dot(O);
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

    r1 = 1;
    r2 = 1;
    d1 = 1;
    d2 = 1;
    d3 = 1;

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

    y0 = zeros(12);
    for i in range(0, 3):
        y0[(i * 3):(i * 3 + 3)] = dcm0[i];
    y0[9:12] = omega0;

    print(scipy.integrate.odeint(f2, y0, linspace(0, 15, 101)));

    converter = IOConverter(IOConverterParam(r1, r2, d1, d2, d3));
    #print(converter.getInput(K, Kd));
