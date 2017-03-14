from numpy import *;
import scipy.linalg;
import serial;
import math;
import time;

class PositionParam:
    def __init__(self, coordinates):
        self.coordinates = coordinates;
    def getCoordinate(self, num):
        return self.coordinates[num];

class PositionResult:
    def __init__(self, xc, Q):
        self.xc = xc;
        self.Q = Q;
    def getPosition(self):
        return self.xc;
    def getDcm(self):
        return self.Q;

class PositionRecoder:
    def __init__(self):
        self.queue = [];
        #self.fp = open("DATA", "w");
    def add(self, t, r, Q):
        #self.fp.write("%.6f,%s,%s,%s,%s\n" % (t, str(r), str(Q[0]), str(Q[1]), str(Q[2])));
        self.queue.append((t, r, Q));
        if len(self.queue) > 100:
            del self.queue[0];
    def getVelocity(self):
        n = len(self.queue) - 1;
        if n > 1:
            r1 = self.queue[n - 1];
            r2 = self.queue[n];
            dt = r2[0] - r1[0];
            dr = r2[1] - r1[1];
            return dr / dt;
    def getAcceleration(self):
        n = len(self.queue) - 1;
        if n > 2:
            r1 = self.queue[n - 2];
            r2 = self.queue[n - 1];
            r3 = self.queue[n];
            dt1 = r2[0] - r1[0];
            dt2 = r3[0] - r2[0];
            dr1 = r2[1] - r1[1];
            dr2 = r3[1] - r2[1];

            v1 = dr1 / dt1;
            v2 = dr2 / dt2;

            dt = r3[0] - r1[0];
            dv = v2 - v1;

            return dv / dt;
    def getAngularVelocity(self):
        n = len(self.queue) - 1;
        if n > 1:
            r1 = self.queue[n - 1];
            r2 = self.queue[n];
            Q = r1[2];
            dQ = r2[2] - r1[2];
            dt = r2[0] - r1[0];
            return (dQ / dt).dot(Q.T);


class PositionSolver:
    def __init__(self, param):
        self.param = param;
    def solve(self, v):
        [x1, y1] = self.param.getCoordinate(0);
        [x2, y2] = self.param.getCoordinate(1);
        [x3, y3] = self.param.getCoordinate(2);
        [x4, z4] = self.param.getCoordinate(3);
        [x5, z5] = self.param.getCoordinate(4);
        [y6, z6] = self.param.getCoordinate(5);
        A = array([[x1, y1, 1], [x2, y2, 1], [x3, y3, 1]]);
        b = array([[v[0]], [v[1]], [v[2]]]);
        
        r = linalg.solve(A, b);
        A1 = r[0][0];
        B1 = r[1][0];
        C1 = r[2][0];

        A = array([[x4, z4, 1], [x5, z5, 1], [A1, -1, 0]]);
        b = array([[v[3]], [v[4]], [B1]]);
        
        r = linalg.solve(A, b);
        A2 = r[0][0];
        B2 = r[1][0];
        C2 = r[2][0];

        A = array([[y6, z6, 1], [B1, -1, 0], [-1, B2, 0]]);
        b = array([[v[5]], [A1], [A2]]);

        r = linalg.solve(A, b);
        A3 = r[0][0];
        B3 = r[1][0];
        C3 = r[2][0];

        A = array([[A1, B1, -1], [A2, -1, B2], [-1, A3, B3]]);
        b = array([[-C1], [-C2], [-C3]]);
        
        r = linalg.solve(A, b);
        xc = r[0][0];
        yc = r[1][0];
        zc = r[2][0];

        e1 = array([1, -A3, -B3]);
        e1 = e1 / linalg.norm(e1);
        e2 = array([-A2, 1, -B2]);
        e2 = e2 / linalg.norm(e2);
        e3 = array([-A1, -B1, 1]);
        e3 = e3 / linalg.norm(e3);

        Q = array([e1, e2, e3]);
        return PositionResult(array([xc, yc, zc]), Q);

class SensorReader:
    def __init__(self, port, baudrate):
        self.serial = serial.Serial(port, baudrate, timeout=1);
    def read(self):
        self.serial.write("M0\r\n");
        data = self.serial.readline().rstrip("\r\n").lstrip("M0,").split(",");
        for i in range(0, 6):
            data[i] = (30.0 - float(data[i])) / 1000;
        return data;

if __name__ == '__main__':
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

    #v = [0.0006, -0.0052, -0.0004, 0.0096, 0.0034, -0.0005];

    recoder = PositionRecoder();
    while True:
        t = time.time();
        v = reader.read();
        result = solver.solve(v);
        #print(result);
        c = result.getPosition();
        Q = result.getDcm();
        #print "%.3f\t%.3f\t%.3f" % (c[0] * 1000, c[1] * 1000, c[2] * 1000);
        #print(c);
        #print(Q);
        time.sleep(1);
        psi = -math.atan(Q[1][0] / Q[1][1]);
        phi = math.asin(Q[1][2]);
        print "%f\t%f" % (psi * 180 / math.pi * 60, phi * 180 / math.pi * 60); 
        #print(Q * Q.T);
        #recoder.add(t, c, Q);
        #print(recoder.getVelocity());
        #print(recoder.getAcceleration());
        #print(recoder.getAngularVelocity());
