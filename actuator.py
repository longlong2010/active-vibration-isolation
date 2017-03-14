from Raspi_MotorHAT.Raspi_MotorHAT import Raspi_MotorHAT, Raspi_DCMotor;

from numpy import *;
import atexit;
import math;

class ActuatorParam:
    def __init__(self, Fc):
        self.Fc = Fc;
    def getCalibration(self):
        return self.Fc;

class Actuator:
    def __init__(self, addr1, addr2, param):
        self.param = param;
        self.h1 = Raspi_MotorHAT(addr = addr1); 
        #self.h2 = Raspi_MotorHAT(addr = addr2);
        def onExit():
            for i in range(0, 4):
                self.h1.getMotor(i + 1).run(Raspi_MotorHAT.RELEASE);
                #self.h2.getMotor(i + 1).run(Raspi_MotorHAT.RELEASE);
        atexit.register(onExit);

    def execute(self, F):
        Fc = self.param.getCalibration();
        for i in range(1, 4):
            m = self.h1.getMotor(i);
            k = (i - 1) * 2;
            if F[k] > 0:
                m.run(Raspi_MotorHAT.FORWARD);
            else:
                m.run(Raspi_MotorHAT.BACKWARD);
            m.setSpeed(int(abs(F[k]) / Fc[k] * 255));

if __name__ == '__main__':
    Fc = array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5]);
    act = Actuator(0x6f, 0x61, ActuatorParam(Fc));
    act.execute(array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]));
