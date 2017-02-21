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
        for i in range(0, 2):
            m1 = self.h1.getMotor(i + 1);
            if F[i] > 0:
                m1.run(Raspi_MotorHAT.FORWARD);
            else:
                m1.run(Raspi_MotorHAT.BACKWARD);

            m1.setSpeed(int(F[i] / Fc[i] * 255));

            #m2 = self.h2.getMotor(i + 1);
            #if F[i + 2] > 0:
            #    m2.run(Raspi_MotorHAT.FORWARD);
            #else:
            #    m2.run(Raspi_MotorHAT.BACKWARD);
            #
            #m2.setSpeed(int(F[i + 2] / Fc[i + 2] * 255));
        
        m1 = self.h1.getMotor(3);
        if F[4] > 0:
            m1.run(Raspi_MotorHAT.FORWARD);
        else:
            m1.run(Raspi_MotorHAT.BACKWARD);
       
        m1.setSpeed(int(F[4] / Fc[i] * 255));

        #m2 = self.h2.getMotor(3);
        #if F[5] > 0:
        #    m2.run(Raspi_MotorHAT.FORWARD);
        #else:
        #    m2.run(Raspi_MotorHAT.BACKWARD);
        #m1.setSpeed(int(F[5] / Fc[i] * 255));

if __name__ == '__main__':
    Fc = array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0]);
    act = Actuator(0x6f, 0x61, ActuatorParam(Fc));
    act.execute(array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]));
