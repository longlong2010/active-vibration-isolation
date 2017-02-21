from numpy import *;

class AttitudeControllerParam:
	def	__init__(self, I, K, Kd):
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
		pass;
	def getControlForce(xc, vc):
		pass;

class AttitudeController:
	def __init__(self, param):
		self.param = param;

	def getControlTorque(self, dcm, omega):
		K = self.param.getK();
		Kd = self.param.getKd();
		dcme = array([dcm[2][1] - dcm[1][2], dcm[0][2] - dcm[2][1], dcm[1][0] - dcm[0][1]]);
		T = 0.5 * dcme * K + omega * Kd;
		return T;

if __name__ == '__main__':
	I = array([11581946.33, 18047496.42, 29576720.71]) * 1e-9;
	K = array([0.1, 0.1, 0.1]);
	Kd = array([-0.1, -0.1, -0.1]);

	acp = AttitudeControllerParam(I, K, Kd);
	ac = AttitudeController(acp);

	dcm = array([[0.9980, -0.0517, -0.0358], [0.0523, 0.9985, 0.0156], [0.0349, -0.0174, 0.9992]]);
	omega = array([0, 0, 0]);

	print(ac.getControlTorque(dcm, omega));
