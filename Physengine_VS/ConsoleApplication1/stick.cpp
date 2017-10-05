#include "physengine.h"
//constants for the stick
const phys M = 10;
const phys h = 0.5;
const phys R = 0.01;

void Stick::initstick() {
	Stick::stick.Ibdia = Vector(0.5L*M*R*R, 1.L / 12.L*M*h*h + 1.L / 4.L*M*R*R, 1.L / 12.L*M*h*h + 1.L / 4.L*M*R*R);
	Stick::stick.m = M;
	Stick::stick.q = Quat(0.L, 1L, 0.L, 0.L);
	Stick::stick.tau = Vector(0.L, -0.L, 0.L);
	Stick::stick.r = Vector();
	Stick::stick.euler = Vector(0.L, 0.L, 0.L);
	Stick::stick.w = Vector(0.5L, 0.1L, 0.6L);
	Stick::stick.F = (0.L, 0.L, 0.L);
}
void Stick::flyingstick() {
	Quat q = Stick::stick.q;//, qi = Stick::stick.qi;
					 //Vector taub = (q*Quat(0,taub)*qi).ext();
					 //Vector wb = (q*Quat(0,wb)*qi).ext();
	Mat33 rotI = q.toRot();
	Mat33 rotIinv = rotI.inv();
	Vector taub = rotIinv*Stick::stick.tau;
	Vector wb = rotIinv*Stick::stick.w;

	Quat dq = (Mat44(wb.V[0], wb.V[1], wb.V[2])*q)*0.5L;

	Stick::stick.q = q + dq*dtime;
	//Stick::stick.q.normalize();
	phys SIbdia[3] = { Stick::stick.Ibdia.V[0], Stick::stick.Ibdia.V[1],Stick::stick.Ibdia.V[2] };
	Vector alphab = Vector((SIbdia[1] - SIbdia[2]) / SIbdia[0] * wb.V[1]*wb.V[2] + taub.V[0] / SIbdia[0],
		(SIbdia[2] - SIbdia[0]) / SIbdia[1] * wb.V[2]*wb.V[0] + taub.V[1] / SIbdia[1],
		(SIbdia[0] - SIbdia[1]) / SIbdia[2] * wb.V[0]*wb.V[1] + taub.V[2] / SIbdia[2]);//paper's formula 6

	wb = wb + alphab*dtime;
	Stick::stick.w = rotI*wb;//여기에서 한번 더 torot을 해주어야하는지 의문이다.

	Stick::stick.v = Stick::stick.v + Stick::stick.F*(1 / Stick::stick.m)*dtime;
	Stick::stick.r = Stick::stick.r + Stick::stick.v*dtime;
}

void Stick::debugstick() {
	ios_base::sync_with_stdio(false); cin.tie(0); cout.tie(0);
	initstick();
	Vector stickendinit = Vector(-1.0L, 0.0L, 0.0L);
	Vector stickendnow = Vector(0, 0, 0);
	for (int i = 0; i < 5000000; i++) {
		if (i % 50000 != 0) continue;
		Mat33 Qrot = Stick::stick.q.toRot();
		stickendnow = Qrot.inv()*stickendinit;
		cout << "quat = " << Stick::stick.q << "norm = " << Stick::stick.q.norm() << '\n';
		cout << "time = " << dtime*(i + 1) << '\n';
		cout << "r = " << Stick::stick.r << '\n';
		cout << "initial position of stick : (1,0,0), now : " << stickendnow << '\n';
		cout << "w = " << Stick::stick.w << '\n';
		Quat Qs = Stick::stick.q;
		Mat33 QsRot = Qs.toRot(), QiRot = QsRot.inv();
		Vector Angmom = (QsRot%Stick::stick.Ibdia)*QiRot*Stick::stick.w;
		cout << "Angular momentum = " << Angmom << ", |L| = " << Angmom.norm() << '\n';
		cout << "-------------------------------------" << '\n';
		flyingstick();
	}
}