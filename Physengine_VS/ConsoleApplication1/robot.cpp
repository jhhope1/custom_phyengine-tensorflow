#include "physengine.h"
void robot::setalpha() {
	//fl.pri.mtalpha = ~~~~
}
void robot::setForce() {
}
void calculate_all_alpha();

void calculate_all_alpha() {
	
}
void robot::timeflow() {
	setalpha();
	setForce();
	Quat qsub[3], qdot;
	Mat33 Qsub[3], Qdot;
	Vector l[3], omega[3], e[3], Qalpha[3], lbtomot[3], lsum[3];
	qdot = Mat44(body.w.V[0], body.w.V[1], body.w.V[2]) * body.q *0.5L;
	Qdot = Mat33(
		body.q.q[0] * qdot.q[0] + body.q.q[1] * qdot.q[1] - body.q.q[2] * qdot.q[2] - body.q.q[3] * qdot.q[3], qdot.q[1] * body.q.q[2] + body.q.q[1] * qdot.q[2] - qdot.q[0] * body.q.q[3] - body.q.q[0] * qdot.q[3], qdot.q[1] * body.q.q[3] + body.q.q[1] * qdot.q[3] + qdot.q[0] * body.q.q[2] + body.q.q[0] * qdot.q[2],
		qdot.q[1] * body.q.q[2] + body.q.q[1] * qdot.q[2] + qdot.q[0] * body.q.q[3] + body.q.q[0] * qdot.q[3], body.q.q[0] * qdot.q[0] - body.q.q[1] * qdot.q[1] + body.q.q[2] * qdot.q[2] - body.q.q[3] * qdot.q[3], qdot.q[2] * body.q.q[3] + body.q.q[2] * qdot.q[3] - qdot.q[0] * body.q.q[1] - body.q.q[0] * qdot.q[1],
		qdot.q[1] * body.q.q[3] + body.q.q[1] * qdot.q[3] - qdot.q[0] * body.q.q[2] - body.q.q[0] * qdot.q[2], qdot.q[2] * body.q.q[3] + body.q.q[2] * qdot.q[3] + qdot.q[0] * body.q.q[1] + body.q.q[0] * qdot.q[1], body.q.q[0] * qdot.q[0] - body.q.q[1] * qdot.q[1] - body.q.q[2] * qdot.q[2] + body.q.q[3] * qdot.q[3]
	)*2.L;
	Leg L;
	for (int legnum = 0; legnum < 4; legnum++) {
		L = *leg[legnum];
		qsub[0] = body.q*L.sub[0].q;
		for (int i = 1; i < 3; i++) qsub[i] = qsub[i - 1] * L.sub[i].q;
		for (int i = 0; i < 3; i++) Qsub[i] = qsub[i].toRot();
		for (int i = 0; i < 3; i++) l[i] = Qsub[i] * L.sub[i].l;
		for (int i = 0; i < 3; i++) e[i] = Qsub[i] * L.sub[i].axis;
		for (int i = 0; i < 3; i++) omega[i] = e[i] * L.sub[i].omega;
		for (int i = 0; i < 3; i++) Qalpha[i] = e[i] * L.sub[i].alpha;
		lbtomot[0] = body.lbtomot[legnum]+l[0]*0.5L;
		for (int i = 1; i < 3; i++)	lbtomot[i] = lbtomot[i - 1] + (l[i] + l[i - 1])*0.5L;
		lsum[0] = l[0];
		for (int i = 1; i < 3; i++) lsum[i] = lsum[i - 1] + l[i];
	}
}