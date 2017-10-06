#include "physengine.h"
void robot::setalpha() {
	//fl.pri.mtalpha = ~~~~
}
void robot::setForce() {
}
void robot::setMass() {
	Mtot = body.m;
	for (int i = 0; i < numLeg; i++) {
		for (int j = 0; j < numsubleg; j++) {
			Mtot += leg[i].sub[j].m;
		}
	}
}
void robot::timeflow() {
	setalpha();
	setForce();
	Quat qsub[numsubleg], qdot;
	Mat33 
		Qb, Qsub[numsubleg], Qdot,
		D,
		Feqa = Matdia(Mtot), Feqalpha,
		Teqa, Teqalpha,
		IQsum[numsubleg];
	Vector
		l[numsubleg][2], lbtomot[numsubleg], lsum[numsubleg], lbtomotb,
		wsb, ws[numsubleg], e[numsubleg], Qalpha[numsubleg],
		w[numsubleg],
		A[numsubleg],
		Feqc = Fg * Mtot,
		sumML,
		Teqc;
	qdot = Mat44(body.w.V[0], body.w.V[1], body.w.V[2]) * body.q *0.5L;
	Qdot = Mat33(
		body.q.q[0] * qdot.q[0] + body.q.q[1] * qdot.q[1] - body.q.q[2] * qdot.q[2] - body.q.q[3] * qdot.q[3], qdot.q[1] * body.q.q[2] + body.q.q[1] * qdot.q[2] - qdot.q[0] * body.q.q[3] - body.q.q[0] * qdot.q[3], qdot.q[1] * body.q.q[3] + body.q.q[1] * qdot.q[3] + qdot.q[0] * body.q.q[2] + body.q.q[0] * qdot.q[2],
		qdot.q[1] * body.q.q[2] + body.q.q[1] * qdot.q[2] + qdot.q[0] * body.q.q[3] + body.q.q[0] * qdot.q[3], body.q.q[0] * qdot.q[0] - body.q.q[1] * qdot.q[1] + body.q.q[2] * qdot.q[2] - body.q.q[3] * qdot.q[3], qdot.q[2] * body.q.q[3] + body.q.q[2] * qdot.q[3] - qdot.q[0] * body.q.q[1] - body.q.q[0] * qdot.q[1],
		qdot.q[1] * body.q.q[3] + body.q.q[1] * qdot.q[3] - qdot.q[0] * body.q.q[2] - body.q.q[0] * qdot.q[2], qdot.q[2] * body.q.q[3] + body.q.q[2] * qdot.q[3] + qdot.q[0] * body.q.q[1] + body.q.q[0] * qdot.q[1], body.q.q[0] * qdot.q[0] - body.q.q[1] * qdot.q[1] - body.q.q[2] * qdot.q[2] + body.q.q[3] * qdot.q[3]
	)*2.L;
	Leg L;
	/*
	index -1 : body의 물리량
	qsub[-1] : body.q
	Qsub[-1] : body.q.toRot()
	Qalpha[-1] : 0
	l[-1] : Qb * body.lbtomot[legnum]
	*/
	Qb = body.q.toRot();
	for (int legnum = 0; legnum < numLeg; legnum++) {
		L = leg[legnum];
		qsub[0] = body.q*L.sub[0].q;
		wsb = Qb * body.w;
		lbtomotb = Qb * body.lbtomot[legnum];

		for (int i = 1; i < numsubleg; i++) qsub[i] = qsub[i - 1] * L.sub[i].q;
		for (int i = 0; i < numsubleg; i++) Qsub[i] = qsub[i].toRot();

		for (int b = 0; b < 2; b++)
			for (int i = 0; i < numsubleg; i++)
				l[i][b] = Qsub[i] * L.sub[i].l[b];
			
		lbtomot[legnum] = lbtomotb + l[0][0];
		
		IQsum[0] = Mat33(
			L.sub[0].Ibdia.V[0] * Qsub[0].mat[0][0], L.sub[0].Ibdia.V[1] * Qsub[0].mat[1][0], L.sub[0].Ibdia.V[2] * Qsub[0].mat[2][0],
			L.sub[0].Ibdia.V[0] * Qsub[0].mat[0][1], L.sub[0].Ibdia.V[1] * Qsub[0].mat[1][1], L.sub[0].Ibdia.V[2] * Qsub[0].mat[2][1],
			L.sub[0].Ibdia.V[0] * Qsub[0].mat[0][2], L.sub[0].Ibdia.V[1] * Qsub[0].mat[1][2], L.sub[0].Ibdia.V[2] * Qsub[0].mat[2][2]
		);
		for (int i = 1; i < numsubleg; i++) {
			IQsum[i] = IQsum[i-1] + Mat33(
				L.sub[i].Ibdia.V[0] * Qsub[i].mat[0][0], L.sub[i].Ibdia.V[1] * Qsub[i].mat[1][0], L.sub[i].Ibdia.V[2] * Qsub[i].mat[2][0],
				L.sub[i].Ibdia.V[0] * Qsub[i].mat[0][1], L.sub[i].Ibdia.V[1] * Qsub[i].mat[1][1], L.sub[i].Ibdia.V[2] * Qsub[i].mat[2][1],
				L.sub[i].Ibdia.V[0] * Qsub[i].mat[0][2], L.sub[i].Ibdia.V[1] * Qsub[i].mat[1][2], L.sub[i].Ibdia.V[2] * Qsub[i].mat[2][2]
			);
		}
		for (int i = 0; i < numsubleg; i++) e[i] = Qsub[i] * L.sub[i].axis;

		ws[0] = wsb + e[0] * L.sub[0].omega;
		for (int i = 1; i < numsubleg; i++) ws[i] = ws[i - 1] + e[i] * L.sub[i].omega;
		for (int i = 0; i < numsubleg; i++) w[i] = Qsub[i].transpose()*ws[i];

		for (int i = 0; i < numsubleg; i++) Qalpha[i] = e[i] * L.sub[i].alpha;

		for (int i = 1; i < numsubleg; i++)	
			lbtomot[i] = lbtomot[i - 1] + l[i][0] + l[i - 1][1];

		phys Ddia = 0;
		Mat33 Dtemp;
		for (int i = 0; i < numsubleg; i++) {
			Ddia += L.sub[i].m * lbtomot[i].normsq();
			Dtemp = Dtemp + dyadic(lbtomot[i])*L.sub[i].m;
		}
		A[0] = wsb * (wsb * lbtomotb)
			+ Qalpha[0] * l[0][0] + ws[0] * (ws[0] * l[0][0]);
		for (int i = 1; i < numsubleg; i++) {
			A[i] = A[i - 1] +
				(Qalpha[i - 1] * l[i - 1][1] + ws[i - 1] * (ws[i - 1] * l[i - 1][1])
					+ Qalpha[i] * l[i][0] + ws[i] * (ws[i] * l[i][0])
					);
		}
		D = Matdia(Ddia) - Dtemp;
		//Feqc = Feqc + F.F;
		for (int i = 0; i < numsubleg; i++) Feqc = Feqc - A[i] * L.sub[i].m;
		for (int i = 0; i < numsubleg; i++) sumML = sumML + lbtomot[i] * L.sub[i].m;
		//Teqc = Teqc + (body.r-F.r)*F.F;
		for (int i = 0; i < numsubleg; i++) {
			Teqc = Teqc +
				Vector(
					(L.sub[i].Ibdia.V[1] - L.sub[i].Ibdia.V[2])*w[i].V[1] * w[i].V[2],
					(L.sub[i].Ibdia.V[2] - L.sub[i].Ibdia.V[0])*w[i].V[2] * w[i].V[0],
					(L.sub[i].Ibdia.V[0] - L.sub[i].Ibdia.V[1])*w[i].V[0] * w[i].V[1]
				);
		}

		for (int i = 0; i < numsubleg; i++) {
			Teqc = Teqc - (IQsum[numsubleg - 1] - (i ? IQsum[i - 1] : Mat33()))* Qalpha[i];
		}
		Teqalpha = Teqalpha + D;
		Teqalpha = Teqalpha + IQsum[numsubleg - 1];
	}
	Feqalpha = Matskew(sumML*(-1.L));
	Teqc = Teqc + sumML*Fg;
	Teqc = Teqc - (Qdot%body.Ibdia)*body.w;
	Teqc = Teqc - (Qb%body.Ibdia)*Qdot.transpose()*wsb;
	Teqalpha = Teqalpha + (Qb%body.Ibdia)*Qb.transpose();
	Teqa = Matskew(sumML);
}
robot::robot() {
	body = Robotbody();
	for (int i = 0; i < numLeg; i++) leg[i] = Leg();
	setMass();
}

Leg::Leg() {
	for (int i = 0; i < numsubleg; i++) {
		sub[i] = subleg();
	}
}

subleg::subleg() {
	
}

Robotbody::Robotbody() {

}