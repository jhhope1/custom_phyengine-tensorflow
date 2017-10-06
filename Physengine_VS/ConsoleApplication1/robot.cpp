#include "physengine.h"
void robot::setalpha() {
	/*냉무*/
}
void robot::setMass() {
	Mtot = body.m;
	for (int i = 0; i < numLeg; i++) {
		for (int j = 0; j < numsubleg; j++) {
			Mtot += leg[i].sub[j].m;
		}
	}
}
pair<Vector,Vector> robot::timeflow() {
	setalpha();
	Flist.clear();
	Quat qsub[numsubleg], qdot;
	Mat33 
		Qb, Qsub[numsubleg], Qdot,
		D,
		Feqa = Matdia(Mtot), Feqalpha,
		Teqa, Teqalpha,
		IQsum[numsubleg];
	Vector
		l[numsubleg][2], lbtomots[numsubleg], lbtomotb,
		wsb, ws[numsubleg], e[numsubleg], Qalpha[numsubleg],
		w[numsubleg],
		A[numsubleg],
		Feqc = Fg * Mtot,
		sumML,
		Teqc,
		asb, alphasb;
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
			
		lbtomots[legnum] = lbtomotb + l[0][0];
		
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
		
		for (int i = 0; i < numsubleg; i++) {
			cout << "L.sub[" << i << "].omega of leg " << legnum << " = " << L.sub[i].omega << endl;
		}
		
		ws[0] = wsb + e[0] * L.sub[0].omega;
		for (int i = 1; i < numsubleg; i++) ws[i] = ws[i - 1] + e[i] * L.sub[i].omega;
		for (int i = 0; i < numsubleg; i++) w[i] = Qsub[i].transpose()*ws[i];

		for (int i = 0; i < numsubleg; i++) Qalpha[i] = e[i] * L.sub[i].alpha;
		
		for (int i = 0; i < numsubleg; i++) {
			cout << "Qalpha[" << i << "] of leg " << legnum << " = " << Qalpha[i] << endl;
		}
		
		for (int i = 1; i < numsubleg; i++)	
			lbtomots[i] = lbtomots[i - 1] + l[i][0] + l[i - 1][1];

		phys Ddia = 0;
		Mat33 Dtemp;
		for (int i = 0; i < numsubleg; i++) {
			Ddia += L.sub[i].m * lbtomots[i].normsq();
			Dtemp = Dtemp + dyadic(lbtomots[i])*L.sub[i].m;
		}
		A[0] = wsb * (wsb * lbtomotb)
			+ Qalpha[0] * l[0][0] + ws[0] * (ws[0] * l[0][0]);
		for (int i = 1; i < numsubleg; i++) {
			A[i] = A[i - 1] +
				(Qalpha[i - 1] * l[i - 1][1] + ws[i - 1] * (ws[i - 1] * l[i - 1][1])
					+ Qalpha[i] * l[i][0] + ws[i] * (ws[i] * l[i][0])
					);
		}
		for (int i = 0; i < numsubleg; i++) {
			cout << "A[" << i << "] of leg " << legnum << " = " << A[i] << endl;
		}
		D = Matdia(Ddia) - Dtemp;

		//Feqc = Feqc + F.F;
		for (int i = 0; i < numsubleg; i++) Feqc = Feqc - A[i] * L.sub[i].m;
		for (int i = 0; i < numsubleg; i++) sumML = sumML + lbtomots[i] * L.sub[i].m;
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

		//Update Quaternions
		for (int i = 0; i < numsubleg; i++) {
			L.sub[i].omega += L.sub[i].alpha * dtime;
			L.sub[i].theta += L.sub[i].omega * dtime; //오차?
			L.sub[i].q = Quat(cos(L.sub[i].theta / 2.L), L.sub[i].axis*sin(L.sub[i].theta / 2.L));
		}

		//set Force
		Vector vlegs = body.vs + wsb*body.lbtomot[legnum];
		Vector pos;
		for (int i = 0; i < numsubleg; i++) {
			vlegs = vlegs + ws[i] * (l[i][0] + l[i][1]);
			pos = body.rs + lbtomots[i] + l[i][1];
			if (pos.V[2] < 0) {
				if (vlegs.V[2] < 0) {
					Flist.push_back(Force(Vector(0,0,Fupscale*Mtot*g),pos));
				}
				else {
					Flist.push_back(Force(Vector(0, 0, Fdownscale*Mtot*g), pos));
				}
			}
			//frictions are ignored
		}
	}
	cout << "Mtot = " << Mtot << endl;
	cout << "Feqc_init = " << Feqc << endl;
	cout << "Teqc_init = " << Teqc << endl;
	for (Force &f : Flist) {
		Feqc = Feqc + f.F;
		Teqc = Teqc + (f.r - body.rs)*f.F;
	}
	cout << "Feqc_second = " << Feqc << endl;
	cout << "Teqc_second = " << Teqc << endl;
	Feqalpha = Matskew(sumML)*(-1.L);
	Teqc = Teqc + sumML*Fg;
	Teqc = Teqc - (Qdot%body.Ibdia)*body.w;
	Teqc = Teqc - (Qb%body.Ibdia)*Qdot.transpose()*wsb;
	Teqalpha = Teqalpha + (Qb%body.Ibdia)*Qb.transpose();
	Teqa = Matskew(sumML);

	//solving equation!!
	/*
	cout << "sumML = "<<sumML << endl;
	cout << "Feqc = " << Feqc << endl;
	cout << "Teqc = " << Teqc << endl;
	cout << "Feqalpha = " << endl << Feqalpha << endl << "Det = " << Feqalpha.det() << endl;
	cout << "Teqalpha = " << endl << Teqalpha << endl << "Det = " << Teqalpha.det() << endl;
	cout << "Feqa = " << endl << Feqa << endl << "Det = " << Feqa.det() << endl;
	cout << "Teqa = " << endl << Teqa << endl << "Det = " << Teqa.det() << endl;
	*/
	Mat33 tmpMat, debugMat;
	Vector TtF;
	if (abs(Feqalpha.det()) < 1e-10) {
		tmpMat = Teqa*Feqa.inv();
		/*
		cout << "succeeded! - tmpMat = " << endl << tmpMat << endl;
		debugMat = Teqalpha - tmpMat * Feqalpha;
		TtF = Teqc - (tmpMat * Feqc);
		cout << "debugMat = " << endl << debugMat << endl << "Det = " << debugMat.det() << endl;
		cout << "TtF = " << endl << TtF << endl;
		*/
		alphasb = (Teqalpha - tmpMat*Feqalpha).inv() * (Teqc - tmpMat * Feqc);
		//cout << "succeeded! - alphasb = " << alphasb << endl;
		asb = Feqa.inv() * (Feqc - Feqalpha * alphasb);
		//cout << "succeeded! - asb = " << asb << endl;
	}
	else {
		tmpMat = Teqalpha * Feqalpha.inv();
		cout << "succeeded! - tmpMat" << endl;
		asb = (Teqa - tmpMat * Feqa).inv() * (Teqc - tmpMat * Feqc);
		cout << "succeeded! - asb" << endl;
		alphasb = Feqalpha.inv() * (Feqc - Feqa * asb);
		cout << "succeeded! - alphasb" << endl;
	}
	//body v,w update
	body.vs = body.vs + asb*dtime;
	body.rs = body.rs + body.vs*dtime;
	body.w = body.w + Qb.transpose()*alphasb*dtime;
	//body quaternion update
	Quat dq = (Mat44(body.w.V[0], body.w.V[1], body.w.V[2])*body.q)*0.5L; //오차?
	body.q = body.q + dq*dtime;
	cout << "function successfully run" << endl;
	for (int i = 0; i < numsubleg; i++) cout << lbtomots[i] << endl;
	return make_pair( asb,alphasb );
}
robot::robot() {
	Flist.reserve(20);
	body = Robotbody();
	for (int i = 0; i < numLeg; i++) leg[i] = Leg();
	//set Axes of Legs
	leg[0].sub[0].axis = Vector(0, 1, 0);
	leg[0].sub[1].axis = Vector(1, 0, 0);
	leg[0].sub[2].axis = Vector(1, 0, 0);

	leg[1].sub[0].axis = Vector(0, -1, 0);
	leg[1].sub[1].axis = Vector(1, 0, 0);
	leg[1].sub[2].axis = Vector(1, 0, 0);

	leg[2].sub[0].axis = Vector(0, 1, 0);
	leg[2].sub[1].axis = Vector(-1, 0, 0);
	leg[2].sub[2].axis = Vector(-1, 0, 0);

	leg[3].sub[0].axis = Vector(0, -1, 0);
	leg[3].sub[1].axis = Vector(-1, 0, 0);
	leg[3].sub[2].axis = Vector(-1, 0, 0);
	
	//tam-timized : NEVER USE THIS CODE FOR OTHER OPERATIONS!!!!
	for (int i = 0; i < numLeg; i++) {
		for (int j = 0; j < numsubleg; j++) {
			for (int b = 0; b < 2; b++) {
				leg[i].sub[j].l[b] = Vector(0,0.05L*(1-(i&2)), 0);
			}
		}
	}

	//set L-vectors of sublegs & body
	setMass();
	setI();
}
void robot::setI() {
	body.Ibdia = 
		Vector(lyb*lyb + lzb*lzb, lzb*lzb + lxb*lxb, lxb*lxb + lyb*lyb)*body.m *(1.L/12.L);
	for (int i = 0; i < numLeg; i++) {
		for (int j = 0; j < numsubleg; j++) {
			leg[i].sub[j].Ibdia = Vector(0.001L,0.001L,0.001L)*(1.L/12.L);
		}
	}
}
Leg::Leg() {
	for (int i = 0; i < numsubleg; i++) sub[i] = subleg();
	for (int i = 0; i < numsubleg; i++) {
		sub[i].m = 0.1L;
	}
	//initial angle
	sub[1].theta = pi / 2.L;
}

subleg::subleg():theta(0),alpha(0),omega(0) {
	cout << "alpha = " << alpha << ",omega = " << omega << endl;
}

Robotbody::Robotbody() {
	//set mass of body
	m = 0.5L;
	//initial location
	rs = Vector(0, 0, 0.4L);
	q = Quat(0, 1.L, 0, 0);

	lbtomot[0] = Vector(lxb / 2.L, lyb / 2.L, 0);
	lbtomot[1] = Vector(-lxb / 2.L, lyb / 2.L, 0);
	lbtomot[2] = Vector(lxb / 2.L, -lyb / 2.L, 0);
	lbtomot[3] = Vector(-lxb / 2.L, -lyb / 2.L, 0);
}