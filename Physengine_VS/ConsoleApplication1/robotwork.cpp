#include "physengine.h"
using namespace std;
void robot::debugrobot() {
	pair<Vector, Vector> a_alpha;

	for (int i = 0; i < 20000; i++) {
		a_alpha = timeflow(dtime*i*100.L);
		//cout << "timeflow out" << endl;
		if (i % printn) continue;
		txt << dtime*i << '\n';
		txt << body.rs << '\n'; //body의 위치 출력
		/*
		cout << "Time : " << i*dtime << '\n';
		cout << "r of Body : " << body.rs << '\n';
		cout << "v of Body : " << body.vs << '\n';
		cout << "a of Body : " << a_alpha.first << '\n';
		cout << "alpha of Body : " << a_alpha.second << '\n';
		cout << "size of Flist = " << Flist.size() << '\n';
		for (Force &f : Flist) {
			cout << "F = " << f.F << ", r = " << f.r << '\n';
		}
		*/
		/*
		for (int i = 0; i < numLeg; i++) {
			for (int j = 0; j < numsubleg; j++) {
				cout << "leg " << i << ", subleg" << j << " 's quat : " << leg[i].sub[j].q << '\n';
			}
		}
		*/
		Quat qsub[numsubleg];
		Mat33 Qsub[numsubleg], Qb;
		Vector
			lbtomotb, lbtomots[numsubleg],
			l[numsubleg][2];
		Qb = body.q.toRot();
		for (int legnum = 0; legnum < numLeg; legnum++) {
			Leg &L = leg[legnum];
			qsub[0] = body.q*L.sub[0].q;
			lbtomotb = Qb * body.lbtomot[legnum];
			txt << body.rs + lbtomotb << '\n'; // Leg의 시작점 출력
			for (int i = 1; i < numsubleg; i++) qsub[i] = qsub[i - 1] * L.sub[i].q;
			for (int i = 0; i < numsubleg; i++) Qsub[i] = qsub[i].toRot();

			for (int b = 0; b < 2; b++)
				for (int i = 0; i < numsubleg; i++)
					l[i][b] = Qsub[i] * L.sub[i].l[b];

			lbtomots[0] = lbtomotb + l[0][0];
			for (int i = 1; i < numsubleg; i++)
				lbtomots[i] = lbtomots[i - 1] + l[i][0] + l[i - 1][1];
			for (int i = 0; i < numsubleg; i++) {
				txt << body.rs + lbtomots[i] + l[i][1] << '\n'; //각 subleg의 끝점 출력
			}
		}
	}
}