/*
	physio.cpp : source file for I/O functions of phyengine's object
	Last Modified : 2017/10/04 by Changki Yun
	Now Available :
		Vector, Mat33, Quat
*/
#include "physengine.h"
ostream& operator<<(ostream& os, const Vector& v) {
	os << "(" << v.V[0] << ", " << v.V[1] << ", " << v.V[2] << ")";
	return os;
}
ostream& operator<< (ostream &os, const Mat33& m) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			os << m.mat[i][j] << "\n "[j < 2];
		}
	}
	return os;
}
ostream& operator<< (ostream &os, const Quat &q) {
	os << "[" << q.q[0] << ",( " << q.q[1] << ", " << q.q[2] << ", " << q.q[3] << ")]";
	return os;
}
Mat33 Matdia(phys a) {
	return Mat33(
		a, 0, 0,
		0, a, 0,
		0, 0, a
	);
}
Mat33 dyadic(Vector u) {
	Vector tmp = Vector(u.V[0] * u.V[1], u.V[1] * u.V[2], u.V[2] * u.V[0]);
	return Mat33(
		u.V[0] * u.V[0], tmp.V[0], tmp.V[2],
		tmp.V[0], u.V[1] * u.V[1], tmp.V[1],
		tmp.V[2], tmp.V[1], u.V[2] * u.V[2]
	);
}
Mat33 dyadic(Vector u, Vector w) {
	return Mat33(u.V[0] * w.V[0], u.V[0] * w.V[1], u.V[0] * w.V[2],
		u.V[1] * w.V[0], u.V[1] * w.V[1], u.V[1] * w.V[2],
		u.V[2] * w.V[0], u.V[2] * w.V[1], u.V[2] * w.V[2]
	);
}
Mat33 Matskew(Vector u) {
	return Mat33(
		0, -u.V[2], u.V[1],
		u.V[2], 0, -u.V[0],
		-u.V[1], u.V[0], 0
	);
}
Quat theta_to_quat(phys th, Vector ax) {
	return Quat(cos(th / 2.L), ax*sin(th / 2.L));
}