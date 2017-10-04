/*
	physio.cpp : source file for I/O functions of phyengine's object
	Last Modified : 2017/10/04 by Changki Yun
	Now Available :
		Vector, Mat33, Quat
*/
#include "physengine.h"
ostream& operator<<(ostream& os, const Vector& v) {
	os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
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
	os << "[" << q.r << ",( " << q.i << ", " << q.j << ", " << q.k << ")]";
	return os;
}