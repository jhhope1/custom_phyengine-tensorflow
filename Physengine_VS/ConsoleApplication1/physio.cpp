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