#pragma once
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
using namespace std;
using phys = long double;
//Basic physics constants
const phys g = 9.81;
const phys pi = 3.14159265358979323846264338;
const phys dtime = 1e-3;

//3D Vector
struct Vector {
	phys V[3];
	Vector(phys x = 0.L, phys y = 0.L, phys z = 0.L) {
		V[0] = x;
		V[1] = y;
		V[2] = z;
	}
	Vector operator+ (Vector w) {
		return Vector(V[0] + w.V[0], V[1] + w.V[1], V[2] + w.V[2]);
	}
	Vector operator- (Vector w) {
		return Vector(V[0] - w.V[0], V[1] - w.V[1], V[2] - w.V[2]);
	}
	Vector operator* (Vector w) {
		return Vector(V[1] * w.V[2] - V[2] * w.V[1], V[2] * w.V[0] - V[0] * w.V[2], V[0] * w.V[1] - V[1] * w.V[0]);
	}
	Vector operator* (phys a) {
		return Vector(V[0]*a, V[1]*a, V[2]*a);
	}
	phys operator% (Vector w) {
		return V[0]*w.V[0] + V[1]*w.V[1] + V[2]*w.V[2];
	}
	phys norm() {
		return sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]);
	}
	friend ostream& operator<< (ostream& os, const Vector& v);
};
//Force Vector
struct Force {
	Vector F, r;
};
//3x3 Matrix
struct Mat33 {
	phys mat[3][3];
	Mat33(phys a = 0., phys b = 0., phys c = 0., phys d = 0., phys e = 0., phys f = 0., phys g = 0., phys h = 0., phys i = 0.) {
		mat[0][0] = a;
		mat[0][1] = b;
		mat[0][2] = c;
		mat[1][0] = d;
		mat[1][1] = e;
		mat[1][2] = f;
		mat[2][0] = g;
		mat[2][1] = h;
		mat[2][2] = i;
	}
	Mat33(Vector dia) {
		mat[0][0] = dia.V[0];
		mat[1][1] = dia.V[1];
		mat[2][2] = dia.V[2];
	}
	Mat33 inv() {
		phys det = 0;
		Mat33 z;
		for (int i = 0; i<3; i++) det += (mat[0][i] *
			(mat[1][(i + 1) % 3] * mat[2][(i + 2) % 3] - mat[1][(i + 2) % 3] * mat[2][(i + 1) % 3]));
		if (abs(det)<1e-10) {
			for (int i = 0; i<3; i++) {
				for (int j = 0; j<3; j++) printf("%.3Lf ", mat[i][j]);
				puts("");
			}
			puts("Determinant is 0!");
			exit(4444);
		}
		det = 1.L / det;
		for (int i = 0; i<3; i++)
			for (int j = 0; j<3; j++)
				z.mat[i][j] = (mat[(j + 1) % 3][(i + 1) % 3] * mat[(j + 2) % 3][(i + 2) % 3] - mat[(j + 1) % 3][(i + 2) % 3] * mat[(j + 2) % 3][(i + 1) % 3])*det;
		return z;
	}
	Mat33 operator+ (Mat33 z) {
		for (int i = 0; i<3; i++)
			for (int j = 0; j<3; j++)
				z.mat[i][j] += mat[i][j];
		return z;
	}
	Mat33 operator- (Mat33 z) {
		for (int i = 0; i<3; i++)
			for (int j = 0; j<3; j++)
				z.mat[i][j] -= mat[i][j];
		return z;
	}
	Mat33 operator* (Mat33 z) {
		Mat33 w;
		for (int i = 0; i<3; i++) {
			for (int j = 0; j<3; j++) {
				for (int k = 0; k<3; k++) {
					w.mat[i][j] += mat[i][k] * z.mat[k][j];
				}
			}
		}
		return w;
	}
	Mat33 operator* (phys a) {
		return Mat33(
			a*mat[0][0], a*mat[0][1], a*mat[0][2],
			a*mat[1][0], a*mat[1][1], a*mat[1][2],
			a*mat[2][0], a*mat[2][1], a*mat[2][2]
		);
	}
	Vector operator* (Vector z) {
		Vector w;
		return Vector(mat[0][0] * z.V[0] + mat[0][1] * z.V[1] + mat[0][2] * z.V[2],
			mat[1][0] * z.V[0] + mat[1][1] * z.V[1] + mat[1][2] * z.V[2],
			mat[2][0] * z.V[0] + mat[2][1] * z.V[1] + mat[2][2] * z.V[2]);
	}
	Mat33 transpose() {
		return Mat33(mat[0][0], mat[1][0], mat[2][0],
			mat[0][1], mat[1][1], mat[2][1],
			mat[0][2], mat[1][2], mat[2][2]);
	}
	Mat33 operator% (Vector dia) {
		return Mat33(
			mat[0][0] * dia.V[0], mat[0][1] * dia.V[1], mat[0][2] * dia.V[2],
			mat[1][0] * dia.V[0], mat[1][1] * dia.V[1], mat[1][2] * dia.V[2],
			mat[2][0] * dia.V[0], mat[2][1] * dia.V[1], mat[2][2] * dia.V[2]
		);
	}
	friend ostream& operator<< (ostream& os, const Mat33& m);
};
Mat33 Matdia(phys);
Mat33 dyadic(Vector, Vector);
Mat33 Matskew(Vector);
//Quarternion
//Note : to rotate a vector v through angle t around z :
// v -> qvq^-1 where q = (cos(t/2), sin(t/2)*z)
struct Quat {
	phys q[4];
	Quat(phys r = 0., phys i = 1., phys j = 0., phys k = 0.) {
		q[0] = r;
		q[1] = i;
		q[2] = j;
		q[3] = k;
	}
	Quat(phys r, Vector v) {
		q[0] = r;
		q[1] = v.V[0];
		q[2] = v.V[1];
		q[3] = v.V[2];
	}
	Quat operator+ (Quat w) {
		return Quat(q[0] + w.q[0], q[1] + w.q[1], q[2] + w.q[2], q[3] + w.q[3]);
	}
	Quat operator- (Quat w) {
		return Quat(q[0] - w.q[0], q[1] - w.q[1], q[2] - w.q[2], q[3] - w.q[3]);
	}
	Quat operator* (Quat w) {
		return Quat(q[0]*w.q[0] - q[1]*w.q[1] - q[2]*w.q[2] - q[3]*w.q[3],
			q[0]*w.q[1] + q[1]*w.q[0] + q[2]*w.q[3] - q[3]*w.q[2],
			q[0]*w.q[2] - q[1]*w.q[3] + q[2]*w.q[0] + q[3]*w.q[1],
			q[0]*w.q[3] + q[1]*w.q[2] - q[2]*w.q[1] + q[3]*w.q[0]);
	}
	Quat operator* (phys d) {
		return Quat(q[0]*d, q[1]*d, q[2]*d, q[3]*d);
	}
	static Quat qinv(Vector v) {
		return Quat(0, v*(-1. / v.norm()));
	}
	Vector ext() {
		return Vector(q[1], q[2], q[3]);
	}
	phys norm() {
		return sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	}
	static Quat qinv(Quat q) {
		return Quat(q.q[0], -q.q[1], -q.q[2], -q.q[3])*(1. / q.norm() / q.norm());
	}
	Mat33 toRot() {
		return Mat33(q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3], 2.L*(q[1]*q[2] - q[0]*q[3]), 2.L*(q[0]*q[2] + q[1]*q[3]),
			2.L*(q[1]*q[2] + q[0]*q[3]), q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3], 2.L*(q[2]*q[3] - q[0]*q[1]),
			2.L*(q[1]*q[3] - q[0]*q[2]), 2.L*(q[0]*q[1] + q[2]*q[3]), q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]
		);
	}
	void normalize() {
		phys t = norm();
		q[0] /= t; q[1] /= t; q[2] /= t; q[3] /= t;
	}
	friend ostream& operator<< (ostream& os, const Quat& q);
};

//4x4 Matrix optimized for Quarternion operation
struct Mat44 {
	phys mat[4][4];
	Mat44(phys a = 0., phys b = 0., phys c = 0.) {
		mat[0][0] = 0.0L;
		mat[0][1] = -a;
		mat[0][2] = -b;
		mat[0][3] = -c;
		mat[1][0] = a;
		mat[1][1] = 0.0L;
		mat[1][2] = c;
		mat[1][3] = -b;
		mat[2][0] = b;
		mat[2][1] = -c;
		mat[2][2] = 0.0L;
		mat[2][3] = a;
		mat[3][0] = c;
		mat[3][1] = b;
		mat[3][2] = -a;
		mat[3][3] = 0.0L;
	}
	Quat operator* (Quat z) {
		return Quat(mat[0][0] * z.q[0] + mat[0][1] * z.q[1] + mat[0][2] * z.q[2] + mat[0][3] * z.q[3],
			mat[1][0] * z.q[0] + mat[1][1] * z.q[1] + mat[1][2] * z.q[2] + mat[1][3] * z.q[3],
			mat[2][0] * z.q[0] + mat[2][1] * z.q[1] + mat[2][2] * z.q[2] + mat[2][3] * z.q[3],
			mat[3][0] * z.q[0] + mat[3][1] * z.q[1] + mat[3][2] * z.q[2] + mat[3][3] * z.q[3]);
	}
};

//Basic Rigid Body
struct Rigidbody {
	Rigidbody() {
		q = Quat(0., 1., 0., 0.); //space -> body v' = q*v*qinv
								  //qi = Quat::qinv(q); // body -> space
	}
	phys m, speed;
	Mat33 invIb;
	Vector Ibdia, r, v, w, euler, F, tau;
	Quat q;
};

//A stick : test of physengine
struct Stick {
	Rigidbody stick;
	void initstick();
	void flyingstick();
	void debugstick();
};
struct Robotbody : public Rigidbody {
	Vector lbtomot[4];
};
struct subleg : public Rigidbody {
	subleg *body;
	Vector axis, l;
	phys theta, omega, alpha;
	void setaxis(Vector v) {
		axis = v;
	}
};
struct Leg {
	subleg sub[3];
};
struct robot {
	Robotbody &body;
	Leg* leg[4]; //0 : fr, 1 : fl, 2 : br, 3 : bl
	void setalpha();
	void setForce();
	void timeflow();
};