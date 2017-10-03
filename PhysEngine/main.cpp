#include <bits/stdc++.h>
using namespace std;
using phys = long double;

//for Cylinder
const double M = 10;
const double h = 0.5;
const double R = 0.01;
const double g = 9.81;
const double pi = 3.14159265358979323846264338;
const phys dtime = 5e-2;

struct Vector {
	phys x, y, z;
	Vector(phys x = 0.L, phys y = 0.L, phys z = 0.L) :x(x), y(y), z(z) {}
	Vector operator+ (Vector w) {
		return Vector(x + w.x, y + w.y, z + w.z);
	}
	Vector operator- (Vector w) {
		return Vector(x - w.x, y - w.y, z - w.z);
	}
	Vector operator* (Vector w) {
		return Vector(y * w.z - z * w.y, z * w.x - x * w.z, x * w.y - y * w.x);
	}
	Vector operator* (phys a) {
		return Vector(x*a, y*a, z*a);
	}
	phys operator% (Vector w) {
		return x*w.x + y*w.y + z*w.z;
	}
	phys norm() {
		return sqrt(x*x + y*y + z*z);
	}
};
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
	Vector operator* (Vector z) {
		Vector w;
		return Vector(mat[0][0] * z.x + mat[0][1] * z.y + mat[0][2] * z.z,
			mat[1][0] * z.x + mat[1][1] * z.y + mat[1][2] * z.z,
			mat[2][0] * z.x + mat[2][1] * z.y + mat[2][2] * z.z);
	}
	Mat33 transpose() {
		return Mat33(mat[0][0], mat[1][0], mat[2][0],
			mat[0][1], mat[1][1], mat[2][1],
			mat[0][2], mat[1][2], mat[2][2]);
	}
};

struct Quat {
	phys r, i, j, k;
	Quat(phys r = 0., phys i = 1., phys j = 0., phys k = 0.) :r(r), i(i), j(j), k(k) {}
	Quat(phys r, Vector v) :r(r), i(v.x), j(v.y), k(v.z) {}
	Quat operator+ (Quat w) {
		return Quat(r + w.r, i + w.i, j + w.j, k + w.k);
	}
	Quat operator- (Quat w) {
		return Quat(r - w.r, i - w.i, j - w.j, k - w.k);
	}
	Quat operator* (Quat w) {
		return Quat(r*w.r - i*w.i - j*w.j - k*w.k,
			r*w.i + i*w.r + j*w.k - k*w.j,
			r*w.j - i*w.k + j*w.r + k*w.i,
			r*w.k + i*w.j - j*w.i + k*w.r);
	}
	Quat operator* (phys d) {
		return Quat(r*d, i*d, j*d, k*d);
	}
	static Quat qinv(Vector v) {
		return Quat(0, v*(-1. / v.norm()));
	}
	Vector ext() {
		return Vector(i, j, k);
	}
	phys norm() {
		return sqrt(r*r + i*i + j*j + k*k);
	}
	static Quat qinv(Quat q) {
		return Quat(q.r, -q.i, -q.j, -q.k)*(1. / q.norm() / q.norm());
	}
	Mat33 toRot() {
		return Mat33(1.L - 2.L*(j*j + k*k), 2.L*(i*j - r*k), 2.L*(r*j + i*k),
			2.L*(i*j + r*k), 1.L - 2.L*(i*i + k*k), 2.L*(j*k - r*i),
			2.L*(i*k - r*j), 2.L*(r*i + j*k), 1.L - 2.L*(i*i + j*j)
		);
	}
};

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
		return Quat(mat[0][0] * z.r + mat[0][1] * z.i + mat[0][2] * z.j + mat[0][3] * z.k,
                    mat[1][0] * z.r + mat[1][1] * z.i + mat[1][2] * z.j + mat[1][3] * z.k,
                    mat[2][0] * z.r + mat[2][1] * z.i + mat[2][2] * z.j + mat[2][3] * z.k,
                    mat[3][0] * z.r + mat[3][1] * z.i + mat[3][2] * z.j + mat[3][3] * z.k);
	}
};

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
Rigidbody stick;
void initstick() {
	stick.Ibdia = Vector(0.5L*M*R*R, 1.L / 12.L*M*h*h + 1.L / 4.L*M*R*R, 1.L / 12.L*M*h*h + 1.L / 4.L*M*R*R);
	stick.m=M;
    stick.q=Quat(0.L,1.L,0.L,0.L);
	stick.tau = Vector(0.,0.,0.);
	stick.r = Vector();
	stick.euler = Vector(0., 0., 0.);
	stick.w = Vector(0., 0.1, 0.);
	stick.F=(0,0,-M*g);
}
void flyingstick() {
	Quat q = stick.q;//, qi = stick.qi;
					 //Vector taub = (q*Quat(0,taub)*qi).ext();
					 //Vector wb = (q*Quat(0,wb)*qi).ext();
	Mat33 rotI = q.toRot();
	Mat33 rotIinv = rotI.inv();
	Vector taub = rotIinv*stick.tau;
	Vector wb = rotIinv*stick.w;

	Quat dq = (Mat44(wb.x, wb.y, wb.z)*q)*0.5L;

	stick.q = q + dq*dtime;

	phys SIbdia[3] = { stick.Ibdia.x, stick.Ibdia.y,stick.Ibdia.z };
    Vector alphab = Vector((SIbdia[1] - SIbdia[2]) / SIbdia[0] * wb.y*wb.z + taub.x / SIbdia[0],
                        (SIbdia[2] - SIbdia[0]) / SIbdia[1] * wb.z*wb.x + taub.y / SIbdia[1],
                        (SIbdia[0] - SIbdia[1]) / SIbdia[2] * wb.x*wb.y + taub.z / SIbdia[2]);//paper's formula 6

	wb = wb + alphab*dtime;
	stick.w = rotI*wb;//여기에서 한번 더 torot을 해주어야하는지 의문이다.

	stick.v= stick.v + stick.F*(1 / stick.m)*dtime;
	stick.r = stick.r + stick.v*dtime;
}
int main() {
	initstick();
	Vector stickendinit=Vector(-1.0L,0.0L,0.0L);
	Vector stickendnow=Vector(0,0,0);
	for(int i=0 ; i<10000 ; i++){
        if(i%100=0) continue;
		Mat33 Qrot = stick.q.toRot();
		stickendnow = Qrot.inv()*stickendinit;
		printf("quaternion=(%.3Lf,  %.3Lf, %.3Lf, %.3Lf)\n", stick.q.r,stick.q.i,stick.q.j,stick.q.k);
        printf("time = %.3Lf\n",dtime*(i+1));
        printf("r = (%.3Lf, %.3Lf, %.3Lf)\n",stick.r.x,stick.r.y,stick.r.z);
        printf("막대기의 한쪽 끝의 처음 위치 = (1, 0, 0), 지금 위치 = (%.3Lf,%.3Lf,%.3Lf)\n",stickendnow.x,stickendnow.y,stickendnow.z);
        printf("w = (%.3Lf %.3Lf %.3Lf)\n",stick.w.x,stick.w.y,stick.w.z);
        flyingstick();
        }
}
