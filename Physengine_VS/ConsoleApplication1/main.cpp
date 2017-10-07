#include "physengine.h"
ofstream txt;
int main() {
	txt = ofstream("motion1.txt");
	txt.setf(ios::fixed);
	txt.precision(12);
	cout.setf(ios::fixed);
	cout.precision(12);
	set_physics_constants();
	ios_base::sync_with_stdio(false); cin.tie(NULL); cout.tie(NULL);
	/*
	Stick x;
	x.initstick();
	x.debugstick();
	*/
	robot R;
	R.debugrobot();
	txt.close();
	return 0;
}