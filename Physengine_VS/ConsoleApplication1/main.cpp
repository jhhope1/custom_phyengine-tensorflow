#include "physengine.h"
int main() {
	cout.setf(ios::fixed);
	cout.precision(12);
	set_physics_constants();
	/*
	Stick x;
	x.initstick();
	x.debugstick();
	*/
	robot R;
	R.debugrobot();
	return 0;
}