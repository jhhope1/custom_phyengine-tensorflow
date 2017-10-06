#include "physengine.h"
using namespace std;
void robot::debugrobot() {
	pair<Vector, Vector> a_alpha;
	for (int i = 0; i < 100000; i++) {
		a_alpha = timeflow();
		//cout << "timeflow out" << endl;
		if (i % 3000) continue;
		cout << "Time : " << i*dtime << '\n';
		cout << "r of Body : " << body.rs << '\n';
		cout << "v of Body : " << body.vs << '\n';
		cout << "a of Body : " << a_alpha.first << '\n';
		cout << "alpha of Body : " << a_alpha.second << '\n';
		cout << "size of Flist = " << Flist.size() << '\n';
		for (Force &f : Flist) {
			cout << "F = " << f.F << ", r = " << f.r << '\n';
		}
	}
}