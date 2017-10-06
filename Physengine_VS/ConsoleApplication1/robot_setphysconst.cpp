#include "physengine.h"
using namespace std;
Vector Fg(0, 0, -g);
phys lxb;
phys lyb;
phys lzb;
void set_physics_constants() { //prior to all functions
	lxb = 0.2L;
	lyb = 0.3L;
	lzb = 0.05L;
}