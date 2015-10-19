#include "Interpolator.h"

typedef std::vector< double > state_type;

class Kinematic {
private:
	Interpolator *spInt, *stInt;
	double kspeed, ksteer, l, psi;
public:
	Kinematic(Interpolator spInt, Interpolator stInt, double ksp, double kst, double l, double psi) {
		this->spInt = &spInt;
		this->stInt = &stInt;
		kspeed = ksp;
		ksteer = kst;
		this->l = l;
    this->psi = psi;
	}

	void operator() (const state_type &x, state_type &dxdt, const double t){
		dxdt[0] = kspeed*spInt->interpolate(t);
		dxdt[1] = (kspeed*spInt->interpolate(t)) / l*tan(ksteer*stInt->interpolate(t) + psi);
	}
};
