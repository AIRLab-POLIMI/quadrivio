#include <boost/circular_buffer.hpp>
#include <gsl/gsl_spline.h>

using namespace boost;

class Interpolator {
private:
	int size;
	circular_buffer<double> values;
	circular_buffer<double> times;
	gsl_interp_accel *acc;
	gsl_spline *spline;
	bool ready;
public:
	Interpolator() {};
	Interpolator(int size);
	double interpolate(double time);
	bool canInterpolate();
	void update(double val, double time);
};
