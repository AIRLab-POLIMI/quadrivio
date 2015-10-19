#include "Interpolator.h"

Interpolator::Interpolator(int size) {
	this->size = size;
	values = boost::circular_buffer<double>(this->size);
	times = boost::circular_buffer<double>(this->size);
	acc = gsl_interp_accel_alloc();
	spline = gsl_spline_alloc(gsl_interp_cspline, this->size);
	ready = false;
}

bool Interpolator::canInterpolate() { return ready; }

double Interpolator::interpolate(double time) {
	if(time > times[size - 1]) {
		double v = (values[size - 1] - values[size - 2]) / (times[size - 1] - times[size - 2]);
		return values[size - 1] + v * (time - times[size - 1]);
	}
	else
		return gsl_spline_eval(spline, time, acc);
}

void Interpolator::update(double val, double time) {
	values.push_back(val);
	times.push_back(time);
	if(times.size() >= size) {
		ready = true;
		times.linearize();
		values.linearize();
		gsl_spline_init(spline, times.array_one().first, values.array_one().first, size);
	}
}

