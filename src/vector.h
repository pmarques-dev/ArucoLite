#ifndef VECTOR_H
#define VECTOR_H

#include <math.h>

// include file with some basic geometry classes

class pt2d_t {
public:
	float x, y;

	pt2d_t() {}
	pt2d_t(float x, float y) : x(x), y(y) {}

	pt2d_t operator+(const pt2d_t &rhs) {
		return pt2d_t(x + rhs.x, y + rhs.y);
	}
	void operator+=(const pt2d_t &rhs) {
		*this = *this + rhs;
	}
	pt2d_t operator-(const pt2d_t &rhs) {
		return pt2d_t(x - rhs.x, y - rhs.y);
	}
	pt2d_t operator-(void) {
		return pt2d_t(-x, -y);
	}
	pt2d_t operator*(float scaler) {
		return pt2d_t(x * scaler, y * scaler);
	}
	void operator*=(float scaler) {
		*this = *this * scaler;
	}
	float norm(void) {
		return sqrt(x * x + y * y);
	}
	float cross(const pt2d_t &rhs) {
		return x * rhs.y - y * rhs.x;
	}
	void normalize(void) {
		float n = norm();
		if (n < 1e-6)
			return;
		x /= n;
		y /= n;
	}
};

class line2d_t {
public:
	// line is defined by a center point "c" and a vector "v"
	pt2d_t c, v;
};

class line_fit_t {
protected:
	float x, y, xx, xy, yy;
	int size;

public:
	line_fit_t() {
		reset();
	}

	void reset(void) {
		size = 0;
		x = 0.0f;
		y = 0.0f;
		xx = 0.0f;
		xy = 0.0f;
		yy = 0.0f;
	}

	void add(float px, float py) {
		x += px;
		y += py;
		xx += px * px;
		xy += px * py;
		yy += py * py;
		size++;
	}

	bool compute(line2d_t &line);
};

// returns true if the lines intersect
bool intersect_lines(line2d_t &l1, line2d_t &l2, pt2d_t &result);

#endif
