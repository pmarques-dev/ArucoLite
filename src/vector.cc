#include "vector.h"

// intersect two segments, one defined by the 2 points (x1,y1) -> (x2,y2) and
// another defined by (x3,y3) -> (x4,y4) and return the intersection in (res_x,res_y)
static bool intersect_segment(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4, float *res_x, float *res_y)
{
	float divider = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	if (fabs(divider) < 1e-3)
		return false;

	float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / divider;

	*res_x = x1 + t * (x2 - x1);
	*res_y = y1 + t * (y2 - y1);

	return true;
}

bool intersect_lines(line2d_t &l1, line2d_t &l2, pt2d_t &result)
{
	return intersect_segment(
		l1.c.x, l1.c.y, l1.c.x + l1.v.x, l1.c.y + l1.v.y,
		l2.c.x, l2.c.y, l2.c.x + l2.v.x, l2.c.y + l2.v.y,
		&result.x, &result.y
	);
}

bool line_fit_t::compute(line2d_t &line)
{
	float tx, ty, x0, x1, theta;

	if (size < 2)
		return false;

	line.c.x = x / size;
	line.c.y = y / size;

	x0 = xx - x * line.c.x;
	x1 = yy - y * line.c.y;

	tx = x0 - x1;
	ty = 2.0f * (xy - x * y / size);

	theta = 0.5f * atan2f(ty, tx);

	line.v.x = cosf(theta);
	line.v.y = sinf(theta);

	return true;
}
