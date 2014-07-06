#include <cstdio>
#include "sgm\mat.h"

using namespace sgm;

int main()
{
	vec3f a(1), b(2);

	auto f = a.dot(b);

	mat33<float> m;

	m.identify_axis(0);
	m.identify_axis(1);
	m.data[2] = vec3f(3, 3, 1);

	mat33<float> inv = inverse(m);

	return 0;
}