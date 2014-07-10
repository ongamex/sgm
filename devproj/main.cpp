#include <iostream>
#include <sgm\sgm.h>
#include <DirectXMath.h>

using namespace sgm;

//template<typename VEC_TYPE>
std::ostream& operator << (std::ostream& o, const vec3f& v)
{
	o << '(' << v[0];
	for(unsigned t = 1; t < vec3f::NUM_ELEMS; ++t)
		 o << ' ' << ',' << v[t];
	o << ')';

	return o;
}

template<typename VEC_TYPE>
VEC_TYPE InitVector()
{
	VEC_TYPE result;
	for(unsigned t = 0; t < VEC_TYPE::NUM_ELEMS; ++t)
	{
		result[t] = (typename VEC_TYPE::DATA_TYPE)(t + 1);
	}

	return result;
}

template<class VEC_TYPE>
void TestVectors() 
{
#define PRINT_VEC(x) std::cout << #x" = " << x << std::endl;
#define TEST_OP(x) 						  \
	{									  \
		std::cout << #x << " => ";		  \
		std::cout << (x) << std::endl;}
	
	std::cout << "\n-------------------------------------\n\n";

	VEC_TYPE a = InitVector<VEC_TYPE>();
	VEC_TYPE b = InitVector<VEC_TYPE>();



	PRINT_VEC(a);
	PRINT_VEC(b);

	TEST_OP(a + b);
	TEST_OP(a - b);

	TEST_OP(cmul(a,b));
	TEST_OP(cmul(a,b));
	TEST_OP(cdiv(a,b));

	TEST_OP(dot(a,b));
	TEST_OP(hsum(a));

	TEST_OP(lerp(a,b,(typename VEC_TYPE::DATA_TYPE)0.5f));
	TEST_OP(sgm::lerp(1,2,0.5));

	for(unsigned t = 0; t < VEC_TYPE::NUM_ELEMS; ++t)
	{
		TEST_OP(VEC_TYPE::get_axis(t, (typename VEC_TYPE::DATA_TYPE)t));
	}
	TEST_OP(VEC_TYPE::get_zero());
	
	{
		std::cout << "\n-------------------------------------\n\n";
		VEC_TYPE x = VEC_TYPE::get_zero();
		x[0] = 1; x[1] = 1;

		PRINT_VEC(x);

		TEST_OP(normalized(x));
		TEST_OP(length(normalized(x)));
	}
}


template<class T>
void initvar(T& v)
{
	fread(&v, sizeof(T), 1, (FILE*)54);
}

template<class T>
void usevar(T& v)
{
	fwrite(&v, sizeof(T), 1, (FILE*)53);
}

int main()
{	
	quat<float> a(1,2,3,4);
	quat<float> b(0.5, 0.6, 0.7, 0.8);

	initvar(a);
	initvar(b);

	a = mul(a,b);

	usevar(a);

	return 0;
}