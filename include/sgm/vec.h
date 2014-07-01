#pragma once

#include "sgmath.h"

SGE_BEGIN_MATH_NAMESPACE

/////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////
#define SGM_TVEC_OPERATOR_SCALE_VECTOR(VEC_TYPE)							\
template<typename TDATA_TYPE> VEC_TYPE<TDATA_TYPE> 							\
	operator*( const TDATA_TYPE& s, const VEC_TYPE<TDATA_TYPE>& d)			\
{																			\
	return d * s;															\
}

#define SGM_TVEC_DEFINE_BASIC_OPS(TVEC)				\
template<class T>									\
T dot(const TVEC<T>& a, const TVEC<T>& b)			\
{													\
	return a.dot(b);								\
}													\
template<class T>									\
T length(const TVEC<T>& a)							\
{													\
	return a.length();								\
}													\
template<class T>									\
TVEC<T> normalized(const TVEC<T>& a)				\
{													\
	return a.normalized();							\
}													\
template<class T>									\
TVEC<T> reflect(const TVEC<T>& i, const TVEC<T>& n)	\
{													\
	return i.reflect(n);							\
}													\
template<class T>									\
TVEC<T> refract(									\
const TVEC<T>& i,									\
const TVEC<T>& n,									\
const typename TVEC<T>::DATA_TYPE& eta)				\
{													\
	return i.refract(n, eta);						\
}													

/////////////////////////////////////////////////////////////////
//2D vector
/////////////////////////////////////////////////////////////////
template<typename TDATA_TYPE>
struct vec2
{
	typedef TDATA_TYPE DATA_TYPE;
	typedef vec2 SELF_TYPE;
	static const unsigned int NUM_ELEMS = 2;

	union
	{
		struct { DATA_TYPE x, y; };
		struct { DATA_TYPE u, v; };
		DATA_TYPE d[NUM_ELEMS];
	};

	vec2()
	{ }

	explicit vec2(const DATA_TYPE& s)
	{
		for(unsigned int t = 0; t < NUM_ELEMS; ++t)
		{
			d[t] = s;
		}
	}

	vec2(const DATA_TYPE& _x, const DATA_TYPE& _y) :
		x(_x),
		y(_y)
	{ }

#include "vec_common.inl"
	
};

SGM_TVEC_OPERATOR_SCALE_VECTOR(vec2)
SGM_TVEC_DEFINE_BASIC_OPS(vec2)

/////////////////////////////////////////////////////////////////
//3D vector
/////////////////////////////////////////////////////////////////
template<typename TDATA_TYPE>
struct vec3
{
	typedef TDATA_TYPE DATA_TYPE;
	typedef vec3 SELF_TYPE;
	static const unsigned int NUM_ELEMS = 3;

	union
	{
		struct { DATA_TYPE x, y, z; };
		struct { DATA_TYPE u, v, w; };
		struct { DATA_TYPE r, g, b; };
		DATA_TYPE d[NUM_ELEMS];
	};

	vec3()
	{ }

	explicit vec3(const DATA_TYPE& s)
	{
		for(unsigned int t = 0; t < NUM_ELEMS; ++t)
		{
			d[t] = s;
		}
	}

	vec3(const DATA_TYPE& _x, const DATA_TYPE& _y, const DATA_TYPE& _z) :
		x(_x),
		y(_y),
		z(_z)
	{ }

#include "vec_common.inl"
	
	SELF_TYPE cross(const SELF_TYPE& v)
	{
		const DATA_TYPE x = (y * v.z) - (v.y * z);
		const DATA_TYPE y = (v.x * z) - (x * v.z);
		const DATA_TYPE z = (x * v.y) - (v.x * y);

		return SELF_TYPE(x, y, z);
	}

};

SGM_TVEC_OPERATOR_SCALE_VECTOR(vec3)
SGM_TVEC_DEFINE_BASIC_OPS(vec3)

template<class T>									
vec3<T> corss(const vec3<T>& a, const vec3<T>& b)
{
	return a.cross(b);
}

/////////////////////////////////////////////////////////////////
//4D vector
/////////////////////////////////////////////////////////////////
template<typename TDATA_TYPE>
struct vec4
{
	typedef TDATA_TYPE DATA_TYPE;
	typedef vec4 SELF_TYPE;
	static const unsigned int NUM_ELEMS = 4;

	union
	{
		struct { DATA_TYPE x, y, z, w; };
		struct { DATA_TYPE r, g, b, a; };
		DATA_TYPE d[NUM_ELEMS];
	};

	vec4()
	{ }

	explicit vec4(const DATA_TYPE& s)
	{
		for(unsigned int t = 0; t < NUM_ELEMS; ++t)
		{
			d[t] = s;
		}
	}

	vec4(const DATA_TYPE& _x, const DATA_TYPE& _y, const DATA_TYPE& _z, const DATA_TYPE& _w) :
		x(_x),
		y(_y),
		z(_z),
		w(_w)
	{ }

#include "vec_common.inl"

};

SGM_TVEC_OPERATOR_SCALE_VECTOR(vec4)
SGM_TVEC_DEFINE_BASIC_OPS(vec4)

/////////////////////////////////////////////////////////////////
//Cleanup all the macros
/////////////////////////////////////////////////////////////////

#undef SGM_TVEC_OPERATOR_SCALE_VECTOR
#undef SGM_TVEC_DEFINE_BASIC_OPS

typedef vec2<int>			vec2i;
typedef vec2<unsigned int>	vec2u;
typedef vec2<float>			vec2f;
typedef vec2<double>		vec2d;

typedef vec3<int>			vec3i;
typedef vec3<unsigned int>	vec3u;
typedef vec3<float>			vec3f;
typedef vec3<double>		vec3d;

typedef vec4<int>			vec4i;
typedef vec4<unsigned int>	vec4u;
typedef vec4<float>			vec4f;
typedef vec4<double>		vec4d;

template <typename T, unsigned int N>
struct vec_type_picker
{

};

template <typename T>
struct vec_type_picker<T, 2>
{
	typedef vec2<T>	value;
};

template <typename T>
struct vec_type_picker<T, 3>
{
	typedef vec3<T>	value;
};

template <typename T>
struct vec_type_picker<T, 4>
{
	typedef vec4<T>	value;
};


SGE_END_MATH_NAMESPACE