#pragma once

#include "sgm_base.h"
#include "math_utils.h"

SGE_BEGIN_MATH_NAMESPACE
									
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
		DATA_TYPE data[NUM_ELEMS];
	};

	vec2()
	{ }

	explicit vec2(const DATA_TYPE& s)
	{
		for(unsigned int t = 0; t < NUM_ELEMS; ++t)
		{
			data[t] = s;
		}
	}

	vec2(const DATA_TYPE& _x, const DATA_TYPE& _y) :
		x(_x),
		y(_y)
	{ }

#include "vec_common.inl"

};

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
		DATA_TYPE data[NUM_ELEMS];
	};

	vec3()
	{ }

	explicit vec3(const DATA_TYPE& s)
	{
		for(unsigned int t = 0; t < NUM_ELEMS; ++t)
		{
			data[t] = s;
		}
	}

	vec3(const DATA_TYPE& _x, const DATA_TYPE& _y, const DATA_TYPE& _z) :
		x(_x),
		y(_y),
		z(_z)
	{ }

#include "vec_common.inl"
	
	//---------------------------------------------------
	//cross product
	//---------------------------------------------------
	SELF_TYPE cross(const SELF_TYPE& v) const
	{
		const DATA_TYPE x = (y * v.z) - (v.y * z);
		const DATA_TYPE y = (v.x * z) - (x * v.z);
		const DATA_TYPE z = (x * v.y) - (v.x * y);

		return SELF_TYPE(x, y, z);
	}
									
	friend vec3 cross(const vec3& a, const vec3& b)
	{
		return a.cross(b);
	}

	//---------------------------------------------------
	//tiple product: dot(v, cross(a,b))
	//---------------------------------------------------
	friend DATA_TYPE triple(const vec3& v, const vec3& a, const vec3& b)
	{
		return v.dot(a.cross(b));
	}
};

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
		DATA_TYPE data[NUM_ELEMS];
	};

	vec4()
	{ }

	explicit vec4(const DATA_TYPE& s)
	{
		for(unsigned int t = 0; t < NUM_ELEMS; ++t)
		{
			data[t] = s;
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

/////////////////////////////////////////////////////////////////
//vec_n vector
/////////////////////////////////////////////////////////////////
template<typename TDATA_TYPE, unsigned int TNUM_ELEMS>
struct vec_n
{
	typedef TDATA_TYPE DATA_TYPE;
	typedef vec_n SELF_TYPE;
	static const unsigned int NUM_ELEMS = TNUM_ELEMS;

	DATA_TYPE data[NUM_ELEMS];

	vec_n() { }

#include "vec_common.inl"

};

/////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////
template<unsigned X, unsigned Y, typename TVEC>
vec2<typename TVEC::DATA_TYPE> permute(const TVEC& vec)
{
	SGE_MATH_STATIC_ASSERT(TVEC::NUM_ELEMS > X, "sgm::permute arhumargument out of range!");
	SGE_MATH_STATIC_ASSERT(TVEC::NUM_ELEMS > Y, "sgm::permute arhumargument out of range!");

	return vec2<typename TVEC::DATA_TYPE>(vec[X], vec[Y]); 
}

template<unsigned X, unsigned Y, unsigned Z, typename TVEC>
vec3<typename TVEC::DATA_TYPE> permute(const TVEC& vec)
{
	SGE_MATH_STATIC_ASSERT(TVEC::NUM_ELEMS > X, "sgm::permute arhumargument out of range!");
	SGE_MATH_STATIC_ASSERT(TVEC::NUM_ELEMS > Y, "sgm::permute arhumargument out of range!");
	SGE_MATH_STATIC_ASSERT(TVEC::NUM_ELEMS > Z, "sgm::permute arhumargument out of range!");

	return vec3<typename TVEC::DATA_TYPE>(vec[X], vec[Y], vec[Z]); 
}

template<unsigned X, unsigned Y, unsigned Z, unsigned W, typename TVEC>
vec4<typename TVEC::DATA_TYPE> permute(const TVEC& vec)
{
	SGE_MATH_STATIC_ASSERT(TVEC::NUM_ELEMS > X, "sgm::permute arhumargument out of range!");
	SGE_MATH_STATIC_ASSERT(TVEC::NUM_ELEMS > Y, "sgm::permute arhumargument out of range!");
	SGE_MATH_STATIC_ASSERT(TVEC::NUM_ELEMS > Z, "sgm::permute arhumargument out of range!");
	SGE_MATH_STATIC_ASSERT(TVEC::NUM_ELEMS > W, "sgm::permute arhumargument out of range!");

	return vec4<typename TVEC::DATA_TYPE>(vec[X], vec[Y], vec[Z], vec[W]); 
}


/////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////
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


/////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////
template <typename T, unsigned int N>
struct vec_type_picker
{
	typedef vec_n<T,N>	value;
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