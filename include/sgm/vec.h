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

	//---------------------------------------------------
	//polar to cartesian
	//---------------------------------------------------
	void polar_to_cartesian(const DATA_TYPE& angle, const DATA_TYPE& radius)
	{
		DATA_TYPE s,c; sgm::sincos(angle, s, c);
		data[0] = s * radius;
		data[1] = c * radius;
	}

	friend SELF_TYPE polar_to_cartesian(const DATA_TYPE& angle, const DATA_TYPE& radius)
	{
		SELF_TYPE result;
		result.polar_to_cartesian(angle, radius);
		return result;
	}

	//---------------------------------------------------
	//cartesian to polar
	//---------------------------------------------------
	void cartesian_to_polar(DATA_TYPE& angle, DATA_TYPE& radius) const
	{
		angle = sgm::atan2(data[1], data[0]);
		radius = length();
	}

	friend void cartesian_to_polar(const SELF_TYPE& vector, DATA_TYPE& angle, DATA_TYPE& radius)
	{
		vector.cartesian_to_polar(angle, radius);
	}

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

	vec3(const vec2<DATA_TYPE>& xy, const DATA_TYPE& _z) :
		x(xy[0]),
		y(xy[1]),
		z(_z)
	{}

	vec3(const DATA_TYPE& _x, const vec2<DATA_TYPE>& yz) :
		x(_x),
		y(yz[0]),
		z(yz[1])
	{}

#include "vec_common.inl"
	
	//---------------------------------------------------
	//cross product
	//---------------------------------------------------
	SELF_TYPE cross(const SELF_TYPE& v) const
	{
		const float tx = (y*v.z) - (v.y*z);
		const float ty = (v.x*z) - (x*v.z);
		const float tz = (x*v.y) - (v.x*y);

		return SELF_TYPE(tx, ty, tz);
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

	//---------------------------------------------------
	//spherical(right-handed with z-up) to cartesian
	//azimuth - the angle on xy plane starting form +x
	//polar - the angle stating form +z and goes to xy plane
	//---------------------------------------------------
	//void spherical_to_cartesian_rh_z_up(const DATA_TYPE& azimuth, const DATA_TYPE& polar, const DATA_TYPE& radius) const
	//{
	//	radius = length();
	//	azimuth = sgm::atan2(data[1], data[0]);
	//	polar = sgm::acos(data[3] / radius);
	//}
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

	vec4(const vec2<DATA_TYPE>& xy, const vec2<DATA_TYPE>& zw) :
		x(xy[0]),
		y(xy[1]),
		z(zw[0]),
		w(zw[1]),
	{ }

	vec4(const vec2<DATA_TYPE>& xy, const DATA_TYPE& z, const DATA_TYPE& w) :
		x(xy[0]),
		y(xy[1]),
		z(z),
		w(w),
	{ }

	vec4(const DATA_TYPE& x, const DATA_TYPE& y, const vec2<DATA_TYPE>& zw) :
		x(x),
		y(y),
		z(zw[0]),
		w(zw[1]),
	{ }

	vec4(const DATA_TYPE& _x, const vec3<DATA_TYPE>& v) :
		x(_x),
		y(v[0]),
		z(v[1]),
		w(v[2])
	{}

	vec4(const vec3<DATA_TYPE>& v, const DATA_TYPE& _w) :
		x(v[0]),
		y(v[1]),
		z(v[2]),
		w(_w)
	{}


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