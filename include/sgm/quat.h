#pragma once

#ifndef SGM_QUAT_H_07112014
#define SGM_QUAT_H_07112014

#include "sgm_base.h"
#include "trigonometry.h"
#include "vec.h"

SGE_BEGIN_MATH_NAMESPACE

/////////////////////////////////////////////////////////////////
//Quaternion
//data layout is (x,y,z,w) where :
//(x,y,z) is the imaginary part
//w is the real part
/////////////////////////////////////////////////////////////////
template<typename TDATA_TYPE>
struct quat
{
	typedef TDATA_TYPE DATA_TYPE;
	typedef quat SELF_TYPE;

	static const unsigned NUM_ELEMS = 4;

	union{
		struct { DATA_TYPE x,y,z,w; };
		DATA_TYPE data[NUM_ELEMS];
	};

	//-----------------------------------------------------------
	//constructors
	//-----------------------------------------------------------
	quat() {}

	quat(const DATA_TYPE& sx, 
		 const DATA_TYPE& sy, 
		 const DATA_TYPE& sz, 
		 const DATA_TYPE& sw)
	{
		data[0] = sx; 
		data[1] = sy; 
		data[2] = sz; 
		data[3] = sw;
	}

	//-----------------------------------------------------------
	friend quat quat_axis_rotation(const vec3<DATA_TYPE>& axis, const DATA_TYPE& angle)
	{
		DATA_TYPE s,c; sgm::sincos(angle * (DATA_TYPE)(0.5), s, c);

		quat result;
		for(unsigned t = 0; t < 3; ++t) result[t] = s * axis[t];
		result[3] = c;

		return result;
	}

	//-----------------------------------------------------------
	friend quat quat_identity()
	{
		return quat(0, 0, 0, (DATA_TYPE)(1.0));
	}

	//-----------------------------------------------------------
	friend quat quat_imaginary()
	{
		return quat((DATA_TYPE)(1.0), (DATA_TYPE)(1.0), (DATA_TYPE)(1.0), 0);
	}

	//-----------------------------------------------------------
	//conjugate
	//-----------------------------------------------------------
	quat conjugate() const
	{
		return quat(-data[0], -data[1], -data[2], data[3]);
	}

	friend quat conjugate(const quat& q)
	{
		return q.conjugate();
	}

	//-----------------------------------------------------------
	//length
	//-----------------------------------------------------------
	DATA_TYPE length_sqr() const
	{
		DATA_TYPE result = 0;

		for(unsigned int t = 0; t < 4; ++t)
		{
			result += data[t] * data[t];
		}

		return result;
	}

	DATA_TYPE length() const
	{
		return sgm::sqrt(length_sqr());
	}

	friend quat length(const quat& q)
	{
		return q.length();
	}

	//-----------------------------------------------------------
	//normalized
	//-----------------------------------------------------------
	quat normalized() const
	{
		const DATA_TYPE invLength = DATA_TYPE(1.0) / length();

		quat result;
		for(unsigned t = 0; t < 4; ++t)
		{
			result[t] = data[t] * invLength;
		}

		return result;
	}

	friend quat normalized(const quat& q)
	{
		return q.normalized();
	}

	//-----------------------------------------------------------
	//inverse
	//-----------------------------------------------------------
	quat inverse() const
	{
		const quat invLensqr = (DATA_TYPE)1.0 / length_sqr();
		return conjugate() * invLensqr;
	}

	friend quat inverse(const quat& q)
	{
		return q.inverse();
	}

	//-----------------------------------------------------------
	//operator[]
	//-----------------------------------------------------------
	DATA_TYPE&			operator[](const unsigned int t)		{ return data[t]; }
	const DATA_TYPE&	operator[](const unsigned int t) const	{ return data[t]; }

	//-----------------------------------------------------------
	// quat + quat
	//-----------------------------------------------------------
	quat& operator+=(const quat& v)
	{
		for(unsigned int t = 0; t < 4; ++t)
		{
			data[t] += v[t];
		}
		return *this;
	}

	quat operator+(const quat& v) const
	{
		quat r;
		for(unsigned int t = 0; t < 4; ++t)
		{
			r[t] = data[t] + v[t];
		}
		return r;
	}

	//---------------------------------------------------------
	//quat - quat
	//---------------------------------------------------------
	quat& operator-=(const quat& v)
	{
		for(unsigned int t = 0; t < 4; ++t)
		{
			data[t] -= v[t];
		}
		return *this;
	}


	quat operator-(const quat& v) const
	{
		SELF_TYPE r;
		for(unsigned int t = 0; t < 4; ++t)
		{
			r[t] = data[t] - v[t];
		}
		return r;
	}

	//---------------------------------------------------------
	//hsum
	//---------------------------------------------------------
	DATA_TYPE hsum() const
	{
		DATA_TYPE r = data[0];
		for(unsigned int t = 1; t < NUM_ELEMS; ++t)
		{
			r += data[t];
		}
		return r;
	}

	friend DATA_TYPE hsum(const SELF_TYPE& v)
	{
		return v.hsum();
	}

	//-----------------------------------------------------------
	//Quat * Scalar
	//-----------------------------------------------------------
	quat operator*=(const DATA_TYPE& s)
	{
		for(unsigned int t = 0; t < 4; ++t)
		{
			data[t] *= s;
		}
		return *this;
	}

	quat operator*(const DATA_TYPE& s) const
	{
		quat r;
		for(unsigned int t = 0; t < 4; ++t)
		{
			r[t] = data[t] * s;
		}
		return r;
	}

	friend quat operator*(const DATA_TYPE& s, const quat& q)
	{
		return q * s;
	}

	//-----------------------------------------------------------
	//Quat / Scalar
	//-----------------------------------------------------------
	quat operator/=(const DATA_TYPE& s)
	{
		for(unsigned int t = 0; t < 4; ++t)
		{
			data[t] /= s;
		}
		return *this;
	}

	quat operator/(const DATA_TYPE& s) const
	{
		quat r;
		for(unsigned int t = 0; t < 4; ++t)
		{
			r[t] = data[t] / s;
		}
		return r;
	}

	friend quat operator/(const DATA_TYPE& s, const quat& q)
	{
		return q / s;
	}

	//-----------------------------------------------------------
	//multiplication
	//-----------------------------------------------------------
	friend quat mul(const quat& p, const quat& q)
	{
		quat r;

		quat v(p[3], -p[2], p[1], p[0]);
		r[0] = cmul(v, q).hsum();

		v = quat(p[2], p[3], -p[0], p[1]);
		r[1] = cmul(v, q).hsum();

		v = quat(-p[1], p[0], p[3], p[2]);
		r[2] = cmul(v, q).hsum();

		v = quat(-p[0], -p[1], -p[2], p[3]);
		r[3] = cmul(v, q).hsum();

		return r;
	}

	//-----------------------------------------------------------
	//Componentwise multiplication
	//-----------------------------------------------------------
	friend quat cmul(const quat& q0, const quat& q1)
	{
		SELF_TYPE r;
		for(unsigned int t = 0; t < 4; ++t)
		{
			r[t] = q0[t] * q1[t];
		}
		return r;
	}
};

SGE_END_MATH_NAMESPACE

#endif