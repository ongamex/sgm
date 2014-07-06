#include "sgmath.h"
#include "vec.h"

SGE_BEGIN_MATH_NAMESPACE
/////////////////////////////////////////////////////////////////
//Quaternion class implementation
//Figure out what to do with quaternion 
//column/row major multiplication problem
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

	quat() {}

	quat(const DATA_TYPE& sx, const DATA_TYPE& sy, const DATA_TYPE& sz, const DATA_TYPE& sw)
	{
		data[0] = sx; data[1] = sy; data[2] = sz; data[3] = sw;
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

	friend quat normalized(const quat& q)
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

	friend quat operator*(const DATA_TYPE& s, const quat& v)
	{
		return v * s;
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

	friend quat operator/(const DATA_TYPE& s, const quat& v)
	{
		return v / s;
	}

};