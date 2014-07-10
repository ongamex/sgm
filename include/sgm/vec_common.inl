/////////////////////////////////////////////////////////////////
//This *.inl contains common vector operation.
//The file is shared between several vector classes.
//Each class that uses that file should have defined as members:
//
//SELF_TYPE - typedev <CLASS_NAME> SELF_TYPE;
//DATA_TYPE - the data storage type
//data[] - an array of type DATA_TYPE with size NUM_ELEMS
//NUM_ELEMS - the number of storage elements (unsigned int)
//
/////////////////////////////////////////////////////////////////

//-----------------------------------------------------------
//gets and identity axis
//-----------------------------------------------------------
static SELF_TYPE get_axis(const unsigned int& axisIndex, const DATA_TYPE& axisLen = DATA_TYPE(1.0))
{
	SELF_TYPE result;
	for(size_t t = 0; t < NUM_ELEMS; ++t)
	{
		result[t] = (DATA_TYPE)(0.0);
	}
	result[axisIndex] = axisLen;
	return result;
}

//-----------------------------------------------------------
//zero vector
//-----------------------------------------------------------
static SELF_TYPE get_zero()
{
	SELF_TYPE result;
	for(size_t t = 0; t < NUM_ELEMS; ++t)
	{
		result[t] = DATA_TYPE(0.0);
	}

	return result;
}

//-----------------------------------------------------------
//operator[]
//-----------------------------------------------------------
DATA_TYPE&			operator[](const unsigned int t)		{ return data[t]; }
const DATA_TYPE&	operator[](const unsigned int t) const	{ return data[t]; }


//-----------------------------------------------------------
//operator== and !=
//-----------------------------------------------------------
bool operator==(const SELF_TYPE& v) const
{
	bool result = true;
	for(unsigned t = 0; t < NUM_ELEMS; ++t)
	{
		result = result && (data[t] == v[t]);
		//not autovect friendly, 
		//if(data[t] != v[t]) return false;
	}

	return result;
}

bool operator!=(const SELF_TYPE& v) const
{
	return !((*this) == v);
}


//-----------------------------------------------------------
//unary operators - +
//-----------------------------------------------------------
SELF_TYPE operator-() const
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		result[t] = - data[t];
	}
	return result;
}

SELF_TYPE operator+() const
{
	return *this;
}

//-----------------------------------------------------------
// Vector + Vector
//-----------------------------------------------------------
SELF_TYPE& operator+=(const SELF_TYPE& v)
{
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		data[t] += v[t];
	}
	return *this;
}

SELF_TYPE operator+(const SELF_TYPE& v) const
{
	SELF_TYPE r(*this);
	r += v;
	return r;
}

//---------------------------------------------------------
//Vector - Vector
//---------------------------------------------------------
SELF_TYPE& operator-=(const SELF_TYPE& v)
{
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		data[t] -= v[t];
	}
	return *this;
}


SELF_TYPE operator-(const SELF_TYPE& v) const
{
	SELF_TYPE r(*this);
	r -= v;
	return r;
}

//---------------------------------------------------------
//Vector * Scalar (and vice versa)
//---------------------------------------------------------
SELF_TYPE& operator*=(const DATA_TYPE& s)
{
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		data[t] *= s;
	}
	return *this;
}

SELF_TYPE operator*(const DATA_TYPE& s) const
{
	SELF_TYPE r(*this);
	r *= s;
	return r;
}

friend SELF_TYPE operator*(const DATA_TYPE& s, const SELF_TYPE& v)
{
	return v * s;
}

//---------------------------------------------------------
//componentwise multiplication
//---------------------------------------------------------
friend SELF_TYPE cmul(const SELF_TYPE& a, const SELF_TYPE& b)
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		r[t] = a[t] * b[t];
	}
	return r;
}

//---------------------------------------------------------
//componentwise division
//---------------------------------------------------------
friend SELF_TYPE cdiv(const SELF_TYPE& a, const SELF_TYPE& b)
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		r[t] = a[t] / b[t];
	}
	return r;
}

//---------------------------------------------------------
//Horizontal sum
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

//---------------------------------------------------------
//min components
//---------------------------------------------------------
SELF_TYPE component_min(const SELF_TYPE& v) const
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		r[t] = sgm::pick_min(*this[t], v[t]);
	}
	return r;
}

friend SELF_TYPE component_min(const SELF_TYPE& a, const SELF_TYPE& b)
{
	a.component_min(b);
}

//---------------------------------------------------------
//max
//---------------------------------------------------------
SELF_TYPE component_max(const SELF_TYPE& v) const
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		r[t] = sgm::pick_max(*this[t], v[t]);
	}
	return r;
}

friend SELF_TYPE component_max(const SELF_TYPE& a, const SELF_TYPE& b)
{
	a.component_max(b);
}

//---------------------------------------------------------
//Dot product
//---------------------------------------------------------
DATA_TYPE dot(const SELF_TYPE& v) const
{
	DATA_TYPE r = data[0] * v[0];
	for(unsigned int t = 1; t < NUM_ELEMS; ++t)
	{
		r += data[t] * v[t];
	}
	return r;
}

friend DATA_TYPE dot(const SELF_TYPE& a, const SELF_TYPE& b)
{
	return a.dot(b);
}

//---------------------------------------------------------
//length
//---------------------------------------------------------
DATA_TYPE length_sqr() const
{
	return dot(*this);
}

DATA_TYPE length() const
{
	return sgm::sqrt(length_sqr());
}

friend DATA_TYPE length(const SELF_TYPE& v)
{
	return v.length();
}

//---------------------------------------------------------
//normalized
//---------------------------------------------------------
SELF_TYPE normalized() const
{
	const DATA_TYPE invLength = DATA_TYPE(1.0) / length();

	SELF_TYPE result;
	for(unsigned int t = 0; t < SELF_TYPE::NUM_ELEMS; ++t)
	{
		result[t] = data[t] * invLength;
	}

	return result;
}

friend SELF_TYPE normalized(const SELF_TYPE& v)
{
	return v.normalized();
}

//---------------------------------------------------------
//reflect
//---------------------------------------------------------
SELF_TYPE reflect(const SELF_TYPE& normal) const
{
	return (*this) + DATA_TYPE(2.0) * dot(normal) * normal;
}

friend DATA_TYPE normalized(const SELF_TYPE& incident, const SELF_TYPE& normal)
{
	return incident.reflect(normal);
}
//---------------------------------------------------------
//refract
//---------------------------------------------------------
SELF_TYPE refract(const SELF_TYPE& normal, const DATA_TYPE& eta) const
{
	const DATA_TYPE one(1.0);
	const DATA_TYPE zero(0.0);

	const DATA_TYPE p = dot(normal);
	const DATA_TYPE k = one - eta*eta * (one -  p*p);
	
	if (k < zero) return SELF_TYPE(zero);
	else return (*this) * eta - (eta * p + sgm::sqrt(k)) * normal;
}

friend SELF_TYPE refract(const SELF_TYPE& inc, const SELF_TYPE& n, DATA_TYPE& eta)				
{													
	return i.refract(n, eta);
}

//---------------------------------------------------------
//distance
//---------------------------------------------------------
SELF_TYPE distance(const SELF_TYPE& other) const
{
	return length(*this - other);
}

friend SELF_TYPE distance(const SELF_TYPE& a, const SELF_TYPE& b)
{
	return a.distance(b);
}

//---------------------------------------------------------
//lerp
//---------------------------------------------------------
//friend SELF_TYPE lerp(const SELF_TYPE& a, const SELF_TYPE& b, const DATA_TYPE& t)
//{
//	return a + (b-a)*t;
//}