/////////////////////////////////////////////////////////////////
//This inl file should be only used in vec.h
//All methods that are written here must be applicable for all
//vec classes (vec2, vec3, vec4, vecN)
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
//Static members
/////////////////////////////////////////////////////////////////
static SELF_TYPE get_axis(const unsigned int& axisIndex, const DATA_TYPE& axisLen = DATA_TYPE(1.0))
{
	SELF_TYPE result;
	for(size_t t = 0; t < NUM_ELEMS; ++t)
	{
		result[t] = (DATA_TYPE)(0.0);
	}
	result[axisIndex] = (DATA_TYPE)(axisLen);
	return result;
}

static SELF_TYPE get_zero()
{
	SELF_TYPE result;
	for(size_t t = 0; t < NUM_ELEMS; ++t)
	{
		result[t] = DATA_TYPE(0.0);
	}

	return result;
}

/////////////////////////////////////////////////////////////////
//other member methods
/////////////////////////////////////////////////////////////////
DATA_TYPE& operator[](const unsigned int t)
{
	return d[t];
}

//---------------------------------------------------------
const DATA_TYPE& operator[](const unsigned int t) const
{
	return d[t];
}

//---------------------------------------------------------
SELF_TYPE operator+(const SELF_TYPE& v) const
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		r[t] = d[t] + v[t];
	}
	return r;
}

//---------------------------------------------------------
SELF_TYPE operator-(const SELF_TYPE& v) const
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		r[t] = d[t] - v[t];
	}
	return r;
}

//---------------------------------------------------------
SELF_TYPE operator*(const SELF_TYPE& v) const
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS, ++t)
	{
		r[t] = d[t] - v[t];
	}
	return;
}

//---------------------------------------------------------
SELF_TYPE operator*(const DATA_TYPE& s) const
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		r[t] = d[t] * s;
	}
	return r;
}

friend SELF_TYPE operator*(const DATA_TYPE& s, const SELF_TYPE& v)
{
	return v * s;
}

//---------------------------------------------------------
SELF_TYPE& operator+=(const SELF_TYPE& v)
{
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		d[t] += v[t];
	}
	return *this;
}

//---------------------------------------------------------
SELF_TYPE& operator-(const SELF_TYPE& v)
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		d[t] -= v[t];
	}
	return *this;
}

//---------------------------------------------------------
SELF_TYPE& operator*=(const DATA_TYPE& s)
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		d[t] *= s;
	}
	return *this;
}

//---------------------------------------------------------
DATA_TYPE hsum() const
{
	DATA_TYPE r = d[0];
	for(unsigned int t = 1; t < NUM_ELEMS; ++t)
	{
		r += d[t];
	}
	return r;
}

//---------------------------------------------------------
SELF_TYPE min_components(const SELF_TYPE& v) const
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		r[t] = sgm::pick_min(*this[t], v[t]);
	}
	return r;
}

//---------------------------------------------------------
SELF_TYPE max_components(const SELF_TYPE& v) const
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_ELEMS; ++t)
	{
		r[t] = sgm::pick_max(*this[t], v[t]);
	}
	return r;
}

//---------------------------------------------------------
DATA_TYPE dot(const SELF_TYPE& v) const
{
	DATA_TYPE r = d[0] * v[0];
	for(unsigned int t = 1; t < NUM_ELEMS; ++t)
	{
		r += d[t] * v[t];
	}
	return r;
}

//---------------------------------------------------------
DATA_TYPE lengthSqr() const
{
	return dot(*this);
}

//---------------------------------------------------------
DATA_TYPE length() const
{
	return sgm::sqrt(lengthSqr());
}

//---------------------------------------------------------
SELF_TYPE normalized() const
{
	const DATA_TYPE invLength = DATA_TYPE(1.0) / length();

	SELF_TYPE result;
	for(unsigned int t = 0; t < SELF_TYPE::NUM_ELEMS; ++t)
	{
		result[t] = d[t] * invLength;
	}

	return result;
}

//---------------------------------------------------------
SELF_TYPE reflect(const SELF_TYPE& normal) const
{
	return (*this) + DATA_TYPE(2.0) * dot(normal) * normal;
}

//---------------------------------------------------------
SELF_TYPE refract(const SELF_TYPE& normal, const DATA_TYPE& eta) const
{
	const DATA_TYPE one(1.0);
	const DATA_TYPE zero(0.0);

	const DATA_TYPE p = dot(normal);
	const DATA_TYPE k = one - eta*eta * (one -  p*p);
	if (k < zero)
		return SELF_TYPE(zero);
	else
		return (*this) * eta - (eta * p + sgm::sqrt(k)) * normal;
}

//---------------------------------------------------------
//global operators
//---------------------------------------------------------									
friend DATA_TYPE dot(const SELF_TYPE& a, const SELF_TYPE& b)			
{													
	return a.dot(b);								
}			
								
friend DATA_TYPE length(const SELF_TYPE& a)							
{													
	return a.length();								
}													
									
friend SELF_TYPE normalized(const SELF_TYPE& a)				
{													
	return a.normalized();							
}		

friend SELF_TYPE reflect(const SELF_TYPE& incident, const SELF_TYPE& normal)	
{													
	return incident.reflect(normal);							
}			

friend SELF_TYPE refract(const SELF_TYPE& inc, const SELF_TYPE& n, DATA_TYPE& eta)				
{													
	return i.refract(n, eta);
}