//---------------------------------------------------
//
//---------------------------------------------------
DATA_TYPE& at(unsigned const int r, const unsigned int c)
{
	return data[r][c];
}

const DATA_TYPE& at(unsigned const int r, const unsigned int c) const
{
	return data[r][c];
}

void identify_axis(const unsigned int i)
{
	data[i] = VEC_TYPE::get_axis(i);
}

//---------------------------------------------------
//Scalars operator* and *=
//---------------------------------------------------
SELF_TYPE operator*(const DATA_TYPE& s) const 
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		result[t] = data[t] * s;
	}
	return result;
}

friend SELF_TYPE operator*(const DATA_TYPE& s, const SELF_TYPE& m)
{
	return m * s;
}

SELF_TYPE& operator*=(const SELF_TYPE& s) 
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		data[t] *= s;
	}
	return *this;
}

//---------------------------------------------------
//operator+ and +=
//---------------------------------------------------
SELF_TYPE operator+(const SELF_TYPE& m) const 
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		result[t] = data[t] + m.data[t];
	}
	return result;
}

SELF_TYPE& operator+=(const SELF_TYPE& m) 
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		data[t] += m.data[t];
	}
	return *this;
}

//---------------------------------------------------
//operator- and -=
//---------------------------------------------------
SELF_TYPE operator-(const SELF_TYPE& m) const
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		result[t] = data[t] - m.data[t];
	}
	return result;
}

SELF_TYPE& operator-=(const SELF_TYPE& m) 
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		data[t] -= m.data[t];
	}
	return *this;
}

//---------------------------------------------------
//Vector * Matrix
//---------------------------------------------------
friend SELF_TYPE mul(const VEC_TYPE_PRE& v, const SELF_TYPE& m)
{
	VEC_TYPE result(0);
	for(int t = 0; t < NUM_ROW; ++t)
	{
		result += data[t] * v[t];
	}
	return result;
}

//---------------------------------------------------
//Matrix * Vector
//---------------------------------------------------
friend SELF_TYPE mul(const SELF_TYPE& m, const VEC_TYPE_PRE& v)
{
		VEC_TYPE_PRE result;
	for(int t = 0; t < NUM_ROW; ++t)
	{
		result[t] = dot(v, data[t]);
	}
	return result;
}

//---------------------------------------------------
//Matrix * Matrix mathematical multiplication
//---------------------------------------------------
friend SELF_TYPE mul(const SELF_TYPE& a, const SELF_TYPE& b)
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		r.data[t] = mul(a.data[t], b);
	}
	return r;
}

//---------------------------------------------------
//Matrix * Matrix componentwise multiplication
//---------------------------------------------------
friend SELF_TYPE cmul(const SELF_TYPE& a, const SELF_TYPE& b)
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		r.data[t] = cmul(a.data[t], b.data[t]);
	}
	return r;
}

//---------------------------------------------------
//transpose
//---------------------------------------------------
SELF_TYPE transpose() const
{
	SELF_TYPE r;

	for(int t = 0; t < NUM_ROW; ++t)
	for(int s = 0; s < NUM_COL; ++s)
	{
		r.at(t,s) = at(s,t);
	}

	return r;
}

friend SELF_TYPE transpose(const SELF_TYPE& m)
{
	return m.transpose();
}