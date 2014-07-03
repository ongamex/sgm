//---------------------------------------------------
DATA_TYPE& at(unsigned const int r, const unsigned int c)
{
	return axis[r][c];
}

//---------------------------------------------------
const DATA_TYPE& at(unsigned const int r, const unsigned int c) const
{
	return axis[r][c];
}

//---------------------------------------------------
void identify_axis(const unsigned int i)
{
	rows[i] = SELF_TYPE::get_axis(i);
}

//---------------------------------------------------
void identify_axis_safe(const unsigned int i)
{
	if(i < NUM_ROW) rows[i] = SELF_TYPE::get_axis(i);
}

//---------------------------------------------------
SELF_TYPE operator*(const DATA_TYPE& s) const 
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		result[t] = axis[t] * s;
	}
	return result;
}

//---------------------------------------------------
SELF_TYPE& operator*=(const SELF_TYPE& s) 
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		axis[t] *= s;
	}
	return *this;
}

//---------------------------------------------------
SELF_TYPE operator+(const SELF_TYPE& m) const 
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		result[t] = axis[t] + m.axis[t];
	}
	return result;
}

//---------------------------------------------------
SELF_TYPE& operator+=(const SELF_TYPE& m) 
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		axis[t] += m.axis[t];
	}
	return *this;
}

//---------------------------------------------------
SELF_TYPE operator-(const SELF_TYPE& m) const
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		result[t] = axis[t] - m.axis[t];
	}
	return result;
}

//---------------------------------------------------
SELF_TYPE& operator-=(const SELF_TYPE& m) 
{
	SELF_TYPE result;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		axis[t] -= m.axis[t];
	}
	return *this;
}

//---------------------------------------------------
VEC_TYPE vec_mul_mat(const VEC_TYPE_PRE& v) const
{
	VEC_TYPE result(0);
	for(int t = 0; t < NUM_ROW; ++t)
	{
		result += axis[t] * v[t];
	}
	return result;
}

//---------------------------------------------------
VEC_TYPE_PRE mat_mul_vec(const VEC_TYPE& v) const
{
	VEC_TYPE_PRE result;
	for(int t = 0; t < NUM_ROW; ++t)
	{
		result[t] = dot(v, axis[t]);
	}
	return result;
}

//---------------------------------------------------
friend SELF_TYPE mul(const VEC_TYPE_PRE& v, const SELF_TYPE& m)
{
	return m.vec_mul_mat(v);
}

//---------------------------------------------------
friend SELF_TYPE mul(const SELF_TYPE&, const VEC_TYPE_PRE& v)
{
	return m.mat_mul_vec(v);
}

//---------------------------------------------------
friend SELF_TYPE mul(const SELF_TYPE& a, const SELF_TYPE& b)
{
	SELF_TYPE r;
	for(unsigned int t = 0; t < NUM_VECS; ++t)
	{
		r.axis[t] = mul(a.axis[t], b);
	}
	return r;
}