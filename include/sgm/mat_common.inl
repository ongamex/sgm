/////////////////////////////////////////////////////////////////
//[TODO][NOTE] Column those operations are 
//not ready for column major operation yet!!!
//Probably a local IS_COLUMN_MAJOR-ish macro will take place
//since there are few differences.
//
//This *.inl contains common matrix operation.
//The file is shared between several matrix classes.
//Each class that uses that file should have defined as members:
//
//SELF_TYPE - typedev <CLASS_NAME> SELF_TYPE;
//DATA_TYPE - the data storage type
//NUM_ROW and NUM_COL - matrix size in mathematical meaning 
//VEC_TYPE - storage vector type for (row/column)
//data[] - an array of type VEC_TYPE with size NUM_VECS
//NUM_VECS - the number of VEC_TYPEs needed to represent the matrix
//VEC_SIZE - the number of data elements in VEC_TYPE
//VEC_TYPE_PRE - premultiplication vector type
/////////////////////////////////////////////////////////////////

//---------------------------------------------------
//Member access methods
//[NOTE]operator[] isn't overloaded because of
//the mixed nature of the matrix storage layout.
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
//[TODO]Column major multiplication order
//---------------------------------------------------
friend SELF_TYPE mul(const VEC_TYPE_PRE& v, const SELF_TYPE& m)
{
	VEC_TYPE result(0);
	for(int t = 0; t < NUM_VECS; ++t)
	{
		result += data[t] * v[t];
	}
	return result;
}

//---------------------------------------------------
//Matrix * Vector
//[TODO]Column major multiplication order
//---------------------------------------------------
friend VEC_TYPE_PRE mul(const SELF_TYPE& m, const VEC_TYPE_PRE& v)
{
	VEC_TYPE_PRE result;
	for(int t = 0; t < NUM_VECS; ++t)
	{
		result[t] = dot(v, data[t]);
	}
	return result;
}

//---------------------------------------------------
//Matrix * Matrix mathematical multiplication
//[TODO]Column major multiplication order
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
	SELF_TYPE result;

	for(int t = 0; t < NUM_ROW; ++t)
	for(int s = 0; s < NUM_COL; ++s)
	{
		result.at(t,s) = at(s,t);
	}

	return result;
}

friend SELF_TYPE transpose(const SELF_TYPE& m)
{
	return m.transpose();
}