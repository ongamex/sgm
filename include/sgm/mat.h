#pragma once

#include "sgmath.h"

SGE_BEGIN_MATH_NAMESPACE

template<typename TDATA_TYPE>
struct mat
{
	typedef typename TDATA_TYPE DATA_TYPE;
	typedef mat SELF_TYPE;

	static const unsigned int NUM_ROW = 3;
	static const unsigned int NUM_COL = 4;

	typedef typename vec_type_picker<DATA_TYPE, NUM_COL>::value		VEC_TYPE;//the storage data type
	typedef typename vec_type_picker<DATA_TYPE, NUM_COL-1>::value	VEC_TYPE_PREV; //used for coord/normal transformations

	typedef typename vec_type_picker<DATA_TYPE, NUM_ROW>::value		VEC_TYPE_PRE;//the vector premultiplication type
	typedef typename vec_type_picker<DATA_TYPE, NUM_ROW-1>::value	VEC_TYPE_PRE_PREV; //used for coord/normal transformations

	VEC_TYPE rows[NUM_ROW];

	mat()
	{}

	mat(const VEC_TYPE& r0, const VEC_TYPE& r1, const VEC_TYPE& r2)
	{
		rows[0] = r0; rows[1] = r1; rows[2] = r2;
	}

	//---------------------------------------------------
	void identify_row(const unsigned int i)
	{
		rows[i] = SELF_TYPE::get_axis(i);
	}

	//---------------------------------------------------
	void identify_row_save(const unsigned int i)
	{
		if(i < NUM_ROW) rows[i] = SELF_TYPE::get_axis(i);
	}

	//---------------------------------------------------
	template<unsigned int i>
	void identify_row_safe()
	{
		if(i < NUM_ROW)
		rows[i] = SELF_TYPE::get_axis(i);
	}

	//---------------------------------------------------
	VEC_TYPE vec_mul_mat(const VEC_TYPE_PRE& v) const
	{
		VEC_TYPE result(0);
		for(int t = 0; t < NUM_ROW; ++t)
		{
			result += rows[t] * v[t];
		}
		return result;
	}

	//---------------------------------------------------
	VEC_TYPE_PRE mat_mul_vec(const VEC_TYPE& v) const
	{
		VEC_TYPE_PRE result(0);
		for(int t = 0; t < NUM_ROW; ++t)
		{
			result[t] += dot(v, rows[t]);
		}
		return result;
	}
};




SGE_END_MATH_NAMESPACE