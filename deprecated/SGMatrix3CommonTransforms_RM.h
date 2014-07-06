#include "SGMatrix34.h"

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix34 mat3_scaling_RM(const SGVector& s)
{
	SGMatrix34 r;

	r.m_rows[0].setValue(s.x, 0.f, 0.f, 0.f);
	r.m_rows[1].setValue(0.f, s.y, 0.f, 0.f);
	r.m_rows[2].setValue(0.f, 0.f, s.z, 0.f);

	return r;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//Rotations
/////////////////////////////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGMatrix34 mat3_rotationX_RM(const float theta)
{
	const float cost = cosf(theta);
	const float sint = sinf(theta);

	SGMatrix34 r;
	r.m_rows[0] = SGVector::V1000;
	r.m_rows[1].setValue(0.f, cost, sint, 0.f);
	r.m_rows[2].setValue(0.f, -sint, cost, 0.f);

	return r;
}

//------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix34 mat3_rotationY_RM(const float theta)
{
	
	const float cost = cosf(theta);
	const float sint = sinf(theta);

	SGMatrix34 r;
	r.m_rows[0].setValue(cost, 0.f, -sint, 0.f);
	r.m_rows[1] = SGVector::V0100;
	r.m_rows[2].setValue(sint, 0.f, cost, 0.f);

	return r;
}

//------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix34 mat3_rotationZ_RM(const float theta)
{
	const float cost = cosf(theta);
	const float sint = sinf(theta);

	SGMatrix34 r;
	r.m_rows[0].setValue(cost, sint, 0.f, 0.f);
	r.m_rows[1].setValue(-sint, cost, 0.f, 0.f);
	r.m_rows[2] = SGVector::V0010;
	
	return r;
}

//------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix34 mat3_rotationZYX_RM(const float z, const float y, const float x)
{
	float ci ( cosf(x)); 
	float cj ( cosf(y)); 
	float ch ( cosf(z)); 
	float si ( sinf(x)); 
	float sj ( sinf(y)); 
	float sh ( sinf(z)); 
	float cc = ci * ch; 
	float cs = ci * sh; 
	float sc = si * ch; 
	float ss = si * sh;

	SGMatrix34 r;
	r.m_rows[0].setValue(cj * ch, cj * sh, -sj, 0.f);
	r.m_rows[1].setValue(sj * sc - cs, sj * ss + cc, cj * si, 0.f);
	r.m_rows[2].setValue(sj * cc + ss,  sj * cs - sc,  cj * ci, 0.f);

	return r;
}

//------------------------------------------------------------------------------
//
SGE_FORCE_INLINE SGMatrix34 mat3_rotationQuat_RM(const SGVector& q)
{
	const float x = q.x;
	const float y = q.y;
	const float z = q.z;
	const float w = q.w;

	SGMatrix34 r;
	r.m_rows[0].setValue(	1.f - 2.f*(y*y - z*z), 
							2.f*(x*y + z*w),
							2.f*(x*z - y*w),
							0.f);

	r.m_rows[1].setValue(	2.f*(x*y - z*w),
							1.f-2.f*(x*x - z*z),
							2.f*(y*z + x*w),
							0.f);

	r.m_rows[2].setValue(	2.f*(x*z + y*w),
							2.f*(y*z - x*w),
							1.f - 2.f*(x*x - y*y),
							0.f);

	/*r.m_rows[0].setValue(	2.f*(x*x + w*w) - 1.f, 
							2.f*(x*y + z*w),
							2.f*(x*z - y*w),
							0.f);

	r.m_rows[1].setValue(	2.f*(x*y - z*w),
							2.f*(y*y + w*w) - 1.f,
							2.f*(y*z + x*w),
							0.f);

	r.m_rows[2].setValue(	2.f*(x*z + y*w),
							2.f*(y*z - x*w),
							2.f*(z*z + w*w) - 1.f,
							0.f);
*/
	return r;
}

//------------------------------------------------------------------------------
//[TODO]
SGE_FORCE_INLINE void mat3_decompose_RM(const SGMatrix34& m, SGVector& scaling, SGMatrix34& rot)
{
	using Math::sqr;

	scaling.x = sqrtf(sqr(m._00()) + sqr(m._01()) + sqr(m._02()));
	scaling.y = sqrtf(sqr(m._10()) + sqr(m._11()) + sqr(m._12()));
	scaling.z = sqrtf(sqr(m._20()) + sqr(m._21()) + sqr(m._22()));
	scaling.w = 1.f;

	const SGVector invScale = vec3_inverse(scaling);

	rot.m_rows[0] = vec3_mul(invScale, m.m_rows[0]);
	rot.m_rows[1] = vec3_mul(invScale, m.m_rows[1]);
	rot.m_rows[2] = vec3_mul(invScale, m.m_rows[2]);
}

#define SG3RANKDECOMPOSE(a, b, c, x, y, z)      \
    if((x) < (y))                   \
    {                               \
        if((y) < (z))               \
        {                           \
            (a) = 2;                \
            (b) = 1;                \
            (c) = 0;                \
        }                           \
        else                        \
        {                           \
            (a) = 1;                \
                                    \
            if((x) < (z))           \
            {                       \
                (b) = 2;            \
                (c) = 0;            \
            }                       \
            else                    \
            {                       \
                (b) = 0;            \
                (c) = 2;            \
            }                       \
        }                           \
    }                               \
    else                            \
    {                               \
        if((x) < (z))               \
        {                           \
            (a) = 2;                \
            (b) = 0;                \
            (c) = 1;                \
        }                           \
        else                        \
        {                           \
            (a) = 0;                \
                                    \
            if((y) < (z))           \
            {                       \
                (b) = 2;            \
                (c) = 1;            \
            }                       \
            else                    \
            {                       \
                (b) = 1;            \
                (c) = 2;            \
            }                       \
        }                           \
    }
                                    
#define XM3_DECOMP_EPSILON 0.0001f

inline bool mat3_decompose2_RM(const SGMatrix34& m, SGVector& outScale, SGMatrix34& rot)
{
	static const SGVector *pvCanonicalBasis[3] = {
		&SGVector::V1000,
		&SGVector::V0100,
		&SGVector::V0010
    };

    SGVector *ppvBasis[3];
	SGMatrix34 matTemp;
	ppvBasis[0] = &matTemp.m_rows[0];
    ppvBasis[1] = &matTemp.m_rows[1];
    ppvBasis[2] = &matTemp.m_rows[2];

    matTemp.m_rows[0] = m.m_rows[0];
    matTemp.m_rows[1] = m.m_rows[1];
    matTemp.m_rows[2] = m.m_rows[2];

    float *pfScales = outScale.m_floats;

    size_t a, b, c;
	pfScales[0] = vec3_lenght(*ppvBasis[0]); 
    pfScales[1] = vec3_lenght(*ppvBasis[1]); 
    pfScales[2] = vec3_lenght(*ppvBasis[2]); 
    pfScales[3] = 0.f;

    SG3RANKDECOMPOSE(a, b, c, pfScales[0], pfScales[1], pfScales[2])

    if(pfScales[a] < XM3_DECOMP_EPSILON)
    {
		ppvBasis[a]->m_floats[0] = pvCanonicalBasis[a]->m_floats[0];
    }
	*ppvBasis[a] = vec3_normalized(*ppvBasis[a]);

    if(pfScales[b] < XM3_DECOMP_EPSILON)
    {
        size_t aa, bb, cc;
        float fAbsX, fAbsY, fAbsZ;

		fAbsX = fabsf(ppvBasis[a]->x);
        fAbsY = fabsf(ppvBasis[a]->y);
        fAbsZ = fabsf(ppvBasis[a]->z);

        SG3RANKDECOMPOSE(aa, bb, cc, fAbsX, fAbsY, fAbsZ)

		*ppvBasis[b] = vec3_cross(*ppvBasis[a], *pvCanonicalBasis[cc]);
    }

    *ppvBasis[b] = vec3_normalized(*ppvBasis[b]);

    if(pfScales[c] < XM3_DECOMP_EPSILON)
    {
        *ppvBasis[c] = vec3_cross(*ppvBasis[a],*ppvBasis[b]);
    }
        
    *ppvBasis[c] = vec3_normalized(*ppvBasis[c]);

	float fDet = mat3_determinant(matTemp);

    // use Kramer's rule to check for handedness of coordinate system
    if(fDet < 0.0f)
    {
        // switch coordinate system by negating the scale and inverting the basis vector on the x-axis
        pfScales[a] = -pfScales[a];
		*ppvBasis[a] = vec4_negative(*ppvBasis[a]);

        fDet = -fDet;
    }

    fDet -= 1.0f;
    fDet *= fDet;

    if(XM3_DECOMP_EPSILON < fDet)
    {
        // Non-SRT matrix encountered
        return false;
    }

    // generate the quaternion from the matrix
    rot = (matTemp);
    return true;
}

#undef SG3RANKDECOMPOSE