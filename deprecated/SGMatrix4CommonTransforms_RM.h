/////////////////////////////////////////////////////////////////////////////////
//Row major matrix4 transfromations
////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "SGMatrix4.h"
#include "SGMatrix34.h"

/////////////////////////////////////////////////////////////////////////////////
//Scaling
/////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGMatrix4 mat4_scaling_RM(const SGVector& s)
{
	SGMatrix4 r;

	r.m_rows[0].setValue(s.x, 0.f, 0.f, 0.f);
	r.m_rows[1].setValue(0.f, s.y, 0.f, 0.f);
	r.m_rows[2].setValue(0.f, 0.f, s.z, 0.f);
	r.m_rows[3] = SGVector::V0001;

	return r;
}

/////////////////////////////////////////////////////////////////////////////////
//Translation
/////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGMatrix4 mat4_translation_RM(const SGVector& v)
{
	return SGMatrix4(	SGVector::V1000,
						SGVector::V0100,
						SGVector::V0010,
						SGVector(v.x, v.y, v.z, 1.f));
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix4 mat4_translation_RM(const float x, const float y, const float z)
{
	return SGMatrix4(	SGVector::V1000,
						SGVector::V0100,
						SGVector::V0010,
						SGVector(x, y, z, 1.f));
}

/////////////////////////////////////////////////////////////////////////////////
//Rotations
/////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE SGMatrix4 mat4_rotationX_RM(const float theta)
{
	return SGMatrix4(mat3_rotationX_RM(theta));
}

//------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix4 mat4_rotationY_RM(const float theta)
{
	return SGMatrix4(mat3_rotationY_RM(theta));
}

//------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix4 mat4_rotationZ_RM(const float theta)
{
	return SGMatrix4(mat3_rotationZ_RM(theta));
}

//------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix4 mat4_rotationZYX_RM(const float z, const float y, const float x)
{
	return SGMatrix4(mat3_rotationZYX_RM(z, y, x));
}

//------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix4 mat4_rotationQuat_RM(const SGVector& q)
{
	return SGMatrix4(mat3_rotationQuat_RM(q));
}

/////////////////////////////////////////////////////////////////////////////////
//Projections
/////////////////////////////////////////////////////////////////////////////////
SGE_FORCE_INLINE void mat4_perspectiveFovLH_RM(SGMatrix4& m, 
	const float FovAngleY, 
    const float AspectHByW, 
    const float NearZ, 
    const float FarZ)
{
	const float    SinFov = sinf( FovAngleY * 0.5f);
	const float    CosFov = cosf( FovAngleY * 0.5f);

	const float Height = CosFov / SinFov;
    const float Width = Height / AspectHByW;
    const float fRange = FarZ / (FarZ - NearZ);

	m.m_rows[0] = SGVector(Width, 0.f, 0.f, 0.f);
	m.m_rows[1] = SGVector(0.f, Height, 0.f, 0.f);
	m.m_rows[2] = SGVector(0.f, 0.f, fRange, 1.f);
	m.m_rows[3] = SGVector(0.f, 0.f, -fRange * NearZ, 0.f);
}

//-------------------------------------------------------------------------------
SGE_FORCE_INLINE SGMatrix4 mat4_perspectiveFovRH_RM( 
	const float FovAngleY, 
    const float AspectWByH, 
    const float NearZ, 
    const float FarZ)
{
	SGMatrix4 m;
	const float yScale = tanf(Math::HALF_PI - (FovAngleY/2.f));
	const float xScale = yScale / AspectWByH;
    const float fRange = FarZ / (NearZ - FarZ);

	m.m_rows[0] = SGVector(xScale, 0.f, 0.f, 0.f);
	m.m_rows[1] = SGVector(0.f, yScale, 0.f, 0.f);
	m.m_rows[2] = SGVector(0.f, 0.f, fRange, -1.f);
	m.m_rows[3] = SGVector(0.f, 0.f, fRange * NearZ, 0.f);

	return m;
}

/////////////////////////////////////////////////////////////////////////////////
//Views
/////////////////////////////////////////////////////////////////////////////////
//Functions expects normalized directions
SGE_FORCE_INLINE SGMatrix4 mat4_lookAtLH_RM(
	const SGVector& eyePos,
	const SGVector& atPoint,
	const SGVector& upDir
)
{
	SGMatrix4 m;
	const SGVector zaxis = vec3_normalized(vec3_sub(atPoint, eyePos));
	const SGVector xaxis = vec3_cross(upDir, zaxis);
	const SGVector yaxis = vec3_cross(zaxis, xaxis);

	SGVector T = vec3_dot3(eyePos, xaxis, yaxis, zaxis);
	T = vec4_negative(T);

	//row 0
	m.m_rows[0].x = xaxis.x;
	m.m_rows[0].y = yaxis.x;
	m.m_rows[0].z = zaxis.x;
	m.m_rows[0].w = 0.f;

	//row 1
	m.m_rows[1].x = xaxis.y;
	m.m_rows[1].x = yaxis.y;
	m.m_rows[1].x = zaxis.y;
	m.m_rows[1].x = 0;

	//row 2
	m.m_rows[2].x = xaxis.z;
	m.m_rows[2].x = yaxis.z;
	m.m_rows[2].x = zaxis.z;
	m.m_rows[2].x = 0;

	//row 3
	m.m_rows[3] = T;
	m.m_rows[3].w = 1.f;

	return m;
}

//--------------------------------------------------------------------------------
//Functions expects normalized directions
SGE_FORCE_INLINE SGMatrix4 mat4_lookAtRH_RM( 
	const SGVector& eyePos,
	const SGVector& atPoint,
	const SGVector& upDir )
{

	SGMatrix4 m;

	const SGVector zaxis = vec3_normalized(vec3_sub(eyePos, atPoint));
	const SGVector xaxis = vec3_cross(upDir, zaxis);
	const SGVector yaxis = vec3_cross(zaxis, xaxis);

	SGVector T = vec3_dot3(eyePos, xaxis, yaxis, zaxis);
	T = vec4_negative(T);

	//row 0
	m.m_rows[0].x = xaxis.x;
	m.m_rows[0].y = yaxis.x;
	m.m_rows[0].z = zaxis.x;
	m.m_rows[0].w = 0.f;

	//row 1
	m.m_rows[1].x = xaxis.y;
	m.m_rows[1].y = yaxis.y;
	m.m_rows[1].z = zaxis.y;
	m.m_rows[1].w = 0;

	//row 2
	m.m_rows[2].x = xaxis.z;
	m.m_rows[2].y = yaxis.z;
	m.m_rows[2].z = zaxis.z;
	m.m_rows[2].w = 0;

	//row 3
	m.m_rows[3] = T;
	m.m_rows[3].w = 1.f;

	return m;
}