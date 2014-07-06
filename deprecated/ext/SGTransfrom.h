////////////////////////////////////////////////////////////////////////////////////
//SGTransform.h
//[NOTE]: This is SEPARATE and OPTIONAL extension to the math library.
//Transformation order is applyed as follow: Scaling->Rotation->Translation
////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "Math\SGMath.h"

class SGTransform  : public TAlignedNew<SceneNode>
{

public :

	SGTransform(const SGVector& _scaling = SGVector::V1111, 
		const SGVector& rotationQuat = SGVector::V0001, 
		const SGVector& _translation = SGVector::V0000):
		scaling(_scaling),
		rotation(rotationQuat),
		translation(_translation)
	{}

	//---------------------------------------------------------------------
	SGE_FORCE_INLINE SGTransform GetInverse() const 
	{
		return SGTransform( vec3_inverse(scaling),
		quat_conjurgate(rotation),
		vec4_negative(translation));
	}

	//---------------------------------------------------------------------
	SGE_FORCE_INLINE SGMatrix4 AsMatrix4_RM() const
	{
		SGMatrix4 transform = mat4_scaling_RM(scaling);
		transform = mat4_mul(transform, mat4_rotationQuat_RM(rotation));
		transform = mat4_mul(transform, mat4_translation_RM(translation));

		return transform;
	}

	//---------------------------------------------------------------------
	SGE_FORCE_INLINE SGTransform CombinationWith(const SGTransform& t) const
	{
		return SGTransform(vec3_mul(scaling, t.scaling),
				quat_mul(rotation, t.rotation),
				vec3_add(translation, t.translation));
	}

	SGE_FORCE_INLINE SGTransform Interpolate(const SGTransform& t, const float f) const
	{
		return SGTransform(
			SGVector(1.f),//(scaling, vec3_scale(vec3_sub(t.scaling, scaling), f)),
			quat_slerp(rotation, t.rotation, f , 0.001f),
			vec3_add(translation, vec3_scale(vec3_sub(t.translation, translation), f))
			);
	}

public :

	SGVector	scaling;
	SGVector	rotation;		//quaternion
	SGVector	translation;	//
};

SGE_FORCE_INLINE SGTransform operator*(const SGTransform& a, const SGTransform& b)
{
	return a.CombinationWith(b);
}