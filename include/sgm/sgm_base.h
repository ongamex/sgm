#pragma once

#ifndef SGM_SGM_BASE_H_07072014
#define SGM_SGM_BASE_H_07072014

#define SGE_BEGIN_MATH_NAMESPACE namespace sgm {
#define SGE_END_MATH_NAMESPACE }

#define SGE_MATH_FORCE_INLINE __forceinline
#define SGE_MATH_STATIC_ASSERT(condition, strLiteralMessage) static_assert(condition, strLiteralMessage)

#define SGM_GLOBAL_CONST __declspec(selectany) extern const

#endif