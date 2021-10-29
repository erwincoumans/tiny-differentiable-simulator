#ifndef _MATRIX_UTILS_HPP
#define _MATRIX_UTILS_HPP

#pragma once

#include "math/conditionals.hpp"

namespace tds {
  
template <typename Algebra> 
typename Algebra::Scalar get_matrix_elem(const typename Algebra::Matrix3& mat, int index)
{
	int i = index % 3;
	int j = index / 3;
	return mat(i,j);
}

template <typename Algebra> 
static typename Algebra::Vector3 matrix_to_euler_xyz(const typename Algebra::Matrix3& mat)
{
	typename Algebra::Vector3 xyz;
    using Scalar = typename Algebra::Scalar;

	// rot =  cy*cz          -cy*sz           sy
	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy

	Scalar fi = get_matrix_elem<Algebra>(mat, 2);
	xyz[0] = tds::where_le(fi, Algebra::one(),
		tds::where_ge(fi, -Algebra::one(),
			Algebra::atan2(-get_matrix_elem<Algebra>(mat, 5), get_matrix_elem<Algebra>(mat, 8)),
			xyz[0] = -Algebra::atan2(get_matrix_elem<Algebra>(mat, 3), get_matrix_elem<Algebra>(mat, 4))),
		Algebra::atan2(get_matrix_elem<Algebra>(mat, 3), get_matrix_elem<Algebra>(mat, 4)));

	xyz[1] = tds::where_le(fi, Algebra::one(),
		tds::where_ge(fi, -Algebra::one(),
			Algebra::asin(get_matrix_elem<Algebra>(mat, 2)),
			-Algebra::half_pi()),
		Algebra::half_pi());

	xyz[2] = tds::where_le(fi, Algebra::one(),
		tds::where_ge(fi, -Algebra::one(),
			Algebra::atan2(-get_matrix_elem<Algebra>(mat, 1), get_matrix_elem<Algebra>(mat, 0)),
			 Algebra::zero()),
		Algebra::zero());
#if 0
	if (fi < Scalar(1.0f))
	{
		if (fi > Scalar(-1.0f))
		{
			assert (xyz[0] == Algebra::atan2(-get_matrix_elem<Algebra>(mat, 5), get_matrix_elem<Algebra>(mat, 8)));
			assert (xyz[1] == Algebra::asin(get_matrix_elem<Algebra>(mat, 2)));
			assert (xyz[2] == Algebra::atan2(-get_matrix_elem<Algebra>(mat, 1), get_matrix_elem<Algebra>(mat, 0)));
		}
		else
		{
			// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
			assert (xyz[0] == -Algebra::atan2(get_matrix_elem<Algebra>(mat, 3), get_matrix_elem<Algebra>(mat, 4)));
			assert (xyz[1] == -Algebra::half_pi());
			assert (xyz[2] == Scalar(0.0));
		}
	}
	else
	{
		// WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
		assert (xyz[0] == Algebra::atan2(get_matrix_elem<Algebra>(mat, 3), get_matrix_elem<Algebra>(mat, 4)));
		assert (xyz[1] == Algebra::half_pi());
		assert (xyz[2] == 0.0);
	}
#endif

	return xyz;
}


template <typename Algebra> 
typename Algebra::Vector3 get_axis_difference_quaternion(const typename Algebra::Quaternion& currentQuat, const typename Algebra::Quaternion desiredQuat)
{
  typename Algebra::Vector3 axisOut;
    
	auto relRot = Algebra::inverse(currentQuat) * desiredQuat;
	typename Algebra::Vector3 angleDiff = matrix_to_euler_xyz<Algebra>(Algebra::quat_to_matrix(relRot));
    //angleDiff = Algebra::get_euler_rpy(relRot);
    
	axisOut[0] = angleDiff[0];
	axisOut[1] = angleDiff[1];
	axisOut[2] = angleDiff[2];
	return axisOut;
}
};//namespace tds
#endif //_MATRIX_UTILS_HPP
