// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "ErrorMinimizersImpl.h"
#include "PointMatcherPrivate.h"
#include "Eigen/SVD"

using namespace Eigen;

template<typename T>
PointToPointWithCovErrorMinimizer<T>::PointToPointWithCovErrorMinimizer(const Parameters& params):
	PointToPointErrorMinimizer<T>("PointToPointWithCovErrorMinimizer", availableParameters(), params),
	sensorStdDev(Parametrizable::get<T>("sensorStdDev"))
{
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPointWithCovErrorMinimizer<T>::compute(const ErrorElements& mPts_const)
{
	ErrorElements mPts = mPts_const;
	typename PointMatcher<T>::TransformationParameters result = PointToPointErrorMinimizer<T>::compute_in_place(mPts);
	
	this->covMatrix = this->estimateCovariance(mPts, result);
	
	return result;
}

//TODO: rewrite this using ErrorElements struct
template<typename T>
typename PointToPointWithCovErrorMinimizer<T>::Matrix PointToPointWithCovErrorMinimizer<T>::estimateCovariance(const ErrorElements& mPts, const TransformationParameters& transformation)
{
	const int max_nbr_point = mPts.reading.getNbPoints();
	
	Matrix covariance(Matrix::Zero(6,6));
	Matrix J_hessian(Matrix::Zero(6,6));
	Matrix d2J_dReadingdX(Matrix::Zero(6, max_nbr_point));
	Matrix d2J_dReferencedX(Matrix::Zero(6, max_nbr_point));
	
	Vector reading_point(Vector::Zero(3));
	Vector reference_point(Vector::Zero(3));
	Vector normal(3);
	Vector reading_direction(Vector::Zero(3));
	Vector reference_direction(Vector::Zero(3));
	
	normal << 1.0, 1.0, 1.0;    // Used for point-to-point computation
	
	T beta = -asin(transformation(2,0));
	T alpha = atan2(transformation(2,1), transformation(2,2));
	T gamma = atan2(transformation(1,0)/cos(beta), transformation(0,0)/cos(beta));
	T t_x = transformation(0,3);
	T t_y = transformation(1,3);
	T t_z = transformation(2,3);
	
	Vector tmp_vector_6(Vector::Zero(6));
	
	int valid_points_count = 0;
	
	//TODO: add missing const
	for(int i = 0; i < max_nbr_point; ++i)
	{
		//if (outlierWeights(0,i) > 0.0)
		{
			reading_point = mPts.reading.features.block(0,i,3,1);
			//int reference_idx = matches.ids(0,i);
			reference_point = mPts.reference.features.block(0,i,3,1);
			
			T reading_range = reading_point.norm();
			reading_direction = reading_point / reading_range;
			T reference_range = reference_point.norm();
			reference_direction = reference_point / reference_range;
			
			T n_alpha = normal(2)*reading_direction(1) - normal(1)*reading_direction(2);
			T n_beta = normal(0)*reading_direction(2) - normal(2)*reading_direction(0);
			T n_gamma = normal(1)*reading_direction(0) - normal(0)*reading_direction(1);
			
			T E = normal(0)*(reading_point(0) - gamma*reading_point(1) + beta*reading_point(2) + t_x - reference_point(0));
			E +=  normal(1)*(gamma*reading_point(0) + reading_point(1) - alpha*reading_point(2) + t_y - reference_point(1));
			E +=  normal(2)*(-beta*reading_point(0) + alpha*reading_point(1) + reading_point(2) + t_z - reference_point(2));
			
			T N_reading = normal(0)*(reading_direction(0) - gamma*reading_direction(1) + beta*reading_direction(2));
			N_reading +=  normal(1)*(gamma*reading_direction(0) + reading_direction(1) - alpha*reading_direction(2));
			N_reading +=  normal(2)*(-beta*reading_direction(0) + alpha*reading_direction(1) + reading_direction(2));
			
			T N_reference = -(normal(0)*reference_direction(0) + normal(1)*reference_direction(1) + normal(2)*reference_direction(2));
			
			// update the hessian and d2J/dzdx
			tmp_vector_6 << normal(0), normal(1), normal(2), reading_range * n_alpha, reading_range * n_beta, reading_range * n_gamma;
			
			J_hessian += tmp_vector_6 * tmp_vector_6.transpose();
			
			tmp_vector_6 << normal(0) * N_reading, normal(1) * N_reading, normal(2) * N_reading, n_alpha * (E + reading_range * N_reading), n_beta * (E + reading_range * N_reading), n_gamma * (E + reading_range * N_reading);
			
			d2J_dReadingdX.block(0,valid_points_count,6,1) = tmp_vector_6;
			
			tmp_vector_6 << normal(0) * N_reference, normal(1) * N_reference, normal(2) * N_reference, reference_range * n_alpha * N_reference, reference_range * n_beta * N_reference, reference_range * n_gamma * N_reference;
			
			d2J_dReferencedX.block(0,valid_points_count,6,1) = tmp_vector_6;
			
			valid_points_count++;
		} // if (outlierWeights(0,i) > 0.0)
	}
	
	Matrix d2J_dZdX(Matrix::Zero(6, 2 * valid_points_count));
	d2J_dZdX.block(0,0,6,valid_points_count) = d2J_dReadingdX.block(0,0,6,valid_points_count);
	d2J_dZdX.block(0,valid_points_count,6,valid_points_count) = d2J_dReferencedX.block(0,0,6,valid_points_count);
	
	Matrix inv_J_hessian = J_hessian.inverse();
	
	covariance = d2J_dZdX * d2J_dZdX.transpose();
	covariance = inv_J_hessian * covariance * inv_J_hessian;
	
	return (sensorStdDev * sensorStdDev) * covariance;
}

template<typename T>
typename PointToPointWithCovErrorMinimizer<T>::Matrix PointToPointWithCovErrorMinimizer<T>::getCovariance() const
{
	return covMatrix;
}

template struct PointToPointWithCovErrorMinimizer<float>;
template struct PointToPointWithCovErrorMinimizer<double>;