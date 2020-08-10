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

#include <iostream>

#include "Eigen/SVD"

#include "ErrorMinimizersImpl.h"
#include "PointMatcherPrivate.h"
#include "Functions.h"

using namespace Eigen;
using namespace std;

typedef PointMatcherSupport::Parametrizable Parametrizable;
typedef PointMatcherSupport::Parametrizable P;
typedef Parametrizable::Parameters Parameters;
typedef Parametrizable::ParameterDoc ParameterDoc;
typedef Parametrizable::ParametersDoc ParametersDoc;

template<typename T>
PointToPlaneErrorMinimizer<T>::PointToPlaneErrorMinimizer(const Parameters& params):
	ErrorMinimizer(name(), availableParameters(), params),
	force2D(Parametrizable::get<T>("force2D"))
{
	LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 2D.");
}

template<typename T>
PointToPlaneErrorMinimizer<T>::PointToPlaneErrorMinimizer(const ParametersDoc paramsDoc, const Parameters& params):
	ErrorMinimizer(name(), paramsDoc, params),
	force2D(Parametrizable::get<T>("force2D"))
{
	LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 2D.");
}


template<typename T, typename MatrixA, typename Vector>
void solvePossiblyUnderdeterminedLinearSystem(const MatrixA& A, const Vector & b, Vector & x) {
	assert(A.cols() == A.rows());
	assert(b.cols() == 1);
	assert(b.rows() == A.rows());
	assert(x.cols() == 1);
	assert(x.rows() == A.cols());

	typedef typename PointMatcher<T>::Matrix Matrix;

	BOOST_AUTO(Aqr, A.fullPivHouseholderQr());
	if (!Aqr.isInvertible())
	{
		// Solve reduced problem R1 x = Q1^T b instead of QR x = b, where Q = [Q1 Q2] and R = [ R1 ; R2 ] such that ||R2|| is small (or zero) and therefore A = QR ~= Q1 * R1
		const int rank = Aqr.rank();
		const int rows = A.rows();
		const Matrix Q1t = Aqr.matrixQ().transpose().block(0, 0, rank, rows);
		const Matrix R1 = (Q1t * A * Aqr.colsPermutation()).block(0, 0, rank, rows);

		const bool findMinimalNormSolution = true; // TODO is that what we want?

		// The under-determined system R1 x = Q1^T b is made unique ..
		if(findMinimalNormSolution){
			// by getting the solution of smallest norm (x = R1^T * (R1 * R1^T)^-1 Q1^T b.
			x = R1.template triangularView<Eigen::Upper>().transpose() * (R1 * R1.transpose()).llt().solve(Q1t * b);
		} else {
			// by solving the simplest problem that yields fewest nonzero components in x
			x.block(0, 0, rank, 1) = R1.block(0, 0, rank, rank).template triangularView<Eigen::Upper>().solve(Q1t * b);
			x.block(rank, 0, rows - rank, 1).setZero();
		}

		x = Aqr.colsPermutation() * x;

		BOOST_AUTO(ax , (A * x).eval());
		if (!b.isApprox(ax, 1e-5)) {
			LOG_INFO_STREAM("PointMatcher::icp - encountered almost singular matrix while minimizing point to plane distance. QR solution was too inaccurate. Trying more accurate approach using double precision SVD.");
			x = A.template cast<double>().jacobiSvd(ComputeThinU | ComputeThinV).solve(b.template cast<double>()).template cast<T>();
			ax = A * x;

			if((b - ax).norm() > 1e-5 * std::max(A.norm() * x.norm(), b.norm())){
				LOG_WARNING_STREAM("PointMatcher::icp - encountered numerically singular matrix while minimizing point to plane distance and the current workaround remained inaccurate."
						<< " b=" << b.transpose()
						<< " !~ A * x=" << (ax).transpose().eval()
						<< ": ||b- ax||=" << (b - ax).norm()
						<< ", ||b||=" << b.norm()
						<< ", ||ax||=" << ax.norm());
			}
		}
	}
	else {
		// Cholesky decomposition
		x = A.llt().solve(b);
	}
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPlaneErrorMinimizer<T>::compute(const ErrorElements& mPts_const)
{
		ErrorElements mPts = mPts_const;
		return compute_in_place(mPts);
}


template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPlaneErrorMinimizer<T>::compute_in_place(ErrorElements& mPts)
{
		const int dim = mPts.reading.features.rows();
		const int nbPts = mPts.reading.features.cols();

		// Adjust if the user forces 2D minimization on XY-plane
		int forcedDim = dim - 1;
		if(force2D && dim == 4)
		{
				mPts.reading.features.conservativeResize(3, Eigen::NoChange);
				mPts.reading.features.row(2) = Matrix::Ones(1, nbPts);
				mPts.reference.features.conservativeResize(3, Eigen::NoChange);
				mPts.reference.features.row(2) = Matrix::Ones(1, nbPts);
				forcedDim = dim - 2;
		}

		// Fetch normal vectors of the reference point cloud (with adjustment if needed)
		const BOOST_AUTO(normalRef, mPts.reference.getDescriptorViewByName("normals").topRows(forcedDim));

		// Note: Normal vector must be precalculated to use this error. Use appropriate input filter.
		assert(normalRef.rows() > 0);

		// Compute cross product of cross = cross(reading X normalRef)
		const Matrix cross = this->crossProduct(mPts.reading.features, normalRef);

		// wF = [weights*cross, weights*normals]
		// F  = [cross, normals]
		Matrix wF(normalRef.rows()+ cross.rows(), normalRef.cols());
		Matrix F(normalRef.rows()+ cross.rows(), normalRef.cols());

		for(int i=0; i < cross.rows(); i++)
		{
				wF.row(i) = mPts.weights.array() * cross.row(i).array();
				F.row(i) = cross.row(i);
		}
		for(int i=0; i < normalRef.rows(); i++)
		{
				wF.row(i + cross.rows()) = mPts.weights.array() * normalRef.row(i).array();
				F.row(i + cross.rows()) = normalRef.row(i);
		}

		// Unadjust covariance A = wF * F'
		const Matrix A = wF * F.transpose();

		const Matrix deltas = mPts.reading.features - mPts.reference.features;

		// dot product of dot = dot(deltas, normals)
		Matrix dotProd = Matrix::Zero(1, normalRef.cols());

		for(int i=0; i<normalRef.rows(); i++)
		{
				dotProd += (deltas.row(i).array() * normalRef.row(i).array()).matrix();
		}

		// b = -(wF' * dot)
		const Vector b = -(wF * dotProd.transpose());

		Vector x(A.rows());

		solvePossiblyUnderdeterminedLinearSystem<T>(A, b, x);

		// Transform parameters to matrix
		Matrix mOut;
		if(dim == 4 && !force2D)
		{
				Eigen::Transform<T, 3, Eigen::Affine> transform;
				// PLEASE DONT USE EULAR ANGLES!!!!
				// Rotation in Eular angles follow roll-pitch-yaw (1-2-3) rule
				/*transform = Eigen::AngleAxis<T>(x(0), Eigen::Matrix<T,1,3>::UnitX())
				 * Eigen::AngleAxis<T>(x(1), Eigen::Matrix<T,1,3>::UnitY())
				 * Eigen::AngleAxis<T>(x(2), Eigen::Matrix<T,1,3>::UnitZ());*/

				transform = Eigen::AngleAxis<T>(x.head(3).norm(),x.head(3).normalized());

				// Reverse roll-pitch-yaw conversion, very useful piece of knowledge, keep it with you all time!
				/*const T pitch = -asin(transform(2,0));
					const T roll = atan2(transform(2,1), transform(2,2));
					const T yaw = atan2(transform(1,0) / cos(pitch), transform(0,0) / cos(pitch));
					std::cerr << "d angles" << x(0) - roll << ", " << x(1) - pitch << "," << x(2) - yaw << std::endl;*/
				transform.translation() = x.segment(3, 3);
				mOut = transform.matrix();

				if (mOut != mOut)
				{
						// Degenerate situation. This can happen when the source and reading clouds
						// are identical, and then b and x above are 0, and the rotation matrix cannot
						// be determined, it comes out full of NaNs. The correct rotation is the identity.
						mOut.block(0, 0, dim-1, dim-1) = Matrix::Identity(dim-1, dim-1);
				}
		}
		else
		{
				Eigen::Transform<T, 2, Eigen::Affine> transform;
				transform = Eigen::Rotation2D<T> (x(0));
				transform.translation() = x.segment(1, 2);

				if(force2D)
				{
						mOut = Matrix::Identity(dim, dim);
						mOut.topLeftCorner(2, 2) = transform.matrix().topLeftCorner(2, 2);
						mOut.topRightCorner(2, 1) = transform.matrix().topRightCorner(2, 1);
				}
				else
				{
						mOut = transform.matrix();
				}
		}
		return mOut;
}


template<typename T>
T PointToPlaneErrorMinimizer<T>::computeResidualError(ErrorElements mPts, const bool& force2D)
{
	const int dim = mPts.reading.features.rows();
	const int nbPts = mPts.reading.features.cols();

	// Adjust if the user forces 2D minimization on XY-plane
	int forcedDim = dim - 1;
	if(force2D && dim == 4)
	{
		mPts.reading.features.conservativeResize(3, Eigen::NoChange);
		mPts.reading.features.row(2) = Matrix::Ones(1, nbPts);
		mPts.reference.features.conservativeResize(3, Eigen::NoChange);
		mPts.reference.features.row(2) = Matrix::Ones(1, nbPts);
		forcedDim = dim - 2;
	}

	// Fetch normal vectors of the reference point cloud (with adjustment if needed)
	const BOOST_AUTO(normalRef, mPts.reference.getDescriptorViewByName("normals").topRows(forcedDim));

	// Note: Normal vector must be precalculated to use this error. Use appropriate input filter.
	assert(normalRef.rows() > 0);

	const Matrix deltas = mPts.reading.features - mPts.reference.features;

	// dotProd = dot(deltas, normals) = d.n
	Matrix dotProd = Matrix::Zero(1, normalRef.cols());
	for(int i = 0; i < normalRef.rows(); i++)
	{
		dotProd += (deltas.row(i).array() * normalRef.row(i).array()).matrix();
	}
	// residual = w*(d.n)²
	dotProd = (mPts.weights.row(0).array() * dotProd.array().square()).matrix();

	// return sum of the norm of each dot product
	return dotProd.sum();
}


template<typename T>
T PointToPlaneErrorMinimizer<T>::getResidualError(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches) const
{
	assert(matches.ids.rows() > 0);

	// Fetch paired points
	typename ErrorMinimizer::ErrorElements mPts(filteredReading, filteredReference, outlierWeights, matches);

	return PointToPlaneErrorMinimizer::computeResidualError(mPts, force2D);
}

template<typename T>
T PointToPlaneErrorMinimizer<T>::getOverlap() const
{

	// Gather some information on what kind of point cloud we have
	const bool hasReadingNoise = this->lastErrorElements.reading.descriptorExists("simpleSensorNoise");
	const bool hasReferenceNoise = this->lastErrorElements.reference.descriptorExists("simpleSensorNoise");
	const bool hasReferenceDensity = this->lastErrorElements.reference.descriptorExists("densities");

	const int nbPoints = this->lastErrorElements.reading.features.cols();
	const int dim = this->lastErrorElements.reading.features.rows();

	// basix safety check
	if(nbPoints == 0)
	{
		throw std::runtime_error("Error, last error element empty. Error minimizer needs to be called at least once before using this method.");
	}

	Eigen::Array<T, 1, Eigen::Dynamic>  uncertainties(nbPoints);

	// optimal case
	if (hasReadingNoise && hasReferenceNoise && hasReferenceDensity)
	{
		// find median density

		Matrix densities = this->lastErrorElements.reference.getDescriptorViewByName("densities");
		vector<T> values(densities.data(), densities.data() + densities.size());

		// sort up to half the values
		nth_element(values.begin(), values.begin() + (values.size() * 0.5), values.end());

		// extract median value
		const T medianDensity = values[values.size() * 0.5];
		const T medianRadius = 1.0/pow(medianDensity, 1/3.0);

		uncertainties = (medianRadius +
						this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise").array() +
						this->lastErrorElements.reference.getDescriptorViewByName("simpleSensorNoise").array());
	}
	else if(hasReadingNoise && hasReferenceNoise)
	{
		uncertainties = this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise") +
						this->lastErrorElements.reference.getDescriptorViewByName("simpleSensorNoise");
	}
	else if(hasReadingNoise)
	{
		uncertainties = this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise");
	}
	else if(hasReferenceNoise)
	{
		uncertainties = this->lastErrorElements.reference.getDescriptorViewByName("simpleSensorNoise");
	}
	else
	{
		LOG_INFO_STREAM("PointToPlaneErrorMinimizer - warning, no sensor noise and density. Using best estimate given outlier rejection instead.");
		return this->getWeightedPointUsedRatio();
	}


	const Vector dists = (this->lastErrorElements.reading.features.topRows(dim-1) - this->lastErrorElements.reference.features.topRows(dim-1)).colwise().norm();


	// here we can only loop through a list of links, but we are interested in whether or not
	// a point has at least one valid match.
	int count = 0;
	int nbUniquePoint = 1;
	Vector lastValidPoint = this->lastErrorElements.reading.features.col(0) * 2.;
	for(int i=0; i < nbPoints; i++)
	{
		const Vector point = this->lastErrorElements.reading.features.col(i);

		if(lastValidPoint != point)
		{
			// NOTE: we tried with the projected distance over the normal vector before:
			// projectionDist = delta dotProduct n.normalized()
			// but this doesn't make sense 


			if(PointMatcherSupport::anyabs(dists(i, 0)) < (uncertainties(0,i)))
			{
				lastValidPoint = point;
				count++;
			}
		}

		// Count unique points
		if(i > 0)
		{
			if(point != this->lastErrorElements.reading.features.col(i-1))
				nbUniquePoint++;
		}

	}
	//cout << "count: " << count << ", nbUniquePoint: " << nbUniquePoint << ", this->lastErrorElements.nbRejectedPoints: " << this->lastErrorElements.nbRejectedPoints << endl;

	return (T)count/(T)(nbUniquePoint + this->lastErrorElements.nbRejectedPoints);
}

template struct PointToPlaneErrorMinimizer<float>;
template struct PointToPlaneErrorMinimizer<double>;
