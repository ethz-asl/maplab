// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
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
#include "CovarianceSampling.h"

#include <vector>
#include <list>
#include <utility>
#include <unordered_map>

// Eigenvalues
#include "Eigen/QR"
#include "Eigen/Eigenvalues"

// CovarianceSamplingDataPointsFilter
template <typename T>
CovarianceSamplingDataPointsFilter<T>::CovarianceSamplingDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("CovarianceSamplingDataPointsFilter", 
		CovarianceSamplingDataPointsFilter::availableParameters(), params),
	nbSample{Parametrizable::get<std::size_t>("nbSample")}
{
	try 
	{
		const std::uint8_t tnm = this->template get<std::uint8_t>("torqueNorm");
		normalizationMethod = TorqueNormMethod(tnm);
	}
	catch (const InvalidParameter& e) 
	{
		normalizationMethod = TorqueNormMethod::Lavg;
	}
}

template <typename T>
typename PointMatcher<T>::DataPoints
CovarianceSamplingDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

template <typename T>
void CovarianceSamplingDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{	
	const std::size_t featDim(cloud.features.rows());
	assert(featDim == 4); //3D pts only
	
	//Check number of points
	const std::size_t nbPoints = cloud.getNbPoints();		
	if(nbSample >= nbPoints)
		return;
	
	//Check if there is normals info
	if (!cloud.descriptorExists("normals"))
		throw InvalidField("OrientNormalsDataPointsFilter: Error, cannot find normals in descriptors.");

	const auto& normals = cloud.getDescriptorViewByName("normals");
	
	std::vector<std::size_t> keepIndexes;
	keepIndexes.resize(nbSample);
	
	///---- Part A, as we compare the cloud with himself, the overlap is 100%, so we keep all points 
	//A.1 and A.2 - Compute candidates
	std::vector<std::size_t> candidates ;
	candidates.resize(nbPoints);
	
	for (std::size_t i = 0; i < nbPoints; ++i) candidates[i] = i;
	
	const std::size_t nbCandidates = candidates.size();
	
	//Compute centroid
	Vector3 center;
	for(std::size_t i = 0; i < featDim - 1; ++i) center(i) = T(0.);
	
	for (std::size_t i = 0; i < nbCandidates; ++i)
		for (std::size_t f = 0; f <= 3; ++f)
			center(f) += cloud.features(f,candidates[i]);
	
	for(std::size_t i = 0; i <= 3; ++i) center(i) /= T(nbCandidates);
	
	//Compute torque normalization
	T Lnorm = 1.0;
	
	if(normalizationMethod == TorqueNormMethod::L1)
	{
		Lnorm = 1.0;
	}
	else if(normalizationMethod == TorqueNormMethod::Lavg)
	{
		Lnorm = 0.0;
		for (std::size_t i = 0; i < nbCandidates; ++i)
			Lnorm += (cloud.features.col(candidates[i]).head(3) - center).norm();
		Lnorm /= nbCandidates;
	}
	else if(normalizationMethod == TorqueNormMethod::Lmax)	
	{	
		const Vector minValues = cloud.features.rowwise().minCoeff();
		const Vector maxValues = cloud.features.rowwise().maxCoeff();
		const Vector3 radii = maxValues.head(3) - minValues.head(3);

		Lnorm = radii.maxCoeff() / 2.; //radii.mean() / 2.; 
	}
	
	//A.3 - Compute 6x6 covariance matrix + EigenVectors
	auto computeCovariance = [Lnorm, nbCandidates, &cloud, &center, &normals, &candidates](Matrix66 & cov) -> void {
			//Compute F matrix, see Eq. (4)
			Eigen::Matrix<T, 6, Eigen::Dynamic> F(6, nbCandidates);
	
			for(std::size_t i = 0; i < nbCandidates; ++i)
			{
				const Vector3 p = cloud.features.col(candidates[i]).head(3) - center; // pi-c
				const Vector3 ni = normals.col(candidates[i]).head(3);
				
				//compute (1 / L) * (pi - c) x ni 
				F.template block<3, 1>(0, i) = (1. / Lnorm) * p.cross(ni);
				//set ni part
				F.template block<3, 1>(3, i) = ni;
			}

			// Compute the covariance matrix Cov = FF'
			cov = F * F.transpose(); 
		};
		
	Matrix66 covariance;
	computeCovariance(covariance);
	
	Eigen::EigenSolver<Matrix66> solver(covariance);		
	const Matrix66  eigenVe = solver.eigenvectors().real();
	const Vector6   eigenVa = solver.eigenvalues().real();
	
	///---- Part B
	//B.1 - Compute the v-6 for each candidate
	std::vector<Vector6, Eigen::aligned_allocator<Vector6>> v; // v[i] = [(pi-c) x ni ; ni ]'
	v.resize(nbCandidates);

	for(std::size_t i = 0; i < nbCandidates; ++i)
	{
		const Vector3 p = cloud.features.col(candidates[i]).head(3) - center; // pi-c
		const Vector3 ni = normals.col(candidates[i]).head(3);
		
		v[i].template block<3, 1>(0, 0) = (1. / Lnorm) * p.cross(ni);
		v[i].template block<3, 1>(3, 0) = ni;
	}
	
	//B.2 - Compute the 6 sorted list based on dot product (vi . Xk) = magnitude, with Xk the kth-EigenVector
	std::vector<std::list<std::pair<int, T>>> L; // contain list of pair (index, magnitude) contribution to the eigens vectors
	L.resize(6);
	
	//sort by decreasing magnitude
	auto comp = [](const std::pair<int, T>& p1, const std::pair<int, T>& p2) -> bool {
			return p1.second > p2.second;
		};
	
	for(std::size_t k = 0; k < 6; ++k)
	{		
		for(std::size_t i = 0; i < nbCandidates; ++i )
		{
			L[k].push_back(std::make_pair(i, std::fabs( v[i].dot(eigenVe.template block<6,1>(0, k)) )));
		}
		
		L[k].sort(comp);
	}	
	
	std::vector<T> t(6, T(0.)); //contains the sums of squared magnitudes
	std::vector<bool> sampledPoints(nbCandidates, false); //maintain flag to avoid resampling the same point in an other list 
	
	///Add point iteratively till we got the desired number of point
	for(std::size_t i = 0; i < nbSample; ++i)
	{
		//B.3 - Equally constrained all eigen vectors		
		// magnitude contribute to t_i where i is the indice of th least contrained eigen vector
		
		//Find least constrained vector
		std::size_t k = 0;
		for (std::size_t i = 0; i < 6; ++i)
		{
			if (t[k] > t[i])
				k = i;
		}
		// Add the point from the top of the list corresponding to the dimension to the set of samples
		while(sampledPoints[L[k].front().first])
			L[k].pop_front(); //remove already sampled point
		
		//Get index to keep
		const std::size_t idToKeep = static_cast<std::size_t>(L[k].front().first);
		L[k].pop_front();
			
		sampledPoints[idToKeep] = true; //set flag to avoid resampling
				
		//B.4 - Update the running total
		for (std::size_t k = 0; k < 6; ++k)
		{
			const T magnitude = v[idToKeep].dot(eigenVe.template block<6, 1>(0, k));
			t[k] += (magnitude * magnitude);
		}
		
		keepIndexes[i] = candidates[idToKeep];
	}

	//TODO: evaluate performances between this solution and sorting the indexes
	// We build map of (old index to new index), in case we sample pts at the begining of the pointcloud
	std::unordered_map<std::size_t, std::size_t> mapidx;
	std::size_t idx = 0;
	
	///(4) Sample the point cloud
	for(std::size_t id : keepIndexes)
	{
		//retrieve index from lookup table if sampling in already switched element
		if(id<idx)
			id = mapidx[id];
		//Switch columns id and idx
		cloud.swapCols(idx, id);	
		//Maintain new index position	
		mapidx[idx] = id;
		//Update index
		++idx;
	}
	cloud.conservativeResize(nbSample);
}

// Compute c = Lambda_6 / Lambda_1, where Lambda_1 <= ... <= Lambda_6
// Stability is obtained where c is as close as possible of 1.0
template <typename T>
T CovarianceSamplingDataPointsFilter<T>::computeConditionNumber(const Matrix66 &cov)
{
	Vector eigenVa = Vector::Identity(6, 1);		
	
	Eigen::EigenSolver<Matrix66> solver(cov);		
	eigenVa = solver.eigenvalues().real();

	return eigenVa.maxCoeff() / eigenVa.minCoeff();
}

template struct CovarianceSamplingDataPointsFilter<float>;
template struct CovarianceSamplingDataPointsFilter<double>;
