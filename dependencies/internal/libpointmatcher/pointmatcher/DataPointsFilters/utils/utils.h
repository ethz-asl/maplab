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
#pragma once

#include "PointMatcher.h"

#include <vector>
#include <algorithm>
#include <cmath>

namespace PointMatcherSupport
{

template<class T>
inline constexpr T pow(const T base, const std::size_t exponent)
{
    return exponent == 0 ? 1 : base * pow(base, exponent - 1);
}

template<typename T>
struct IdxCompare
{
	typedef typename PointMatcher<T>::Vector Vector;
	const Vector& target;

	IdxCompare(const typename PointMatcher<T>::Vector& target): target(target) {}

	bool operator()(size_t a, size_t b) const { return target(a, 0) < target(b, 0); }
};


template<typename T>
std::vector<size_t> 
sortIndexes(const typename PointMatcher<T>::Vector& v)
{
	// initialize original index locations
	const size_t idxSize = v.size();
	std::vector<size_t> idx(idxSize);
	for(size_t i=0; i < idxSize; ++i) idx[i]=i;

	// sort indexes based on comparing values in v
	std::sort(idx.begin(), idx.end(), IdxCompare<T>(v));

	return idx;
}

template<typename T>
typename PointMatcher<T>::Vector
sortEigenValues(const typename PointMatcher<T>::Vector& eigenVa)
{
	// sort the eigenvalues in ascending order
	typename PointMatcher<T>::Vector eigenVaSort = eigenVa;
	std::sort(eigenVaSort.data(), eigenVaSort.data() + eigenVaSort.size());
	return eigenVaSort;
}

template<typename T>
typename PointMatcher<T>::Vector 
serializeEigVec(const typename PointMatcher<T>::Matrix& eigenVe)
{
	// serialize row major
	const int eigenVeDim = eigenVe.cols();
	typename PointMatcher<T>::Vector output(eigenVeDim*eigenVeDim);
	for(int k=0; k < eigenVeDim; ++k)
	{
		output.segment(k*eigenVeDim, eigenVeDim) = 
			eigenVe.row(k).transpose();
	}

	return output;
}

template<typename T>
T computeDensity(const typename PointMatcher<T>::Matrix& NN)
{
	//volume in meter
	const T volume = (4./3.)*M_PI*std::pow(NN.colwise().norm().maxCoeff(), 3);

	//volume in decimeter
	//T volume = (4./3.)*M_PI*std::pow(NN.colwise().norm().maxCoeff()*10.0, 3);
	//const T minVolume = 4.18e-9; // minimum of volume of one millimeter radius
	//const T minVolume = 0.42; // minimum of volume of one centimeter radius (in dm^3)

	//if(volume < minVolume)
	//	volume = minVolume;

	return T(NN.cols())/(volume);
}
template<typename T>
typename PointMatcher<T>::Vector
computeNormal(const typename PointMatcher<T>::Vector& eigenVa, const typename PointMatcher<T>::Matrix& eigenVe)
{
	// Keep the smallest eigenvector as surface normal
	const int nbEigenCol = eigenVe.cols();
	int smallestId(0);
	T smallestValue(std::numeric_limits<T>::max());
	for(int j = 0; j < nbEigenCol ; ++j)
	{
		if (eigenVa(j) < smallestValue)
		{
			smallestId = j;
			smallestValue = eigenVa(j);
		}
	}

  return eigenVe.col(smallestId);
}

template<typename T>
size_t argMax(const typename PointMatcher<T>::Vector& v)
{
	//FIXME: Change that to use the new API. the new Eigen API (3.2.8) allows this with the call maxCoeff. See the section Visitors in https://eigen.tuxfamily.org/dox/group__TutorialReductionsVisitorsBroadcasting.html
	const int size(v.size());
	T maxVal(0);
	size_t maxIdx(0);
	for (int i = 0; i < size; ++i)
	{
		if (v[i] > maxVal)
		{
			maxVal = v[i];
			maxIdx = i;
		}
	}
	return maxIdx;
}

};
