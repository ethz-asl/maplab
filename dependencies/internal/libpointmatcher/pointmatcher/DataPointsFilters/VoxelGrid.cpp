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
#include "VoxelGrid.h"


// VoxelGridDataPointsFilter
template <typename T>
VoxelGridDataPointsFilter<T>::VoxelGridDataPointsFilter() :
	vSizeX(1),
	vSizeY(1),
	vSizeZ(1),
	useCentroid(true),
	averageExistingDescriptors(true) 
{
}

template <typename T>
VoxelGridDataPointsFilter<T>::VoxelGridDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("VoxelGridDataPointsFilter", 
		VoxelGridDataPointsFilter::availableParameters(), params),
	vSizeX(Parametrizable::get<T>("vSizeX")),
	vSizeY(Parametrizable::get<T>("vSizeY")),
	vSizeZ(Parametrizable::get<T>("vSizeZ")),
	useCentroid(Parametrizable::get<bool>("useCentroid")),
	averageExistingDescriptors(Parametrizable::get<bool>("averageExistingDescriptors"))
{
}

template <typename T>
typename PointMatcher<T>::DataPoints
VoxelGridDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

template <typename T>
void VoxelGridDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	const unsigned int numPoints(cloud.features.cols());
	const int featDim(cloud.features.rows());
	const int descDim(cloud.descriptors.rows());
	const int timeDim(cloud.times.rows());
	const unsigned int labelDim(cloud.descriptorLabels.size());

	assert (featDim == 3 || featDim == 4);

	int insertDim(0);
	if (averageExistingDescriptors)
	{
		// TODO: this should be in the form of an assert
		// Validate descriptors and labels
		for(unsigned int i = 0; i < labelDim ; ++i)
			insertDim += cloud.descriptorLabels[i].span;
		if (insertDim != descDim)
			throw InvalidField("VoxelGridDataPointsFilter: Error, descriptor labels do not match descriptor data");
		//TODO: timeDim too?
	}

	// TODO: Check that the voxel size is not too small, given the size of the data
	// TODO: Check if sizes are positive

	// Calculate number of divisions along each axis
	Vector minValues = cloud.features.rowwise().minCoeff();
	Vector maxValues = cloud.features.rowwise().maxCoeff();

	const T minBoundX = minValues.x() / vSizeX;
	const T maxBoundX = maxValues.x() / vSizeX;
	const T minBoundY = minValues.y() / vSizeY;
	const T maxBoundY = maxValues.y() / vSizeY;
	T minBoundZ = 0;
	T maxBoundZ = 0;

	if (featDim == 4)
	{
		minBoundZ = minValues.z() / vSizeZ;
		maxBoundZ = maxValues.z() / vSizeZ;
	}

	// number of divisions is total size / voxel size voxels of equal length + 1
	// with remaining space
	const unsigned int numDivX = 1 + maxBoundX - minBoundX;
	const unsigned int numDivY = 1 + maxBoundY - minBoundY;;
	unsigned int numDivZ = 0;

	// If a 3D point cloud
	if (featDim == 4 )
	  numDivZ = 1 + maxBoundZ - minBoundZ;

	unsigned int numVox = numDivX * numDivY;
	if (featDim == 4)
		numVox *= numDivZ;
	
	if(numVox == 0)
	{
		throw InvalidParameter("VoxelGridDataPointsFilter: The number of voxel couldn't be computed. There might be NaNs in the feature matrix. Use the fileter RemoveNaNDataPointsFilter before this one if it's the case.");
	}

	// Assume point cloud is randomly ordered
	// compute a linear index of the following type
	// i, j, k are the component indices
	// nx, ny number of divisions in x and y components
	// idx = i + j * nx + k * nx * ny
	std::vector<unsigned int> indices(numPoints);

	// vector to hold the first point in a voxel
	// this point will be ovewritten in the input cloud with
	// the output value

	std::vector<Voxel> voxels;

	// try allocating vector. If too big return error
	try 
	{
		voxels = std::vector<Voxel>(numVox);
	} 
	catch (std::bad_alloc&) 
	{
		throw InvalidParameter((boost::format("VoxelGridDataPointsFilter: Memory allocation error with %1% voxels.  Try increasing the voxel dimensions.") % numVox).str());
	}

	for (unsigned int p = 0; p < numPoints; ++p)
	{	
		const unsigned int i = floor(cloud.features(0,p)/vSizeX - minBoundX);
		const unsigned int j = floor(cloud.features(1,p)/vSizeY- minBoundY);
		unsigned int k = 0;
		unsigned int idx = 0;
		if ( featDim == 4 )
		{
			k = floor(cloud.features(2,p)/vSizeZ - minBoundZ);
			idx = i + j * numDivX + k * numDivX * numDivY;
		}
		else
		{
			idx = i + j * numDivX;
		}

		const unsigned int pointsInVox = voxels[idx].numPoints + 1;

		if (pointsInVox == 1)
		{
			voxels[idx].firstPoint = p;
		}

		voxels[idx].numPoints = pointsInVox;

		indices[p] = idx;

	}


	// store which points contain voxel position
	std::vector<unsigned int> pointsToKeep;

	// Store voxel centroid in output
	if (useCentroid)
	{
		// Iterate through the indices and sum values to compute centroid
		for (unsigned int p = 0; p < numPoints ; ++p)
		{
			const unsigned int idx = indices[p];
			const unsigned int firstPoint = voxels[idx].firstPoint;

			// If this is the first point in the voxel, leave as is
			// if not sum up this point for centroid calculation
			if (firstPoint != p)
			{
				// Sum up features and descriptors (if we are also averaging descriptors)

				for (int f = 0; f < (featDim - 1); ++f)
				{
					cloud.features(f,firstPoint) += cloud.features(f,p);
				}

				if (averageExistingDescriptors) 
				{
					for (int d = 0; d < descDim; ++d)
					{
						cloud.descriptors(d,firstPoint) += cloud.descriptors(d,p);
					}
					for (int d = 0; d < timeDim; ++d)
					{
						cloud.times(d,firstPoint) += cloud.times(d,p);
					}
				}
			}
		}

		// Now iterating through the voxels
		// Normalize sums to get centroid (average)
		// Some voxels may be empty and are discarded
		for(unsigned int idx = 0; idx < numVox; ++idx)
		{
			const unsigned int numPoints = voxels[idx].numPoints;
			const unsigned int firstPoint = voxels[idx].firstPoint;
			if(numPoints > 0)
			{
				for (int f = 0; f < (featDim - 1); ++f)
					cloud.features(f,firstPoint) /= numPoints;

				if (averageExistingDescriptors) 
				{
					for ( int d = 0; d < descDim; ++d )
						cloud.descriptors(d,firstPoint) /= numPoints;
					for ( int d = 0; d < timeDim; ++d )
						cloud.times(d,firstPoint) /= numPoints;
				}
				pointsToKeep.push_back(firstPoint);
			}
		}
	}
	else
	{
		// Although we don't sum over the features, we may still need to sum the descriptors
		if (averageExistingDescriptors)
		{
			// Iterate through the indices and sum values to compute centroid
			for (unsigned int p = 0; p < numPoints ; ++p)
			{
				const unsigned int idx = indices[p];
				const unsigned int firstPoint = voxels[idx].firstPoint;

				// If this is the first point in the voxel, leave as is
				// if not sum up this point for centroid calculation
				if (firstPoint != p)
				{
					for (int d = 0; d < descDim; ++d)
					{
						cloud.descriptors(d,firstPoint) += cloud.descriptors(d,p);
					}
					for (int d = 0; d < timeDim; ++d)
					{
						cloud.times(d,firstPoint) += cloud.times(d,p);
					}
				}
			}
		}

		for (unsigned int idx = 0; idx < numVox; ++idx)
		{
			const unsigned int numPoints = voxels[idx].numPoints;
			const unsigned int firstPoint = voxels[idx].firstPoint;

			if (numPoints > 0)
			{
				// get back voxel indices in grid format
				// If we are in the last division, the voxel is smaller in size
				// We adjust the center as from the end of the last voxel to the bounding area
				unsigned int i = 0;
				unsigned int j = 0;
				unsigned int k = 0;
				if (featDim == 4)
				{
					k = idx / (numDivX * numDivY);
					if (k == numDivZ)
						cloud.features(3,firstPoint) = maxValues.z() - (k-1) * vSizeZ/2;
					else
						cloud.features(3,firstPoint) = k * vSizeZ + vSizeZ/2;
				}

				j = (idx - k * numDivX * numDivY) / numDivX;
				if (j == numDivY)
					cloud.features(2,firstPoint) = maxValues.y() - (j-1) * vSizeY/2;
				else
					cloud.features(2,firstPoint) = j * vSizeY + vSizeY / 2;

				i = idx - k * numDivX * numDivY - j * numDivX;
				if (i == numDivX)
					cloud.features(1,firstPoint) = maxValues.x() - (i-1) * vSizeX/2;
				else
					cloud.features(1,firstPoint) = i * vSizeX + vSizeX / 2;

				// Descriptors : normalize if we are averaging or keep as is
				if (averageExistingDescriptors) 
				{
					for ( int d = 0; d < descDim; ++d)
						cloud.descriptors(d,firstPoint) /= numPoints;
					for ( int d = 0; d < timeDim; ++d)
						cloud.times(d,firstPoint) /= numPoints;
				}
				pointsToKeep.push_back(firstPoint);
			}
		}
	}

	// Move the points to be kept to the start
	// Bring the data we keep to the front of the arrays then
	// wipe the leftover unused space.
	std::sort(pointsToKeep.begin(), pointsToKeep.end());
	int numPtsOut = pointsToKeep.size();
	for (int i = 0; i < numPtsOut; ++i)
	{
		const int k = pointsToKeep[i];
		assert(i <= k);
		cloud.features.col(i) = cloud.features.col(k);
		if (cloud.descriptors.rows() != 0)
			cloud.descriptors.col(i) = cloud.descriptors.col(k);
		if (cloud.times.rows() != 0)
			cloud.times.col(i) = cloud.times.col(k);

	}
	
	cloud.conservativeResize(numPtsOut);
	
	//cloud.features.conservativeResize(Eigen::NoChange, numPtsOut);
	//
	//if (cloud.descriptors.rows() != 0)
	//	cloud.descriptors.conservativeResize(Eigen::NoChange, numPtsOut);
	//if (cloud.times.rows() != 0)
	//	cloud.times.conservativeResize(Eigen::NoChange, numPtsOut)
}

template struct VoxelGridDataPointsFilter<float>;
template struct VoxelGridDataPointsFilter<double>;


