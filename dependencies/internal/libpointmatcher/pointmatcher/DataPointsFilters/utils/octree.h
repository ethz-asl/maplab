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

#include <cstdlib> 
#include <vector>
#include "PointMatcher.h"

#include "utils.h"

/*!
 * \class octree.h
 * \brief Octree class for DataPoints spatial representation
 *
 * \author Mathieu Labussiere (<mathieu dot labu at gmail dot com>)
 * \date 24/05/2018
 * \version 0.1
 *
 * \date 01/06/2018
 * \version 0.2
 *
 * Octree/Quadtree implementation for decomposing point cloud. 
 * The current implementation use the data structure PointMatcher<T>::DataPoints. 
 * It ensures that each node has either (8/4) or 0 childs. 
 *
 * Can create an octree with the 2 following crieterions:
 *	- max number of data by node
 *	- max size of a node (or stop when only one or zero data element is available)
 *
 * After construction, we can apply a process by creating a Visitor, implementing the following method:
 *	```cpp
 *	template<typename T>
 *	struct Visitor {
 *		bool operator()(Octree<T>& oc);
 *	};
 *	```
 *
 * Remark:
 *	- Current implementation only store the indexes of the points from the pointcloud.
 *	- Data element are exclusively contained in leaves node.
 *	- Some leaves node contains no data (to ensure (8/4) or 0 childs).
 *
 */

template < typename T, std::size_t dim >
class Octree_ 
{
public:		
	using PM = PointMatcher<T>;
	using DP = typename PM::DataPoints; /**/
	using Id = typename DP::Index; /**/
	
	using Data = typename DP::Index; /**/
	using DataContainer = std::vector<Data>;
	
	using Point = Eigen::Matrix<T,dim,1>;
	
//private:
	static constexpr std::size_t nbCells = PointMatcherSupport::pow(2, dim);
	
private:
	struct BoundingBox 
	{
			Point center;
			T 	radius;
	};
	
	Octree_* parent;
	Octree_* cells[nbCells];
	
	/******************************************************
	 *	Cells id are assigned as their position 
	 *   from the center (+ greater than center, - lower than center)
	 *
	 *		for 3D case									for 2D case
	 *
	 *	  	0	1	2	3	4	5	6	7		  	0	1	2	3
	 * 	x:	-	+	-	+	-	+	-	+		x:	-	+	-	+
	 * 	y:	-	-	+	+	-	-	+	+		y:	-	-	+	+	
	 * 	z:	-	-	-	-	+	+	+	+
	 *
	 *****************************************************/
	
	BoundingBox bb;
	
	DataContainer data;	
	
	std::size_t depth;
		
public:
	Octree_();
	Octree_(const Octree_<T,dim>& o); //Deep-copy
	Octree_(Octree_<T,dim>&& o);
	
	virtual ~Octree_();
	
	Octree_<T,dim>& operator=(const Octree_<T,dim>& o);//Deep-copy
	Octree_<T,dim>& operator=(Octree_<T,dim>&& o);
	
	bool isLeaf() const;
	bool isRoot() const;
	bool isEmpty()const;
	
	inline std::size_t idx(const Point& pt) const;
	inline std::size_t idx(const DP& pts, const Data d) const;
	
	std::size_t getDepth() const;
	
	T getRadius() const;
	Point getCenter() const;
	
	DataContainer * getData();
	Octree_<T, dim>* operator[](std::size_t idx);
	
	// Build tree from DataPoints with a specified stop parameter
	bool build(const DP& pts, size_t maxDataByNode=1, T maxSizeByNode=T(0.), bool parallelBuild=false);

protected:
	//real build function
	bool build(const DP& pts, DataContainer&& datas, BoundingBox&& bb, size_t maxDataByNode=1, T maxSizeByNode=T(0.), bool parallelBuild=false);
	
	inline DataContainer toData(const DP& pts, const std::vector<Id>& ids);
	
public:	
	template < typename Callback >
	bool visit(Callback& cb);
};
	
#include "octree.hpp"

template<typename T> using Quadtree = Octree_<T,2>;
template<typename T> using Octree = Octree_<T,3>;

