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

#ifndef __POINTMATCHER_IOFUNCTIONS_H
#define __POINTMATCHER_IOFUNCTIONS_H

#include <iosfwd>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <stdio.h>

namespace PointMatcherSupport
{

extern const bool isBigEndian; //!< true if platform is big endian
extern const int oneBigEndian; //!< is always a big endian independent of the platforms endianness

template <typename T>
struct ConverterToAndFromBytes 
{
	union 
	{
		T v;
		char bytes[sizeof(T)];
	};

	ConverterToAndFromBytes(T v = static_cast<T>(0)) : v(v) {}

	void swapBytes()
	{
		ConverterToAndFromBytes tmp(this->v);
		std::reverse_copy(tmp.bytes, tmp.bytes + sizeof(T), bytes);
	}

	friend std::ostream & operator << (std::ostream & out, const ConverterToAndFromBytes & c)
	{
		out.write(c.bytes, sizeof(T));
		return out;
	}
	friend std::istream & operator >> (std::istream & in, ConverterToAndFromBytes & c)
	{
		in.read(c.bytes, sizeof(T));
		return in;
	}
};

template<typename Matrix>
std::ostream & writeVtkData(bool writeBinary,const Matrix & data, std::ostream & out)
{
	if(writeBinary)
	{
		typedef typename Matrix::Scalar TargetDataType;
		for(int r = 0; r < data.rows(); r++)
		{
			for(int c = 0; c < data.cols(); c++)
			{
				ConverterToAndFromBytes<TargetDataType> converter(static_cast<TargetDataType>(data(r, c)));
				if(!isBigEndian)
				{
					converter.swapBytes();
				}
				out << converter;
			}
		}
	}
	else 
	{
		out << data;
	}

	return out;
}

template<typename DataType, typename MatrixRef>
std::istream & readVtkData(bool readBinary, MatrixRef into, std::istream & in)
{
	typedef typename MatrixRef::Scalar TargetDataType;
	
	for(int r = 0; r < into.rows(); r++)
	{
		for(int c = 0; c < into.cols(); c++)
		{
			TargetDataType & dest = into(r, c);
			if(readBinary)
			{
				ConverterToAndFromBytes<DataType> converter;
				in >> converter;
				if(!isBigEndian){
					converter.swapBytes();
				}
				dest = converter.v;
			}
			else 
			{
				in >> dest;
			}
		}
	}

	return in;
}

template<typename MatrixRef>
std::istream & readVtkData(std::string dataType, bool readBinary, MatrixRef into, std::istream & in)
{
	if(dataType == "float")
	{
		return readVtkData<float>(readBinary, into, in);
	} 
	else if (dataType == "double") 
	{
		return readVtkData<double>(readBinary, into, in);
	} 
	else if (dataType == "unsigned_int") 
	{
		return readVtkData<unsigned int>(readBinary, into, in);
	}
	else 
	{
		throw std::runtime_error(std::string("Unsupported data type : " + dataType + "! Expected 'float' or 'double'."));
	}
}

//! Replaces getline for handling windows style CR/LF line endings
std::istream & safeGetLine( std::istream& is, std::string & t);


} // PointMatcherSupport

#endif // __POINTMATCHER_IOFUNCTIONS_H
