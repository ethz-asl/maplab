/*

Copyright (c) 2013, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef MOTION_DEFINES_H_
#define MOTION_DEFINES_H_

#include <string>

namespace mav_planning_utils
{

//namespace DerivativesP
//{

//enum DerivativeP
//{
//  p = 0, v = 1, a = 2, j = 3, s = 4, none = -1
//};
//}
//typedef DerivativesP::DerivativeP DerivativeP;

class DerivativesP
{
public:
  enum
  {
    p = 0, v = 1, a = 2, j = 3, s = 4, none = -1
  };

  static int toInt(const std::string & str)
  {
    if (str == "p")
      return p;
    else if (str == "v")
      return v;
    else if (str == "a")
      return a;
    else if (str == "j")
      return j;
    else if (str == "s")
      return s;
    else
      return none;
  }

  static std::string toString(int d)
  {
    if (d >= 0 && d < 5)
    {
      const char* text[5] = {"p", "v", "a", "j", "s"};
      return std::string(text[d]);
    }
    else
      return std::string("-");
  }
};

class DerivativesO
{
public:
  enum
  {
    o = 0, w = 1, wd = 2, wdd = 3, none = -1
  };
};
//typedef DerivativesO::DerivativeO DerivativeO;


//std::string derivativeP2String(int d)
//{
//  if (d >= 0 && d < 5)
//  {
//    const char* text[5] = {"p", "v", "a", "j", "s"};
//    return std::string(text[d]);
//  }
//  else
//    return std::string("-");
//}
//
//
//std::string derivativeO2String(int d)
//{
//  if (d >= 0 && d < 4)
//  {
//    const char* text[4] = {"o", "w", "wd", "wdd"};
//    return std::string(text[d]);
//  }
//  else
//    return std::string("-");
//}

} //namespace mav_planning_utils

#endif /* MOTION_DEFINES_H_ */
