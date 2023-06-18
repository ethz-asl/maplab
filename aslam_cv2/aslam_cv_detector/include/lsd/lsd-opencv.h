/*M///////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2008-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef _OPENCV_LSD_HPP_
#define _OPENCV_LSD_HPP_
#ifdef __cplusplus

#include <opencv2/core/core.hpp>

namespace aslamcv {
using cv::InputArray;
using cv::OutputArray;
using cv::InputOutputArray;
using cv::Mat;
using cv::Point;
using cv::Size;
using cv::Ptr;

enum {LSD_NO_SIZE_LIMIT = -1,
      LSD_REFINE_NONE   = 0,
      LSD_REFINE_STD    = 1,
      LSD_REFINE_ADV    = 2,


};

class LineSegmentDetector : public cv::Algorithm
{
public:
/**
 * Detect lines in the input image with the specified ROI.
 *
 * @param _image    A grayscale(CV_8UC1) input image.
 *                  If only a roi needs to be selected, use
 *                  lsd_ptr->detect(image(roi), ..., lines);
 *                  lines += Scalar(roi.x, roi.y, roi.x, roi.y);
 * @param _lines    Return: A vector of Vec4i elements specifying the beginning and ending point of a line.
 *                          Where Vec4i is (x1, y1, x2, y2), point 1 is the start, point 2 - end.
 *                          Returned lines are strictly oriented depending on the gradient.
 * @param _roi      Return: ROI of the image, where lines are to be found. If specified, the returning
 *                          lines coordinates are image wise.
 * @param width     Return: Vector of widths of the regions, where the lines are found. E.g. Width of line.
 * @param prec      Return: Vector of precisions with which the lines are found.
 * @param nfa       Return: Vector containing number of false alarms in the line region, with precision of 10%.
 *                          The bigger the value, logarithmically better the detection.
 *                              * -1 corresponds to 10 mean false alarms
 *                              * 0 corresponds to 1 mean false alarm
 *                              * 1 corresponds to 0.1 mean false alarms
 *                          This vector will be calculated _only_ when the objects type is REFINE_ADV
 */
    virtual void detect(InputArray _image, OutputArray _lines,
                        OutputArray width = cv::noArray(), OutputArray prec = cv::noArray(),
                        OutputArray nfa = cv::noArray()) = 0;

/**
 * Draw lines on the given canvas.
 *
 * @param image     The image, where lines will be drawn.
 *                  Should have the size of the image, where the lines were found
 * @param lines     The lines that need to be drawn
 */
    virtual void drawSegments(InputOutputArray _image, InputArray lines) = 0;

/**
 * Draw both vectors on the image canvas. Uses blue for lines 1 and red for lines 2.
 *
 * @param image     The image, where lines will be drawn.
 *                  Should have the size of the image, where the lines were found.
 * @param lines1    The first lines that need to be drawn. Color - Blue.
 * @param lines2    The second lines that need to be drawn. Color - Red.
 * @return          The number of mismatching pixels between lines1 and lines2.
 */
    virtual int compareSegments(const Size& size, InputArray lines1, InputArray lines2, InputOutputArray _image = cv::noArray()) = 0;

/**
 * Find all line elements that are *not* fullfilling the angle and range requirenmnets.
 * Take all lines, whose angle(line_segment) is outside [min_angle, max_angle] range.
 *
 * @param lines         Input lines.
 * @param filtered      The output vector of lines not containing those fulfilling the requirement.
 * @param min_angle     The min angle to be considered in degrees. Should be in range [0..180].
 * @param max_angle     The max angle to be considered in degrees. Should be >= min_angle and widthin range [0..180].
 * @return              Returns the number of line segments not included in the output vector.
 */
    CV_WRAP virtual int filterOutAngle(InputArray lines, OutputArray filtered, float min_angle, float max_angle) = 0;

/**
 * Find all line elements that are fullfilling the angle and range requirenmnets.
 * Take all lines, whose angle(line_segment) is inside [min_angle, max_angle] range.
 * The opposite of the filterOutAngle method.
 *
 * @param lines         Input lines.
 * @param filtered      The output vector of lines not containing those fulfilling the requirement.
 * @param min_angle     The min angle to be considered in degrees. Should be in range [0..180].
 * @param max_angle     The max angle to be considered in degrees. Should be >= min_angle and widthin range [0..180].
 * @return              Returns the number of line segments not included in the output vector.
 */
    CV_WRAP virtual int retainAngle(InputArray lines, OutputArray filtered, float min_angle, float max_angle) = 0;

/**
 * Find all line elements that *are* fullfilling the size requirenmnets.
 * Lines which are shorter than max_length and longer than min_length.
 *
 * @param lines         Input lines.
 * @param filtered      The output vector of lines containing those fulfilling the requirement.
 * @param max_length    Maximum length of the line segment.
 * @param min_length    Minimum length of the line segment.
 * @return              Returns the number of line segments not included in the output vector.
 */
    CV_WRAP virtual int filterSize(InputArray lines, OutputArray filtered, float min_length, float max_length = LSD_NO_SIZE_LIMIT) = 0;

/*
 * Find itnersection point of 2 lines.
 *
 * @param line1         First line in format Vec4i(x1, y1, x2, y2).
 * @param line2         Second line in the same format as line1.
 * @param P             The point where line1 and line2 intersect.
 * @return              The value in variable P is only valid when the return value is true.
 *                      Otherwise, the lines are parallel and the value can be ignored.
 */
    CV_WRAP virtual bool intersection(InputArray line1, InputArray line2, Point& P) = 0;

    virtual ~LineSegmentDetector() {};
};

//! Returns a pointer to a LineSegmentDetector class.
CV_EXPORTS Ptr<LineSegmentDetector> createLineSegmentDetectorPtr(
    int _refine = LSD_REFINE_STD, double _scale = 0.8,
    double _sigma_scale = 0.6, double _quant = 2.0, double _ang_th = 22.5,
    double _log_eps = 0, double _density_th = 0.7, int _n_bins = 1024);

}
#endif
#endif
