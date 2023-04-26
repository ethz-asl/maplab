/*
 *    Filename: dird-map-filtering.cc
 *  Created on: April 12, 2021
 *      Author: Florian Achermann (acfloria@ethz.ch)
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid-map-amo/grid-map-filtering.h"

#include <glog/logging.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/opencv.hpp>

namespace amo {

void inpaint_layer(std::unique_ptr<grid_map::GridMap>& map,
    std::string input_layer,
    std::string output_layer,
    double radius) {
  CHECK_NOTNULL(map);
  CHECK_GT(radius, 0.0);

  map->add("inpaint_mask", 0.0);

  for (grid_map::GridMapIterator iterator(*map); !iterator.isPastEnd(); ++iterator) {
    if (!map->isValid(*iterator, input_layer)) {
      map->at("inpaint_mask", *iterator) = 1.0;
    }
  }

  cv::Mat originalImage, mask, filledImage;
  const float minValue = map->get(input_layer).minCoeffOfFinites();
  const float maxValue = map->get(input_layer).maxCoeffOfFinites();

  grid_map::GridMapCvConverter::toImage<float, 1>(*map, input_layer, CV_32FC1, minValue, maxValue,
                                                          originalImage);
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(*map, "inpaint_mask", CV_8UC1, mask);

  const double radiusInPixels = radius / map->getResolution();
  cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);

  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(filledImage, output_layer, *map, minValue, maxValue);
  map->erase("inpaint_mask");
}

}  // namespace amo
