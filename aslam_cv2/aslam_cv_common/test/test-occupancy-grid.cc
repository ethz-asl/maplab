#include <aslam/common/entrypoint.h>
#include <eigen-checks/gtest.h>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>

#include "aslam/common/occupancy-grid.h"

typedef aslam::common::WeightedKeypoint<double, double, size_t> Point;
typedef aslam::common::WeightedOccupancyGrid<Point> WeightedOccupancyGrid;

TEST(OccupancyGrid, addPointOrReplaceWeakestIfCellFull) {
  WeightedOccupancyGrid grid(2.0, 2.0, 1.0, 1.0);

  static constexpr size_t kMaxNumPointsPerCell = 2u;
  EXPECT_TRUE(
      grid.addPointOrReplaceWeakestIfCellFull(Point(0.5, 0.0, 0.9, 0), kMaxNumPointsPerCell));
  EXPECT_TRUE(
      grid.addPointOrReplaceWeakestIfCellFull(Point(0.5, 0.0, 1.0, 1), kMaxNumPointsPerCell));
  EXPECT_FALSE(
      grid.addPointOrReplaceWeakestIfCellFull(Point(0.5, 0.0, 0.8, 2), kMaxNumPointsPerCell));
  // Higher weight replaces point.
  EXPECT_TRUE(
      grid.addPointOrReplaceWeakestIfCellFull(Point(0.5, 0.0, 1.1, 3), kMaxNumPointsPerCell));
  // The cell should now contain the second and last added points with a weight of 1.1 and 1.0.
  const WeightedOccupancyGrid::PointList& cell = grid.getGridCell(0.5, 0.0);
  ASSERT_EQ(cell.size(), 2u);
  EXPECT_EQ(cell[0].weight, 1.1);
  EXPECT_EQ(cell[1].weight, 1.0);
  EXPECT_EQ(cell[0].id, 3u);
  EXPECT_EQ(cell[1].id, 1u);

  ASSERT_TRUE(grid.getGridCell(0.0, 1.5).empty());
  ASSERT_TRUE(grid.getGridCell(1.5, 1.5).empty());
  ASSERT_TRUE(grid.getGridCell(1.5, 0.0).empty());

  // Test on second cell.
  EXPECT_TRUE(
      grid.addPointOrReplaceWeakestIfCellFull(Point(0.5, 1.0, 1.0, 4), kMaxNumPointsPerCell));
  EXPECT_TRUE(
      grid.addPointOrReplaceWeakestIfCellFull(Point(0.5, 1.5, 1.0, 5), kMaxNumPointsPerCell));
  EXPECT_FALSE(
      grid.addPointOrReplaceWeakestIfCellFull(Point(0.5, 1.5, 1.0, 6), kMaxNumPointsPerCell));
}

TEST(OccupancyGrid, removeWeightedPointsFromOverfullCells) {
  WeightedOccupancyGrid grid(2.0, 2.0, 1.0, 1.0);

  grid.addPointUnconditional(Point(0.5, 0.0, 0.3, 0));
  grid.addPointUnconditional(Point(0.5, 0.0, 0.2, 1));
  grid.addPointUnconditional(Point(0.5, 0.0, 0.4, 2));

  ASSERT_EQ(grid.getNumPoints(), 3u);
  ASSERT_TRUE(grid.getGridCell(0.0, 1.5).empty());
  ASSERT_TRUE(grid.getGridCell(1.5, 1.5).empty());
  ASSERT_TRUE(grid.getGridCell(1.5, 0.0).empty());

  // Remove points from full cells.
  static constexpr size_t kMaxNumPointsPerCell = 2u;
  EXPECT_EQ(
      grid.removeWeightedPointsFromOverfullCells(kMaxNumPointsPerCell), 1u);

  const WeightedOccupancyGrid::PointList& cell = grid.getGridCell(0.5, 0.5);
  ASSERT_EQ(cell.size(), kMaxNumPointsPerCell);

  // The points should be sorted by descending score.
  EXPECT_EQ(cell[0].id, 2u);
  EXPECT_EQ(cell[1].id, 0u);
}

TEST(OccupancyGrid, CellIndexing) {
  WeightedOccupancyGrid grid(2.0, 2.0, 1.0, 1.0);

  // Add N points to cell: 2:00 1:10 0:01 1:11
  grid.addPointUnconditional(Point(0.5, 0.5, 1.0, 0));
  grid.addPointUnconditional(Point(0.5, 0.5, 1.0, 1));
  grid.addPointUnconditional(Point(0.5, 1.5, 1.0, 0));
  grid.addPointUnconditional(Point(1.5, 1.5, 1.0, 0));

  EXPECT_EQ(grid.getGridCell(0.5, 0.5).size(), 2u);
  EXPECT_EQ(grid.getGridCell(1.5, 0.5).size(), 0u);
  EXPECT_EQ(grid.getGridCell(0.5, 1.5).size(), 1u);
  EXPECT_EQ(grid.getGridCell(1.5, 1.5).size(), 1u);

  // Test corner cases.
  grid.reset();
  grid.addPointUnconditional(Point(0.5, 0.5, 1.0, 0)); // Cell 0,0

  grid.addPointUnconditional(Point(0.5, 1.0, 1.0, 1)); // Cell 0,1
  grid.addPointUnconditional(Point(0.5, 1.0, 1.0, 1)); // Cell 0,1

  grid.addPointUnconditional(Point(1.0, 1.0, 1.0, 2)); // Cell 1,1
  grid.addPointUnconditional(Point(1.0, 1.0, 1.0, 2)); // Cell 1,1
  grid.addPointUnconditional(Point(1.0, 1.0, 1.0, 2)); // Cell 1,1

  EXPECT_EQ(grid.getGridCell(0.5, 0.5).size(), 1u);
  EXPECT_EQ(grid.getGridCell(1.5, 0.5).size(), 0u);
  EXPECT_EQ(grid.getGridCell(0.5, 1.5).size(), 2u);
  EXPECT_EQ(grid.getGridCell(1.5, 1.5).size(), 3u);
}

TEST(OccupancyGrid, AddInvalidPointCoordinates) {
  const double kGridSize = 2.0;
  WeightedOccupancyGrid grid(kGridSize, kGridSize, 1.0, 1.0);

  EXPECT_DEATH(grid.addPointUnconditional(Point(-0.1, 0.1, 1.0, 0)), "^");
  EXPECT_DEATH(grid.addPointUnconditional(Point(-0.1, -0.1, 1.0, 1)), "^");
  EXPECT_DEATH(grid.addPointUnconditional(Point(0.1, -0.1, 1.0, 2)), "^");
  EXPECT_DEATH(grid.addPointUnconditional(Point(0.1, kGridSize, 1.0, 3)), "^");
  EXPECT_DEATH(grid.addPointUnconditional(Point(kGridSize, 0.1, 1.0, 4)), "^");
  EXPECT_DEATH(grid.addPointUnconditional(Point(kGridSize, kGridSize, 1.0, 5)), "^");

  static constexpr size_t kMaxNumPointsPerCell = 2u;
  EXPECT_DEATH(
      grid.addPointOrReplaceWeakestIfCellFull(Point(-0.1, 0.1, 1.0, 0),
                                              kMaxNumPointsPerCell), "^");
  EXPECT_DEATH(
      grid.addPointOrReplaceWeakestIfCellFull(Point(-0.1, -0.1, 1.0, 1),
                                              kMaxNumPointsPerCell), "^");
  EXPECT_DEATH(
      grid.addPointOrReplaceWeakestIfCellFull(Point(0.1, -0.1, 1.0, 2),
                                              kMaxNumPointsPerCell), "^");
  EXPECT_DEATH(
      grid.addPointOrReplaceWeakestIfCellFull(Point(0.1, kGridSize, 1.0, 3),
                                              kMaxNumPointsPerCell), "^");
  EXPECT_DEATH(
      grid.addPointOrReplaceWeakestIfCellFull(Point(kGridSize, 0.1, 1.0, 4),
                                              kMaxNumPointsPerCell), "^");
  EXPECT_DEATH(
      grid.addPointOrReplaceWeakestIfCellFull(Point(kGridSize, kGridSize, 1.0, 5),
                                              kMaxNumPointsPerCell), "^");
}

TEST(OccupancyGrid, AddPointOrReplaceWeakestNearestPoints) {
  const double kGridSize = 50.0;
  const double kGridCellSize = 10.0;
  const double kMinDistanceBetweenPoints = 5.0;
  WeightedOccupancyGrid grid(kGridSize, kGridSize, kGridCellSize, kGridCellSize);

  // Test rejection of the same point that violates the min. distance.
  grid.addPointOrReplaceWeakestNearestPoints(Point(0.1, 0.1, 1.0, 0), kMinDistanceBetweenPoints);
  grid.addPointOrReplaceWeakestNearestPoints(Point(0.1, 0.1, 3.0, 1), kMinDistanceBetweenPoints);
  grid.addPointOrReplaceWeakestNearestPoints(Point(0.1, 0.1, 2.0, 2), kMinDistanceBetweenPoints);
  ASSERT_EQ(grid.getNumPoints(), 1u);
  ASSERT_EQ(grid.getGridCell(0.1, 0.1).size(), 1u);
  EXPECT_EQ(grid.getGridCell(0.1, 0.1)[0].id, 1u);  // Point with id 1 has highest score of 3.0

  // Point2 should be selected over point1 as it is closer than the min. distance and has
  // higher score.
  grid.reset();
  Point point1(7.5, 7.5, 1.0, 1);
  Point point2(7.5 + 0.95 * static_cast<double>(kMinDistanceBetweenPoints), 7.5, 2.0, 2);

  grid.addPointOrReplaceWeakestNearestPoints(point1, kMinDistanceBetweenPoints);
  grid.addPointOrReplaceWeakestNearestPoints(point2, kMinDistanceBetweenPoints);
  ASSERT_EQ(grid.getNumPoints(), 1u);
  ASSERT_EQ(grid.getGridCell(5.0, 7.5).size(), 0u);
  ASSERT_EQ(grid.getGridCell(15.0, 7.5).size(), 1u);
  // The id of the first point in the grid cell has to be 2u only point 2 should remain in this
  // cell.
  EXPECT_EQ(grid.getGridCell(15.0, 7.5)[0].id, 2u);

  // Point4 should be rejected as it is too close to point3 and point3 has a higher score.
  grid.reset();
  Point point3(7.5, 7.5, 2.0, 3);
  Point point4(7.5 + 0.95 * static_cast<double>(kMinDistanceBetweenPoints), 7.5, 1.0, 4);

  grid.addPointOrReplaceWeakestNearestPoints(point3, kMinDistanceBetweenPoints);
  grid.addPointOrReplaceWeakestNearestPoints(point4, kMinDistanceBetweenPoints);
  ASSERT_EQ(grid.getNumPoints(), 1u);
  ASSERT_EQ(grid.getGridCell(15.0, 7.5).size(), 0u);
  ASSERT_EQ(grid.getGridCell(5.0, 7.5).size(), 1u);
  EXPECT_EQ(grid.getGridCell(5.0, 7.5)[0].id, 3u);

  // No point should be rejected as the distance is equal to the allowed min. distance.
  grid.reset();
  Point point5(7.5, 7.5, 2.0, 5);
  Point point6(7.5 + static_cast<double>(kMinDistanceBetweenPoints), 7.5, 1.0, 6);
  grid.addPointOrReplaceWeakestNearestPoints(point5, kMinDistanceBetweenPoints);
  grid.addPointOrReplaceWeakestNearestPoints(point6, kMinDistanceBetweenPoints);
  ASSERT_EQ(grid.getNumPoints(), 2u);
  ASSERT_EQ(grid.getGridCell(5.0, 7.5).size(), 1u);
  EXPECT_EQ(grid.getGridCell(5.0, 7.5)[0].id, 5u);
  ASSERT_EQ(grid.getGridCell(15.0, 7.5).size(), 1u);
  EXPECT_EQ(grid.getGridCell(15.0, 7.5)[0].id, 6u);

  // Now we add a point in the middle that should reject point point5 and point6 as it has a higher
  // score.
  Point point7(7.5 + 0.5 * static_cast<double>(kMinDistanceBetweenPoints), 7.5, 5.0, 7);
  grid.addPointOrReplaceWeakestNearestPoints(point7, kMinDistanceBetweenPoints);
  ASSERT_EQ(grid.getNumPoints(), 1u);
  ASSERT_EQ(grid.getGridCell(5.0, 7.5).size(), 0u);
  ASSERT_EQ(grid.getGridCell(15.0, 7.5).size(), 1u);
  EXPECT_EQ(grid.getGridCell(15.0, 7.5)[0].id, 7u);
}

TEST(OccupancyGrid, AddPointOrReplaceWeakestNearestPointsRandom) {
  const double kMinDistanceBetweenPoints = 5.0;
  const double kWidth = 752.0;
  const double kHeight = 480.0;
  WeightedOccupancyGrid grid(kHeight, kWidth, kMinDistanceBetweenPoints,
                             kMinDistanceBetweenPoints);

  // Generate some random points.
  const size_t kNumRandomPoints = 1e3;
  Eigen::Matrix2Xd random_points(2, kNumRandomPoints);
  random_points.setRandom();
  random_points = (random_points.array() + 1.0) / 2.0;
  random_points.row(0) *= kHeight;
  random_points.row(1) *= kWidth;

  const double kCrazyHugeScore = 10.0;
  for (int idx = 0; idx < kNumRandomPoints; ++idx) {
    grid.addPointOrReplaceWeakestNearestPoints(
        Point(random_points(0,idx), random_points(1, idx),
              kCrazyHugeScore, idx), kMinDistanceBetweenPoints);
  }
  const size_t initial_num_points = grid.getNumPoints();

  // Let's add the same points again but with a lower score.
  Eigen::VectorXd random_score(kNumRandomPoints);
  random_score.setRandom();

  const int kIndexSecondAddition = 1e6;
  for (int idx = 0; idx < kNumRandomPoints; ++idx) {
    grid.addPointOrReplaceWeakestNearestPoints(
        Point(random_points(0,idx), random_points(1,idx),
              random_score(idx), kIndexSecondAddition), kMinDistanceBetweenPoints);
  }

  // Make sure no points got removed.
  EXPECT_EQ(grid.getNumPoints(), initial_num_points);
  WeightedOccupancyGrid::PointList points;
  grid.getAllPointsInGrid(&points);
  EXPECT_EQ(points.size(), grid.getNumPoints());
}

TEST(OccupancyGrid, GetOccupancyMask) {
  // Create a grid with some points.
  const double kGridSize = 100.0;
  const double kCellSize = 25.0;
  WeightedOccupancyGrid grid(kGridSize, kGridSize, kCellSize, kCellSize);
  grid.addPointUnconditional(Point(37, 37, 1.0, 0));
  grid.addPointUnconditional(Point(37, 37, 1.0, 0));
  grid.addPointUnconditional(Point(75, 75, 1.0, 2));
  ASSERT_EQ(grid.getNumPoints(), 3u);

  // Get the mask.
  const WeightedOccupancyGrid::CoordinatesType kMaskRadiusAroundPointsPx = 10.0;
  const size_t kMaxPointsPerCell = 2u;
  cv::Mat mask = grid.getOccupancyMask(kMaskRadiusAroundPointsPx, kMaxPointsPerCell);

  // Cell 0,0 should be masked completly as it is full. Check that the cell is completely masked.
  cv::Mat cell_00(mask, cv::Rect(25, 25, kMaskRadiusAroundPointsPx, kMaskRadiusAroundPointsPx));
  EXPECT_EQ(cv::countNonZero(cell_00), 0);

  cv::Mat cell_00_oversize(mask, cv::Rect(24, 24, kCellSize + 2, kCellSize + 2));
  EXPECT_EQ(cv::countNonZero(cell_00_oversize), 4 * 25 + 4);

  // Border cases for cell mask.
  EXPECT_EQ(mask.at<unsigned char>(25, 25), 0);
  EXPECT_EQ(mask.at<unsigned char>(25, 49), 0);
  EXPECT_EQ(mask.at<unsigned char>(49, 25), 0);
  EXPECT_EQ(mask.at<unsigned char>(49, 49), 0);

  EXPECT_EQ(mask.at<unsigned char>(24, 24), 255);
  EXPECT_EQ(mask.at<unsigned char>(24, 50), 255);
  EXPECT_EQ(mask.at<unsigned char>(50, 24), 255);
  EXPECT_EQ(mask.at<unsigned char>(50, 50), 255);

  // Check that the neighborhood around the points is masked.
  EXPECT_EQ(mask.at<unsigned char>(75, 75), 0);
  EXPECT_EQ(mask.at<unsigned char>(75 + kMaskRadiusAroundPointsPx, 75), 0);
  EXPECT_EQ(mask.at<unsigned char>(75 - kMaskRadiusAroundPointsPx, 75), 0);
  EXPECT_EQ(mask.at<unsigned char>(75, 75 + kMaskRadiusAroundPointsPx), 0);
  EXPECT_EQ(mask.at<unsigned char>(75, 75 - kMaskRadiusAroundPointsPx), 0);

  // Further away from the point the image shouldn't be masked.
  EXPECT_EQ(mask.at<unsigned char>(75 + kMaskRadiusAroundPointsPx + 1, 75), 255);
  EXPECT_EQ(mask.at<unsigned char>(75 - kMaskRadiusAroundPointsPx - 1, 75), 255);
  EXPECT_EQ(mask.at<unsigned char>(75, 75 + kMaskRadiusAroundPointsPx + 1), 255);
  EXPECT_EQ(mask.at<unsigned char>(75, 75 - kMaskRadiusAroundPointsPx - 1), 255);
}

TEST(OccupancyGrid, InvalidGridParameters) {
  // Zero sized grid.
  EXPECT_DEATH(WeightedOccupancyGrid(0.0, 1.0, 1.0, 1.0), "^");
  EXPECT_DEATH(WeightedOccupancyGrid(1.0, 0.0, 1.0, 1.0), "^");
  EXPECT_DEATH(WeightedOccupancyGrid(0.0, 0.0, 1.0, 1.0), "^");

  // Cell size bigger than grid.
  EXPECT_DEATH(WeightedOccupancyGrid(1.0, 1.0, 2.0, 0.5), "^");
  EXPECT_DEATH(WeightedOccupancyGrid(1.0, 1.0, 0.5, 2.0), "^");
  EXPECT_DEATH(WeightedOccupancyGrid(1.0, 1.0, 2.0, 2.0), "^");
}

ASLAM_UNITTEST_ENTRYPOINT
