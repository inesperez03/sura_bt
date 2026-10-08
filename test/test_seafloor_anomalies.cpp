#include <vector>

#include "gtest/gtest.h"
#include "sura_bt/seafloor_anomaly_nodes.hpp"

namespace
{
geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

std::vector<geometry_msgs::msg::Point> floorPoints()
{
  std::vector<geometry_msgs::msg::Point> points;
  for (int x = 0; x < 20; ++x) {
    for (int y = 0; y < 20; ++y) {
      points.push_back(point(x * 0.25 + 0.125, y * 0.25 + 0.125,
        4.0 + x * 0.005));
    }
  }
  return points;
}

TEST(SeafloorMap, AccumulatesScansAndIgnoresIsolatedEcho)
{
  sura_bt::SeafloorMap map(0.25, 1000);
  const auto floor = floorPoints();
  map.addPoints(std::vector<geometry_msgs::msg::Point>(floor.begin(), floor.begin() + 200));
  map.addPoints(std::vector<geometry_msgs::msg::Point>(floor.begin() + 200, floor.end()));
  EXPECT_EQ(map.size(), 400U);
  EXPECT_TRUE(map.anomalyCentroids(0.5, 3, 1.0).empty());
  map.addPoints({point(2.125, 2.125, 5.5)});
  EXPECT_TRUE(map.anomalyCentroids(0.5, 3, 1.0).empty());
}

TEST(SeafloorMap, FindsSeparateGroupsAcrossScans)
{
  sura_bt::SeafloorMap map(0.25, 1000);
  map.addPoints(floorPoints());
  map.addPoints({point(1.125, 1.125, 6.0), point(1.375, 1.125, 6.0)});
  EXPECT_TRUE(map.anomalyCentroids(0.5, 3, 1.0).empty());
  map.addPoints({point(1.625, 1.125, 6.0), point(3.125, 3.125, 2.0),
    point(3.375, 3.125, 2.0), point(3.625, 3.125, 2.0)});
  const auto centers = map.anomalyCentroids(0.5, 3, 1.0);
  ASSERT_EQ(centers.size(), 2U);
  EXPECT_NEAR(centers[0].x, 1.375, 1e-6);
  EXPECT_NEAR(centers[0].y, 1.125, 1e-6);
  EXPECT_NEAR(centers[0].z, 5.01, 0.01);
  EXPECT_NEAR(centers[1].x, 3.375, 1e-6);
  EXPECT_NEAR(centers[1].z, 3.03, 0.01);
}

TEST(SeafloorMap, EvictsOldestCellsAtLimit)
{
  sura_bt::SeafloorMap map(0.25, 2);
  map.addPoints({point(0.125, 0.125, 1.0), point(0.375, 0.125, 1.0)});
  map.addPoints({point(0.125, 0.125, 1.0), point(0.625, 0.125, 1.0)});
  EXPECT_EQ(map.size(), 2U);
}
}  // namespace
