// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <CGAL/convex_hull_2.h>
#include <iostream>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;

int main() {
  // Polygon with exact coordinates that produce the bug
  Polygon_2 footprint;
  footprint.push_back(Point_2(-37.481, -315.382));
  footprint.push_back(Point_2(-32.167, -325.104));
  footprint.push_back(Point_2(-25.43, -321.417));
  footprint.push_back(Point_2(-30.042, -313.027));
  footprint.push_back(Point_2(-33.912, -315.139));
  footprint.push_back(Point_2(-34.63, -313.823));

  std::cout << "Is simple: " << footprint.is_simple() << std::endl;
  std::cout << "Is convex: " << footprint.is_convex() << std::endl;
  std::cout << "Orientation: " << (footprint.is_counterclockwise_oriented() ? "CCW" : "CW") << std::endl;

  std::cout << "Input polygon vertices:" << std::endl;
  for (auto it = footprint.vertices_begin(); it != footprint.vertices_end(); ++it) {
    std::cout << "(" << it->x() << ", " << it->y() << ")" << std::endl;
  }

  // Compute minimum rectangle, using first the convex_hull_2
  // We found that, in some cases, using directly the footprint, we got shapes not at all representing the BB.
  std::vector<Point_2> first;
  std::vector<Point_2> second;
  std::vector<Point_2> hull_points;
  CGAL::convex_hull_2(footprint.vertices_begin(), footprint.vertices_end(), std::back_inserter(hull_points));
  CGAL::min_rectangle_2(footprint.begin(), footprint.end(), std::back_inserter(first)); // Don't make convex hull
  CGAL::min_rectangle_2(hull_points.begin(), hull_points.end(), std::back_inserter(second)); // First, use convex hull

  std::cout << "min_rectangle_2 returned " << first.size() << " points:" << std::endl;
  for (size_t i = 0; i < first.size(); ++i) {
    std::cout << "Point " << i << ": (" << first[i].x() << ", " << first[i].y() << ")" << std::endl;
  }

  std::cout << "convex_hull_2, then min_rectangle_2 returned " << second.size() << " points:" << std::endl;
  for (size_t i = 0; i < second.size(); ++i) {
    std::cout << "Point " << i << ": (" << second[i].x() << ", " << second[i].y() << ")" << std::endl;
  }

  return 0;
}
