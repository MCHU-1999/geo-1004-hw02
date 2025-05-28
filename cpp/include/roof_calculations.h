#ifndef ROOF_CALCULATIONS_H
#define ROOF_CALCULATIONS_H

#include <vector>
#include <string>
#include <utility>
#include "json.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/bounding_box.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;

using json = nlohmann::json;

struct PlaneCoordinateSystem {
    Vector_3 normal;
    Point_3 origin;
    Vector_3 u; // local x-axis
    Vector_3 v; // local y-axis
};


/**
 * @brief Analyzes a roof surface by computing its area and orientation.
 * 
 * @param outer_ring The outer boundary of the polygon representing the roof surface.
 * @param inner_rings A list of inner boundaries (holes) within the polygon.
 * @return A pair containing:
 *  - the surface area (as a double)
 *  - the orientation classification (as a string)
 */
std::pair<double, std::string> analyse_roof_surface(
    const std::vector<Point_3> &outer_ring,
    const std::vector<std::vector<Point_3>> &inner_rings
);

/**
 * @brief Calculate the azimuth from the normal vector
 * 
 * @param normal The 3D surface normal vector.
 * @return Azimuth angle in degrees, in the range [0, 360).
 */
double calculate_orientation_azimuth(const Kernel::Vector_3 &normal);

PlaneCoordinateSystem compute_plane_coordinate_system(const std::vector<Point_3> &polygon_points);
Point_2 project_point_to_local_plane(const Point_3 &p, const PlaneCoordinateSystem &plane_cs);

#endif // ROOF_CALCULATIONS_H
