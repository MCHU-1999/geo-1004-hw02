#include "roof_calculations.h"
#include <vector>
#include <string>

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

struct PlaneCoordinateSystem {
    Vector_3 normal;
    Point_3 origin;
    Vector_3 u; // local x-axis
    Vector_3 v; // local y-axis
};

// Based on the cross-product algorithm as explained in my DTM course notes.
Vector_3 calculate_polygon_normal(const std::vector<Point_3> &polygon_points) {
    // Knowing that a valid polygon is planar and has at least 3 points, we can calculate the normal.
    const Point_3 &A = polygon_points[0];
    const Point_3 &B = polygon_points[1];
    const Point_3 &C = polygon_points[2];

    Vector_3 v1 = B - A;
    Vector_3 v2 = C - A;

    Vector_3 normal = CGAL::cross_product(v1, v2);

    // Normalise the length of the normal vector.
    double length = std::sqrt(normal.squared_length());
    normal = normal / length;

    // Make sure it points upwards since we are looking at roofs.
    if (normal.z() < 0) normal = -normal;

    return normal;
}


// Convert a normal vector to one of 8 orientations or "horizontal"
std::string classify_orientation(const Vector_3 &normal) {
    if (std::abs(normal.z()) > 0.95) // If Z is close to 1, it's near vertical thus horizontal
        return "horizontal";

    // Compute azimuth in degrees, atan2(x, y) makes it clockwise from north.
    double angle = std::atan2(normal.x(), normal.y()) * 180.0 / CGAL_PI;
    if (angle < 0) angle += 360;

    // Map the angle_deg to one of the 8 (given) compass directions
    static const std::array<std::string, 8> directions = {"NE", "EN", "ES", "SE", "SW", "WS", "WN", "NW"};
    int index = static_cast<int>(angle / 45.0) % 8;
    return directions[index];
}

// Since we're intending to use calculate the area of a polygon in 2D, we need to project the polygon to a plane.
// We define this plane by a normal vector, an origin and two vectors that define the plane.
// The two vectors are orthogonal to each other and are perpendicular to the normal vector.
// The origin is the first point of the polygon, so we know that it lies on the plane.
PlaneCoordinateSystem compute_plane_coordinate_system(const std::vector<Point_3> &polygon_points) {
    Vector_3 normal = calculate_polygon_normal(polygon_points);
    Point_3 origin = polygon_points[0];

    // Create a vector from origin to second point
    Vector_3 vec = polygon_points[1] - origin;

    // Project vec onto the plane (subtract the normal component)
    double dot = vec * normal;
    vec = vec - (normal * dot);

    // Normalise the vector
    double length = std::sqrt(vec.squared_length());

    // The local x- and y-axis, which are perpendicular to the normal vector and orthogonal to each other.
    // We also make sure to normalise the length of the vectors. Since we want scale-independent 3D to 2D projections.
    Vector_3 u = vec / length;
    Vector_3 v = CGAL::cross_product(normal, u);

    return {normal, origin, u, v};
}


// Helper function to project a point to the local plane defined by the plane coordinate system.
Point_2 project_point_to_local_plane(const Point_3 &p, const PlaneCoordinateSystem &plane_cs) {
    Vector_3 op = p - plane_cs.origin;
    double x = op * plane_cs.u;
    double y = op * plane_cs.v;
    return {x, y};
}


// Helper function to calculate the signed area of a ring.
// A Counter-Clockwise (CCW) ring yields a positive area, while a CW ring yields a negative area.
double signed_area(const std::vector<Point_2> &ring, const Point_2 &origin) {
    double area = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const Point_2 &p1 = ring[i];
        const Point_2 &p2 = ring[(i + 1) % ring.size()];
        area += 0.5 * ((p1.x() - origin.x()) * (p2.y() - origin.y()) - (p2.x() - origin.x()) * (p1.y() - origin.y()));
    }
    return area;
}


// Combining the functions above to calculate the area of a polygon.
double calculate_polygon_area(const std::vector<Point_3> &polygon_points,
                              const std::vector<std::vector<Point_3> > &inner_rings) {
    PlaneCoordinateSystem plane_cs = compute_plane_coordinate_system(polygon_points);
    std::vector<Point_2> outer_ring_2d, inner_ring_2d;

    // Project the outer ring to the local plane.
    for (const auto &p: polygon_points)
        outer_ring_2d.push_back(project_point_to_local_plane(p, plane_cs));

    // Compute the bounding box of the 2D projected outer ring and do -1 for both coordinates.
    CGAL::Bbox_2 bbox = CGAL::bounding_box(outer_ring_2d.begin(), outer_ring_2d.end()).bbox();
    Point_2 origin_2d(bbox.xmin() - 1.0, bbox.ymin() - 1.0);

    // Make sure that the area of the outer ring is positive, in case its orientation is wrong.
    double area = std::abs(signed_area(outer_ring_2d, origin_2d));

    // Project the inner rings to the local plane.
    for (const auto &ring: inner_rings) {
        std::vector<Point_2> ring_2d;
        for (const auto &p: ring)
            ring_2d.push_back(project_point_to_local_plane(p, plane_cs));

        // Subtract the area of the inner ring(s) from the total area.
        // We make sure that the area is positive and then subtracted from the total area, since holes are negative.
        area -= std::abs(signed_area(ring_2d, origin_2d));
    }

    return area;
}


std::pair<double, std::string> analyse_roof_surface(
    const std::vector<Point_3> &outer_ring,
    const std::vector<std::vector<Point_3>> &inner_rings
) {
    Vector_3 normal = calculate_polygon_normal(outer_ring);
    std::string orientation = classify_orientation(normal);
    double area = calculate_polygon_area(outer_ring, inner_rings);
    return {area, orientation};
}
