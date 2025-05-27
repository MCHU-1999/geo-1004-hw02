#include "geometric_difference.h"
#include <limits>
#include <fstream>
#include <iostream>
#include <CGAL/Random.h>

// static CGAL random generator
static CGAL::Random cgal_rand;  // This will be shared across all functions

// function to sample points uniformly on mesh surface
std::vector<Point_3> sample_points_from_mesh(const Mesh& mesh, int num_samples) {
    std::vector<Point_3> sampled_points;
    std::vector<FT> face_areas;
    FT total_area = 0.0;

    // calculate area of each face for weighted sampling
    for (auto f : mesh.faces()) {
        auto it = CGAL::vertices_around_face(mesh.halfedge(f), mesh).begin();
        const Point_3& a = mesh.point(*it); it++;
        const Point_3& b = mesh.point(*it); it++;
        const Point_3& c = mesh.point(*it);

        // Calculate triangle area using cross product
        Kernel::Vector_3 ab = b - a;
        Kernel::Vector_3 ac = c - a;
        Kernel::Vector_3 cross = CGAL::cross_product(ab, ac);
        FT area = std::sqrt(cross.squared_length()) / 2.0;

        face_areas.push_back(area);
        total_area += area;
    }

    // sample points proportionally to face areas
    for (int i = 0; i < num_samples; i++) {
        // Use CGAL's random generator
        double random_area = cgal_rand.get_double(0.0, CGAL::to_double(total_area));
        FT cumulative_area = 0.0;
        auto face_it = mesh.faces().begin();
        size_t face_idx = 0;

        for (auto f : mesh.faces()) {
            cumulative_area += face_areas[face_idx];
            if (CGAL::to_double(cumulative_area) >= random_area) {
                face_it = mesh.faces().begin();
                std::advance(face_it, face_idx);
                break;
            }
            face_idx++;
        }

        // sample point on selected triangle
        auto it = CGAL::vertices_around_face(mesh.halfedge(*face_it), mesh).begin();
        const Point_3& a = mesh.point(*it); it++;
        const Point_3& b = mesh.point(*it); it++;
        const Point_3& c = mesh.point(*it);

        Point_3 sampled_point = sample_point_on_triangle(a, b, c);
        sampled_points.push_back(sampled_point);
    }

    return sampled_points;
}

// Sample a random point on a triangle using barycentric coordinates
Point_3 sample_point_on_triangle(const Point_3& a, const Point_3& b, const Point_3& c) {
    // Use CGAL's random generator
    double r1 = cgal_rand.get_double(0.0, 1.0);
    double r2 = cgal_rand.get_double(0.0, 1.0);

    // Ensure point is inside triangle
    if (r1 + r2 > 1.0) {
        r1 = 1.0 - r1;
        r2 = 1.0 - r2;
    }

    double u = r1;
    double v = r2;
    double w = 1.0 - u - v;

    // Barycentric interpolation
    Kernel::Vector_3 va = a - CGAL::ORIGIN;
    Kernel::Vector_3 vb = b - CGAL::ORIGIN;
    Kernel::Vector_3 vc = c - CGAL::ORIGIN;

    Kernel::Vector_3 result = w * va + u * vb + v * vc;
    return CGAL::ORIGIN + result;
}

// Calculate Hausdorff distance between two point sets
FT hausdorff_distance(const std::vector<Point_3>& set_a, const std::vector<Point_3>& set_b) {
    FT dist_a_to_b = directed_hausdorff_distance(set_a, set_b);
    FT dist_b_to_a = directed_hausdorff_distance(set_b, set_a);
    return std::max(dist_a_to_b, dist_b_to_a);
}

// Calculate directed Hausdorff distance (from one set to another)
FT directed_hausdorff_distance(const std::vector<Point_3>& from_set, const std::vector<Point_3>& to_set) {
    FT max_distance = 0.0;

    for (const Point_3& point : from_set) {
        FT min_distance = point_to_set_distance(point, to_set);
        max_distance = std::max(max_distance, min_distance);
    }

    return max_distance;
}

// Find minimum distance from a point to a set of points
FT point_to_set_distance(const Point_3& point, const std::vector<Point_3>& point_set) {
    FT min_distance = std::numeric_limits<double>::max();

    for (const Point_3& target_point : point_set) {
        FT distance = std::sqrt(CGAL::squared_distance(point, target_point));
        min_distance = std::min(min_distance, distance);
    }

    return min_distance;
}

// Export sampled points to PLY format for visualization
void export_points_to_ply(const std::vector<Point_3>& points, const std::string& filename) {
    std::ofstream out(filename);

    // Write PLY header
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << points.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "end_header\n";

    // Write points with color (red for visibility)
    for (const auto& p : points) {
        out << p.x() << " " << p.y() << " " << p.z() << " 255 0 0\n";
    }

    out.close();
    std::cout << "Exported " << points.size() << " points to " << filename << std::endl;
}

// Export combined point clouds with different colors
void export_combined_points_to_ply(const std::vector<Point_3>& points_lod13,
                                   const std::vector<Point_3>& points_lod22,
                                   const std::string& filename) {
    std::ofstream out(filename);

    // Write PLY header
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << (points_lod13.size() + points_lod22.size()) << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "end_header\n";

    // Write LoD 1.3 points in red
    for (const auto& p : points_lod13) {
        out << p.x() << " " << p.y() << " " << p.z() << " 255 0 0\n";
    }

    // Write LoD 2.2 points in blue
    for (const auto& p : points_lod22) {
        out << p.x() << " " << p.y() << " " << p.z() << " 0 0 255\n";
    }

    out.close();
    std::cout << "Exported combined point cloud to " << filename << std::endl;
}