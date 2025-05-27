#include <vector>
#include <random>
#include <limits>
#include <fstream>
#include <iostream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>
#include <unordered_map>
#include "json.hpp"
using json = nlohmann::json;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef Kernel::FT FT;

// Sample a random point on a triangle using barycentric coordinates
Point_3 sample_point_on_triangle(const Point_3& a, const Point_3& b, const Point_3& c) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);

    double r1 = uniform_dist(gen);
    double r2 = uniform_dist(gen);

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


// Function to sample points uniformly on mesh surface
std::vector<Point_3> sample_points_from_mesh(const Mesh& mesh, int num_samples = 1000) {
    std::vector<Point_3> sampled_points;
    std::vector<FT> face_areas;
    FT total_area = 0.0;

    // Calculate area of each face for weighted sampling
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

    // Set up random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);

    // Sample points proportionally to face areas
    for (int i = 0; i < num_samples; i++) {
        // Select face based on area weight
        double random_area = uniform_dist(gen) * CGAL::to_double(total_area);
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

        // Sample point on selected triangle
        auto it = CGAL::vertices_around_face(mesh.halfedge(*face_it), mesh).begin();
        const Point_3& a = mesh.point(*it); it++;
        const Point_3& b = mesh.point(*it); it++;
        const Point_3& c = mesh.point(*it);

        Point_3 sampled_point = sample_point_on_triangle(a, b, c);
        sampled_points.push_back(sampled_point);
    }

    return sampled_points;
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

// Calculate directed Hausdorff distance (from one set to another)
FT directed_hausdorff_distance(const std::vector<Point_3>& from_set, const std::vector<Point_3>& to_set) {
    FT max_distance = 0.0;

    for (const Point_3& point : from_set) {
        FT min_distance = point_to_set_distance(point, to_set);
        max_distance = std::max(max_distance, min_distance);
    }

    return max_distance;
}

// Calculate Hausdorff distance between two point sets
FT hausdorff_distance(const std::vector<Point_3>& set_a, const std::vector<Point_3>& set_b) {
    FT dist_a_to_b = directed_hausdorff_distance(set_a, set_b);
    FT dist_b_to_a = directed_hausdorff_distance(set_b, set_a);
    return std::max(dist_a_to_b, dist_b_to_a);
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


// Function to get mesh for specific LoD
bool get_mesh_for_lod(json& j, const std::string& building_key, const std::string& target_lod, Mesh& mesh) {
    std::vector<std::vector<int>> vertices = j["vertices"].get<std::vector<std::vector<int>>>();
    std::unordered_map<int, CGAL::SM_Vertex_index> index_map;

    // Look for the specific LoD
    for (auto& g: j["CityObjects"][building_key]["geometry"].items()) {
        if (g.value()["lod"] == target_lod) {
            mesh.clear();
            for (auto& shell : g.value()["boundaries"]) {
                for (auto& surface : shell) {
                    for (auto& ring : surface) {
                        std::vector<CGAL::SM_Vertex_index> face_idx;
                        for (auto& v : ring) {
                            if (index_map.find(v.get<int>()) != index_map.end()) {
                                face_idx.push_back(index_map[v.get<int>()]);
                            } else {
                                CGAL::SM_Vertex_index idx = mesh.add_vertex(
                                    Point_3(
                                        vertices[v.get<int>()][0],
                                        vertices[v.get<int>()][1],
                                        vertices[v.get<int>()][2]
                                    )
                                );
                                index_map[v.get<int>()] = idx;
                                face_idx.push_back(idx);
                            }
                        }
                        mesh.add_face(face_idx);
                    }
                }
            }
            return true;
        }
    }
    return false;
}
