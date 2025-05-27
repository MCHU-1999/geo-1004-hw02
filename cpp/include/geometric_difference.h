#ifndef GEOMETRIC_DIFFERENCE_H
#define GEOMETRIC_DIFFERENCE_H

#include <vector>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef Kernel::FT FT;

// Function declarations for geometric difference calculations
std::vector<Point_3> sample_points_from_mesh(const Mesh& mesh, int num_samples = 1000);
Point_3 sample_point_on_triangle(const Point_3& a, const Point_3& b, const Point_3& c);
FT hausdorff_distance(const std::vector<Point_3>& set_a, const std::vector<Point_3>& set_b);
FT directed_hausdorff_distance(const std::vector<Point_3>& from_set, const std::vector<Point_3>& to_set);
FT point_to_set_distance(const Point_3& point, const std::vector<Point_3>& point_set);

// Visualization functions
void export_points_to_ply(const std::vector<Point_3>& points, const std::string& filename);
void export_combined_points_to_ply(const std::vector<Point_3>& points_lod13, 
                                   const std::vector<Point_3>& points_lod22, 
                                   const std::string& filename);

#endif // GEOMETRIC_DIFFERENCE_H

//neelabh