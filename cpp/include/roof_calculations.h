#ifndef ROOF_CALCULATIONS_H
#define ROOF_CALCULATIONS_H

#include <vector>
#include <array>
#include <string>
#include "json.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;

using json = nlohmann::json;

// Structure to hold roof analysis results
struct RoofAnalysisResult {
    double area;
    std::string orientation;
};

/**
 * Process all roof surfaces in the CityJSON model to calculate and add area and orientation
 * @param j The CityJSON model
 * @param vertices The transformed vertices of the model
 */
void process_roof_surfaces(json &j, const std::vector<std::array<double, 3>> &vertices);

/**
 * Analyze a single roof surface and return its area and orientation
 * @param outer_ring The outer boundary of the roof surface
 * @param inner_rings The inner boundaries (holes) of the roof surface
 * @return A RoofAnalysisResult with the calculated area and orientation
 */
RoofAnalysisResult analyse_roof_surface(
    const std::vector<Point_3> &outer_ring,
    const std::vector<std::vector<Point_3>> &inner_rings
);

#endif // ROOF_CALCULATIONS_H
