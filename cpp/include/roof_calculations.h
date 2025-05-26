#ifndef ROOF_CALCULATIONS_H
#define ROOF_CALCULATIONS_H

#include <vector>
#include <string>
#include <utility>
#include "json.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;

using json = nlohmann::json;

std::pair<double, std::string> analyse_roof_surface(
    const std::vector<Point_3> &outer_ring,
    const std::vector<std::vector<Point_3>> &inner_rings
);

double calculate_orientation_azimuth(const Kernel::Vector_3 &normal);

#endif // ROOF_CALCULATIONS_H
