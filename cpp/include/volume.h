#ifndef VOLUME_H
#define VOLUME_H

#include <iostream>
#include <string>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include "json.hpp" //-- it is in the /include/ folder
using json = nlohmann::json;

typedef CGAL::Exact_predicates_inexact_constructions_kernel   Kernel;
typedef Kernel::Point_3                                      Point_3;
typedef Kernel::FT                                                FT;
typedef CGAL::Surface_mesh<Point_3>                             Mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

bool bld_mesh_from_json(json &j, std::string key, Mesh &mesh);
bool triangulate_mesh(Mesh &mesh, bool verbose = false);
FT volume_from_mesh(const Mesh &mesh);

#endif
