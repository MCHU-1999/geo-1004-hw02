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

/**
 * @brief Builds a mesh from a JSON object using a specific key.
 * 
 * @param j JSON object containing geometry and transformation data.
 * @param key Key in the JSON that identifies the object to process.
 * @param mesh Output mesh to populate.
 * @return True if the mesh was successfully built, false otherwise.
 */
bool bld_mesh_from_json(json &j, std::string key, Mesh &mesh);

/**
 * @brief Triangulates all faces in the given mesh.
 * 
 * @param mesh The mesh to be triangulated.
 * @param verbose If true, prints diagnostic information during processing.
 * @return True if triangulation was successful, false otherwise.
 */
bool triangulate_mesh(Mesh &mesh, bool verbose = false);

/**
 * @brief Computes the volume enclosed by the mesh.
 * 
 * @param mesh The input mesh, assumed to be closed and oriented.
 * @return The computed volume as a scalar of type FT.
 */
FT volume_from_mesh(const Mesh &mesh);


#endif