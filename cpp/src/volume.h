#ifndef VOLUME_H
#define VOLUME_H

#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Constrained_triangulation_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/IO/PLY.h>

#include "json.hpp" //-- it is in the /include/ folder
using json = nlohmann::json;

#include "roof_calculations.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel   Kernel;
typedef Kernel::Point_3                                      Point_3;
typedef Kernel::Point_2                                      Point_2;
typedef Kernel::FT                                                FT;
typedef CGAL::Surface_mesh<Point_3>                             Mesh;

// Just for handling polygons with holes
typedef CGAL::Triangulation_vertex_base_2<Kernel>                 Vb;
typedef CGAL::Constrained_triangulation_face_base_2<Kernel>       Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>              TDS;
typedef CGAL::Exact_predicates_tag                              Itag;
typedef CGAL::Constrained_triangulation_2<Kernel, TDS, Itag>      CT;
typedef CGAL::Polygon_2<Kernel>                            Polygon_2;


namespace PMP = CGAL::Polygon_mesh_processing;

/**
 * @brief Builds a mesh from a JSON object using a specific key and LoD.
 * 
 * @param j JSON object containing geometry and transformation data.
 * @param building_key Key in the JSON that identifies the object to process.
 * @param target_lod The interested LoD level.
 * @param mesh Output mesh to populate.
 * @return True if the mesh was successfully built, false otherwise.
 */
bool get_mesh_for_lod(json& j, const std::string& building_key, const std::string& target_lod, Mesh& mesh);

/**
 * @brief Builds a mesh from a JSON object using a specific key.
 * 
 * This function will iterate through all the LoD levels in this list (vector) {"2.2", "2.1", "2.0", "2", "1.3", "1.2", "1.1", "1.0", "1"} 
 * and try to find at least something with volume (so anything that's not LoD 0 is fine)
 * 
 * @param j JSON object containing geometry and transformation data.
 * @param key Key in the JSON that identifies the object to process.
 * @param mesh Output mesh to populate.
 * @return True if the mesh was successfully built, false otherwise.
 */
bool mesh_from_json(json &j, std::string key, Mesh &mesh);

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