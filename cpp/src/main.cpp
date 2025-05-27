/*
+------------------------------------------------------------------------------+
|                                                                              |
|                                 Hugo Ledoux                                  |
|                             h.ledoux@tudelft.nl                              |
|                                  2025-05-07                                  |
|                                                                              |
+------------------------------------------------------------------------------+
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/helpers.h>


// Add these includes to your existing code
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/property_map.h>

#include <unordered_map>
#include <CGAL/Min_quadrilateral_2.h>
#include <CGAL/Polygon_2.h>

//-- https://github.com/nlohmann/json
//-- used to read and write (City)JSON
#include "json.hpp" //-- it is in the /include/ folder
using json = nlohmann::json;

// including our functions
#include "roof_calculations.h"
#include "volume.h"
#include "geometric_difference.h"


typedef CGAL::Exact_predicates_inexact_constructions_kernel   Kernel;
typedef CGAL::Polygon_2<Kernel>                            Polygon_2;
typedef Kernel::FT                                                FT;
typedef Kernel::Point_2                                      Point_2;
typedef Kernel::Point_3                                      Point_3;
typedef Kernel::Vector_2                                    Vector_2;
typedef Kernel::Vector_3                                    Vector_3;
typedef CGAL::Surface_mesh<Point_3>                             Mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

// Declarations
/**
 * @brief Extracts and transforms 3D vertices from a JSON object.
 * @param j A City JSON object
 * 
 * @return A vector of transformed 3D vertices as arrays of doubles.
 */
std::vector<std::array<double, 3>> get_vertices(json &j);

/**
 * @brief Visit roof surfaces and compute their orientation and area, and write that into json object.
 * @param j A City JSON object
 * @param vertices A vector of transformed 3D vertices as arrays of doubles.
 * 
 * @return Nothing
 */
void compute_roof_area_orientation(json &j, const std::vector<std::array<double, 3>> &vertices);

/**
 * @brief Visit and extract the dominant axis of the building using the LOD0 footprint, and write that as azimuth orientation.
 * @param j A City JSON object
 * @param vertices A vector of transformed 3D vertices as arrays of doubles.
 * 
 * @return Nothing
 */
void compute_footprint_orientation(json &j, const std::vector<std::array<double, 3>> &vertices);

/**
 * @brief Calculate and add an attribute "volume" into json object. 
 * @param j A City JSON object
 * @param vertices A vector of transformed 3D vertices as arrays of doubles.
 * 
 * @return Nothing
 */
void compute_volume(json &j);

/**
 * @brief Calculate and add an attribute "hausdorff_lod_22_13" into json object. 
 * @param j A City JSON object
 * 
 * @return Nothing
 */
void compute_hausdorff(json &j);


// Main function
int main(int argc, const char *argv[]) {
  //-- will read the file passed as an argument or twobuildings.city.json if nothing is passed
  const char *filename = (argc > 1) ? argv[1] : "../../data/nextbk_2b.city.json";

  std::cout << "Processing: " << filename << std::endl;
  std::ifstream input(filename);
  json j;
  input >> j; //-- store the content of the file in a nlohmann::json object
  input.close();

  //-- get scale from cityJSON
  const std::vector<double> &scale = j["transform"]["scale"].get<std::vector<double>>();

  //-- get vertices from cityJSON
  auto vertices = get_vertices(j);

  //-- print out the number of Buildings in the file
  int nobuildings = 0;
  for (auto &co: j["CityObjects"]) {
    if (co["type"] == "Building") nobuildings += 1;
  }
  std::cout << "There are " << nobuildings << " Buildings in the file" << std::endl;
  std::cout << "Number of vertices " << j["vertices"].size() << std::endl;

  //-- Calculate and add an attribute "orientation"
  compute_footprint_orientation(j, vertices);

  //-- Calculate and add attributes "area" "orientation" into roofSurface
  compute_roof_area_orientation(j, vertices);

  //-- Calculate and add an attribute "volume"
  compute_volume(j);

  //-- Calculate and add an attribute "hausdorff_lod_22_13"
  compute_hausdorff(j);

  //-- write to disk the modified city model (out.city.json)
  std::ofstream o("out.city.json");
  o << j.dump(2) << std::endl;
  o.close();

  return 0;
}

void compute_footprint_orientation(json &j, const std::vector<std::array<double, 3>> &vertices) {
  for (auto &co: j["CityObjects"].items()) {
    // const std::string &building_id = co.key();
    auto &obj = co.value();
    if (obj["type"] != "Building") continue;

    for (const auto &geom: obj["geometry"]) {
      if (geom["lod"] != "0") continue;

      // Fallback to -1.0 if there is no footprint geometry to be found.
      if (!geom.contains("boundaries") || geom["boundaries"].empty() ||
          !geom["boundaries"][0].is_array() || geom["boundaries"][0].empty()) {
        std::cerr << "Building " << co.key() << " has no valid LoD0 footprint.\n";
        continue;
      }

      const auto ring = geom["boundaries"][0][0]; // LOD0 always has 1 outer ring.

      // Get the 3D coordinates from the vertex indices, then push them to 2D since the footprint is flat.
      Polygon_2 footprint;
      for (const auto &v_index: ring) {
        int index = v_index.get<int>();
        const auto &pt = vertices[index];
        footprint.push_back(Point_2(pt[0], pt[1]));
      }

      // compute the minimum rectangle
      std::vector<Point_2> p_m;
      CGAL::min_rectangle_2(footprint.vertices_begin(), footprint.vertices_end(), std::back_inserter(p_m));

      // Determine the dominant vector, aka the longest vector.
      // Since it's a rectangle, we only have to compare p_m[0] and p_m[1] against p_m[1] and p_m[2].
      Vector_2 v1 = p_m[1] - p_m[0];
      Vector_2 v2 = p_m[2] - p_m[1];
      Vector_2 dominant_vec_2d = (v1.squared_length() > v2.squared_length()) ? v1 : v2; // All hail C++ if-statements.

      // Make sure it points upwards.
      if (dominant_vec_2d.y() < 0) dominant_vec_2d = -dominant_vec_2d;
      Vector_3 dominant_vec_3d(dominant_vec_2d.x(), dominant_vec_2d.y(), 1);

      // Compute the azimuth for the overall building orientation
      double azimuth = calculate_orientation_azimuth(dominant_vec_3d);
      // std::cout << "Building " << co.key() << " footprint orientation: " << azimuth << " degrees\n";

      // Write the footprint orientation
      obj["attributes"]["orientation"] = azimuth;
    }
  }
}

// Visit and extract the dominant axis of the building using the LOD0 footprint, and write that as azimuth orientation.
void compute_roof_area_orientation(json &j, const std::vector<std::array<double, 3>> &vertices) {
  for (auto &co: j["CityObjects"].items()) {
    for (auto &g: co.value()["geometry"]) {
      std::string lod = g["lod"].get<std::string>();
      if (g["type"] == "Solid") {
        for (size_t i = 0; i < g["boundaries"].size(); i++) {
          for (size_t k = 0; k < g["boundaries"][i].size(); k++) {
            int sem_index = g["semantics"]["values"][i][k];

            // Get only the RoofSurfaces
            if (g["semantics"]["surfaces"][sem_index]["type"].get<std::string>() == "RoofSurface") {

              // Extract outer ring points
              std::vector<Point_3> outer_ring;
              for (auto &v_idx: g["boundaries"][i][k][0]) {
                int idx = v_idx.get<int>();
                auto &pt = vertices[idx];
                outer_ring.emplace_back(pt[0], pt[1], pt[2]);
              }

              // Extract inner rings points (holes)
              std::vector<std::vector<Point_3>> inner_rings;
              for (size_t r = 1; r < g["boundaries"][i][k].size(); r++) {
                std::vector<Point_3> inner_ring;
                for (auto &v_idx: g["boundaries"][i][k][r]) {
                  int idx = v_idx.get<int>();
                  auto &pt = vertices[idx];
                  inner_ring.emplace_back(pt[0], pt[1], pt[2]);
                }
                inner_rings.push_back(inner_ring);
              }

              // Print outer ring
              // std::cout << "Outer ring points:" << std::endl;
              // for (const auto &p: outer_ring) {
              //   std::cout << "(" << p.x() << ", " << p.y() << ", " << p.z() << ")" << std::endl;
              // }
              //
              // // Print inner rings
              // for (size_t r = 0; r < inner_rings.size(); ++r) {
              //   std::cout << "Inner ring " << r << " points:" << std::endl;
              //   for (const auto &p: inner_rings[r]) {
              //     std::cout << "(" << p.x() << ", " << p.y() << ", " << p.z() << ")" << std::endl;
              //   }
              // }

              // Big boy function
              auto [roof_area, roof_orientation] = analyse_roof_surface(outer_ring, inner_rings);

              // Print results
              // std::cout << "Building: " << co.key() << ", LoD: " << lod
              //     << ", RoofSurface area: " << roof_area << " m^2, "
              //     << "orientation: " << roof_orientation << std::endl;
              
              // Write results
              g["semantics"]["surfaces"][sem_index]["area"] = roof_area;
              g["semantics"]["surfaces"][sem_index]["orientation"] = roof_orientation;
            }
          }
        }
      }
    }
  }
}

void compute_volume(json &j){
  const std::vector<double> &scale = j["transform"]["scale"].get<std::vector<double>>();
  
  for (auto &co: j["CityObjects"].items()) {
    if (co.value()["type"] != "Building" || !co.value().contains("children"))
      continue;

    const std::vector<std::string> &children = co.value()["children"].get<std::vector<std::string>>();
    Mesh mesh;
    FT vol = 0.0;
    for (auto &child: children) {
      if (!bld_mesh_from_json(j, child, mesh)) {
        // Error
        std::cerr << "Failed to convert building to mesh since this object doesn't have LoD >= 1.0" << std::endl;
        continue;
      }
      if (!triangulate_mesh(mesh)) {
        // Error
        std::cerr << "Failed to triangulate mesh" << std::endl;
        continue;
      }
      vol += volume_from_mesh(mesh);
    }

    vol = vol * scale[0] * scale[1] * scale[2];
    // std::cout << "Volume for object " << co.key() << ": " << vol << std::endl;
    co.value()["attributes"]["volume"] = vol;
  }
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

// calculate geometric difference (Hausdorff distance) between LoD 1.3 and LoD 2.2
void compute_hausdorff(json &j) {
  const std::vector<double> &scale = j["transform"]["scale"].get<std::vector<double>>();

  for (auto& co : j["CityObjects"].items()) {
    if (co.value()["type"] != "Building" || !co.value().contains("children"))
      continue;

    const std::vector<std::string>& children = co.value()["children"].get<std::vector<std::string>>();

    // Get LoD 1.3 and LoD 2.2 meshes
    Mesh mesh_lod13, mesh_lod22;
    bool has_lod13 = false, has_lod22 = false;

    for (const auto& child : children) {
      if (!has_lod13 && get_mesh_for_lod(j, child, "1.3", mesh_lod13)) {
        triangulate_mesh(mesh_lod13);
        has_lod13 = true;
      }
      if (!has_lod22 && get_mesh_for_lod(j, child, "2.2", mesh_lod22)) {
        triangulate_mesh(mesh_lod22);
        has_lod22 = true;
      }
    }

    if (has_lod13 && has_lod22) {
      // Sample points from both meshes
      std::vector<Point_3> points_lod13 = sample_points_from_mesh(mesh_lod13, 1000);
      std::vector<Point_3> points_lod22 = sample_points_from_mesh(mesh_lod22, 1000);

      // ADD THESE LINES - Export visualization files
      // export_points_to_ply(points_lod13, co.key() + "_lod13_points.ply");
      // export_points_to_ply(points_lod22, co.key() + "_lod22_points.ply");
      // export_combined_points_to_ply(points_lod13, points_lod22, co.key() + "_combined_points.ply");

      // Calculate Hausdorff distance
      FT hausdorff_dist = hausdorff_distance(points_lod13, points_lod22);

      // apply scale transformation
      hausdorff_dist = hausdorff_dist * scale[0]; // Assuming uniform scaling

      std::cout << "Hausdorff distance for object " << co.key() << ": " << hausdorff_dist << std::endl;
      co.value()["attributes"]["hausdorff_lod_22_13"] = CGAL::to_double(hausdorff_dist);
    } else {
      std::cout << "Warning: Could not find both LoD 1.3 and LoD 2.2 for object " << co.key() << std::endl;
      co.value()["attributes"]["hausdorff_lod_22_13"] = -1.0; // Indicate missing data
    }
  }
}

// Returns all vertices scaled to their actual values, no translation necessary for our purposes.
std::vector<std::array<double, 3>> get_vertices(json &j) {
  std::vector<std::array<double, 3>> transformed_vertices;
  for (auto &v: j["vertices"]) {
    std::vector<int> vi = v;
    double x = vi[0] * j["transform"]["scale"][0].get<double>();
    double y = vi[1] * j["transform"]["scale"][1].get<double>();
    double z = vi[2] * j["transform"]["scale"][2].get<double>();
    transformed_vertices.push_back({x, y, z});
  }
  return transformed_vertices;
}
