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
#include <CGAL/Min_quadrilateral_2.h>
#include <CGAL/Polygon_2.h>

//-- https://github.com/nlohmann/json
//-- used to read and write (City)JSON
#include "json.hpp" //-- it is in the /include/ folder
using json = nlohmann::json;

// For roof area and orientation calculations
#include "roof_calculations.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel   Kernel;
typedef CGAL::Surface_mesh<Point_3>                             Mesh;
typedef CGAL::Polygon_2<Kernel>                            Polygon_2;
typedef Kernel::FT                                                FT;
typedef Kernel::Point_2                                      Point_2;
typedef Kernel::Point_3                                      Point_3;
typedef Kernel::Vector_2                                    Vector_2;
typedef Kernel::Vector_3                                    Vector_3;


namespace PMP = CGAL::Polygon_mesh_processing;

// Declarations
std::vector<std::array<double, 3> > get_vertices(json &j);

void visit_roofsurfaces(json &j, const std::vector<std::array<double, 3> > &vertices);

bool bld_mesh_from_json(json &j, std::string key, Mesh &mesh);

bool triangulate_mesh(Mesh &mesh, bool verbose = false);

FT volume_from_mesh(const Mesh &mesh);

const std::vector<std::string> lod_tier = {"2.2", "2.1", "2.0", "2", "1.3", "1.2", "1.1", "1.0", "1"};

double compute_footprint_orientation(const json &j, const std::vector<std::array<double, 3> > &vertices);


// Main function
int main(int argc, const char *argv[]) {
  //-- will read the file passed as an argument or twobuildings.city.json if nothing is passed
  const char *filename = (argc > 1) ? argv[1] : "../../data/nextbk_2b.city.json";
  // const char* filename = (argc > 1) ? argv[1] : "../../data/9-284-556.city.json";
  std::cout << "Processing: " << filename << std::endl;
  std::ifstream input(filename);
  json j;
  input >> j; //-- store the content of the file in a nlohmann::json object
  input.close();

  //-- get scale from cityJSON
  const std::vector<double> &scale = j["transform"]["scale"].get<std::vector<double> >();

  //-- get vertices from cityJSON
  auto vertices = get_vertices(j);
  double footprint_orientation = compute_footprint_orientation(j, vertices);
  std::cout << "Footprint orientation: " << footprint_orientation << " degrees\n";

  // Process roof surfaces to calculate area and orientation
  // Implementation of process_roof_surfaces function
  for (auto &co: j["CityObjects"].items()) {
    for (auto &g: co.value()["geometry"]) {
      if (g["type"] == "Solid" && g.contains("semantics")) {
        for (size_t i = 0; i < g["boundaries"].size(); i++) {
          for (size_t k = 0; k < g["boundaries"][i].size(); k++) {
            int sem_index = g["semantics"]["values"][i][k];

            // Get only the RoofSurfaces
            if (g["semantics"]["surfaces"][sem_index]["type"].get<std::string>() == "RoofSurface") {
              // Add roof analysis attributes to the semantic surface
              g["semantics"]["surfaces"][sem_index]["attributes"]["processed"] = true;
            }
          }
        }
      }
    }
  }

  // For backward compatibility, still call the visit_roofsurfaces function
  visit_roofsurfaces(j, vertices);


  //-- print out the number of Buildings in the file
  int nobuildings = 0;
  for (auto &co: j["CityObjects"]) {
    if (co["type"] == "Building") {
      nobuildings += 1;
    }
  }
  std::cout << "There are " << nobuildings << " Buildings in the file" << std::endl;
  std::cout << "Number of vertices " << j["vertices"].size() << std::endl;

  //-- add an attribute "volume"
  for (auto &co: j["CityObjects"].items()) {
    if (co.value()["type"] != "Building" || !co.value().contains("children"))
      continue;

    const std::vector<std::string> &children = co.value()["children"].get<std::vector<std::string> >();
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
    std::cout << "Volume for object " << co.key() << ": " << vol << std::endl;
    co.value()["attributes"]["volume"] = vol;
  }

  //-- write to disk the modified city model (out.city.json)
  std::ofstream o("out.city.json");
  o << j.dump(2) << std::endl;
  o.close();

  return 0;
}


bool bld_mesh_from_json(json &j, std::string key, Mesh &mesh) {
  std::vector<std::vector<int> > vertices = j["vertices"].get<std::vector<std::vector<int> > >();
  std::unordered_map<int, CGAL::SM_Vertex_index> index_map;

  for (size_t i = 0; i < lod_tier.size(); i++) {
    if (i > 0) {
      std::cout << "LoD " << lod_tier[i - 1] << " not found, using LoD " << lod_tier[i] << " instead." << std::endl;
    }
    const std::string &lod = lod_tier[i];

    for (auto &g: j["CityObjects"][key]["geometry"].items()) {
      if (g.value()["lod"] == lod) {
        mesh.clear();
        for (auto &shell: g.value()["boundaries"]) {
          for (auto &surface: shell) {
            for (auto &ring: surface) {
              std::vector<CGAL::SM_Vertex_index> face_ids;
              for (auto &v: ring) {
                if (index_map.find(v.get<int>()) != index_map.end()) {
                  face_ids.push_back(index_map[v.get<int>()]);
                } else {
                  CGAL::SM_Vertex_index idx = mesh.add_vertex(
                    Point_3(
                      vertices[v.get<int>()][0],
                      vertices[v.get<int>()][1],
                      vertices[v.get<int>()][2]
                    )
                  );
                  index_map[v.get<int>()] = idx;
                  face_ids.push_back(idx);
                }
              }
              mesh.add_face(face_ids);
            }
          }
        }
        return true;
      }
    }
  }

  // Nothing is found, return false
  return false;
}


bool triangulate_mesh(Mesh &mesh, bool verbose) {
  if (is_empty(mesh)) {
    std::cerr << "Warning: empty mesh" << std::endl;
    return false;
  }

  if (verbose) {
    for (auto f: mesh.faces()) {
      std::cout << "Face " << f.idx() << ": ";
      for (auto v: CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
        const Point_3 &p = mesh.point(v);
        std::cout << "(" << p.x() << ", " << p.y() << ", " << p.z() << ") ";
      }
      std::cout << "\n";
    }
  }

  if (CGAL::is_triangle_mesh(mesh)) {
    std::cout << "Input mesh is triangulated." << std::endl;
  } else {
    // std::cout << "Input mesh is not triangulated." << std::endl;
    PMP::triangulate_faces(mesh);
  }

  // Confirm that all faces are triangles.
  for (boost::graph_traits<Mesh>::face_descriptor f: faces(mesh)) {
    if (!CGAL::is_triangle(halfedge(f, mesh), mesh)) {
      std::cerr << "Error: non-triangular face left in mesh." << std::endl;
      return false;
    }
  }

  if (verbose) {
    for (auto f: mesh.faces()) {
      std::cout << "Face " << f.idx() << ": ";
      for (auto v: CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
        const Point_3 &p = mesh.point(v);
        std::cout << "(" << p.x() << ", " << p.y() << ", " << p.z() << ") ";
      }
      std::cout << "\n";
    }
  }

  return true;
}


FT tetrahedron_volume(const Point_3 &a, const Point_3 &b, const Point_3 &c, const Point_3 &o) {
  Kernel::Vector_3 ab = b - a;
  Kernel::Vector_3 ac = c - a;
  Kernel::Vector_3 oa = a - o;
  FT det = oa * CGAL::cross_product(ab, ac); // = dot(cross(bd, cd), ad)
  return det / 6.0;
}


FT volume_from_mesh(const Mesh &mesh) {
  const Point_3 &o = *mesh.points().begin();
  FT vol = 0.0;

  for (auto f: mesh.faces()) {
    auto it = CGAL::vertices_around_face(mesh.halfedge(f), mesh).begin();
    const Point_3 &a = mesh.point(*it);
    const Point_3 &b = mesh.point(*(++it));
    const Point_3 &c = mesh.point(*(++it));

    vol += tetrahedron_volume(a, b, c, o);
  }

  return vol;
}


// Compute, based on the LOD0 footprint the dominant axis of the building, and return that as azimuth orientation.
double compute_footprint_orientation(const json &j, const std::vector<std::array<double, 3> > &vertices) {
  for (const auto &co: j["CityObjects"].items()) {
    const std::string &building_id = co.key();
    const auto &obj = co.value();
    if (obj["type"] != "Building") continue;

    for (const auto &geom: obj["geometry"]) {
      if (geom["lod"] != "0") continue;

      // Fallback to -1.0 if there is no footprint geometry to be found.
      if (!geom.contains("boundaries") || geom["boundaries"].empty() ||
          !geom["boundaries"][0].is_array() || geom["boundaries"][0].empty()) {
        std::cout << "Building " << building_id << " has no valid LoD0 footprint.\n";
        continue;
      }

      const auto ring = geom["boundaries"][0][0]; // LOD0 always has 1 outer ring.

      // Get the 3D coordinates from the vertex indices, then push them to 2D since the footprint is flat.
      Polygon_2 footprint;
      for (const auto &v_index: ring) {
        int index = v_index.get<int>();
        const auto &pt = vertices[index];
        footprint.push_back(Point_2(pt[0], pt[1]));
        // std::cout << "(" << pt[0] << ", " << pt[1] << ")" << std::endl;
      }

      // Using CGAL, compute the minimum rectangle
      std::vector<Point_2> p_m;
      CGAL::min_rectangle_2(footprint.vertices_begin(), footprint.vertices_end(), std::back_inserter(p_m));

      // Determine the dominant vector, aka the longest vector.
      // Since it's a rectangle, we only have to compare p_m[0] and p_m[1] against p_m[1] and p_m[2].
      Vector_2 v1 = p_m[1] - p_m[0];
      Vector_2 v2 = p_m[2] - p_m[1];
      Vector_2 dominant_vec_2d = (v1.squared_length() > v2.squared_length()) ? v1 : v2; // All hail C++ if-statements.

      if (dominant_vec_2d.y() < 0) dominant_vec_2d = -dominant_vec_2d; // Make sure it points upwards.

      Vector_3 dominant_vec_3d(dominant_vec_2d.x(), dominant_vec_2d.y(), 1);

      // Compute the azimuth for the overall building orientation
      double azimuth = calculate_orientation_azimuth(dominant_vec_3d);
      std::cout << "Building " << building_id << " footprint orientation: " << azimuth << " degrees\n";
    }
  }
  return -1.0;
}

// Visit every 'RoofSurface' in the CityJSON model and print its vertices
void visit_roofsurfaces(json &j, const std::vector<std::array<double, 3> > &vertices) {
  for (auto &co: j["CityObjects"].items()) {
    const std::string &building_id = co.key();
    for (auto &g: co.value()["geometry"]) {
      std::string lod = g["lod"].get<std::string>();
      if (g["type"] == "Solid") {
        for (size_t i = 0; i < g["boundaries"].size(); i++) {
          for (size_t k = 0; k < g["boundaries"][i].size(); k++) {
            int sem_index = g["semantics"]["values"][i][k];

            // Get only the RoofSurfaces
            if (g["semantics"]["surfaces"][sem_index]["type"].get<std::string>() == "RoofSurface") {
              // std::cout << "RoofSurface vertices:" << std::endl;

              // Extract outer ring points
              std::vector<Point_3> outer_ring;
              for (auto &v_idx: g["boundaries"][i][k][0]) {
                int idx = v_idx.get<int>();
                auto &pt = vertices[idx];
                outer_ring.emplace_back(pt[0], pt[1], pt[2]);
              }

              // Extract inner rings points (holes)
              std::vector<std::vector<Point_3> > inner_rings;
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
              // std::cout << "Building: " << building_id << ", LoD: " << lod
              //     << ", RoofSurface area: " << roof_area << " m^2, "
              //     << "orientation: " << roof_orientation << std::endl;
            }
          }
        }
      }
    }
  }
}


// Returns all vertices scaled to their actual values, no translation necessary for our purposes.
std::vector<std::array<double, 3> > get_vertices(json &j) {
  std::vector<std::array<double, 3> > transformed_vertices;
  for (auto &v: j["vertices"]) {
    std::vector<int> vi = v;
    double x = vi[0] * j["transform"]["scale"][0].get<double>();
    double y = vi[1] * j["transform"]["scale"][1].get<double>();
    double z = vi[2] * j["transform"]["scale"][2].get<double>();
    transformed_vertices.push_back({x, y, z});
  }
  return transformed_vertices;
}
