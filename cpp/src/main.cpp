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

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/helpers.h>


//-- https://github.com/nlohmann/json
//-- used to read and write (City)JSON
#include "json.hpp" //-- it is in the /include/ folder

using json = nlohmann::json;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3                                    Point_3;
typedef CGAL::Surface_mesh<Point_3>                           Mesh;
typedef Kernel::FT                                              FT;

namespace PMP = CGAL::Polygon_mesh_processing;

int   get_no_roof_surfaces(json &j);
void  list_all_vertices(json& j);
void  visit_roofsurfaces(json &j);

bool bld_mesh_from_json(json& j, std::string key, Mesh& mesh);
bool triangulate_mesh(Mesh& mesh, bool verbose=false);
FT volume_from_mesh(const Mesh& mesh);
const std::vector<std::string> lod_tier = {"2.2", "2.1", "2.0", "2", "1.3", "1.2", "1.1", "1.0", "1"};


int main(int argc, const char * argv[]) {
  //-- will read the file passed as argument or twobuildings.city.json if nothing is passed
  const char* filename = (argc > 1) ? argv[1] : "../../data/nextbk_2b.city.json";
  std::cout << "Processing: " << filename << std::endl;
  std::ifstream input(filename);
  json j;
  input >> j; //-- store the content of the file in a nlohmann::json object
  input.close();

  //-- get scale from cityJSON
  const std::vector<double>& scale = j["transform"]["scale"].get<std::vector<double>>();

  //-- get total number of RoofSurface in the file
  int noroofsurfaces = get_no_roof_surfaces(j);
  std::cout << "Total RoofSurface: " << noroofsurfaces << std::endl;

  // list_all_vertices(j);
  // visit_roofsurfaces(j);

  //-- print out the number of Buildings in the file
  int nobuildings = 0;
  for (auto& co : j["CityObjects"]) {
    if (co["type"] == "Building") {
      nobuildings += 1;
    }
  }
  std::cout << "There are " << nobuildings << " Buildings in the file" << std::endl;

  //-- print out the number of vertices in the file
  std::cout << "Number of vertices " << j["vertices"].size() << std::endl;

  //-- add an attribute "volume"
  for (auto& co : j["CityObjects"].items()) {
    if (co.value()["type"] != "Building" || !co.value().contains("children"))
      continue;

    const std::vector<std::string>& children = co.value()["children"].get<std::vector<std::string>>();
    Mesh mesh;
    FT vol = 0.0;
    for (auto& child: children) {
      if (!bld_mesh_from_json(j, child, mesh)) {
        // Error
        std::cerr << "Failed to convert building to mesh since this object doesn't have LoD >= 1.0" << std::endl;
      }
      if (!triangulate_mesh(mesh)) {
        // Error
        std::cerr << "Failed to triangulate mesh" << std::endl;
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


bool bld_mesh_from_json(json& j, std::string key, Mesh& mesh) {

  std::vector<std::vector<int>> vertices = j["vertices"].get<std::vector<std::vector<int>>>();
  std::unordered_map<int, CGAL::SM_Vertex_index> index_map;
  
  for (size_t i = 0; i < lod_tier.size(); i++) {
    if (i > 0) {
      std::cout << "LoD " << lod_tier[i-1] << " not found, using LoD " << lod_tier[i] << " instead." << std::endl;
    }
    const std::string& lod = lod_tier[i];

    for (auto& g: j["CityObjects"][key]["geometry"].items()) {
      if (g.value()["lod"] == lod) {
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
  }
  
  // Nothing is found, return false
  return false;
}


bool triangulate_mesh(Mesh& mesh, bool verbose) { 
  if (is_empty(mesh)) {
    std::cerr << "Warning: empty mesh" << std::endl;
    return false;
  }

  if (verbose) {
    for (auto f: mesh.faces()) {
      std::cout << "Face " << f.idx() << ": ";
      for (auto v: CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
        const Point_3& p = mesh.point(v);
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
        const Point_3& p = mesh.point(v);
        std::cout << "(" << p.x() << ", " << p.y() << ", " << p.z() << ") ";
      }
      std::cout << "\n";
    }
  }

  return true;
}

FT tetrahedron_volume(const Point_3& a, const Point_3& b, const Point_3& c, const Point_3& o) {
  Kernel::Vector_3 ab = b - a;
  Kernel::Vector_3 ac = c - a;
  Kernel::Vector_3 oa = a - o;
  FT det = oa * CGAL::cross_product(ab, ac);  // = dot(cross(bd, cd), ad)
  return det / 6.0;
}

FT volume_from_mesh(const Mesh& mesh) {
  const Point_3 o = *mesh.points().begin();
  FT vol = 0.0;

  for (auto f: mesh.faces()) {
    auto it = CGAL::vertices_around_face(mesh.halfedge(f), mesh).begin();
    const Point_3& a = mesh.point(*it); it++;
    const Point_3& b = mesh.point(*it); it++;
    const Point_3& c = mesh.point(*it);

    vol += tetrahedron_volume(a, b, c, o);
  }

  return vol;
}



// Visit every 'RoofSurface' in the CityJSON model and output its geometry (the arrays of indices)
// Useful to learn to visit the geometry boundaries and at the same time check their semantics.
void visit_roofsurfaces(json &j) {
  for (auto& co : j["CityObjects"].items()) {
    for (auto& g : co.value()["geometry"]) {
      if (g["type"] == "Solid") {
        for (int i = 0; i < g["boundaries"].size(); i++) {
          for (int j = 0; j < g["boundaries"][i].size(); j++) {
            int sem_index = g["semantics"]["values"][i][j];
            if (g["semantics"]["surfaces"][sem_index]["type"].get<std::string>().compare("RoofSurface") == 0) {
              std::cout << "RoofSurface: " << g["boundaries"][i][j] << std::endl;
            }
          }
        }
      }
    }
  }
}


// Returns the number of 'RooSurface' in the CityJSON model
int get_no_roof_surfaces(json &j) {
  int total = 0;
  for (auto& co : j["CityObjects"].items()) {
    for (auto& g : co.value()["geometry"]) {
      if (g["type"] == "Solid") {
        for (auto& shell : g["semantics"]["values"]) {
          for (auto& s : shell) {
            if (g["semantics"]["surfaces"][s.get<int>()]["type"].get<std::string>().compare("RoofSurface") == 0) {
              total += 1;
            }
          }
        }
      }
    }
  }
  return total;
}


// CityJSON files have their vertices compressed: https://www.cityjson.org/specs/1.1.1/#transform-object
// this function visits all the surfaces and print the (x,y,z) coordinates of each vertex encountered
void list_all_vertices(json& j) {
  for (auto& co : j["CityObjects"].items()) {
    std::cout << "= CityObject: " << co.key() << std::endl;
    for (auto& g : co.value()["geometry"]) {
      if (g["type"] == "Solid") {
        for (auto& shell : g["boundaries"]) {
          for (auto& surface : shell) {
            for (auto& ring : surface) {
              std::cout << "---" << std::endl;
              for (auto& v : ring) { 
                std::vector<int> vi = j["vertices"][v.get<int>()];
                double x = (vi[0] * j["transform"]["scale"][0].get<double>()) + j["transform"]["translate"][0].get<double>();
                double y = (vi[1] * j["transform"]["scale"][1].get<double>()) + j["transform"]["translate"][1].get<double>();
                double z = (vi[2] * j["transform"]["scale"][2].get<double>()) + j["transform"]["translate"][2].get<double>();
                std::cout << std::setprecision(2) << std::fixed << v << " (" << x << ", " << y << ", " << z << ")" << std::endl;                
              }
            }
          }
        }
      }
    }
  }
}