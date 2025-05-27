#include <iostream>
#include <string>
#include <vector>
#include <array>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/helpers.h>

//-- https://github.com/nlohmann/json
//-- used to read and write (City)JSON
#include "json.hpp" //-- it is in the /include/ folder
using json = nlohmann::json;

typedef CGAL::Exact_predicates_inexact_constructions_kernel   Kernel;
typedef Kernel::FT                                                FT;
typedef Kernel::Point_3                                      Point_3;
typedef CGAL::Surface_mesh<Point_3>                             Mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

const std::vector<std::string> lod_tier = {"2.2", "2.1", "2.0", "2", "1.3", "1.2", "1.1", "1.0", "1"};

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
