#include "volume.h"

const std::vector<std::string> lod_tier = {"2.2", "2.1", "2.0", "2", "1.3", "1.2", "1.1", "1.0", "1"};

struct Point2Compare {
  static constexpr double EPSILON = 1e-9;
  
  bool operator()(const Point_2& a, const Point_2& b) const {
    if (std::abs(a.x() - b.x()) > EPSILON) {
      return a.x() < b.x();
    }
    return a.y() < b.y() - EPSILON;
  }
};

// Function to get mesh for specific LoD
bool get_mesh_for_lod(json& j, const std::string& building_key, const std::string& target_lod, Mesh& mesh) {
  std::vector<std::array<int, 3>> vertices = j["vertices"].get<std::vector<std::array<int, 3>>>();
  const std::array<double, 3> &scale = j["transform"]["scale"].get<std::array<double, 3>>();
  std::unordered_map<int, CGAL::SM_Vertex_index> index_map;

  // Look for the specific LoD
  for (auto& g: j["CityObjects"][building_key]["geometry"].items()) {
    if (g.value()["lod"] == target_lod) {
      mesh.clear();
      for (auto& shell : g.value()["boundaries"]) {
        for (auto& surface : shell) {
          if (surface.size() > 1) {
            // Oh shoot we have a hole here
            // construct 2+ non-intersecting nested polygons
            std::vector<Point_3> outer_ring_3;
            for (auto& v : surface[0]) {
              outer_ring_3.push_back(
                Point_3(
                  vertices[v.get<int>()][0] * scale[0],
                  vertices[v.get<int>()][1] * scale[1],
                  vertices[v.get<int>()][2] * scale[2]
                )
              );
            }

            // Create a local PlaneCoordinateSystem
            const PlaneCoordinateSystem& plane_cs = compute_plane_coordinate_system(outer_ring_3);

            // Map Point_2 to CGAL::SM_Vertex_index for later use
            std::map<Point_2, CGAL::SM_Vertex_index, Point2Compare> point_lift_map;

            std::vector<Polygon_2> all_rings;
            for (size_t i = 0; i < surface.size(); i++) {
              Polygon_2 ring_2;
              for (auto& v : surface[i]) {
                CGAL::SM_Vertex_index vertex_idx;
                if (index_map.find(v.get<int>()) == index_map.end()) {
                  vertex_idx = mesh.add_vertex(
                    Point_3(
                      vertices[v.get<int>()][0] * scale[0],
                      vertices[v.get<int>()][1] * scale[1],
                      vertices[v.get<int>()][2] * scale[2]
                    )
                  );
                  index_map[v.get<int>()] = vertex_idx;
                } else {
                  vertex_idx = index_map[v.get<int>()];
                }
                Point_2 projected = project_point_to_local_plane(mesh.point(vertex_idx), plane_cs);
                point_lift_map.insert({projected, vertex_idx});
                ring_2.push_back(projected);
              }
              all_rings.push_back(ring_2);
            }

            // Insert the polygons into a constrained triangulation
            CT ct;
            for (auto& polygon : all_rings) {
              ct.insert_constraint(polygon.vertices_begin(), polygon.vertices_end(), true);
            }

            // Take out the triangles, map to mesh_pt_id and add faces into mesh
            for (auto fit = ct.finite_faces_begin(); fit != ct.finite_faces_end(); ++fit) {
              // if (!ct.is_i is_in_domain(fit)) continue;  // Skip triangles outside
              
              std::vector<CGAL::SM_Vertex_index> tri_idx;
              for (int i = 0; i < 3; ++i) {
                Point_2 &p2 = fit->vertex(i)->point();
                tri_idx.push_back(point_lift_map[p2]);
                // std::cout << "id=" << point_lift_map[p2] << "\n";
              }
              mesh.add_face(tri_idx);
            }
          } else {
            std::vector<CGAL::SM_Vertex_index> vertex_ids;
            for (auto& ring : surface) {
              for (auto& v : ring) {
                if (index_map.find(v.get<int>()) != index_map.end()) {
                  vertex_ids.push_back(index_map[v.get<int>()]);
                } else {
                  CGAL::SM_Vertex_index vertex_idx = mesh.add_vertex(
                    Point_3(
                      vertices[v.get<int>()][0] * scale[0],
                      vertices[v.get<int>()][1] * scale[1],
                      vertices[v.get<int>()][2] * scale[2]
                    )
                  );
                  index_map[v.get<int>()] = vertex_idx;
                  vertex_ids.push_back(vertex_idx);
                }
              }
              mesh.add_face(vertex_ids);
            }
          }
        }
      }
      return true;
    }
  }
  // Nothing is found, return false
  return false;
}

bool mesh_from_json(json &j, std::string key, Mesh &mesh) {
  for (size_t i = 0; i < lod_tier.size(); i++) {
    if (i > 0) {
      std::cout << "LoD " << lod_tier[i - 1] << " not found, using LoD " << lod_tier[i] << " instead." << std::endl;
    }
    if (get_mesh_for_lod(j, key, lod_tier[i], mesh)) return true;
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

  if (!PMP::triangulate_faces(mesh)) {
    // Confirm that all faces are triangles.
    std::cout << "Warning: non-triangular face left in mesh." << std::endl;
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
