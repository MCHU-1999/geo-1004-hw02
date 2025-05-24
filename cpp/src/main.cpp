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
#include <iomanip>

//-- https://github.com/nlohmann/json
//-- used to read and write (City)JSON
#include "json.hpp" //-- it is in the /include/ folder

using json = nlohmann::json;

int get_no_roof_surfaces(json &j);
std::vector<std::array<double,3>> list_all_vertices(json& j);
void visit_roofsurfaces(json &j, const std::vector<std::array<double,3>>& vertices);



int main(int argc, const char * argv[]) {
  //-- will read the file passed as argument or twobuildings.city.json if nothing is passed
  const char* filename = (argc > 1) ? argv[1] : "../../data/nextbk_2b.city.json";
  // const char* filename = (argc > 1) ? argv[1] : "../../data/9-284-556.city.json";
  std::cout << "Processing: " << filename << std::endl;
  std::ifstream input(filename);
  json j;
  input >> j; //-- store the content of the file in a nlohmann::json object
  input.close();

  //-- get total number of RoofSurface in the file
  int noroofsurfaces = get_no_roof_surfaces(j);
  std::cout << "Total RoofSurface: " << noroofsurfaces << std::endl;

  auto vertices = list_all_vertices(j);
  visit_roofsurfaces(j, vertices);

  //-- print out the number of Buildings in the file
  int nobuildings = 0;
  for (auto& co : j["CityObjects"]) {
    if (co["type"] == "Building") {
      nobuildings += 1;
    }
  }
  std::cout << "There are " << nobuildings << " Buildings in the file" << std::endl;
  std::cout << "Number of vertices " << j["vertices"].size() << std::endl;

  //-- add an attribute "volume"
  for (auto& co : j["CityObjects"]) {
    if (co["type"] == "Building") {
      co["attributes"]["volume"] = -1;
    }
  }

  //-- write to disk the modified city model (out.city.json)
  std::ofstream o("out.city.json");
  o << j.dump(2) << std::endl;
  o.close();

  return 0;
}


// Visit every 'RoofSurface' in the CityJSON model and output its geometry (the arrays of indices)
// Useful to learn to visit the geometry boundaries and at the same time check their semantics.
void visit_roofsurfaces(json &j, const std::vector<std::array<double,3>>& vertices) {
  for (auto& co : j["CityObjects"].items()) {
    for (auto& g : co.value()["geometry"]) {
      if (g["type"] == "Solid") {
        for (size_t i = 0; i < g["boundaries"].size(); i++) {         // shells
          for (size_t k = 0; k < g["boundaries"][i].size(); k++) {      // surfaces
            int sem_index = g["semantics"]["values"][i][k];
            if (g["semantics"]["surfaces"][sem_index]["type"].get<std::string>() == "RoofSurface") {
              std::cout << "RoofSurface vertices:" << std::endl;
              // iterate rings
              for (size_t r = 0; r < g["boundaries"][i][k].size(); r++) {
                for (auto& v_idx : g["boundaries"][i][k][r]) {
                  int idx = v_idx.get<int>();
                  auto& pt = vertices[idx];
                  std::cout << std::fixed << std::setprecision(6)
                            << "(" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")" << std::endl;
                }
              }
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

// Returns all vertices transformed (scale + translate)
std::vector<std::array<double,3>> list_all_vertices(json& j) {
  std::vector<std::array<double,3>> transformed_vertices;
  for (auto& v : j["vertices"]) {
    std::vector<int> vi = v;
    double x = (vi[0] * j["transform"]["scale"][0].get<double>()) + j["transform"]["translate"][0].get<double>();
    double y = (vi[1] * j["transform"]["scale"][1].get<double>()) + j["transform"]["translate"][1].get<double>();
    double z = (vi[2] * j["transform"]["scale"][2].get<double>()) + j["transform"]["translate"][2].get<double>();
    transformed_vertices.push_back({x,y,z});
  }
  return transformed_vertices;
}