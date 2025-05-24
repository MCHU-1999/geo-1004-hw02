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

//-- https://github.com/nlohmann/json
//-- used to read and write (City)JSON
#include "json.hpp" //-- it is in the /include/ folder
#include "roof_calculations.h" // For roof area and orientation calculations

// CGAL includes
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;

using json = nlohmann::json;


// Declarations
int get_no_roof_surfaces(json &j);

std::vector<std::array<double, 3> > get_vertices(json &j);

void visit_roofsurfaces(json &j, const std::vector<std::array<double, 3> > &vertices);

RoofAnalysisResult analyse_roof_surface(
    const std::vector<Point_3> &outer_ring,
    const std::vector<std::vector<Point_3> > &inner_rings
);


int main(int argc, const char *argv[]) {
    //-- will read the file passed as an argument or twobuildings.city.json if nothing is passed
    const char *filename = (argc > 1) ? argv[1] : "../../data/nextbk_2b.city.json";
    // const char* filename = (argc > 1) ? argv[1] : "../../data/9-284-556.city.json";
    std::cout << "Processing: " << filename << std::endl;
    std::ifstream input(filename);
    json j;
    input >> j; //-- store the content of the file in a nlohmann::json object
    input.close();

    //-- get the total number of RoofSurface in the file
    int noroofsurfaces = get_no_roof_surfaces(j);
    std::cout << "Total RoofSurface: " << noroofsurfaces << std::endl;

    auto vertices = get_vertices(j);

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
    for (auto &co: j["CityObjects"]) {
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


// Visit every 'RoofSurface' in the CityJSON model and print its vertices
void visit_roofsurfaces(json &j, const std::vector<std::array<double, 3>> &vertices) {
    for (auto &co: j["CityObjects"].items()) {
        std::string building_id = co.key();
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

                            // Big boy function
                            RoofAnalysisResult result = analyse_roof_surface(outer_ring, inner_rings);

                            // Print results
                            std::cout << "Building: " << building_id << ", LoD: " << lod
                                      << ", RoofSurface area: " << result.area << " m^2, "
                                      << "orientation: " << result.orientation << std::endl;
                        }
                    }
                }
            }
        }
    }
}


// Returns all vertices transformed (scale + translate)
std::vector<std::array<double, 3> > get_vertices(json &j) {
    std::vector<std::array<double, 3> > transformed_vertices;
    for (auto &v: j["vertices"]) {
        std::vector<int> vi = v;
        double x = (vi[0] * j["transform"]["scale"][0].get<double>()) + j["transform"]["translate"][0].get<double>();
        double y = (vi[1] * j["transform"]["scale"][1].get<double>()) + j["transform"]["translate"][1].get<double>();
        double z = (vi[2] * j["transform"]["scale"][2].get<double>()) + j["transform"]["translate"][2].get<double>();
        transformed_vertices.push_back({x, y, z});
    }
    return transformed_vertices;
}

// Count the total number of RoofSurfaces in the file
int get_no_roof_surfaces(json &j) {
    int count = 0;
    for (auto &co: j["CityObjects"].items()) {
        for (auto &g: co.value()["geometry"]) {
            if (g["type"] == "Solid" && g.contains("semantics")) {
                for (size_t i = 0; i < g["boundaries"].size(); i++) {
                    for (size_t k = 0; k < g["boundaries"][i].size(); k++) {
                        int sem_index = g["semantics"]["values"][i][k];
                        if (g["semantics"]["surfaces"][sem_index]["type"].get<std::string>() == "RoofSurface") {
                            count++;
                        }
                    }
                }
            }
        }
    }
    return count;
}
