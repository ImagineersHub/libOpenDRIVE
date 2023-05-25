#include "Lane.h"
#include "LaneSection.h"
#include "Math.hpp"
#include "Mesh.h"
#include "OpenDriveMap.h"
#include "Road.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
int main(int argc, char** argv)
{
    if (argc < 2)
    {
        printf("ERROR: too few arguments\n");
        return -1;
    }
    std::string       xodr_file = argv[1];
    odr::OpenDriveMap odr_map(xodr_file);
    const double      eps = 0.1;

    std::vector<odr::Vec3D> lane_pts;
    std::vector<odr::Vec3D> roadmark_pts;

    for (odr::Road road : odr_map.get_roads())
    {
        printf("road: %s, length: %.2f\n", road.id.c_str(), road.length);
        for (odr::LaneSection lanesection : road.get_lanesections())
        {
            const double s_start = lanesection.s0;
            const double s_end = road.get_lanesection_end(lanesection);

            for (odr::Lane lane : lanesection.get_lanes())
            {
                auto lane_mesh = road.get_lane_mesh(lane, eps);
                lane_pts.insert(lane_pts.end(), lane_mesh.vertices.begin(), lane_mesh.vertices.end());

                auto roadmarks = lane.get_roadmarks(s_start, s_end);
                for (const auto& roadmark : roadmarks)
                {
                    auto roadmark_mesh = road.get_roadmark_mesh(lane, roadmark, eps);
                    roadmark_pts.insert(roadmark_pts.end(), roadmark_mesh.vertices.begin(), roadmark_mesh.vertices.end());
                }
            }
        }
    }

    printf("Finished, got %lu lane points, %lu roadmark points\n", lane_pts.size(), roadmark_pts.size());

    odr::RoadNetworkMesh road_network_mesh = odr_map.get_road_network_mesh(eps);
    printf("Got road network mesh\n");

    std::string::size_type pos = xodr_file.find_last_of(".");
    std::string            substr = xodr_file.substr(0, pos);

    // Create the new filename
    std::string newFilename = substr + ".obj";
    std::cout << "Export road mesh to: " << newFilename << std::endl;
    std::ofstream out_file(newFilename);
    out_file << road_network_mesh.get_mesh().get_obj() << std::endl;
    out_file.close();

    return 0;
}
