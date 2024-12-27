#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "db.h"

#include "geo/geo.h"
#include "geo/line.h"
#include "quadtree.h"

namespace bg = boost::geometry;

#define TEST_STOPS
#define TEST_SINGLE_POINT

using PointUFI = int;
using RoadUFI = int;
using RoadDirection = std::string;
using RoadLength = double;
using PointMap = std::map<PointUFI, PointUFI>;

struct d_line_simple {
    int roadufi;
    int from_ufi;
    int to_ufi;
    g_line line;
};

struct nearest_road_info
{
    RoadUFI road_ufi;
    RoadDirection road_direction;
    PointUFI from_ufi;
    PointUFI to_ufi;
    g_point closest_point;
    d_segment nearest_segment;
};

struct road_info
{
    RoadUFI road_ufi;
    std::string ezi_road_name_label;
    RoadDirection direction_code;
    PointUFI from_ufi;
    PointUFI to_ufi;
    RoadLength road_length_meters;
    g_line line;
    bool reversed = false;
};


using bg_point = bg::model::point<double, 2, bg::cs::geographic<bg::degree>>;

using PointUFI = int;
using RoadUFI = int;

struct neighbor
{
    PointUFI next_point_ufi = 0;
    RoadUFI road_ufi = 0;
    RoadLength road_length = 0.0;
};
using neighbor_map = std::map<PointUFI, std::vector<neighbor>>;

struct next_step_final
{
    int next_point_ufi = 0;
    double road_length = 0.0;
    d_line_simple l;
};

class SearchGraph {

private:

    #define START_POINT_UFI 0
    #define GOAL_POINT_UFI 1

    std::map<PointUFI, g_point> points_coords;
    std::map<RoadUFI, d_line> roads_info_map;
    neighbor_map neighbors_map;

    void get_points();
    void get_neighbors();
    void get_roads_info(std::vector<int> roadufis);
    
    double heuristic(PointUFI current, PointUFI goal);

    std::pair<next_step_map, PointMap> gen_extra_info(nearest_road_info start_road_info, nearest_road_info goal_road_info);

    std::pair<std::vector<d_line_simple>, RoadLength> astar(PointUFI start, PointUFI goal, next_step_map special_neighbors, PointMap skip_neighbors);

public:

    QuadTree quadtree;

    SearchGraph();

    nearest_road_info find_nearest_road(double lon, double lat);

    std::pair<std::vector<d_line_simple>, RoadLength> search_path(double lon1, double lat1, double lon2, double lat2);

    std::vector<road_info> get_path_info(const std::vector<d_line_simple> &path);

    void build();
};

void test_searchgraph();