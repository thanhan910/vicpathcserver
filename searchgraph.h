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

enum enum_point_type {
    UNKNOWN = 0,
    ROADPOINT = 1,
    STOP = 2
};

struct dt_point_key {
    enum_point_type point_type = UNKNOWN;
    int point_id = 0;
};
struct next_dt_point_edge {
    dt_point_key next_point_key;
    int road_ufi = -1;
    double distance_meter = 0.0;
    g_line line = {};
};
struct pt_stop_time {
    int stop_id;
    int gtfs_mode_id;
    std::string trip_id;
    std::string arrival_time;
    std::string departure_time;
    int stop_sequence;
    int next_stop_sequence;
};

struct pt_stop_time_data {
    std::string arrival_time;
    std::string departure_time;
};

using t_reg_dt_point_single = std::unordered_map<int, g_point>;
using t_reg_dt_point_all = std::unordered_map<enum_point_type, t_reg_dt_point_single>;

using t_reg_next_dt_point_single = std::unordered_map<int, std::vector<next_dt_point_edge>>;
using t_reg_next_dt_point_all = std::unordered_map<enum_point_type, t_reg_next_dt_point_single>;

using t_reg_pt_stop_stop_times = std::unordered_map<int, std::vector<pt_stop_time>>;
using t_reg_pt_stop_times = std::unordered_map<std::string, std::map<int, pt_stop_time>>;

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

    t_reg_dt_point_all reg_points;
    t_reg_next_dt_point_all reg_next_points;
    t_reg_pt_stop_stop_times reg_stop_times;
    t_reg_pt_stop_times reg_next_stop_times;

    void build_reg_road_points();
    void build_reg_road_next_points();
    void build_reg_stop_points();
    void build_reg_stop_next_points();
    void build_data_structures();
    
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