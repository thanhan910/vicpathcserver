#include "../db.h"
#include "../geo/geo.h"

#include <map>
#include <vector>

using pt_stop_id = int;
using pt_trip_id = std::string;

struct pt_stop_time {
    pt_trip_id trip_id;
    std::string arrival_time;
    std::string departure_time;
    pt_stop_id stop_id;
    int stop_sequence;
    int next_stop_sequence;
};

struct pt_stop_time_key {
    std::string trip_id;
    int stop_sequence;
};

struct pt_stop {
    std::string stop_name;
    double stop_lat;
    double stop_lon;
    std::vector<pt_stop_time_key> stop_times;
};

using map_pt_stop_times = std::map<pt_trip_id, std::map<int, pt_stop_time>>;
using map_pt_stops = std::map<pt_stop_id, pt_stop>;

using rd_point_ufi = int;

struct rd_point {
    g_point point;
};

using map_rd_points = std::map<rd_point_ufi, g_point>;

struct pt_stop_segment {
    int road_ufi;
    pt_stop_id to_stop_id;
    double distance;
    double duration;
};
