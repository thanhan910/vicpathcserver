#include "searchgraph.h"

#include "geo/line.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "db.h"
#include "geo/geo.h"
#include "quadtree.h"

namespace bg = boost::geometry;

#define TEST_STOPS
#define TEST_SINGLE_POINT

void SearchGraph::get_points()
{
    // Initialize MongoDB instance and client

    mongocxx::collection points_collection = db["points"];

    // Vector to store documents from MongoDB
    std::vector<bsoncxx::document::value> points_coords_mongo_get;

    // Get the estimated document count
    auto collection_count = points_collection.estimated_document_count();

    // Iterate over the collection and append documents to the vector
    for (auto &&doc : points_collection.find({}))
    {
        points_coords_mongo_get.push_back(bsoncxx::document::value(doc));
    }

    // Convert BSON documents to the desired map structure
    for (const auto &doc : points_coords_mongo_get)
    {
        auto view = doc.view();
        PointUFI id = view["_id"].get_int32().value;
        bsoncxx::array::view coordinates_tuple = view["coords"].get_array().value;
        g_point coordinates = {coordinates_tuple[0].get_double(), coordinates_tuple[1].get_double()};

        points_coords[id] = coordinates;
    }

    // // Output the neighbors map for verification
    // for (const auto& [id, neighbor_list] : neighbors) {
    //     std::cout << "ID: " << id << "\n";
    //     for (const auto& [neighbor_id, neighbor_name, neighbor_distance] : neighbor_list) {
    //         std::cout << "  Neighbor ID: " << neighbor_id << ", Name: " << neighbor_name << ", Distance: " << neighbor_distance << "\n";
    //     }
    // }

    // return neighbors;
}

void SearchGraph::get_neighbors()
{
    // Initialize MongoDB instance and client

    mongocxx::collection neighbours_collection = db["points_neighbours"];

    // Vector to store documents from MongoDB
    std::vector<bsoncxx::document::value> neighbors_mongo;

    // // Get the estimated document count
    // auto collection_count = neighbours_collection.estimated_document_count();

    // Iterate over the collection and append documents to the vector
    for (auto &&doc : neighbours_collection.find({}))
    {
        neighbors_mongo.push_back(bsoncxx::document::value(doc));
    }

    // Convert BSON documents to the desired map structure
    for (const auto &doc : neighbors_mongo)
    {
        auto view = doc.view();
        int id = view["_id"].get_int32().value;
        std::vector<neighbor> neighbor_list;

        for (const auto &neighbor : view["neighbours"].get_array().value)
        {
            auto neighbor_view = neighbor.get_array().value;
            PointUFI neighbor_id = neighbor_view[0].get_int32().value;
            RoadUFI road_id = neighbor_view[1].get_int32().value;
            RoadLength neighbor_distance = neighbor_view[2].get_double().value;
            neighbor_list.push_back({neighbor_id, road_id, neighbor_distance});
        }

        neighbors_map[id] = neighbor_list;
    }
}

// // Function to calculate geodesic distance between two points (lon1, lat1) and (lon2, lat2)
// double SearchGraph::geodesic_distance(double lat1, double lon1, double lat2, double lon2)
// {
//     bg_point p1(lat1, lon1), p2(lat2, lon2);
//     return bg::distance(p1, p2) * 1000.0; // Convert to meters
// }

double SearchGraph::heuristic(PointUFI current, PointUFI goal)
{
    auto [lon1, lat1] = points_coords[current];
    auto [lon2, lat2] = points_coords[goal];
    bg_point p1(lat1, lon1), p2(lat2, lon2);
    return bg::distance(p1, p2) * 1000.0; // Convert to meters
}

void SearchGraph::get_roads_info(std::vector<int> roadufi_list) {
    std::vector<int> roadufis = {};
    for (int i = 0; i < roadufi_list.size(); i++)
    {
        if (roads_info_map.find(roadufi_list[i]) == roads_info_map.end())
        {
            roadufis.push_back(roadufi_list[i]);
        }
    }
    if (roadufis.size() == 0)
    {
        return;
    }
    
    std::string sql;
    if (roadufis.size() == 1)
    {
        sql = "SELECT ufi, ezi_road_name_label, direction_code, from_ufi, to_ufi, ST_AsText(geom) FROM vmtrans.tr_road_all WHERE ufi = " + std::to_string(roadufis[0]) + ";";
    }
    else {
        sql = "SELECT ufi, ezi_road_name_label, direction_code, from_ufi, to_ufi, ST_AsText(geom) FROM vmtrans.tr_road_all WHERE ufi IN (";
        for (int i = 0; i < roadufis.size(); i++)
        {
            sql += std::to_string(roadufis[i]);
            if (i != roadufis.size() - 1)
            {
                sql += ",";
            }
        }
        sql += ");";
    }
    pqxx::work txn(conn);
    pqxx::result result = txn.exec(sql);
    std::vector<d_line> lines;
    for (auto row : result)
    {
        RoadUFI road_ufi = row[0].as<int>();
        std::string ezi_road_name_label = row[1].as<std::string>();
        RoadDirection direction_code = row[2].as<RoadDirection>();
        PointUFI from_ufi = row[3].as<PointUFI>();
        PointUFI to_ufi = row[4].as<PointUFI>();
        std::string multiline_wkt = row[5].as<std::string>();
        roads_info_map[road_ufi] = {road_ufi, ezi_road_name_label, direction_code, from_ufi, to_ufi, g_line{multiline_wkt}};
    }
}

std::pair<next_step_map, PointMap> SearchGraph::gen_extra_info(nearest_road_info start, nearest_road_info goal)
{
    std::vector<d_point_on_line> points_on_line;
    points_on_line.push_back({START_POINT_UFI, start.closest_point, start.road_ufi, start.nearest_segment.pos});
    points_on_line.push_back({GOAL_POINT_UFI, goal.closest_point, goal.road_ufi, goal.nearest_segment.pos});
    points_coords[START_POINT_UFI] = start.closest_point;
    points_coords[GOAL_POINT_UFI] = goal.closest_point;
    
    get_roads_info({start.road_ufi, goal.road_ufi});

    std::vector<d_line> lines;
    lines.push_back(roads_info_map[start.road_ufi]);
    lines.push_back(roads_info_map[goal.road_ufi]);

    next_step_map special_neighbors = generate_next_steps(lines, points_on_line);

    for (const auto &p : special_neighbors)
    {
        std::cout << "Point: " << p.first << std::endl;
        for (const auto &n : p.second)
        {
            std::cout << "  Neighbor: " << n.next_point_ufi << ", Road: " << n.roadufi << ", Name: " << roads_info_map[n.roadufi].ezi_road_name_label << ", Direction: " << roads_info_map[n.roadufi].direction_code << ", Length: " << n.line.length() << std::endl;
        }
    }

    PointMap skip_neighbors = {
        {start.from_ufi, start.to_ufi},
        {start.to_ufi, start.from_ufi},
        {goal.from_ufi, goal.to_ufi},
        {goal.to_ufi, goal.from_ufi},
    };

    return {special_neighbors, skip_neighbors};
}

// using FrontierItem = std::tuple<double, double, PointUFI, std::vector<d_line_simple>>;

struct FrontierItem {
    double f;
    double g;
    PointUFI pointufi;
    std::vector<d_line_simple> path;
    bool operator>(const FrontierItem &other) const
    {
        return f > other.f;
    }
};

std::pair<std::vector<d_line_simple>, RoadLength> SearchGraph::astar(PointUFI start, PointUFI goal, next_step_map special_neighbors, PointMap skip_neighbors)
{
    // std::vector<FrontierItem> frontier;
    std::priority_queue<FrontierItem, std::vector<FrontierItem>, std::greater<FrontierItem>> frontier;
    std::set<PointUFI> visited;
    std::vector<d_line_simple> path = {};

    // frontier.emplace_back(START_POINT_UFI, 0, start, path);
    frontier.push({0, 0, start, path});

    while (!frontier.empty())
    {
        auto [_, cost, current_pointufi, current_path] = frontier.top();
        frontier.pop();
        // std::cout << "Current path size: " << current_path.size() << std::endl;

        if (current_pointufi == goal)
        {
            return std::make_pair(current_path, cost);
        }

        if (current_pointufi == GOAL_POINT_UFI)
        {
            return std::make_pair(path, cost);
        }

        if (visited.find(current_pointufi) != visited.end())
        {
            continue;
        }

        visited.insert(current_pointufi);

        std::vector<next_step_final> neighbor_points = {};
        if (neighbors_map.find(current_pointufi) != neighbors_map.end())
        {
            for (const auto &p : neighbors_map[current_pointufi])
            {
                neighbor_points.push_back(
                    {p.next_point_ufi, p.road_length, {p.road_ufi, {}}}
                );
            }
        }
        if (special_neighbors.find(current_pointufi) != special_neighbors.end())
        {
            for (const auto &p : special_neighbors[current_pointufi])
            {
                neighbor_points.push_back(
                    {p.next_point_ufi, p.line.length(), {p.roadufi, p.line}}
                );
            }
        }

        for (const next_step_final &next : neighbor_points)
        {
            if (skip_neighbors.find(current_pointufi) != skip_neighbors.end() && skip_neighbors[current_pointufi] == next.next_point_ufi)
            {
                continue;
            }

            if (visited.find(next.next_point_ufi) == visited.end())
            {
                double heuristic_cost = heuristic(next.next_point_ufi, goal);
                std::vector<d_line_simple> new_path = current_path;
                new_path.push_back(next.l);
                frontier.push({cost + next.road_length + heuristic_cost, cost + next.road_length, next.next_point_ufi, new_path});
            }
        }
    }

    return std::make_pair(path, 0.0);
}

SearchGraph::SearchGraph() {}

nearest_road_info SearchGraph::find_nearest_road(double lon, double lat)
{
    g_point p = {lon, lat};

    auto [nearestSegment, closestPoint, minDistance, quads] = quadtree.find_nearest_segment(p);

    RoadUFI roadufi = nearestSegment.roadufi;

    
    get_roads_info({roadufi});

    return {
        roadufi, 
        roads_info_map[roadufi].direction_code,
        roads_info_map[roadufi].from_ufi,
        roads_info_map[roadufi].to_ufi,
        closestPoint, 
        nearestSegment
    };
}

std::pair<std::vector<d_line_simple>, RoadLength> SearchGraph::search_path(double lon1, double lat1, double lon2, double lat2)
{

    std::cout << "Finding nearest road for start point" << std::endl;
    nearest_road_info start = find_nearest_road(lon1, lat1);
    nearest_road_info goal = find_nearest_road(lon2, lat2);
    std::cout << "Start road: " << start.road_ufi << ", Goal road: " << goal.road_ufi << std::endl;

    auto [special_neighbors, skip_neighbors] = gen_extra_info(start, goal);

    return astar(START_POINT_UFI, GOAL_POINT_UFI, special_neighbors, skip_neighbors); // Assuming start is 0 and goal is 1
}


std::vector<std::tuple<RoadUFI, std::string, RoadDirection, double, g_line>> SearchGraph::get_path_info(const std::vector<d_line_simple> &path)
{
    std::vector<int> roadufis = {};
    for (const auto &line : path)
    {
        roadufis.push_back(line.roadufi);
    }

    get_roads_info(roadufis);

    std::vector<std::tuple<RoadUFI, std::string, RoadDirection, double, g_line>> roads_info;
    for (const auto &line : path)
    {
        if (line.line.points.size() == 0)
        {
            roads_info.push_back(
                std::make_tuple(
                    line.roadufi,
                    roads_info_map[line.roadufi].ezi_road_name_label,
                    roads_info_map[line.roadufi].direction_code,
                    roads_info_map[line.roadufi].line.length(),
                    roads_info_map[line.roadufi].line
                )
            );
        }
        else {
            roads_info.push_back(
                std::make_tuple(
                    line.roadufi,
                    roads_info_map[line.roadufi].ezi_road_name_label,
                    roads_info_map[line.roadufi].direction_code,
                    line.line.length(),
                    line.line
                )
            );
        }
    }

    return roads_info;
}


void SearchGraph::build()
{
    // Populate the points_coords and neighbors collections from MongoDB (similar to Python code)
    // Example usage:
    std::chrono::steady_clock::time_point begin, end;
    begin = std::chrono::steady_clock::now();
    get_neighbors();
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    // Calculate size on RAM
    std::cout << "Size of neighbors_map: " << sizeof(neighbors_map) << " bytes" << std::endl;
    begin = std::chrono::steady_clock::now();
    get_points();
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
    // Calculate size on RAM
    std::cout << "Size of points_coords: " << sizeof(points_coords) << " bytes" << std::endl;

    quadtree.gen_quadtree();
}

void test_searchgraph()
{

    SearchGraph searchgraph = SearchGraph();

    searchgraph.build();

    std::chrono::steady_clock::time_point begin, end;

    begin = std::chrono::steady_clock::now();
    double lon1 = 144.9631, lat1 = -37.8136; // Melbourne
    double lon2 = 145.0458, lat2 = -37.8768; // Nearby suburb
    auto [path, cost] = searchgraph.search_path(lon1, lat1, lon2, lat2);
    std::cout << "Path: ";
    for (const auto &p : path)
        std::cout << p.roadufi << " ";
    std::cout << "\nTotal Cost: " << cost << std::endl;
    end = std::chrono::steady_clock::now();
    std::cout << "Find shortest path = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    // Sample 10 points and find the nearest roads of each
    std::vector<double> sample_lons = {
        144.9631, 145.0458, 144.9731, 144.9831, 144.9931, 145.0031, 145.0131, 145.0231, 145.0331, 145.0431};
    std::vector<double> sample_lats = {
        -37.8136, -37.8768, -37.8236, -37.8336, -37.8436, -37.8536, -37.8636, -37.8736, -37.8836, -37.8936};
    for (int i = 0; i < 10; i++)
    {
        double lon = sample_lons[i], lat = sample_lats[i];
        auto nearest = searchgraph.find_nearest_road(lon, lat);
        std::cout << "Point " << i << " (" << lon << ", " << lat << "): ";
        std::cout << nearest.road_ufi << " " << std::endl;
    }
    // g++ main.cpp -o test $(pkg-config --cflags --libs libpqxx libpq libmongocxx-static) -I/vcpkg/installed/x64-linux/include -L/vcpkg/installed/x64-linux/lib
};