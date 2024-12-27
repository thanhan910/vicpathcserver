#pragma once

#include <vector>
#include <string>
#include <map>

struct g_point
{
    double x, y;
};

struct g_segment
{
    g_point p1, p2;
    g_segment(const g_point &p1, const g_point &p2) : p1{p1}, p2{p2} {};
    g_point midpoint() const;
};

struct g_aligned_segment
{
    double x1, y1, x2, y2;
    bool is_vertical;
    g_aligned_segment(const g_point &p1, const g_point &p2) : x1{p1.x}, y1{p1.y}, x2{p2.x}, y2{p2.y}, is_vertical{p1.x == p2.x} {};
    g_aligned_segment(const double x1, const double y1, const double x2, const double y2, const bool is_vertical) : x1{x1}, y1{y1}, x2{x2}, y2{y2}, is_vertical{is_vertical} {};
    bool containsPoint(const g_point &p) const;
};

struct g_boundary
{
    double x_min, y_min, x_max, y_max;
    bool containsPoint(const g_point &p) const;
    double width() const { return x_max - x_min; };
    double height() const { return y_max - y_min; };
    double x_mid() const { return (x_min + x_max) / 2; };
    double y_mid() const { return (y_min + y_max) / 2; };
};

struct d_segment : g_segment
{
    int roadufi;
    int pos;
    d_segment(const g_point &p1, const g_point &p2, const int& roadufi, const int& pos) : g_segment(p1, p2), roadufi{roadufi}, pos{pos} {};
};

#define STRATEGY_EUCLIDEAN "euclidean"
#define STRATEGY_SPHERICAL "spherical"
#define STRATEGY_GEODESIC "geodesic"
#define STRATEGY_MANHATTAN "manhattan"

namespace geo {
    bool intersects(const g_segment &s, const g_aligned_segment &as);
    bool intersects(const g_segment &s, const g_boundary &b);
    double distance(const g_point &p, const double d, const bool is_vertical, std::string strategy);
    double distance(const g_point &p1, const g_point &p2, std::string strategy);
    double distance(const g_point &p, const g_boundary &b, std::string strategy);

    g_point projectionPoint(const g_point &p, const g_segment &s, std::string strategy);
    g_point closestPoint(const g_point &p, const g_segment &s, std::string strategy);
    double projectionDistance(const g_point &p, const g_segment &s, std::string strategy);
    double closestDistance(const g_point &p, const g_segment &s, std::string strategy);
}

struct d_point_on_line
{
    int pointufi;
    g_point point;
    int pos;
};

struct next_step
{
    int next_point_ufi = 0;
    g_line line;
};
using next_step_list = std::vector<next_step>;
using next_step_map = std::map<int, next_step_list>;

struct g_line {
    std::vector<g_point> points;
    g_line() = default;
    g_line(const std::vector<g_point> &points) : points{points} {};
    g_line(const std::string &wkt);
    double length() const;
};

#define DIRECTION_FORWARD "F"
#define DIRECTION_REVERSE "R"
#define DIRECTION_BOTH "B"

struct d_line
{
    int roadufi;
    int from_ufi;
    int to_ufi;
    std::string road_direction;
    g_line line;
    next_step_map generate_next_steps(std::vector<d_point_on_line> &points_on_line);
};