#pragma once

#include "geo.h"
#include <map>

struct g_point_on_line
{
    g_point point;
    int pos;
};

struct g_line {
    std::vector<g_point> points;
    g_line() = default;
    g_line(const std::vector<g_point> &points) : points{points} {};
    g_line(const std::string &wkt);
    double length() const;
    g_line reverse() const;
    std::vector<g_line> split_line(std::vector<g_point_on_line> &points_on_line);
};

struct d_point_on_line
{
    int pointufi;
    g_point point;
    int roadufi;
    int pos;
};

struct next_step
{
    int next_point_ufi = 0;
    int roadufi;
    g_line line;
};

using next_step_map = std::map<int, std::vector<next_step>>;

#define DIRECTION_FORWARD "F"
#define DIRECTION_REVERSE "R"
#define DIRECTION_BOTH "B"

struct d_line
{
    int roadufi;
    std::string ezi_road_name_label;
    std::string direction_code;
    int from_ufi;
    int to_ufi;
    double road_length_meters;
    g_line line;
    next_step_map generate_next_steps(std::vector<d_point_on_line> &points_on_line);
};

next_step_map generate_next_steps(std::vector<d_line> &lines, std::vector<d_point_on_line> &points_on_line);