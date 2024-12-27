#include "line.h"

#include <sstream>
#include <algorithm>

std::vector<g_line> parseWKTtoMultiLineString(const std::string &wkt)
{
    std::vector<g_line> multiLineString;

    // Remove the "MULTILINESTRING(" and ")" from the WKT string
    std::string stripped_wkt = wkt.substr(16, wkt.size() - 17);
    std::istringstream lines_stream(stripped_wkt);
    std::string line;

    while (std::getline(lines_stream, line, ')'))
    {
        // Skip any commas at the beginning of each line string segment
        if (!line.empty() && line[0] == ',')
            line = line.substr(2);

        g_line lineString;
        std::istringstream points_stream(line.substr(1)); // Remove the leading '('
        std::vector<std::string> lineStr;
        // lineStr.push_back(line);
        // Split the line by comma
        line = line.substr(1);
        for (int i = 0; i < line.size(); i++)
        {
            if (line[i] == ',')
            {
                lineStr.push_back(line.substr(0, i));
                line = line.substr(i + 1);
                i = 0;
            }
            else if (line[i] == ' ')
            {
                lineStr.push_back(line.substr(0, i));
                line = line.substr(i + 1);
                i = 0;
            }
        }
        lineStr.push_back(line);

        std::string point;
        while (std::getline(points_stream, point, ','))
        {
            // Split the point into two strings by space
            std::istringstream point_stream(point);
#ifdef USE_STRING
            std::string x_str, y_str;
            std::getline(point_stream, x_str, ' ');
            std::getline(point_stream, y_str, ' ');
            lineString.emplace_back(x_str, y_str);
#else
            double x, y;
            point_stream >> x >> y;
            lineString.points.push_back({x, y});
#endif
        }

        multiLineString.push_back(lineString);
    }
    return multiLineString;
}

g_line::g_line(const std::string &wkt)
{
    // Remove the "MULTILINESTRING((" and "))" from the WKT string
    std::string stripped_wkt = wkt.substr(17, wkt.size() - 19);
    std::istringstream points_stream(stripped_wkt);
    points.clear();
    std::string point;
    while (std::getline(points_stream, point, ','))
    {
        // Split the point into two strings by space
        std::istringstream point_stream(point);
        double x, y;
        point_stream >> x >> y;
        points.push_back({x, y});
    }
}

double g_line::length() const
{
    double length = 0.0;
    for (int i = 0; i < points.size() - 1; i++)
    {
        length += geo::distance(points[i], points[i + 1], STRATEGY_GEODESIC);
    }
    return length;
}

g_line g_line::reverse() const
{
    g_line line;
    for (int i = points.size() - 1; i >= 0; i--)
    {
        line.points.push_back(points[i]);
    }
    return line;
}

next_step_map d_line::generate_next_steps(std::vector<d_point_on_line> &points_on_line)
{
    next_step_map next_steps;
    std::sort(points_on_line.begin(), points_on_line.end(), [this](const d_point_on_line &a, const d_point_on_line &b) {
        if (a.pos == b.pos)
        {
            g_point& p1 = line.points[a.pos];
            g_point& p2 = line.points[a.pos + 1];
            if (p1.x == p2.x)
            {
                return (p1.y < p2.y) ? a.point.y < b.point.y : a.point.y > b.point.y;
            }
            else
            {
                return (p1.x < p2.x) ? a.point.x < b.point.x : a.point.x > b.point.x;
            }
        }
        else
        {
            return a.pos < b.pos;
        }
    });
    if (direction_code == DIRECTION_FORWARD || direction_code == DIRECTION_BOTH)
    {
        g_line newline0;
        for (int j = 0; j <= points_on_line[0].pos; j++)
        {
            newline0.points.push_back(line.points[j]);
        }
        newline0.points.push_back(points_on_line[0].point);
        next_steps[from_ufi].push_back({points_on_line[0].pointufi, roadufi, newline0});
        
        for (int i = 0; i < points_on_line.size() - 1; i++)
        {
            g_line newline;
            newline.points.push_back(points_on_line[i].point);
            for (int j = points_on_line[i].pos + 1; j <= points_on_line[i + 1].pos; j++)
            {
                newline.points.push_back(line.points[j]);
            }
            newline.points.push_back(points_on_line[i + 1].point);
            next_steps[points_on_line[i].pointufi].push_back({points_on_line[i + 1].pointufi, roadufi, newline});
        }

        g_line newline1;
        for (int j = points_on_line[points_on_line.size() - 1].pos; j < line.points.size(); j++)
        {
            newline1.points.push_back(line.points[j]);
        }
        next_steps[points_on_line[points_on_line.size() - 1].pointufi].push_back({to_ufi, roadufi, newline1});
    }
    if (direction_code == DIRECTION_REVERSE || direction_code == DIRECTION_BOTH)
    {
        g_line newline0;
        for (int j = 0; j <= points_on_line[0].pos; j++)
        {
            newline0.points.push_back(line.points[j]);
        }
        newline0.points.push_back(points_on_line[0].point);
        next_steps[to_ufi].push_back({points_on_line[0].pointufi, roadufi, newline0});
        
        for (int i = 0; i < points_on_line.size() - 1; i++)
        {
            g_line newline;
            newline.points.push_back(points_on_line[i].point);
            for (int j = points_on_line[i].pos + 1; j <= points_on_line[i + 1].pos; j++)
            {
                newline.points.push_back(line.points[j]);
            }
            newline.points.push_back(points_on_line[i + 1].point);
            next_steps[points_on_line[i].pointufi].push_back({points_on_line[i + 1].pointufi, roadufi, newline});
        }

        g_line newline1;
        for (int j = points_on_line[points_on_line.size() - 1].pos; j < line.points.size(); j++)
        {
            newline1.points.push_back(line.points[j]);
        }
        next_steps[points_on_line[points_on_line.size() - 1].pointufi].push_back({from_ufi, roadufi, newline1});
    }
    return next_steps;
}

next_step_map generate_next_steps(std::vector<d_line> &lines, std::vector<d_point_on_line> &points_on_line)
{
    std::map<int, std::vector<d_point_on_line>> points_lines_map;
    for (auto &point_on_line : points_on_line)
    {
        points_lines_map[point_on_line.roadufi].push_back(point_on_line);
    }
    next_step_map next_steps;
    for (auto &line : lines)
    {
        if (points_lines_map.find(line.roadufi) != points_lines_map.end())
        {
            next_step_map sub_next_steps = line.generate_next_steps(points_lines_map[line.roadufi]);
            for (auto &sub_next_step : sub_next_steps)
            {
                for (auto &next_step : sub_next_step.second)
                {
                    next_steps[sub_next_step.first].push_back(next_step);
                }
            }
        }
    }
    return next_steps;
}
