#include "geo.h"

#include <cmath>
#include <sstream>
#include <algorithm>

namespace g_geometry {
    g_point g_segment::midpoint() const
    {
        return g_point{(p1.x + p2.x) / 2, (p1.y + p2.y) / 2};
    }

    bool g_aligned_segment::containsPoint(const g_point &p) const
    {
        if (is_vertical)
        {
            return p.x == x1 && (p.y >= y1 && p.y <= y2 || p.y >= y2 && p.y <= y1);
        }
        else
        {
            return p.y == y1 && (p.x >= x1 && p.x <= x2 || p.x >= x2 && p.x <= x1);
        }
    }

    bool g_boundary::containsPoint(const g_point &p) const
    {
        return p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max;
    }

    bool intersects(const g_segment &s, const g_aligned_segment &as)
    {
        if (as.is_vertical)
        {
            bool crossLine = (s.p1.x < as.x1 && s.p2.x > as.x1) || (s.p1.x > as.x1 && s.p2.x < as.x1);
            if (!crossLine)
            {
                if (s.p1.x == as.x1)
                    return as.containsPoint(s.p1);
                if (s.p2.x == as.x1)
                    return as.containsPoint(s.p2);
                else
                    return false;
            }
        }
        else
        {
            bool crossLine = (s.p1.y < as.y1 && s.p2.y > as.y1) || (s.p1.y > as.y1 && s.p2.y < as.y1);
            if (!crossLine)
            {
                if (s.p1.y == as.y1)
                    return as.containsPoint(s.p1);
                else if (s.p2.y == as.y1)
                    return as.containsPoint(s.p2);
                else
                    return false;
            }
        }

        double delta_y = s.p2.y - s.p1.y;
        double delta_x = s.p2.x - s.p1.x;
        if (as.is_vertical)
        {
            double line_intersect_y = s.p1.y + delta_y * (as.x1 - s.p1.x) / delta_x;
            return (line_intersect_y >= as.y1 && line_intersect_y <= as.y2) || (line_intersect_y <= as.y1 && line_intersect_y >= as.y2);
        }
        else
        {
            double line_intersect_x = s.p1.x + delta_x * (as.y1 - s.p1.y) / delta_y;
            return (line_intersect_x >= as.x1 && line_intersect_x <= as.x2) || (line_intersect_x <= as.x1 && line_intersect_x >= as.x2);
        }
    }

    bool intersects(const g_segment &s, const g_boundary &b)
    {
        if (b.containsPoint(s.p1) || b.containsPoint(s.p2))
        {
            return true;
        }
        g_aligned_segment as1{b.x_min, b.y_min, b.x_min, b.y_max, true};
        g_aligned_segment as2{b.x_min, b.y_min, b.x_max, b.y_min, false};
        g_aligned_segment as3{b.x_max, b.y_min, b.x_max, b.y_max, true};
        g_aligned_segment as4{b.x_min, b.y_max, b.x_max, b.y_max, false};
        return intersects(s, as1) || intersects(s, as2) || intersects(s, as3) || intersects(s, as4);
    }

    double distance(const g_point &p, const double d, const bool is_vertical, std::string strategy)
    {
        if (is_vertical)
        {
            return std::abs(p.x - d);
        }
        else
        {
            return std::abs(p.y - d);
        }
    }

    double distance(const g_point &p1, const g_point &p2, std::string strategy)
    {
        if (strategy == "euclidean")
        {
            return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
        }
        else if (strategy == "manhattan")
        {
            return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
        }
        else { // Default to euclidean
            return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
        }
    }

    double distance(const g_point &p, const g_boundary &b, std::string strategy)
    {
        bool ge_x_min = (p.x >= b.x_min);
        bool le_x_max = (p.x <= b.x_max);
        bool ge_y_min = (p.y >= b.y_min);
        bool le_y_max = (p.y <= b.y_max);

        if (ge_x_min && le_x_max && ge_y_min && le_y_max)
        {
            return 0.0;
        }
        if (ge_x_min && le_x_max && !ge_y_min)
        {
            return distance(p, b.y_min, true, strategy);
        }
        if (ge_x_min && le_x_max && !le_y_max)
        {
            return distance(p, b.y_max, true, strategy);
        }
        if (ge_y_min && le_y_max && !ge_x_min)
        {
            return distance(p, b.x_min, false, strategy);
        }
        if (ge_y_min && le_y_max && !le_x_max)
        {
            return distance(p, b.x_max, false, strategy);
        }
        if (!ge_x_min && !ge_y_min)
        {
            return distance(p, g_point{b.x_min, b.y_min}, strategy);
        }
        if (!ge_x_min && !le_y_max)
        {
            return distance(p, g_point{b.x_min, b.y_max}, strategy);
        }
        if (!le_x_max && !ge_y_min)
        {
            return distance(p, g_point{b.x_max, b.y_min}, strategy);
        }
        if (!le_x_max && !le_y_max)
        {
            return distance(p, g_point{b.x_max, b.y_max}, strategy);
        }
        return -1.0;
    }
    
    g_point projectionPoint(const g_point &p, const g_segment &s, std::string strategy)
    {
        // Project p onto the line segment, clamp to endpoints
        double delta_p_p1_x = p.x - s.p1.x;
        double delta_p_p1_y = p.y - s.p1.y;
        double delta_s_x = s.p2.x - s.p1.x;
        double delta_s_y = s.p2.y - s.p1.y;

        double dot = delta_p_p1_x * delta_s_x + delta_p_p1_y * delta_s_y;
        double len_sq = delta_s_x * delta_s_x + delta_s_y * delta_s_y;
        double param = (len_sq != 0) ? dot / len_sq : -1;

        double xx, yy;
        if (param < 0)
        {
            xx = s.p1.x;
            yy = s.p1.y;
        }
        else if (param > 1)
        {
            xx = s.p2.x;
            yy = s.p2.y;
        }
        else
        {
            xx = s.p1.x + param * delta_s_x;
            yy = s.p1.y + param * delta_s_y;
        }
        return {xx, yy};
    }

    g_point closestPoint(const g_point &p, const g_segment &s, std::string strategy)
    {
        g_point projection = projectionPoint(p, s, strategy);
        
        // Check if the closest point is on the line segment
        double x_min = std::min(s.p1.x, s.p2.x);
        double x_max = std::max(s.p1.x, s.p2.x);
        double y_min = std::min(s.p1.y, s.p2.y);
        double y_max = std::max(s.p1.y, s.p2.y);
        bool ge_x_min = projection.x >= x_min;
        bool le_x_max = projection.x <= x_min;
        bool ge_y_min = projection.y >= y_min;
        bool le_y_max = projection.y <= y_min;
        if (ge_x_min && le_x_max && ge_y_min && le_y_max)
        {
            return projection;
        }
        else
        {
            if (s.p1.x == s.p2.x)
            {
                if (s.p1.y < s.p2.y)
                {
                    return (projection.y < s.p1.y) ? s.p1 : s.p2;
                }
                else
                {
                    return (projection.y < s.p2.y) ? s.p2 : s.p1;
                }
            }
            else if (s.p1.x < s.p2.x)
            {
                return (projection.x < s.p1.x) ? s.p1 : s.p2;
            }
            else
            {
                return (projection.x < s.p2.x) ? s.p2 : s.p1;
            }
        }
    }

    double projectionDistance(const g_point &p, const g_segment &s, std::string strategy)
    {
        g_point projection = projectionPoint(p, s, strategy);
        return distance(p, projection, strategy);
    }

    double closestDistance(const g_point &p, const g_segment &s, std::string strategy)
    {
        g_point closest = closestPoint(p, s, strategy);
        return distance(p, closest, strategy);
    }

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
                lineString.points.emplace_back(x, y);
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
            points.emplace_back(x, y);
        }
    }

    double g_line::length() const
    {
        double length = 0.0;
        for (int i = 0; i < points.size() - 1; i++)
        {
            length += distance(points[i], points[i + 1], STRATEGY_GEODESIC);
        }
        return length;
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
        if (road_direction == DIRECTION_FORWARD || road_direction == DIRECTION_BOTH)
        {
            g_line newline0;
            for (int j = 0; j <= points_on_line[0].pos; j++)
            {
                newline0.points.push_back(line.points[j]);
            }
            newline0.points.push_back(points_on_line[0].point);
            next_steps[from_ufi].push_back({points_on_line[0].pointufi, newline0});
            
            for (int i = 0; i < points_on_line.size() - 1; i++)
            {
                g_line newline;
                newline.points.push_back(points_on_line[i].point);
                for (int j = points_on_line[i].pos + 1; j <= points_on_line[i + 1].pos; j++)
                {
                    newline.points.push_back(line.points[j]);
                }
                newline.points.push_back(points_on_line[i + 1].point);
                next_steps[points_on_line[i].pointufi].push_back({points_on_line[i + 1].pointufi, newline});
            }

            g_line newline1;
            for (int j = points_on_line[points_on_line.size() - 1].pos; j < line.points.size(); j++)
            {
                newline1.points.push_back(line.points[j]);
            }
            next_steps[points_on_line[points_on_line.size() - 1].pointufi].push_back({to_ufi, newline1});
        }
        if (road_direction == DIRECTION_REVERSE || road_direction == DIRECTION_BOTH)
        {
            g_line newline0;
            for (int j = 0; j <= points_on_line[0].pos; j++)
            {
                newline0.points.push_back(line.points[j]);
            }
            newline0.points.push_back(points_on_line[0].point);
            next_steps[to_ufi].push_back({points_on_line[0].pointufi, newline0});
            
            for (int i = 0; i < points_on_line.size() - 1; i++)
            {
                g_line newline;
                newline.points.push_back(points_on_line[i].point);
                for (int j = points_on_line[i].pos + 1; j <= points_on_line[i + 1].pos; j++)
                {
                    newline.points.push_back(line.points[j]);
                }
                newline.points.push_back(points_on_line[i + 1].point);
                next_steps[points_on_line[i].pointufi].push_back({points_on_line[i + 1].pointufi, newline});
            }

            g_line newline1;
            for (int j = points_on_line[points_on_line.size() - 1].pos; j < line.points.size(); j++)
            {
                newline1.points.push_back(line.points[j]);
            }
            next_steps[points_on_line[points_on_line.size() - 1].pointufi].push_back({from_ufi, newline1});
        }
        return next_steps;
    }
}