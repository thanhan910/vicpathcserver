#include "geo.h"

#include <cmath>

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
            return (line_intersect_y >= as.y1 && line_intersect_y <= as.y2);
        }
        else
        {
            double line_intersect_x = s.p1.x + delta_x * (as.y1 - s.p1.y) / delta_y;
            return (line_intersect_x >= as.x1 && line_intersect_x <= as.x2);
        }
    }

    bool intersects(const g_segment &s, const g_boundary &b)
    {
        return false;
    }

    double distance(const double d1, const double d2, const bool is_vertical)
    {
        return std::abs(d1 - d2);
    }

    double distance(const g_point &p1, const g_point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

    double distance(const g_point &p, const g_boundary &b)
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
            return distance(p.y, b.y_min, true);
        }
        if (ge_x_min && le_x_max && !le_y_max)
        {
            return distance(p.y, b.y_max, true);
        }
        if (ge_y_min && le_y_max && !ge_x_min)
        {
            return distance(p.x, b.x_min, false);
        }
        if (ge_y_min && le_y_max && !le_x_max)
        {
            return distance(p.x, b.x_max, false);
        }
        if (!ge_x_min && !ge_y_min)
        {
            return distance(p, g_point{b.x_min, b.y_min});
        }
        if (!ge_x_min && !le_y_max)
        {
            return distance(p, g_point{b.x_min, b.y_max});
        }
        if (!le_x_max && !ge_y_min)
        {
            return distance(p, g_point{b.x_max, b.y_min});
        }
        if (!le_x_max && !le_y_max)
        {
            return distance(p, g_point{b.x_max, b.y_max});
        }
        return -1.0;
    }
    
    g_point projectionPoint(const g_point &p, const g_segment &s)
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

    g_point closestPoint(const g_point &p, const g_segment &s)
    {
        g_point projection = projectionPoint(p, s);
        
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
    double projectionDistance(const g_point &p, const g_segment &s)
    {
        g_point projection = projectionPoint(p, s);
        return distance(p, projection);
    }
    double closestDistance(const g_point &p, const g_segment &s)
    {
        g_point closest = closestPoint(p, s);
        return distance(p, closest);
    }
}