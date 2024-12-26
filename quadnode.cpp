#include "quadnode.h"
#include <limits>

QuadNode::QuadNode(g_boundary boundary, int capacity) : boundary(boundary), capacity(capacity), divided(false), segment_count(0) {}

bool QuadNode::insert(d_segment segment)
{
    // if (!(boundary.intersects_segment(segment)))
    //     return false;
    if (!gg::intersects(segment, boundary))
        return false;

    if (!divided)
    {
        // if ((segments.size() < capacity || (boundary.x_max - boundary.x_min) < 0.000001 || (boundary.y_max - boundary.y_min) < 0.000001))
        // If the number of segments is less than the capacity or the width or height of the boundary is less than 0.000001
        if ((segments.size() < capacity || boundary.width() < 0.000001 || boundary.height() < 0.000001))
        {
            segments.push_back(segment);
            segment_count++;
            return true;
        }
        else
        {
            subdivide();
        }
    }

    for (QuadNode *child : children)
    {
        if (child->insert(segment))
        {
            segment_count++;
            return true;
        }
    }
    return false;
}

void QuadNode::subdivide()
{
    // double x_mid = (boundary.x_min + boundary.x_max) / 2;
    // double y_mid = (boundary.y_min + boundary.y_max) / 2;
    // double x_mid = (bg::get<bg::min_corner, 0>(boundary) + bg::get<bg::max_corner, 0>(boundary)) / 2;
    // double y_mid = (bg::get<bg::min_corner, 1>(boundary) + bg::get<bg::max_corner, 1>(boundary)) / 2;
    // double x_min = bg::get<bg::min_corner, 0>(boundary);
    // double y_min = bg::get<bg::min_corner, 1>(boundary);
    // double x_max = bg::get<bg::max_corner, 0>(boundary);
    // double y_max = bg::get<bg::max_corner, 1>(boundary);

    double x_mid = boundary.x_mid();
    double y_mid = boundary.y_mid();
    double x_min = boundary.x_min;
    double y_min = boundary.y_min;
    double x_max = boundary.x_max;
    double y_max = boundary.y_max;

    children.push_back(new QuadNode(g_boundary{x_min, y_min, x_mid, y_mid}, capacity));
    children.push_back(new QuadNode(g_boundary{x_min, y_mid, x_mid, y_max}, capacity));
    children.push_back(new QuadNode(g_boundary{x_mid, y_min, x_max, y_mid}, capacity));
    children.push_back(new QuadNode(g_boundary{x_mid, y_mid, x_max, y_max}, capacity));

    for (const d_segment &seg : segments)
    {
        for (QuadNode *child : children)
        {
            child->insert(seg);
        }
    }

    // Clear the segments vector
    segments.clear();

    divided = true;
}

std::vector<QuadNode *> QuadNode::nearestSegment(const g_point &point, double &minDistance, d_segment &nearestSegment, g_point &nearestPoint) const
{
     if (divided)
    {
        std::vector<QuadNode *> container_quads;
        std::vector<QuadNode *> non_container_quads;
        for (QuadNode *child : children)
        {
            // double x_min = bg::get<bg::min_corner, 0>(child->boundary);
            // double y_min = bg::get<bg::min_corner, 1>(child->boundary);
            // double x_max = bg::get<bg::max_corner, 0>(child->boundary);
            // double y_max = bg::get<bg::max_corner, 1>(child->boundary);
            // double point_x = bg::get<0>(point);
            // double point_y = bg::get<1>(point);
            // bool boundary_contains_point = x_min <= point_x && point_x <= x_max && y_min <= point_y && point_y <= y_max;
            if (boundary.containsPoint(point))
            {
                container_quads.push_back(child);
            }
            else
            {
                non_container_quads.push_back(child);
            }
        }
        for (QuadNode *child : container_quads)
        {
            if (child->segment_count == 0)
            {
                continue;
            }
            else
            {
                auto quad = child->nearestSegment(point, minDistance, nearestSegment, nearestPoint);
                quad.push_back(const_cast<QuadNode *>(this));
                return quad;
            }
        }
        std::vector<QuadNode *> quads;
        for (QuadNode *child : non_container_quads)
        {
            if (child->segment_count == 0)
            {
                continue;
            }
            auto quad = child->nearestSegment(point, minDistance, nearestSegment, nearestPoint);
            quads = quad;
        }
        quads.push_back(const_cast<QuadNode *>(this));
        return quads;
    }

    else
    {
        std::vector<QuadNode *> quads;
        // Segment nearest;
        for (const d_segment &segment : segments)
        {
            // Point closest = segment.nearestPoint(point);
            g_point closestPoint = gg::closestPoint(point, segment);
            double distance = gg::distance(point, closestPoint);
            if (distance < minDistance)
            {
                nearestPoint = closestPoint;
                minDistance = distance;
                nearestSegment = segment;
            }
        }
        quads.push_back(const_cast<QuadNode *>(this));
        return quads;
    }
}

AnswerNearestSegment QuadNode::find_nearest_segment(const g_point &p)
{
    double min_distance = std::numeric_limits<double>::max();
    d_segment nearest_segment = {{0, 0}, {0, 0}, -1, 0};
    g_point nearest_point = {0, 0};
    auto quads = nearestSegment(p, min_distance, nearest_segment, nearest_point);
    return {nearest_segment, nearest_point, min_distance, quads};
}