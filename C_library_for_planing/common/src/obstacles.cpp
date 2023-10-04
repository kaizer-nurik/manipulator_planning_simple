#include "../include/obstacles.h"

#include <cmath>
#include <vector>
bool polygons_collide(const Polygon &polygonA, const Polygon &polygonB)
{ // true -- not collide, false -- collide
    std::vector<Vector2D> edgesA = polygonA.getEdges();
    std::vector<Vector2D> edgesB = polygonB.getEdges();

    // Check for overlapping projections on each axis of polygon A
    for (const Vector2D &edge : edgesA)
    {
        Vector2D axis = edge.perpendicular();
        Projection projectionA = projectPolygon(polygonA, axis);
        Projection projectionB = projectPolygon(polygonB, axis);

        if (!projectionA.overlaps(projectionB))
        {
            return false; // Separating axis found, no collision
        }
    }

    // Check for overlapping projections on each axis of polygon B
    for (const Vector2D &edge : edgesB)
    {
        Vector2D axis = edge.perpendicular();
        Projection projectionA = projectPolygon(polygonA, axis);
        Projection projectionB = projectPolygon(polygonB, axis);

        if (!projectionA.overlaps(projectionB))
        {
            return false; // Separating axis found, no collision
        }
    }

    return true; // No separating axis found, polygons collide
}

double Vector2D::dotProduct(const Vector2D &other) const
{
    return x * other.x + y * other.y;
}

// Perpendicular vector
Vector2D Vector2D::perpendicular() const
{
    return Vector2D(-y, x);
}
bool Vector2D::operator==(const Vector2D &other) const
{
    double tolerance = 1e-6;
    return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y)) <= tolerance;
}
double Vector2D::length() const
{
    return std::sqrt(x * x + y * y);
}

// Get the edges of the polygon
std::vector<Vector2D> Polygon::getEdges() const
{
    std::vector<Vector2D> edges;
    int numPoints = points.size();
    for (int i = 0; i < numPoints; i++)
    {
        const Vector2D &p1 = points[i];
        const Vector2D &p2 = points[(i + 1) % numPoints];
        edges.push_back(Vector2D(p2.x - p1.x, p2.y - p1.y));
    }
    return edges;
}

// Projection of a polygon onto a given axis

    // Check if two projections overlap
    bool Projection::overlaps(const Projection &other) const
    {
        return (max >= other.min) && (other.max >= min);
    }


// Calculate the projection of a polygon onto a given axis
Projection projectPolygon(const Polygon &polygon, const Vector2D &axis)
{
    double min = polygon.points[0].dotProduct(axis);
    double max = min;

    for (const Vector2D &point : polygon.points)
    {
        double projection = point.dotProduct(axis);
        if (projection < min)
        {
            min = projection;
        }
        else if (projection > max)
        {
            max = projection;
        }
    }

    return Projection(min, max);
}

// Check if two polygons collide using the Separating Axis Theorem (SAT)


bool isConvexPolygon(const Polygon &p)
{
    std::vector<Vector2D> polygon = p.points;
    int n = polygon.size();
    if (n < 3)
    {
        // A polygon with less than 3 vertices is not considered convex
        return false;
    }

    bool isClockwise = false;
    bool isCounterClockwise = false;

    for (int i = 0; i < n; ++i)
    {
        const Vector2D &p1 = polygon[i];
        const Vector2D &p2 = polygon[(i + 1) % n];
        const Vector2D &p3 = polygon[(i + 2) % n];

        double crossProduct = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);

        if (crossProduct > 0)
        {
            isClockwise = true;
        }
        else if (crossProduct < 0)
        {
            isCounterClockwise = true;
        }

        if (isClockwise && isCounterClockwise)
        {
            // Cross products have different signs, indicating concave angles
            return false;
        }
    }

    // If we didn't find any concave angles, the polygon is convex
    return true;
}
