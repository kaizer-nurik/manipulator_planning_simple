#pragma once
#include <vector>
#include "geometry.h"

struct Vector2D {
    Vector2D(double xVal, double yVal) : x(xVal), y(yVal) {} //  Constructor
    Vector2D(const Vector2D& _other) = default; // Copy constructor
    Vector2D& operator=(const Vector2D& other) = default; // Copy asigment
    Vector2D(Vector2D&& _other) = default; //move constructor
    Vector2D& operator=(Vector2D&& other) = default;//move asigment
    ~Vector2D()=default;//Destructor
    double x;
    double y;

    // Dot product of two vectors
    double dotProduct(const Vector2D& other) const {
        return x * other.x + y * other.y;
    }

    double operator*(const Vector2D& other)const{
        return x * other.x + y * other.y;
    }
    // Perpendicular vector
    Vector2D perpendicular() const {
        return Vector2D(-y, x);
    }
};

struct Polygon {
    std::vector<Vector2D> points;

    Polygon(const std::vector<Vector2D>& vertexes) : points(vertexes) {}
    Polygon(const Polygon& _other) = default; // Copy constructor
    Polygon& operator=(const Polygon& other) = default; // Copy asigment
    Polygon(Polygon&& _other) = default; //move constructor
    Polygon& operator=(Polygon&& other) = default;//move asigment
    ~Polygon()= default;//Destructor

    // Get the edges of the polygon
    std::vector<Vector2D> getEdges() const {
        std::vector<Vector2D> edges;
        int numPoints = points.size();
        for (int i = 0; i < numPoints; i++) {
            const Vector2D& p1 = points[i];
            const Vector2D& p2 = points[(i + 1) % numPoints];
            edges.push_back(Vector2D(p2.x - p1.x, p2.y - p1.y));
        }
        return edges;
    }
};

// Projection of a polygon onto a given axis
struct Projection {
    double min;
    double max;

    Projection(double minValue, double maxValue) : min(minValue), max(maxValue) {}
    Projection(const Projection& _other) = default; // Copy constructor
    Projection& operator=(const Projection& other) = default; // Copy asigment
    Projection(Projection&& _other) = default; //move constructor
    Projection& operator=(Projection&& other) = default;//move asigment
    ~Projection()= default;//Destructor
    // Check if two projections overlap
    bool overlaps(const Projection& other) const {
        return (max >= other.min) && (other.max >= min);
    }
};

// Calculate the projection of a polygon onto a given axis
Projection projectPolygon(const Polygon& polygon, const Vector2D& axis) {
    double min = polygon.points[0].dotProduct(axis);
    double max = min;

    for (const Vector2D& point : polygon.points) {
        double projection = point.dotProduct(axis);
        if (projection < min) {
            min = projection;
        }
        else if (projection > max) {
            max = projection;
        }
    }

    return Projection(min, max);
}

// Check if two polygons collide using the Separating Axis Theorem (SAT)
bool checkCollision(const Polygon& polygonA, const Polygon& polygonB) {  // true -- not collide, false -- collide
    std::vector<Vector2D> edgesA = polygonA.getEdges();
    std::vector<Vector2D> edgesB = polygonB.getEdges();

    // Check for overlapping projections on each axis of polygon A
    for (const Vector2D& edge : edgesA) {
        Vector2D axis = edge.perpendicular();
        Projection projectionA = projectPolygon(polygonA, axis);
        Projection projectionB = projectPolygon(polygonB, axis);

        if (!projectionA.overlaps(projectionB)) {
            return false;  // Separating axis found, no collision
        }
    }

    // Check for overlapping projections on each axis of polygon B
    for (const Vector2D& edge : edgesB) {
        Vector2D axis = edge.perpendicular();
        Projection projectionA = projectPolygon(polygonA, axis);
        Projection projectionB = projectPolygon(polygonB, axis);

        if (!projectionA.overlaps(projectionB)) {
            return false;  // Separating axis found, no collision
        }
    }

    return true;  // No separating axis found, polygons collide
}

bool isConvexPolygon(const Polygon& p) {
    std::vector<Vector2D> polygon = p.points;
    int n = polygon.size();
    if (n < 3) {
        // A polygon with less than 3 vertices is not considered convex
        return false;
    }

    bool isClockwise = false;
    bool isCounterClockwise = false;

    for (int i = 0; i < n; ++i) {
        const Vector2D& p1 = polygon[i];
        const Vector2D& p2 = polygon[(i + 1) % n];
        const Vector2D& p3 = polygon[(i + 2) % n];

        double crossProduct = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);

        if (crossProduct > 0) {
            isClockwise = true;
        }
        else if (crossProduct < 0) {
            isCounterClockwise = true;
        }

        if (isClockwise && isCounterClockwise) {
            // Cross products have different signs, indicating concave angles
            return false;
        }
    }

    // If we didn't find any concave angles, the polygon is convex
    return true;
}