#pragma once
#include <cmath>
#include <vector>
struct Vector2D {
    double x;
    double y;

    Vector2D(double xVal, double yVal) : x(xVal), y(yVal) {}

    // Dot product of two vectors
    double dotProduct(const Vector2D& other) const ;

    // Perpendicular vector
    Vector2D perpendicular() const ;
    bool operator==(const Vector2D& other) const;
    double length()const;
};

struct Polygon {
    std::vector<Vector2D> points;

     Polygon(const std::vector<Vector2D>& vertexes) : points(vertexes) {}
    // Get the edges of the polygon
    std::vector<Vector2D> getEdges() const ;
};

// Projection of a polygon onto a given axis
struct Projection {
    double min;
    double max;

    Projection(double minValue, double maxValue) : min(minValue), max(maxValue) {}

    // Check if two projections overlap
    bool overlaps(const Projection& other) const ;
};

// Calculate the projection of a polygon onto a given axis
Projection projectPolygon(const Polygon& polygon, const Vector2D& axis) ;


// Check if two polygons collide using the Separating Axis Theorem (SAT)
bool polygons_collide(const Polygon &polygonA, const Polygon &polygonB);


bool isConvexPolygon(const Polygon& p) ;