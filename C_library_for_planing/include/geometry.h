#pragma once
#include <cmath>
#include <vector>
#include <cmath>
#include <algorithm> 
#include <iterator>

struct Point {
    double x;
    double y;

    Point(double xVal, double yVal) : x(xVal), y(yVal) {}
};

struct Segment {
    Point start;
    Point end;

    Segment(const Point& startPoint, const Point& endPoint) : start(startPoint), end(endPoint) {}
};

double distanceToSegment(const Point& point, const Segment& segment) {
    double segmentLength = std::sqrt(std::pow(segment.end.x - segment.start.x, 2) + std::pow(segment.end.y - segment.start.y, 2));

    if (segmentLength == 0.0) {
        // If the segment has zero length, the distance is the Euclidean distance between the point and the segment start
        return std::sqrt(std::pow(point.x - segment.start.x, 2) + std::pow(point.y - segment.start.y, 2));
    }

    // Calculate the normalized direction vector of the segment
    double directionX = (segment.end.x - segment.start.x) / segmentLength;
    double directionY = (segment.end.y - segment.start.y) / segmentLength;

    // Calculate the vector from the segment start to the point
    double vectorX = point.x - segment.start.x;
    double vectorY = point.y - segment.start.y;

    // Calculate the dot product between the vector and the segment direction
    double dotProduct = vectorX * directionX + vectorY * directionY;

    if (dotProduct <= 0.0) {
        // If the dot product is less than or equal to zero, the closest point is the segment start
        return std::sqrt(std::pow(point.x - segment.start.x, 2) + std::pow(point.y - segment.start.y, 2));
    }

    if (dotProduct >= segmentLength) {
        // If the dot product is greater than or equal to the segment length, the closest point is the segment end
        return std::sqrt(std::pow(point.x - segment.end.x, 2) + std::pow(point.y - segment.end.y, 2));
    }

    // Calculate the closest point on the segment
    double closestPointX = segment.start.x + dotProduct * directionX;
    double closestPointY = segment.start.y + dotProduct * directionY;

    // Calculate the distance between the point and the closest point on the segment
    return std::sqrt(std::pow(point.x - closestPointX, 2) + std::pow(point.y - closestPointY, 2));
}

// double distanceToLine(const Point& point, const Line &line)
// {
//     Segment segment(Point(line.getcoord()[0], line.getcoord()[1]), Point(line.getcoord()[2], line.getcoord()[3]));
//     return distanceToSegment(point, segment);
// }

// double distanceToCircle(const Point& point, const Circle &circle)
// {
//     return std::sqrt((point.x - circle.getcoord()[0])*(point.x - circle.getcoord()[0]) 
//     + (point.y - circle.getcoord()[1])*(point.y - circle.getcoord()[1]));
// }

// double distanceToSquare(const Point& point, const Square &square)
// {
//     std::vector<Segment> segments;
//     for (auto i=0u; i<7; i+=2)
//     {
//         segments.push_back(Segment(Point(square.getcoord()[i], square.getcoord()[i+1]), 
//         Point(square.getcoord()[(i+2)%8], square.getcoord()[(i+3)%8])));
//     }
//     std::vector<double> distances;
//     for (const auto &i:segments)
//         distances.push_back(distanceToSegment(point, i));

//     auto result = std::min_element(std::begin(distances), std::end(distances));
//     if (std::end(distances)!=result)
//         return *result;
//     return 0.0;
// }
