#pragma once
#include <vector>
#include "geometry.h"

class Obstacle 
{
public:
    virtual void Draw() const = 0;
    virtual double distancetoPoint(const Point &point) const = 0;
    std::string name;
};

class Line : public Obstacle
{
public:
    Line(double x1, double y1, double x2, double y2)
        : x1_(x1), y1_(y1), x2_(x2), y2_(y2) {name="Line";}

    void Draw() const override {
        std::cout << name << " obstacle: (" << x1_ << ", " << y1_ << ") to (" << x2_ << ", " << y2_ << ")" << std::endl;
    }

    double distancetoPoint(const Point &point) const override
    {
        Segment segment(Point(this->getcoord()[0], this->getcoord()[1]), Point(this->getcoord()[2], this->getcoord()[3]));
        return distanceToSegment(point, segment);
    }

    std::vector<double> getcoord() const
    {
        return {x1_, y1_, x2_, y2_};
    }
private:
    double x1_;
    double y1_;
    double x2_;
    double y2_;
};


class Circle : public Obstacle 
{
public:
    Circle(double centerX, double centerY, double radius)
        : centerX_(centerX), centerY_(centerY), radius_(radius) {name="Circle";}

    void Draw() const override {
        std::cout << name <<" obstacle: Center (" << centerX_ << ", " << centerY_ << "), Radius: " << radius_ << std::endl;
    }
    
    double distancetoPoint(const Point &point) const override
    {
        return std::sqrt((point.x - this->getcoord()[0])*(point.x - this->getcoord()[0]) 
    + (point.y - this->getcoord()[1])*(point.y - this->getcoord()[1]));
    }

    std::vector<double> getcoord() const
    {
        return {centerX_, centerY_, radius_};
    }

private:
    double centerX_;
    double centerY_;
    double radius_;
};


class Square : public Obstacle 
{
public:
    Square(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
        : x1_(x1), y1_(y1), x2_(x2), y2_(y2), x3_(x3), y3_(y3), x4_(x4), y4_(y4) {name="Square";}

    void Draw() const override {
        std::cout << name << " obstacle: (" << x1_ << ", " << y1_ << "), (" << x2_ << ", " << y2_ << "), ("
         << x3_ << ", " << y3_ << "), (" << x4_ << ", " << y4_ << ")" << std::endl;
    }
       
    double distancetoPoint(const Point &point) const override
    {
        std::vector<Segment> segments;
        for (auto i=0u; i<7; i+=2)
        {
            segments.push_back(Segment(Point(this->getcoord()[i], this->getcoord()[i+1]), 
            Point(this->getcoord()[(i+2)%8], this->getcoord()[(i+3)%8])));
        }
        std::vector<double> distances;
        for (const auto &i:segments)
            distances.push_back(distanceToSegment(point, i));

        auto result = std::min_element(std::begin(distances), std::end(distances));
        if (std::end(distances)!=result)
            return *result;
        return 0.0;
    }

    std::vector<double> getcoord() const
    {
        return {x1_, y1_, x2_, y2_, x3_, y3_, x4_, y4_};
    }

private:
    double x1_;
    double y1_;
    double x2_;
    double y2_;
    double x3_;
    double y3_;
    double x4_;
    double y4_;
};