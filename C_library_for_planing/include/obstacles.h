#pragma once

class Obstacle {
public:
    virtual void Draw() const = 0;
    std::string name;
};

class Line : public Obstacle {
public:
    Line(double x1, double y1, double x2, double y2)
        : x1_(x1), y1_(y1), x2_(x2), y2_(y2) {name="Line";}

    void Draw() const override {
        std::cout << name << " obstacle: (" << x1_ << ", " << y1_ << ") to (" << x2_ << ", " << y2_ << ")" << std::endl;
    }

private:
    double x1_;
    double y1_;
    double x2_;
    double y2_;
};

class Circle : public Obstacle {
public:
    Circle(double centerX, double centerY, double radius)
        : centerX_(centerX), centerY_(centerY), radius_(radius) {name="Circle";}

    void Draw() const override {
        std::cout << name <<" obstacle: Center (" << centerX_ << ", " << centerY_ << "), Radius: " << radius_ << std::endl;
    }

private:
    double centerX_;
    double centerY_;
    double radius_;
};

class Square : public Obstacle {
public:
    Square(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4)
        : x1_(x1), y1_(y1), x2_(x2), y2_(y2), x3_(x3), y3_(y3), x4_(x4), y4_(y4) {name="Square";}

    void Draw() const override {
        std::cout << name << " obstacle: (" << x1_ << ", " << y1_ << "), (" << x2_ << ", " << y2_ << "), ("
         << x3_ << ", " << y3_ << "), (" << x4_ << ", " << y4_ << ")" << std::endl;
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