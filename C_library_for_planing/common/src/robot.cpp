#include "robot.h"
#include <assert.h>
#include "./obstacles.h"
#include <cmath>
Robot::Robot(const Robot &other)
{
    for (Joint a : other.joints)
    {
        this->AddJoint(a.length, a.width, a.limits);
    }
    this->dof_ = other.dof_;
    this->configuration = other.configuration;
}

void Robot::AddJoint(double length, double width, const std::vector<double> &limits)
{
    Joint joint;
    joint.length = length;
    joint.width = width;
    joint.limits = limits;
    joints.push_back(joint);
    dof_++;
}

void Robot::printJointDetails() const
{
    std::cout << "Robot DOF: " << dof_ << std::endl;

    for (const auto &joint : joints)
    {
        std::cout << "Joint length: " << joint.length << std::endl;
        std::cout << "Joint width: " << joint.width << std::endl;
        std::cout << "Joint limits: ";

        for (const auto &limit : joint.limits)
        {
            std::cout << limit << " ";
        }

        std::cout << std::endl;
    }
}

Robot Robot::operator-(Robot const rhs) const
{
    Robot result(*this);
    for (int joint_index = 0; joint_index < result.configuration.size(); joint_index++)
    {
        result.configuration[joint_index] = result.configuration[joint_index] - rhs.configuration[joint_index];
    }
    return result;
}
Robot Robot::operator+(Robot const rhs) const
{
    Robot result(*this);
    assert(this->dof_==rhs.dof_);
    for (int joint_index = 0; joint_index < result.configuration.size(); joint_index++)
    {
        result.configuration[joint_index] = result.configuration[joint_index] + rhs.configuration[joint_index];
    }
    return result;
}

Robot Robot::operator/(double const rhs) const
{
    Robot result(*this);
    for (int joint_index = 0; joint_index < result.configuration.size(); joint_index++)
    {
        result.configuration[joint_index] = result.configuration[joint_index] / rhs;
    }
    return result;
}

Robot Robot::operator*(double const rhs) const
{
    Robot result(*this);
    for (int joint_index = 0; joint_index < result.configuration.size(); joint_index++)
    {
        result.configuration[joint_index] = result.configuration[joint_index] * rhs;
    }
    return result;
}

double Robot::distance(const Robot& other) const{
    double result = 0;
    for (int i = 0; i < configuration.size(); i++)
    {
       
        result += std::abs(configuration[i] - other.configuration[i]);
        // assert(!std::isnan(result));
    }
    return result;
}
double fix_fmod(double angle){
    double angle_from_0_to_360 = angle - 360.0 * floor( angle / 360.0 );

    if (angle_from_0_to_360 > 180){
        angle_from_0_to_360 -= 360;
    }  
    return angle_from_0_to_360;

}
bool GoalPoint::is_goal(const double& x, const double& y, const double& angle) const{
    // std::cout<<(x )<<" "<<(y )<<" "<<std::abs(fix_fmod(angle))<<" "<<goalpoint.x<<" "<<goalpoint.y<<std::endl;
    // std::cout<<(x - goalpoint.x)<<" "<<(y - goalpoint.y)<<" "<<std::abs(fix_fmod(angle)-angle1_)<<std::endl;
     return ((
        std::sqrt(((x - goalpoint.x) * (x - goalpoint.x))+((y - goalpoint.y) * (y - goalpoint.y))) < delta)
        && (std::abs(fix_fmod(angle-angle1_)) < angle2_) );};

Vector2D end_effector(const Robot &robot, const std::vector<double>& config)
{
    Vector2D ef(0.0, 0.0);
    double angle = 0.0;
    for (auto i = 0u; i < robot.dof_; i++)
    {
        angle += config[i];
        ef.x += robot.joints[i].length * std::cos((angle) * M_PI / 180.0);
        ef.y += robot.joints[i].length * std::sin((angle) * M_PI / 180.0);
    }
    // std::cout<<ef.x<<' '<<ef.y<<std::endl;
    return ef;
}

bool collide(const Robot &robot, const std::vector<double> angles, const std::vector<Polygon> &poligons) //true - collide, false - not collide
{
    bool flag = false;
    double x0 = 0.0;
    double y0 = 0.0;
    double angle = 0.0;
    std::vector<Polygon> joints;
    for (int i=0; i<robot.dof_; i++)
    {
        //std::cout << x0 << ' ' << y0 << ' ' << robot.dof_ << ' ';
        angle += angles[i];
        double x1 = x0 - sin(angle*M_PI/180.0)*robot.joints[i].width/2;
        double y1 = y0 + cos(angle*M_PI/180.0)*robot.joints[i].width/2;
        double x2 = x0 - sin(angle*M_PI/180.0)*robot.joints[i].width/2 + cos(angle*M_PI/180.0)*robot.joints[i].length;
        double y2 = y0 + cos(angle*M_PI/180.0)*robot.joints[i].width/2 + sin(angle*M_PI/180.0)*robot.joints[i].length;
        double x3 = x0 + sin(angle*M_PI/180.0)*robot.joints[i].width/2 + cos(angle*M_PI/180.0)*robot.joints[i].length;
        double y3 = y0 - cos(angle*M_PI/180.0)*robot.joints[i].width/2 + sin(angle*M_PI/180.0)*robot.joints[i].length;
        double x4 = x0 + sin(angle*M_PI/180.0)*robot.joints[i].width/2;
        double y4 = y0 - cos(angle*M_PI/180.0)*robot.joints[i].width/2;
        joints.push_back(Polygon({Vector2D(x1, y1), Vector2D(x2, y2), Vector2D(x3, y3), Vector2D(x4, y4)}));
        x0+=cos(angle*M_PI/180.0)*robot.joints[i].length;
        y0+=sin(angle*M_PI/180.0)*robot.joints[i].length;
        
    }
    //std::cout << joints.size() << ' ' << poligons.size() << std::endl;

    for (int i=0; i<joints.size(); i++)
    {
        for (int j=0; j<poligons.size(); j++)
        {
            //std::cout << 1 << ' ' << (polygons_collide(joints[i], poligons[j])) << ' ';
            if ((polygons_collide(joints[i], poligons[j]))) 
            {
                //std::cout << "collision occured\n";
                return true;
            }
        }
    }
    // std::cout << std::endl;
    for (int i=0; i<joints.size(); i++)
    {
        for (int j=0; j<joints.size(); j++)
        {
            if (std::abs(j-i)<2) continue;
            if (polygons_collide(joints[i], joints[j])) return true;
        }
    }
    return flag;
}

float GoalPoint::distance(const Robot &robot, const std::vector<double> config) const{
    Vector2D coords = end_effector(robot,config);
    return std::sqrt((coords.x-goalpoint.x)*(coords.x-goalpoint.x)+(coords.y-goalpoint.y)*(coords.y-goalpoint.y));
}


bool GoalPoint::is_goal(const Robot& pos)const{
    Vector2D q = end_effector(pos, pos.configuration);
    double angle = 0.0;
    for (auto i = 0u; i < pos.dof_; i++)
    {
        angle += pos.configuration[i];
    }
    return is_goal(q.x, q.y, angle);
}