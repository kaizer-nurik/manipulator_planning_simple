#include "planner.h"
#include "parsing.h"
#include <chrono>
#include <typeinfo>
#include <random>

template <typename T >
class Timer
{
public:
	using duration_t = T;
	using clock_t = std::chrono::steady_clock;
	using time_point_t = clock_t::time_point;

	void pause()
	{
		auto end = clock_t::now();
		m_duration += std::chrono::duration_cast<duration_t>(end - m_begin);
		is_stopped = true;
		std::cout << name << " is stopped: " << m_duration.count() << std::endl;
	}

	void reset()
	{
		m_begin = clock_t::now();
		is_stopped = false;
		std::cout << name << " is reset" << std::endl;
	}

	void time()
	{
		auto end = clock_t::now();
		m_duration += std::chrono::duration_cast<duration_t>(end - m_begin);
		std::cout << name << " time " << m_duration.count() << ' ' << typeid(m_duration.count()).name() << std::endl;
	}


	Timer(std::string Name) : m_begin(clock_t::now()), m_duration(0), is_stopped(false), name(Name)
	{
		std::cout << name << " is started: " << std::endl;
	}

	~Timer() noexcept
	{
		try
		{
			if (!is_stopped)
			{
				auto end = clock_t::now();
				m_duration += std::chrono::duration_cast<duration_t>(end - m_begin);
				std::cout << name << " is stopped: " << m_duration.count() << ' ' << "milliseconds" << std::endl;
			}
			//std::cout << typeid(duration_t).name() << ' ' << m_duration.count() << std::endl;
		}
		catch (...)
		{
			std::abort();
		}

	}

private:
	time_point_t m_begin;
	duration_t m_duration;
	bool is_stopped;
	std::string name;
};


bool run_benchmark(std::string input_file_name, int N_tests)
{
    Timer<std::chrono::milliseconds> t("timer");
    std::vector<Polygon> polygons;
    Robot robot = Robot();
    GoalPoint goal(0.0, 0.0, 0.0, 0.0);
    

    bool read_normally = read_scene(input_file_name, polygons, robot, goal);
    std::cout << read_normally << std::endl;
    if (! read_normally)
    {
        return false;
    }

    std::random_device rd;
	std::mt19937 engine{ 5 };
	std::uniform_real_distribution<double> dist{ 0.0, 1.0 };
    double PI = std::acos(-1);
    int success = 0;
    if (robot.dof_ == 1)
    {
        goal.angle2_ = 2*PI;
        for (int test=0; test<N_tests; test++)
        {
            double x = cos(dist(engine) * 2 * PI)*robot.joints[0].length;
            double y = sin(dist(engine) * 2 * PI)*robot.joints[0].length;
            goal.goalpoint.x = x;
            goal.goalpoint.y = y;

            Planner planner("trajectory.csv");
            bool b = planner.AStar(robot, goal, polygons);
            success+=b;
            if (b) 
            {
                std::cout << "test_"<< test << "successed, angle was " << std::acos(x) << std::endl;
            }
            else  
            {
                std::cout << "test_"<< test << "faild, angle was " << std::acos(x) << std::endl;
            }
        }
        std::cout << "testing completed: " << success << '/' << N_tests << " successes" << std::endl;
    }
    return true;

}