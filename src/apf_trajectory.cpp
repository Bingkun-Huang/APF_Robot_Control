#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
using namespace std;

const double k_att = 0.6;       // k for Attractive Field 
const double k_rep = 0.6;      //  k for Repulsive  Field 
const double d_th = 0.2;        // distance when act Rep Field 
const double step_size = 0.01;  // change step
const double tolerance = 0.05;  // tolerance for reaching the goal

vector<double> robot_pos = {0.2, 0.2, 0.2};   // initial position xyz
vector<double> goal_pos = {0.7, 0.7, 0.7};    // goal position xyz 
vector<double> obstacle_pos = {0.5, 0.5, 0.5}; // obstacle position xyz
double obstacle_radius = 0.1;                      // radius of ball obstacle 

double cal_Distance(const vector<double>& p1, const vector<double>& p2) 
{
    double dist = 0.0;
    for (size_t i = 0; i < 3; ++i) 
    {
        dist = dist + pow(p1[i] - p2[i], 2);
    }
    return sqrt(dist);
}

vector<double> cal_Att_Force(const vector<double>& robot_pos, const vector<double>& goal_pos) 
{
    vector<double> force(3, 0.0);
    for (size_t i = 0; i < 3; ++i) 
    {
        force[i] = -k_att * (robot_pos[i] - goal_pos[i]);
    }
    return force;
}

vector<double> cal_Rep_Force(const vector<double>& robot_pos, const vector<double>& obs_pos,double obstacle_radius) 
{
    vector<double> force(3, 0);
    double d = cal_Distance(robot_pos, obs_pos); 
    double real_distance = d - obstacle_radius; 

    const double max_rep_force = 10.0; 
    //const double max_rep_force = 5.0;  // when it is to big 
    if (real_distance <= d_th)
     {

        double eta = min(k_rep * (1 / real_distance - 1 / d_th) / (real_distance * real_distance), max_rep_force);
        for (size_t i = 0; i < 3; ++i) 
        {
            force[i] = eta * (robot_pos[i] - obs_pos[i]) / d;
        }
    }
    return force;
}


vector<double> updateRobotPosition(const std::vector<double>& robot_pos,const std::vector<double>& goal_pos,const std::vector<double>& obstacle_pos,double obstacle_radius) 
{
    std::vector<double> total_force(3, 0);

    vector<double> F_att = cal_Att_Force(robot_pos, goal_pos);

    vector<double> F_rep = cal_Rep_Force(robot_pos, obstacle_pos, obstacle_radius);

    for (size_t i = 0; i < 3; ++i)
     {
        total_force[i] = F_att[i] + F_rep[i];
    }

    vector<double> new_position(3, 0.0);
    for (size_t i = 0; i < 3; ++i) 
    {
        new_position[i] = robot_pos[i] + step_size * total_force[i];
        double noise = (rand() % 100 - 50) / 100000.0; // +- -0.0005
        new_position[i] = new_position[i] + noise;
    }
    return new_position;
}

int main() {
    cout << "--------- APF trajectory ---------" << endl;

    vector<vector<double>> path;
    path.push_back(robot_pos);
    // output init-parameters of goal/fist/obstacle positons and redaious 
    ofstream init_file("init_positions.txt");
    if (!init_file.is_open())
    {
        std::cerr << "Error in init_positions.txt file" << endl;
        return -1;
    }

    init_file << robot_pos[0] << ", " << robot_pos[1] << ", " <<robot_pos[2] << "\n";
    init_file << goal_pos[0] << ", " << goal_pos[1] << ", " << goal_pos[2] << "\n";
    init_file << obstacle_pos[0] << ", " << obstacle_pos[1] << ", " << obstacle_pos[2] << "\n";
    init_file << obstacle_radius << "\n";
    init_file.close();

    cout << "Initial positions saved to 'init_positions.txt'." << endl;
    // output trajectory 
    ofstream file("trajectory.txt");
    if (!file.is_open()) 
    {
        cerr << "Error in trajectory file" << endl;
        return -1;
    }

    file << robot_pos[0] << ", " << robot_pos[1] << ", " << robot_pos[2] << "\n";

    int iteration = 0;
    while (cal_Distance(robot_pos, goal_pos) > tolerance)
    {
        robot_pos = updateRobotPosition(robot_pos, goal_pos, obstacle_pos, obstacle_radius);
        path.push_back(robot_pos);

        file << robot_pos[0] << ", " << robot_pos[1] << ", " << robot_pos[2] << "\n";
        
        double distance_to_goal = cal_Distance(robot_pos, goal_pos);
        cout << "Step " << iteration << ": Position = ["
                  << robot_pos[0] << " " << robot_pos[1] << " " << robot_pos[2] << ']'
                  << " Distance to Goal = " << distance_to_goal << endl;

        iteration++;
        if (iteration > 10000) 
        {
            cout << "Error : too much step iteration" << endl;
            break;
        }
    }
    file.close();
    cout << "For all " << iteration << " iterations!" << endl;
    return 0;
}
