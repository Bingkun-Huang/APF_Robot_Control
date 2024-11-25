#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

struct Point 
{
    double x, y, z;
};


bool readInitPositions(const std::string& filePath, Point& start, Point& goal, Point& obstacle, double& obstacle_radius) {
    std::ifstream file(filePath);
    if (!file.is_open()) 
    {
        std::cerr << "Error: Could not open " << filePath << std::endl;
        return -1;
    }

    std::string line;
    int line_num = 0; 
    while (std::getline(file, line)) 
    {
        std::stringstream ss(line);
        
       if (line_num == 0) 
       {  
            ss >> start.x;
            ss.ignore(1); 
            ss >> start.y;
            ss.ignore(1);
            ss >> start.z;
        } 
        else if (line_num == 1) 
        { 
            ss >> goal.x;
            ss.ignore(1);
            ss >> goal.y;
            ss.ignore(1);
            ss >> goal.z;
        } 
        else if (line_num == 2) 
        { 
            ss >> obstacle.x;
            ss.ignore(1);
            ss >> obstacle.y;
            ss.ignore(1);
            ss >> obstacle.z;
        } 
        else if (line_num == 3) 
        { 
            ss >> obstacle_radius;
        }
        line_num++;
    }

    file.close();
    return true;
}

std::vector<Point> readTrajectory(const std::string& filePath) 
{
    std::ifstream file(filePath);
    std::vector<Point> trajectory;
    if (!file.is_open()) 
    {
        std::cerr << "Error: Could not open " << filePath << std::endl;
        return trajectory;
    }

    std::string line;
    while (std::getline(file, line)) 
    {
        std::stringstream ss(line);
        Point point;
        char comma;
        ss >> point.x >> comma >> point.y >> comma >> point.z;
        trajectory.push_back(point);
    }

    file.close();
    return trajectory;
}

// XML writting 
void generateXML(const std::string& outputPath, const Point& start, const Point& goal, const Point& obstacle, double obstacle_radius, const std::vector<Point>& trajectory) {
    std::ofstream file(outputPath);
    if (!file.is_open()) 
    {
        std::cerr << "Error: Could not create " << outputPath << std::endl;
        return;
    }

    // head of XML
    file << "<mujoco model=\"trajectory_visualization\">\n";
    file << "  <worldbody>\n";

    // initital position
    file << "    <body name=\"start\" pos=\"" 
         << start.x << " " << start.y << " " << start.z << "\">\n";
    file << "      <geom type=\"sphere\" size=\"0.04\" rgba=\"0 1 0 1\" />\n"; 
    file << "    </body>\n";

    // goal position
    file << "    <body name=\"goal\" pos=\"" 
         << goal.x << " " << goal.y << " " << goal.z << "\">\n";
    file << "      <geom type=\"sphere\" size=\"0.04\" rgba=\"1 0 0 1\" />\n"; 
    file << "    </body>\n";

    // obstacle 
    file << "    <body name=\"obstacle\" pos=\"" 
         << obstacle.x << " " << obstacle.y << " " << obstacle.z << "\">\n";
    file << "      <geom type=\"sphere\" size=\"" << obstacle_radius << "\" rgba=\"0 0 1 1\" />\n"; // 蓝色
    file << "    </body>\n";

    // points of trajectory
    for (size_t i = 0; i < trajectory.size(); ++i) 
    {
        file << "    <body name=\"trajectory_point" << i + 1 
             << "\" pos=\"" << trajectory[i].x << " " 
             << trajectory[i].y << " " << trajectory[i].z << "\">\n";
        file << "      <geom type=\"sphere\" size=\"0.01\" rgba=\"1 1 0 1\" />\n"; 
        file << "    </body>\n";
    }

    file << "  </worldbody>\n";
    file << "</mujoco>\n";

    file.close();
    std::cout << "XML file generated: " << outputPath << std::endl;
}

int main() {

    std::string initFile = "init_positions.txt";
    std::string trajectoryFile = "trajectory.txt";
    std::string outputFile = "output_trajectory.xml";

    Point start, goal, obstacle;
    double obstacle_radius;
    if (!readInitPositions(initFile, start, goal, obstacle, obstacle_radius))
    {
        return -1;
    }


    std::vector<Point> trajectory = readTrajectory(trajectoryFile);
    if (trajectory.empty())
    {
        std::cerr << "Error: No trajectory points found!" << std::endl;
        return -1;
    }


    generateXML(outputFile, start, goal, obstacle, obstacle_radius, trajectory);

    return 0;
}
