#include <iostream>
#include <cmath>
#include <vector>

// Define constants
const double zeta = 1.0; // Gravitational gain constant
const double eta = 1.0;  // Repulsive force gain
const double Q_star = 1.0; // Threshold distance for obstacles

// Define point structure
struct Point {
    double x;
    double y;
};

// Define class for the robot
class Robot {
private:
    Point position;

public:
    Robot(double x, double y) : position({x, y}) {}

    Point getPosition() const {
        return position;
    }
};

// Define class for obstacles
class Obstacle {
private:
    Point position;

public:
    Obstacle(double x, double y) : position({x, y}) {}

    Point getPosition() const {
        return position;
    }
};

// Define class for APF planner
class APFPlanner {
private:
    Robot robot;
    Point goal;
    std::vector<Obstacle> obstacles;

public:
    APFPlanner(Robot r, Point g, std::vector<Obstacle> obs) : robot(r), goal(g), obstacles(obs) {}

    // Calculate attractive force
    Point calculateAttractiveForce() {
        Point robotPos = robot.getPosition();
        double dx = goal.x - robotPos.x;
        double dy = goal.y - robotPos.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        // Only move towards the goal if the distance is greater than some threshold
        if (distance > 0.1) {
            return {(zeta * dx) / distance, (zeta * dy) / distance};
        } else {
            return {0, 0};
        }
    }

    // Calculate repulsive force
    Point calculateRepulsiveForce() {
        Point robotPos = robot.getPosition();
        double repulsiveForceX = 0, repulsiveForceY = 0;
        
        for (const auto& obstacle : obstacles) {
            double dx = robotPos.x - obstacle.getPosition().x;
            double dy = robotPos.y - obstacle.getPosition().y;
            double distance = sqrt(dx*dx + dy*dy);
            
            if (distance <= Q_star) {
                repulsiveForceX += (eta * (1/distance - 1/Q_star) * (1/distance*distance) * dx);
                repulsiveForceY += (eta * (1/distance - 1/Q_star) * (1/distance*distance) * dy);
            }
        }
        
        return {repulsiveForceX, repulsiveForceY};
    }

    // Calculate resultant force
    Point calculateResultantForce() {
        Point attractiveForce = calculateAttractiveForce();
        Point repulsiveForce = calculateRepulsiveForce();

        return {attractiveForce.x + repulsiveForce.x, attractiveForce.y + repulsiveForce.y};
    }

    // Move the robot based on the resultant force
    void moveRobot() {
        Point resultantForce = calculateResultantForce();
        Point robotPos = robot.getPosition();
        
        // Update the position of the robot
        robotPos.x += resultantForce.x;
        robotPos.y += resultantForce.y;
        robotPos = clipToBoundary(robotPos); // Ensure robot stays within boundaries
        
        // Update the robot's position
        robot = Robot(robotPos.x, robotPos.y);
    }

    // Clip the robot's position to stay within boundaries
    Point clipToBoundary(Point pos) {
        // Implement boundary clipping logic here if needed
        return pos;
    }
};

int main() {
    // Define initial robot position, goal position, and obstacles
    Robot robot(0, 0);
    Point goal = {10, 10};
    std::vector<Obstacle> obstacles = {Obstacle(5, 5), Obstacle(8, 8)}; // Example obstacles

    // Create APF planner
    APFPlanner planner(robot, goal, obstacles);

    // Move the robot towards the goal
    for (int i = 0; i < 100; ++i) { // Simulate 100 iterations
        planner.moveRobot();
        Point robotPos = planner.calculateResultantForce();
        std::cout << "Robot Position: (" << robotPos.x << ", " << robotPos.y << ")\n";
    }

    return 0;
}
