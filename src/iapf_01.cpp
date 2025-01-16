#include <cmath>
#include <vector>
#include "matplotlibcpp.h"
#include <iostream>
namespace plt = matplotlibcpp;

// Define the constants
const double ZETA = 0.5; // Gravitational gain constant
const double EETA = 0.5; // Repulsive force gain
const double Q_STAR = .9; // Threshold of the barrier's acting distance
const double STEP_SIZE = 0.1; // Step size for the movement of the robot

// Define the structure for a point in 2D space
struct Point {
    double x;
    double y;
};

// Function to calculate the distance between two points
double distance(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}



// // Function to calculate the attractive force
// Point F_att(Point q, Point q_goal) {
//     double d = distance(q, q_goal);
//     Point F;
//     F.x = -ZETA * (q.x - q_goal.x);
//     F.y = -ZETA * (q.y - q_goal.y);
//     return F;
// }

// Function to calculate the attractive force
Point F_att(Point q, Point q_goal) {
    double d = distance(q, q_goal);
    Point F;
    if (d <= Q_STAR) {
        F.x = -ZETA * (q.x - q_goal.x);
        F.y = -ZETA * (q.y - q_goal.y);
    } else {
        // If distance is greater than Q_STAR, scale the force by Q_STAR
        double scale_factor = -ZETA * Q_STAR;
        F.x = scale_factor * (q.x - q_goal.x);
        F.y = scale_factor * (q.y - q_goal.y);
    }
    return F;
}

// // Function to calculate the repulsive force
// Point F_rep(Point q, Point q_obstacle) {
//     double D = distance(q, q_obstacle);
//     Point F = {0, 0};
//     if (D <= Q_STAR) {
//         F.x = EETA * (1/D - 1/Q_STAR) * 1/pow(D, 2) * (q.x - q_obstacle.x);
//         F.y = EETA * (1/D - 1/Q_STAR) * 1/pow(D, 2) * (q.y - q_obstacle.y);
//     }
//     return F;
// }

// Function to calculate the repulsive force
Point F_rep(Point q, Point q_obstacle, Point q_goal, double d) {
    double D = distance(q, q_obstacle);
    Point F = {0, 0};
    if (D <= Q_STAR) {
        // Avoid division by zero
        if (D == 0) return F;
        
        double scale_factor1 = EETA * (1 / D - 1 / Q_STAR) * (d * d / (D * D));
        double scale_factor2 = EETA * (1 / D - 1 / Q_STAR) * (1 / D - 1 / Q_STAR) * d;
        
        F.x = scale_factor1 * (q.x - q_obstacle.x) + scale_factor2 * (q.x - q_goal.x);
        F.y = scale_factor1 * (q.y - q_obstacle.y) + scale_factor2 * (q.y - q_goal.y);
    }
    return F;
}


// Function to calculate the total force
Point F_total(Point q, Point q_goal, std::vector<Point> obstacles) {
    Point F_total = F_att(q, q_goal);
    for (Point q_obstacle : obstacles) {
        Point F_obstacle = F_rep(q, q_obstacle, q_goal, distance(q, q_goal));
        F_total.x += F_obstacle.x;
        F_total.y += F_obstacle.y;
    }
    return F_total;
}

int main() {
    // Define the current point, goal point, and obstacles
    Point q = {-5, -5};
    Point q_goal = {10, 10};
    std::vector<Point> obstacles = {{9, 9}, {7, 6}, {3, 3.5}, {5, 4.5}, {-1,-0.7},{-1, -0.8}};

    // Initialize the path with the starting point
    std::vector<double> x_path = {q.x};
    std::vector<double> y_path = {q.y};

    // Move the robot from q to q_goal according to F
    while (distance(q, q_goal) > STEP_SIZE) {
        Point F = F_total(q, q_goal, obstacles);
        // std::cout << F.x << std::endl;
        // std::cout << F.y << std::endl;
        q.x += STEP_SIZE * F.x;
        q.y += STEP_SIZE * F.y;
        x_path.push_back(q.x);
        y_path.push_back(q.y);
    }

    // Plot the path
    plt::plot(x_path, y_path, "ro-");

    // Plot the obstacles
    for (Point q_obstacle : obstacles) {
        plt::plot({q_obstacle.x}, {q_obstacle.y}, "bs");
    }
        plt::xlabel("X");
        plt::ylabel("Y");
        plt::title("Path Planning with Artificial Potential Field");
    plt::show();

    return 0;
}
