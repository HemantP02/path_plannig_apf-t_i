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

// Function to calculate the rotation angle between two vectors
double rotation_angle(Point q_current, Point q_previous) {
    // Calculate the vector between the current and previous points
    double dx = q_current.x - q_previous.x;
    double dy = q_current.y - q_previous.y;

    // Calculate the angle using atan2
    double angle = atan2(dy, dx);

    return angle;
}

// Function to smooth the path curve
std::vector<Point> smooth_path(std::vector<Point>& path, double alpha, double beta, int max_iterations) {
    std::vector<Point> smoothed_path = path;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        for (size_t i = 1; i < path.size() - 1; ++i) {
            double dx = alpha * (path[i].x - smoothed_path[i].x) + beta * (smoothed_path[i - 1].x - 2 * smoothed_path[i].x + smoothed_path[i + 1].x);
            double dy = alpha * (path[i].y - smoothed_path[i].y) + beta * (smoothed_path[i - 1].y - 2 * smoothed_path[i].y + smoothed_path[i + 1].y);

            smoothed_path[i].x += dx;
            smoothed_path[i].y += dy;
        }
    }

    return smoothed_path;
}

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
    std::vector<Point> obstacles = {{9, 9}, {7, 6}, {3, 3.5}, {5, 4.5}, {-1,-0.7}, {-1, -0.8}};

    // Initialize the path with the starting point
    std::vector<double> x_path = {q.x};
    std::vector<double> y_path = {q.y};

    // Initialize the previous point to the starting point
    Point q_previous = q;

    // Move the robot from q to q_goal according to F
    while (distance(q, q_goal) > STEP_SIZE) {
        // Calculate the total force
        Point F = F_total(q, q_goal, obstacles);

        // Calculate the current rotation angle
        double theta_i = rotation_angle(q, q_previous);

        // Adjust the step size based on the rotation angle
        double adjusted_step_size = STEP_SIZE;
        if (theta_i != 0) {
            double rotation_angle_diff = std::abs(theta_i - rotation_angle(q_previous, q));
            if (rotation_angle_diff < M_PI / 6.0) {
                adjusted_step_size = 2 * STEP_SIZE;
            } else if (rotation_angle_diff <= M_PI / 3.0) {
                adjusted_step_size = STEP_SIZE;
            } else {
                adjusted_step_size = STEP_SIZE / 2.0;
            }
        }

        // Update the previous point
        q_previous = q;

        // Update the position of the robot
        q.x += adjusted_step_size * F.x;
        q.y += adjusted_step_size * F.y;

        // Update the path
        x_path.push_back(q.x);
        y_path.push_back(q.y);
    }

    // Plot the path
    plt::plot(x_path, y_path, "ro-");

    // Plot the obstacles
    for (Point q_obstacle : obstacles) {
        plt::plot({q_obstacle.x}, {q_obstacle.y}, "bs");
    }

    // Set labels and title
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Path Planning with Artificial Potential Field");

    // Show the plot
    plt::show();

    return 0;
}