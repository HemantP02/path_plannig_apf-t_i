#include <iostream>
#include <cmath>
#include <vector>

// Define constants
const double ZETA = 1.0; // Attractive force constant
const double EETA = 1.0; // Repulsive force constant
const double Q_STAR = 1.0; // Threshold distance

// Define point structure
struct Point {
    double x, y;
};

// Function to calculate distance between two points
double distance(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

// Function to calculate attractive force
Point F_att(Point q, Point q_goal) {
    Point F;
    F.x = -ZETA * (q.x - q_goal.x);
    F.y = -ZETA * (q.y - q_goal.y);
    return F;
}

// Function to calculate repulsive force
Point F_rep(Point q, std::vector<Point>& obstacles, Point q_goal, double d) {
    Point F = {0, 0};
    for (const auto& obstacle : obstacles) {
        double D = distance(q, obstacle);
        if (D <= Q_STAR) {
            if (D == 0) continue; // Avoid division by zero
            double scale_factor1 = EETA * (1 / D - 1 / Q_STAR) * (d * d / (D * D));
            double scale_factor2 = EETA * (1 / D - 1 / Q_STAR) * (1 / D - 1 / Q_STAR) * d;
            F.x += scale_factor1 * (q.x - obstacle.x) + scale_factor2 * (q.x - q_goal.x);
            F.y += scale_factor1 * (q.y - obstacle.y) + scale_factor2 * (q.y - q_goal.y);
        }
    }
    return F;
}

// Function to calculate net force
Point net_force(Point q, std::vector<Point>& obstacles, Point q_goal) {
    Point att_force = F_att(q, q_goal);
    double d = distance(q, q_goal);
    Point rep_force = F_rep(q, obstacles, q_goal, d);
    Point net_force;
    net_force.x = att_force.x - rep_force.x;
    net_force.y = att_force.y - rep_force.y;
    return net_force;
}

Point find_equilibrium_point(Point q_goal, std::vector<Point>& obstacles, Point initial_guess) {
    const double step_size = 0.01;
    const int max_iterations = 1000;
    const double convergence_threshold = 0.001;

    for (double x = initial_guess.x - 1; x <= initial_guess.x + 1; x += step_size) {
        for (double y = initial_guess.y - 1; y <= initial_guess.y + 1; y += step_size) {
            Point current_point = {x, y};
            Point net = net_force(current_point, obstacles, q_goal);

            if (distance(net, {0, 0}) < convergence_threshold) {
                return current_point;
            }
        }
    }

    // If no equilibrium point is found, return the initial guess
    return initial_guess;
}


int main() {
    // Define goal point
    Point q_goal = {7, 7};

    // Define obstacle points
    std::vector<Point> obstacles = {{5, 5}, {7, 8}, {3, 9}};

    // Initial guess for equilibrium point
    Point initial_guess = {2, 2};

    // Find equilibrium point
    Point equilibrium_point = find_equilibrium_point(q_goal, obstacles, initial_guess);

    // Output equilibrium point
    std::cout << "Equilibrium point: (" << equilibrium_point.x << ", " << equilibrium_point.y << ")\n";

    return 0;
}
