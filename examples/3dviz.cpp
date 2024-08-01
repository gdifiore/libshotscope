#include "FlightVisualizer.hpp"
#include "Simulator.hpp"
#include "golf_ball.hpp"
#include "atmosphere.hpp"
#include "math_utils.hpp"

#include <iostream>


int main() {
    const golfBall ball{0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
    const atmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

    GolfBallPhysicsVariables physVars(ball, atmos);
    GolfBallFlight flight(physVars, ball, atmos);

    Simulator simulator(physVars, flight);

    std::vector<Vector3D> trajectory = simulator.runSimulation();

    std::cout << "Trajectory size: " << trajectory.size() << std::endl;

    try {
        FlightVisualizer visualizer(800, 600, "3D Trajectory");
        printf("setting trajectory\n");
        visualizer.setTrajectory(trajectory);
        printf("run\n");
        visualizer.run();
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}