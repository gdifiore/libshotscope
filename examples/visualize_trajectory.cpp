/**
 * @file visualize_trajectory.cpp
 * @brief Generate an interactive HTML visualization of golf ball flight physics
 *
 * This program simulates a full golf shot including aerial flight and bounces,
 * then generates an HTML file with an interactive 3D visualization.
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <sstream>
#include <cmath>
#include "BallState.hpp"
#include "FlightPhase.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "ground_surface.hpp"
#include "physics_constants.hpp"

struct TrajectoryPoint
{
	float time;
	float x, y, z;
	float vx, vy, vz;
	std::string phase;
};

std::string generateHTML(const std::vector<TrajectoryPoint>& trajectory, const golfBall& ball)
{
	// Convert trajectory to JSON
	std::ostringstream jsonData;
	jsonData << std::fixed << std::setprecision(2);
	jsonData << "[\n";
	for (size_t i = 0; i < trajectory.size(); ++i)
	{
		const auto& pt = trajectory[i];
		jsonData << "  {\"t\":" << pt.time
		         << ",\"x\":" << pt.x
		         << ",\"y\":" << pt.y
		         << ",\"z\":" << pt.z
		         << ",\"vx\":" << pt.vx
		         << ",\"vy\":" << pt.vy
		         << ",\"vz\":" << pt.vz
		         << ",\"phase\":\"" << pt.phase << "\"}";
		if (i < trajectory.size() - 1) jsonData << ",";
		jsonData << "\n";
	}
	jsonData << "]";

	// Initial conditions for display
	std::ostringstream initialConditions;
	initialConditions << std::fixed << std::setprecision(1);
	initialConditions << "Ball speed: " << ball.exitSpeed << " mph<br>"
	                  << "Launch angle: " << ball.launchAngle << "°<br>"
	                  << "Spin rate: " << ball.backspin << " rpm";

	// Generate complete HTML
	return R"HTML(<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Golf Ball Flight Visualizer</title>
    <style>
        body {
            margin: 0;
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            display: flex;
            flex-direction: column;
            height: 100vh;
        }
        #header {
            background: rgba(255, 255, 255, 0.95);
            padding: 20px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            margin: 0 0 10px 0;
            color: #333;
            font-size: 24px;
        }
        #info {
            color: #666;
            font-size: 14px;
        }
        #container {
            flex: 1;
            display: flex;
            gap: 20px;
            padding: 20px;
            overflow: hidden;
        }
        #canvas-container {
            flex: 2;
            background: white;
            border-radius: 10px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.2);
            position: relative;
            overflow: hidden;
        }
        canvas {
            display: block;
            width: 100%;
            height: 100%;
        }
        #controls {
            flex: 0 0 300px;
            background: rgba(255, 255, 255, 0.95);
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.2);
            overflow-y: auto;
        }
        .control-group {
            margin-bottom: 20px;
        }
        .control-group label {
            display: block;
            margin-bottom: 5px;
            color: #333;
            font-weight: 500;
            font-size: 14px;
        }
        .control-group input[type="range"] {
            width: 100%;
        }
        .control-group .value {
            color: #667eea;
            font-weight: bold;
        }
        button {
            width: 100%;
            padding: 10px;
            margin: 5px 0;
            border: none;
            border-radius: 5px;
            background: #667eea;
            color: white;
            font-size: 14px;
            font-weight: 500;
            cursor: pointer;
            transition: background 0.3s;
        }
        button:hover {
            background: #5568d3;
        }
        .stats {
            background: #f5f5f5;
            padding: 15px;
            border-radius: 5px;
            margin-top: 20px;
        }
        .stat-row {
            display: flex;
            justify-content: space-between;
            margin: 5px 0;
            font-size: 13px;
        }
        .stat-label {
            color: #666;
        }
        .stat-value {
            color: #333;
            font-weight: bold;
        }
        .view-button {
            background: #764ba2;
        }
        .view-button:hover {
            background: #653a8a;
        }
    </style>
</head>
<body>
    <div id="header">
        <h1>Golf Ball Flight Physics Visualizer</h1>
        <div id="info">)HTML" + initialConditions.str() + R"HTML(</div>
    </div>
    <div id="container">
        <div id="canvas-container">
            <canvas id="canvas"></canvas>
        </div>
        <div id="controls">
            <div class="control-group">
                <label>View Angle: <span class="value" id="angleValue">30°</span></label>
                <input type="range" id="viewAngle" min="0" max="90" value="30">
            </div>
            <div class="control-group">
                <label>View Rotation: <span class="value" id="rotationValue">45°</span></label>
                <input type="range" id="viewRotation" min="0" max="360" value="45">
            </div>
            <div class="control-group">
                <label>Zoom: <span class="value" id="zoomValue">1.0x</span></label>
                <input type="range" id="zoom" min="0.5" max="3" step="0.1" value="1.0">
            </div>
            <button class="view-button" onclick="setView('side')">Side View</button>
            <button class="view-button" onclick="setView('top')">Top View</button>
            <button class="view-button" onclick="setView('3d')">3D View</button>
            <button onclick="toggleAnimation()">Toggle Animation</button>

            <div class="stats">
                <h3 style="margin-top: 0; color: #333;">Trajectory Statistics</h3>
                <div class="stat-row">
                    <span class="stat-label">Total Distance:</span>
                    <span class="stat-value" id="totalDistance">-</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Max Height:</span>
                    <span class="stat-value" id="maxHeight">-</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Flight Time:</span>
                    <span class="stat-value" id="flightTime">-</span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Bounces:</span>
                    <span class="stat-value" id="bounceCount">-</span>
                </div>
            </div>
        </div>
    </div>

    <script>
        const trajectoryData = )HTML" + jsonData.str() + R"HTML(;

        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');

        let viewAngle = 30;
        let viewRotation = 45;
        let zoomLevel = 1.0;
        let animating = false;
        let animationFrame = 0;

        // Calculate statistics
        let maxHeight = 0;
        let totalDistance = 0;
        let bounceCount = 0;
        let lastPhase = '';

        trajectoryData.forEach((pt, i) => {
            maxHeight = Math.max(maxHeight, pt.z);
            if (i === trajectoryData.length - 1) {
                totalDistance = Math.sqrt(pt.x * pt.x + pt.y * pt.y);
            }
            if (pt.phase === 'bounce' && lastPhase !== 'bounce') {
                bounceCount++;
            }
            lastPhase = pt.phase;
        });

        const flightTime = trajectoryData[trajectoryData.length - 1].t;

        document.getElementById('totalDistance').textContent = totalDistance.toFixed(1) + ' yd';
        document.getElementById('maxHeight').textContent = maxHeight.toFixed(1) + ' ft';
        document.getElementById('flightTime').textContent = flightTime.toFixed(2) + ' s';
        document.getElementById('bounceCount').textContent = bounceCount;

        function resizeCanvas() {
            const container = canvas.parentElement;
            canvas.width = container.clientWidth;
            canvas.height = container.clientHeight;
            draw();
        }

        function project3D(x, y, z) {
            const angleRad = viewAngle * Math.PI / 180;
            const rotRad = viewRotation * Math.PI / 180;

            // Rotate around Z axis
            const xr = x * Math.cos(rotRad) - y * Math.sin(rotRad);
            const yr = x * Math.sin(rotRad) + y * Math.cos(rotRad);

            // Project to 2D
            const scale = 3 * zoomLevel;
            const px = canvas.width / 2 + xr * scale;
            const py = canvas.height / 2 - (z * Math.cos(angleRad) + yr * Math.sin(angleRad)) * scale;

            return { x: px, y: py };
        }

        function draw() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // Draw ground plane
            ctx.fillStyle = '#e8f5e9';
            ctx.fillRect(0, canvas.height * 0.7, canvas.width, canvas.height * 0.3);

            // Draw grid
            ctx.strokeStyle = '#c8e6c9';
            ctx.lineWidth = 1;
            for (let i = -200; i <= 200; i += 20) {
                const p1 = project3D(i, -200, 0);
                const p2 = project3D(i, 200, 0);
                ctx.beginPath();
                ctx.moveTo(p1.x, p1.y);
                ctx.lineTo(p2.x, p2.y);
                ctx.stroke();

                const p3 = project3D(-200, i, 0);
                const p4 = project3D(200, i, 0);
                ctx.beginPath();
                ctx.moveTo(p3.x, p3.y);
                ctx.lineTo(p4.x, p4.y);
                ctx.stroke();
            }

            // Draw trajectory
            const endFrame = animating ? animationFrame : trajectoryData.length;

            // Draw path
            ctx.beginPath();
            ctx.strokeStyle = '#1976d2';
            ctx.lineWidth = 2;

            for (let i = 0; i < endFrame; i++) {
                const pt = trajectoryData[i];
                const projected = project3D(pt.x, pt.y, pt.z);

                if (i === 0) {
                    ctx.moveTo(projected.x, projected.y);
                } else {
                    ctx.lineTo(projected.x, projected.y);
                }
            }
            ctx.stroke();

            // Draw bounce markers
            let lastWasBounce = false;
            for (let i = 0; i < endFrame; i++) {
                const pt = trajectoryData[i];
                if (pt.phase === 'bounce' && !lastWasBounce) {
                    const projected = project3D(pt.x, pt.y, pt.z);
                    ctx.fillStyle = '#ff5722';
                    ctx.beginPath();
                    ctx.arc(projected.x, projected.y, 5, 0, Math.PI * 2);
                    ctx.fill();

                    // Draw vertical line to ground
                    const groundProj = project3D(pt.x, pt.y, 0);
                    ctx.strokeStyle = '#ff5722';
                    ctx.lineWidth = 1;
                    ctx.setLineDash([3, 3]);
                    ctx.beginPath();
                    ctx.moveTo(projected.x, projected.y);
                    ctx.lineTo(groundProj.x, groundProj.y);
                    ctx.stroke();
                    ctx.setLineDash([]);
                }
                lastWasBounce = (pt.phase === 'bounce');
            }

            // Draw current position if animating
            if (animating && endFrame > 0) {
                const pt = trajectoryData[endFrame - 1];
                const projected = project3D(pt.x, pt.y, pt.z);

                // Ball
                ctx.fillStyle = '#ffffff';
                ctx.strokeStyle = '#333';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.arc(projected.x, projected.y, 8, 0, Math.PI * 2);
                ctx.fill();
                ctx.stroke();

                // Shadow on ground
                const shadowProj = project3D(pt.x, pt.y, 0);
                ctx.fillStyle = 'rgba(0,0,0,0.2)';
                ctx.beginPath();
                ctx.arc(shadowProj.x, shadowProj.y, 6, 0, Math.PI * 2);
                ctx.fill();
            }

            // Draw start and end markers
            if (!animating || endFrame === trajectoryData.length) {
                const start = trajectoryData[0];
                const end = trajectoryData[trajectoryData.length - 1];

                const startProj = project3D(start.x, start.y, start.z);
                ctx.fillStyle = '#4caf50';
                ctx.beginPath();
                ctx.arc(startProj.x, startProj.y, 6, 0, Math.PI * 2);
                ctx.fill();

                const endProj = project3D(end.x, end.y, end.z);
                ctx.fillStyle = '#f44336';
                ctx.beginPath();
                ctx.arc(endProj.x, endProj.y, 6, 0, Math.PI * 2);
                ctx.fill();
            }
        }

        function animate() {
            if (!animating) return;

            animationFrame++;
            if (animationFrame >= trajectoryData.length) {
                animationFrame = 0;
            }

            draw();
            setTimeout(() => requestAnimationFrame(animate), 16);
        }

        function toggleAnimation() {
            animating = !animating;
            if (animating) {
                animationFrame = 0;
                animate();
            } else {
                draw();
            }
        }

        function setView(type) {
            if (type === 'side') {
                viewAngle = 0;
                viewRotation = 90;
                document.getElementById('viewAngle').value = 0;
                document.getElementById('viewRotation').value = 90;
            } else if (type === 'top') {
                viewAngle = 89;
                viewRotation = 0;
                document.getElementById('viewAngle').value = 89;
                document.getElementById('viewRotation').value = 0;
            } else if (type === '3d') {
                viewAngle = 30;
                viewRotation = 45;
                document.getElementById('viewAngle').value = 30;
                document.getElementById('viewRotation').value = 45;
            }
            updateLabels();
            draw();
        }

        function updateLabels() {
            document.getElementById('angleValue').textContent = viewAngle + '°';
            document.getElementById('rotationValue').textContent = viewRotation + '°';
            document.getElementById('zoomValue').textContent = zoomLevel.toFixed(1) + 'x';
        }

        document.getElementById('viewAngle').addEventListener('input', (e) => {
            viewAngle = parseFloat(e.target.value);
            updateLabels();
            draw();
        });

        document.getElementById('viewRotation').addEventListener('input', (e) => {
            viewRotation = parseFloat(e.target.value);
            updateLabels();
            draw();
        });

        document.getElementById('zoom').addEventListener('input', (e) => {
            zoomLevel = parseFloat(e.target.value);
            updateLabels();
            draw();
        });

        window.addEventListener('resize', resizeCanvas);
        resizeCanvas();
    </script>
</body>
</html>)HTML";
}

int main()
{
	std::cout << "=== Golf Ball Flight Visualizer ===\n\n";

	// Setup ball with realistic shot parameters - 7 iron shot with slice
	const golfBall ball{
		0.0,    // x0
		0.0,    // y0
		0.0,    // z0
		135.0,  // exitSpeed (mph) - typical 7 iron ball speed
		15.0,   // launchAngle (degrees)
		20.0,   // direction (degrees) - aiming 20° to the right
		5500.0, // backspin (rpm)
		500.0   // sidespin (rpm) - slice spin
	};

	const atmosphericData atmos{
		70.0,  // temperature (F)
		0.0,   // wind x
		0.0,   // wind y
		0.0,   // wind z
		0.0,   // wind direction
		50.0,  // humidity
		29.92  // pressure
	};

	// Fairway ground conditions
	GroundSurface ground;
	ground.height = 0.0F;
	ground.restitution = 0.35F;    // Fairway bounces
	ground.frictionStatic = 0.6F;  // Moderate friction
	ground.firmness = 0.7F;        // Firm fairway

	std::cout << "Simulating 7-iron shot with slice:\n";
	std::cout << "  Ball speed: " << ball.exitSpeed << " mph\n";
	std::cout << "  Launch angle: " << ball.launchAngle << "°\n";
	std::cout << "  Direction: " << ball.direction << "°\n";
	std::cout << "  Backspin: " << ball.backspin << " rpm\n";
	std::cout << "  Sidespin: " << ball.sidespin << " rpm\n\n";

	// Initialize physics
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	AerialPhase aerialPhase(physicsVars, ball, atmos);
	BouncePhase bouncePhase(physicsVars, ball, atmos, ground);

	// Initialize state
	BallState state;
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	const float theta_rad = ball.launchAngle * M_PI / 180.0F;
	const float phi_rad = ball.direction * M_PI / 180.0F;

	state.position[0] = 0.0F;
	state.position[1] = 0.0F;
	state.position[2] = 0.0F;
	state.velocity[0] = v0_fps * std::cos(theta_rad) * std::cos(phi_rad);
	state.velocity[1] = v0_fps * std::cos(theta_rad) * std::sin(phi_rad);
	state.velocity[2] = v0_fps * std::sin(theta_rad);
	state.acceleration[0] = 0.0F;
	state.acceleration[1] = 0.0F;
	state.acceleration[2] = -physics_constants::GRAVITY_FT_PER_S2;
	state.currentTime = 0.0F;

	aerialPhase.initialize(state);

	// Simulation parameters
	const float dt = 0.01F;
	const int maxSteps = 1000;
	std::vector<TrajectoryPoint> trajectory;

	FlightPhase* currentPhase = &aerialPhase;
	std::string phaseName = "aerial";

	std::cout << "Running simulation...\n";

	for (int step = 0; step < maxSteps; ++step)
	{
		// Record trajectory point
		TrajectoryPoint pt;
		pt.time = state.currentTime;
		pt.x = state.position[0] / physics_constants::YARDS_TO_FEET;
		pt.y = state.position[1] / physics_constants::YARDS_TO_FEET;
		pt.z = state.position[2];
		pt.vx = state.velocity[0];
		pt.vy = state.velocity[1];
		pt.vz = state.velocity[2];
		pt.phase = phaseName;
		trajectory.push_back(pt);

		// Execute physics step
		currentPhase->calculateStep(state, dt);

		// Check for phase transitions
		if (currentPhase->isPhaseComplete(state))
		{
			if (phaseName == "aerial")
			{
				currentPhase = &bouncePhase;
				phaseName = "bounce";
				std::cout << "  Transitioned to bounce phase at t=" << state.currentTime << "s\n";
			}
			else if (phaseName == "bounce")
			{
				std::cout << "  Ball settled at t=" << state.currentTime << "s\n";
				break;
			}
		}
	}

	std::cout << "Simulation complete. Recorded " << trajectory.size() << " points.\n";

	// Calculate final statistics
	float maxHeight = 0.0F;
	for (const auto& pt : trajectory)
	{
		maxHeight = std::max(maxHeight, pt.z);
	}
	float totalDistance = sqrt(
		trajectory.back().x * trajectory.back().x +
		trajectory.back().y * trajectory.back().y
	);

	std::cout << "\nFlight Statistics:\n";
	std::cout << "  Carry distance: " << totalDistance << " yards\n";
	std::cout << "  Max height: " << maxHeight << " feet\n";
	std::cout << "  Flight time: " << trajectory.back().time << " seconds\n";

	// Generate HTML file
	std::cout << "\nGenerating visualization...\n";
	std::string html = generateHTML(trajectory, ball);

	std::ofstream outFile("trajectory_visualization.html");
	if (outFile.is_open())
	{
		outFile << html;
		outFile.close();
		std::cout << "Visualization saved to: trajectory_visualization.html\n";
		std::cout << "Open this file in a web browser to view the interactive visualization.\n";
	}
	else
	{
		std::cerr << "Error: Could not create output file.\n";
		return 1;
	}

	return 0;
}
