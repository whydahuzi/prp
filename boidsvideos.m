figure('Position', [500 200 1100 1000])
grid on;

close all
clear
clc

% Initialize boids
numBoids = 50;
positions_ground = rand(numBoids, 2) * 50; % 50x50 area for ground boids
positions_ground(:, 3) = 0; % Z-axis for ground boids
velocities_ground = randn(numBoids, 3) / 10;

positions_sky = rand(numBoids, 2) * 50; % 50x50 area for sky boids
positions_sky(:, 3) = 10; % Z-axis for sky boids
velocities_sky = randn(numBoids, 3) / 10;

visibleRange = 5; % Boids within 5 units are considered neighbors
T = 1000;
borderBuffer = 10;
minVelocity = 0.5;
maxVelocity = 1.5;
separationDistance = 10;
borderDistance = 30; % Adjust as needed
borderFollowingFactor = 0.1; % Adjust the strength of border following
maxHeadingChange = 10; % Adjust the threshold for heading change

% Set up the video writer object
videoFile = 'boids_simulation.mp4';
videoWriter = VideoWriter(videoFile, 'MPEG-4');
videoWriter.FrameRate = 30; % Set the frame rate

open(videoWriter); % Open the video writer object

while 1
    for i = 1:numBoids
        % Ground Boids
        v1_ground = alignment(positions_ground, velocities_ground, i, visibleRange);
        v2_ground = cohesion(positions_ground, i, visibleRange); % Cohesion
        v3_ground = separation(positions_ground, i, separationDistance); % Separation
        v4_ground = borderFollowing(positions_ground, i, borderDistance, borderFollowingFactor);
        v5_ground = smoothHeading(velocities_ground, i, maxHeadingChange);

        velocities_ground(i, :) = velocities_ground(i, :) + v1_ground + v2_ground + v3_ground + v4_ground + v5_ground;
        if norm(velocities_ground(i, :)) < minVelocity
            velocities_ground(i, :) = (velocities_ground(i, :) / norm(velocities_ground(i, :))) * minVelocity;
        end
        if norm(velocities_ground(i, :)) > maxVelocity
            velocities_ground(i, :) = (velocities_ground(i, :) / norm(velocities_ground(i, :))) * maxVelocity;
        end
        positions_ground(i, :) = positions_ground(i, :) + velocities_ground(i, :);
        positions_ground(i, 3) = 0; % Fix z-axis for ground boids

        % Sky Boids
        v1_sky = alignment(positions_sky, velocities_sky, i, visibleRange);
        v2_sky = cohesion(positions_sky, i, visibleRange);
        v3_sky = separation(positions_sky, i, separationDistance);
        v4_sky = borderFollowing(positions_sky, i, borderDistance, borderFollowingFactor);
        v5_sky = smoothHeading(velocities_sky, i, maxHeadingChange);

        velocities_sky(i, :) = velocities_sky(i, :) + v1_sky + v2_sky + v3_sky + v4_sky + v5_sky;
        if norm(velocities_sky(i, :)) < minVelocity
            velocities_sky(i, :) = (velocities_sky(i, :) / norm(velocities_sky(i, :))) * minVelocity;
        end
        if norm(velocities_sky(i, :)) > maxVelocity
            velocities_sky(i, :) = (velocities_sky(i, :) / norm(velocities_sky(i, :))) * maxVelocity;
        end
        positions_sky(i, :) = positions_sky(i, :) + velocities_sky(i, :);
        positions_sky(i, 3) = 10; % Fix z-axis for sky boids

        % Add mutual influence between ground and sky boids
        mutualInfluenceFactor = 0.01; % Adjust the strength of mutual influence
        velocities_ground(i, :) = velocities_ground(i, :) + mutualInfluenceFactor * (positions_sky(i, :) - positions_ground(i, :));
        velocities_sky(i, :) = velocities_sky(i, :) + mutualInfluenceFactor * (positions_ground(i, :) - positions_sky(i, :));

        % Border avoidance for ground boids
        if positions_ground(i, 1) < borderBuffer
            velocities_ground(i, 1) = abs(velocities_ground(i, 1)); % Turn right
        elseif positions_ground(i, 1) > 50 - borderBuffer
            velocities_ground(i, 1) = -abs(velocities_ground(i, 1)); % Turn left
        end
        if positions_ground(i, 2) < borderBuffer
            velocities_ground(i, 2) = abs(velocities_ground(i, 2)); % Turn up
        elseif positions_ground(i, 2) > 50 - borderBuffer
            velocities_ground(i, 2) = -abs(velocities_ground(i, 2)); % Turn down
        end

        % Border avoidance for sky boids
        if positions_sky(i, 1) < borderBuffer
            velocities_sky(i, 1) = abs(velocities_sky(i, 1)); % Turn right
        elseif positions_sky(i, 1) > 50 - borderBuffer
            velocities_sky(i, 1) = -abs(velocities_sky(i, 1)); % Turn left
        end
        if positions_sky(i, 2) < borderBuffer
            velocities_sky(i, 2) = abs(velocities_sky(i, 2)); % Turn up
        elseif positions_sky(i, 2) > 50 - borderBuffer
            velocities_sky(i, 2) = -abs(velocities_sky(i, 2)); % Turn down
        end
    end

    % Visualization
    plot3(positions_ground(:, 1), positions_ground(:, 2), positions_ground(:, 3), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b'); % Ground boids
    hold on;
    plot3(positions_sky(:, 1), positions_sky(:, 2), positions_sky(:, 3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % Sky boids
    hold off;

    xlim([0 50]);
    ylim([0 50]);
    zlim([0 20]);

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Boids Simulation: Ground (Blue) and Sky (Red)');
    grid on;
    drawnow;

    % Capture the plot as a frame and write it to the video
    frame = getframe(gcf);
    writeVideo(videoWriter, frame);

    % Pause for a short time to slow down the simulation
    pause(0.01);
end

% Close the video writer object
close(videoWriter);

%% Helper Functions
function v1 = alignment(positions, velocities, i, visibleRange)
    neighbors = findNeighbors(positions, i, visibleRange);
    if isempty(neighbors)
        v1 = [0, 0, 0];
    else
        avgVelocity = mean(velocities(neighbors, :), 1);
        v1 = (avgVelocity - velocities(i, :)) * 0.1;
    end
end

function v2 = cohesion(positions, i, visibleRange)
    neighbors = findNeighbors(positions, i, visibleRange);
    if isempty(neighbors)
        v2 = [0, 0, 0];
    else
        avgPosition = mean(positions(neighbors, :), 1);
        v2 = (avgPosition - positions(i, :)) * 0.01;
    end
end

function v3 = separation(positions, i, separationDistance)
    neighbors = findNeighbors(positions, i, separationDistance);
    if isempty(neighbors)
        v3 = [0, 0, 0];
    else
        v3 = [0, 0, 0];
        for neighbor = neighbors'
            distance = positions(i, :) - positions(neighbor, :);
            if norm(distance) < separationDistance
                v3 = v3 + (positions(i, :) - positions(neighbor, :));
            end
        end
        v3 = v3 * 0.1;
    end
end

function v4 = borderFollowing(positions, i, borderDistance, borderFollowingFactor)
    v4 = [0, 0, 0];
    if positions(i, 1) < borderDistance
        v4(1) = borderFollowingFactor * (borderDistance - positions(i, 1));
    elseif positions(i, 1) > 50 - borderDistance
        v4(1) = borderFollowingFactor * (50 - borderDistance - positions(i, 1));
    end
    if positions(i, 2) < borderDistance
        v4(2) = borderFollowingFactor * (borderDistance - positions(i, 2));
    elseif positions(i, 2) > 50 - borderDistance
        v4(2) = borderFollowingFactor * (50 - borderDistance - positions(i, 2));
    end
end

function v5 = smoothHeading(velocities, i, maxHeadingChange)
    heading = atan2(velocities(i, 2), velocities(i, 1));
    desiredHeading = atan2(mean(velocities(:, 2)), mean(velocities(:, 1)));
    headingChange = desiredHeading - heading;
    if abs(headingChange) > maxHeadingChange
        headingChange = sign(headingChange) * maxHeadingChange;
    end
    newHeading = heading + headingChange;
    speed = norm(velocities(i, :));
    v5 = [cos(newHeading), sin(newHeading), 0] * speed - velocities(i, :);
end

function neighbors = findNeighbors(positions, i, range)
    distances = sqrt(sum((positions - positions(i, :)).^2, 2));
    neighbors = find(distances < range & distances > 0);
end