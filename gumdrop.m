gax = geoaxes(Basemap="satellite");
latlimits = [44.3135 44.3534];
lonlimits = [-72.0227 -71.9544];
geolimits(latlimits, lonlimits);
geocenter = [mean(latlimits) mean(lonlimits) 0];
refHeight = 400;
hold on

% Load or define the predefined ROI
interactiveROI = false;
load predefinedROI.mat % Assuming this loads takeoffLat, takeoffLon, landLat, landLon, llapoints
helperPlotTakeoffROILanding(gax, takeoffLat, takeoffLon, landLat, landLon, llapoints);

if interactiveROI
    [visLimitsLat, visLimitsLon] = geolimits; 
    [takeoffLat, takeoffLon] = helperTakeoffSelectionFcn(gax, visLimitsLat, visLimitsLon);
    [llapoints, xyzpoints] = helperPolygonSelectionFcn(gax, visLimitsLat, visLimitsLon, geocenter);
    [landLat, landLon] = helperLandSelectionFcn(gax, visLimitsLat, visLimitsLon);
end

% Perform the initial decomposition of coverage
polygons = coverageDecomposition(xyzpoints(:, 1:2));

% Group polygons based on adjacency into clusters of desired size
numDrones = 2; 
numPolygons = numel(polygons);
subregionsPerDrone = floor(numPolygons / numDrones);

% Adjacency grouping with contiguous clusters
clusters = {};
visited = false(1, numPolygons); % Track visited subregions

for i = 1:numPolygons
    if ~visited(i)
        cluster = {polygons{i}};
        visited(i) = true;
        
        % Expand cluster to meet the target size of contiguous subregions
        for j = 1:numPolygons
            if ~visited(j) && numel(cluster) < subregionsPerDrone
                if any(cellfun(@(poly) checkAdjacency(poly, polygons{j}), cluster))
                    cluster{end + 1} = polygons{j};
                    visited(j) = true;
                end
            end
        end
        clusters{end + 1} = cluster; % Add cluster to clusters list
    end
end

% Adjust clusters to ensure an even distribution
finalClusters = {};
for i = 1:numDrones
    finalClusters{i} = [];
end
for i = 1:numel(clusters)
    droneIdx = mod(i - 1, numDrones) + 1;
    finalClusters{droneIdx} = [finalClusters{droneIdx}, clusters{i}];
end

% Display results for each drone
for droneIdx = 1:numDrones
    fprintf('Drone %d is assigned the following contiguous areas:\n', droneIdx);
    for clusterIdx = 1:numel(finalClusters{droneIdx})
        disp('Subregion:');
        disp(finalClusters{droneIdx}{clusterIdx});
    end
end

% Convert final polygons to geodetic coordinates for visualization
subAreasLLA = {};  
for i = 1:numDrones
    for j = 1:numel(finalClusters{i})
        currentPolygon = finalClusters{i}{j};
        
        altitude = refHeight * ones(size(currentPolygon, 1), 1);
        localENU = [currentPolygon(:, 1), currentPolygon(:, 2), altitude];
        subArea = enu2lla(localENU, geocenter, "flat");
        subAreasLLA{end + 1} = subArea(:, 1:2); 
    end
end

% Visualize the assigned areas in geodetic coordinates
cs = uavCoverageSpace(Polygons=subAreasLLA, ...
                      UnitWidth=100, ...
                      Sidelap=0, ...
                      ReferenceHeight=refHeight, ...
                      UseLocalCoordinates=false, ...
                      ReferenceLocation=geocenter);

ax = cs.show(Parent=gax, LineWidth=1.25);

% Function to check adjacency based on shared vertices
function isAdjacent = checkAdjacency(poly1, poly2)
    % Check if poly1 and poly2 share any vertices
    isAdjacent = ~isempty(intersect(poly1, poly2, 'rows'));
end

% PATHFINDING CODE ADDED HERE:
mwLS = [42.3013 -71.375 0];
latlim = [mwLS(1)-0.003 mwLS(1)+0.003];
lonlim = [mwLS(2)-0.003 mwLS(2)+0.003];

function paths = uav_coverage_planner_method(launch_point, polygon_array, sensor_coverage_width, reference_location)
    % Initialize the coverage space with all polygons
    cs = uavCoverageSpace(Polygons=polygon_array, UseLocalCoordinates=false, ReferenceLocation=reference_location);
    cs.UnitWidth = sensor_coverage_width;

    paths = cell(1, length(polygon_array));

    for i = 1:length(polygon_array)
        % Step 1: Calculate Optimal Sweep Angle
        sweep_angle = calculate_optimal_sweep_angle(polygon_array{i}, sensor_coverage_width);

        % Step 2: Set Coverage Pattern with Calculated Sweep Angle
        setCoveragePattern(cs, i, SweepAngle=sweep_angle);  % Set the sweep angle specifically for each polygon
    end

    % Step 3: Initialize the UAV coverage planner with coverage space
    cp = uavCoveragePlanner(cs, Solver="Exhaustive");

    % Step 4: Plan the path from the launch point to cover all polygons
    [waypoints, solnInfo] = plan(cp, [launch_point(1), launch_point(2), 0]);

    % Step 5: Store the generated waypoints and solution info
    paths = struct('waypoints', waypoints, 'solution_info', solnInfo);
end

function sweep_angle = calculate_optimal_sweep_angle(polygon, sensor_coverage_width)
    % Get the centroid of the polygon for orientation calculation
    centroid = mean(polygon, 1);
    centroid_3d = [centroid, 0];

    % Use Principal Component Analysis (PCA) to determine the major axis orientation
    centered_polygon = polygon - centroid;
    [~, ~, V] = svd(centered_polygon);

    % Major axis orientation
    angle_major_axis = atan2d(V(2, 1), V(1, 1));

    % Alternative angles for comparison
    alternative_angles = [angle_major_axis, mod(angle_major_axis + 90, 180)];
    
    % Initialize minimum distance and best angle
    min_distance = inf;
    best_angle = angle_major_axis;

    % Evaluate the distance for each angle and select the minimum
    for angle = alternative_angles
        % Update coverage pattern with the current angle
        cs_temp = uavCoverageSpace(Polygons={polygon}, UseLocalCoordinates=false, ReferenceLocation=centroid_3d);
        cs_temp.UnitWidth = sensor_coverage_width;
        setCoveragePattern(cs_temp, 1, SweepAngle=angle);

        % Plan dummy path to get the coverage distance
        cp_temp = uavCoveragePlanner(cs_temp, Solver="Exhaustive");
        [~, solnInfo] = plan(cp_temp, [centroid_3d]); % Assume takeoff at centroid with altitude 0
        
        % If curr angle is smaller, update angle
        if solnInfo.DistanceCost < min_distance
            min_distance = solnInfo.DistanceCost;
            best_angle = angle;
        end
    end

    % Return the angle that minimized the path distance
    fprintf("sweep angle: %d \n", rad2deg(best_angle))
    sweep_angle = best_angle;
end

% USE CASE
launch_point = [42.30089, -71.3752];
polygon_array = {... 
    [42.3028, -71.37527; 42.30325, -71.37442; 42.3027, -71.3736; 42.3017, -71.37378; 42.3019, -71.375234], ...
    [42.30035, -71.3762; 42.2999, -71.3734; 42.2996, -71.37376; 42.2999, -71.37589]
};
sensor_coverage_width = 20;
reference_location = [42.3013 -71.375 0];

% Call the function to generate paths for each polygon
paths = uav_coverage_planner_method(launch_point, polygon_array, sensor_coverage_width, reference_location);

% Visualization
fig = figure;
g = geoaxes(fig, Basemap="satellite");
geolimits([42.297, 42.305], [-71.378, -71.372])
geoplot(launch_point(1), launch_point(2), 'ro', 'MarkerSize', 10, 'DisplayName', 'Launch Point');
hold on;
for i = 1:length(paths)
    geoplot(paths.waypoints(:,1), paths.waypoints(:,2), 'LineWidth', 1.5, 'DisplayName', ['Path ' num2str(i)]);
end
legend;
hold off;
