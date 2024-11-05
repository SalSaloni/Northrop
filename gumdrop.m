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
function paths = uav_coverage_planner_method(launch_point, polygon_array, sensor_coverage_width)
    % Initialize paths cell array to store paths and solution info for each polygon
    paths = cell(1, length(polygon_array));
    
    % Create coverage space object with all polygons in polygon_array
    cs = uavCoverageSpace(Polygons=polygon_array, UseLocalCoordinates=false, ReferenceHeight=400);  % Reference height is set for demonstration
    
    % Set unit width for the entire coverage space
    cs.UnitWidth = sensor_coverage_width;

    for i = 1:length(polygon_array)
        % Step 1: Calculate Optimal Sweep Angle
        sweep_angle = calculate_sweep_angle(polygon_array{i}, sensor_coverage_width);
        fprintf("sweep angle %d: %d", i, rad2deg(sweep_angle))
        % Step 2: Set Coverage Pattern for each polygon with the calculated sweep angle
        setCoveragePattern(cs, i, 'SweepAngle', sweep_angle);
    end

    % Step 3: Initialize the UAV coverage planner with the configured coverage space
    cp = uavCoveragePlanner(cs, Solver="Exhaustive");

    % Step 4: Plan the path from launch point
    [waypoints, solnInfo] = plan(cp, [launch_point(1), launch_point(2), 0]);

    % Step 5: Store the waypoints and solution info
    paths{1} = struct('waypoints', waypoints, 'solution_info', solnInfo);
end

function sweep_angle = calculate_sweep_angle(polygon, sensor_coverage_width)
    % Get the longest diagonal in the polygon
    longest_diagonal = find_longest_diagonal(polygon);

    % Calculate the angle along this diagonal
    x1 = longest_diagonal(1); y1 = longest_diagonal(2);
    x2 = longest_diagonal(3); y2 = longest_diagonal(4);
    sweep_angle = atan2d(y2 - y1, x2 - x1);  % Angle in degrees
end

function longest_diagonal = find_longest_diagonal(polygon)
    % Helper function to find the longest diagonal in the polygon
    max_distance = 0;
    longest_diagonal = [];

    % Loop over each pair of vertices in the polygon
    for i = 1:size(polygon, 1)
        for j = i+1:size(polygon, 1)
            % Calculate Euclidean distance between points
            distance = sqrt((polygon(i,1) - polygon(j,1))^2 + (polygon(i,2) - polygon(j,2))^2);
            if distance > max_distance
                max_distance = distance;
                longest_diagonal = [polygon(i,1), polygon(i,2), polygon(j,1), polygon(j,2)];
            end
        end
    end
end


%USE CASE
launch_point = [42.30089, -71.3752];  % Example launch point
polygon_array = {... 
    [42.3028, -71.37527; 42.30325, -71.37442; 42.3027, -71.3736; 42.3017, -71.37378; 42.3019, -71.375234], ...
    [42.30035, -71.3762; 42.2999, -71.3734; 42.2996, -71.37376; 42.2999, -71.37589]
};
sensor_coverage_width = 20;  % Sensor coverage width in meters

% Call the function to generate paths for each polygon
paths = uav_coverage_planner_method(launch_point, polygon_array, sensor_coverage_width);

% Visualization (optional, if required for your project)
mwLS = [42.3013 -71.375 0];
latlim = [mwLS(1)-0.003 mwLS(1)+0.003];
lonlim = [mwLS(2)-0.003 mwLS(2)+0.003];
fig = figure;
g = geoaxes(fig,Basemap="satellite");
geolimits(latlim,lonlim)
geoplot(launch_point(1), launch_point(2), 'ro', 'MarkerSize', 10, 'DisplayName', 'Launch Point');
hold on;
for i = 1:length(paths)
    geoplot(paths{i}.waypoints(:,1), paths{i}.waypoints(:,2), 'LineWidth', 1.5, 'DisplayName', ['Path ' num2str(i)]);
end
legend;
hold off;

