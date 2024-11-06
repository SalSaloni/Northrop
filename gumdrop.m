% Main script
gax = geoaxes(Basemap="satellite");
latlimits = [44.3135 44.3534];
lonlimits = [-72.0227 -71.9544];
geolimits(latlimits, lonlimits);
geocenter = [mean(latlimits) mean(lonlimits) 0];
refHeight = 400;
hold on

% Set the number of UAVs
numDrones = 1;  % Adjust the number of UAVs

% Define distinct colors for each UAV path
colors = lines(numDrones); % Generates a colormap with `numDrones` distinct colors

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

% Define the grid parameters based on the bounding box of the convex polygon
gridSizeX = 6;  % Increase the number of grid cells to get finer coverage
gridSizeY = 6;

% Get bounding box of the region
xMin = min(xyzpoints(:,1));
xMax = max(xyzpoints(:,1));
yMin = min(xyzpoints(:,2));
yMax = max(xyzpoints(:,2));
xGrid = linspace(xMin, xMax, gridSizeX + 1);
yGrid = linspace(yMin, yMax, gridSizeY + 1);

% Create a polyshape for the main convex polygon for intersection checks
mainPoly = polyshape(xyzpoints(:,1), xyzpoints(:,2));

% Generate subregions as polygons within the grid that overlap with the main region
subPolygons = {};
for i = 1:gridSizeX
    for j = 1:gridSizeY
        % Define vertices for each grid cell
        subPolygonVertices = [
            xGrid(i), yGrid(j);
            xGrid(i+1), yGrid(j);
            xGrid(i+1), yGrid(j+1);
            xGrid(i), yGrid(j+1)
        ];
        
        % Convert the grid cell to a polyshape and check intersection with the main polygon
        subPolygonShape = polyshape(subPolygonVertices(:,1), subPolygonVertices(:,2));
        
        % Only include subregions that intersect with the main polygon
        if overlaps(mainPoly, subPolygonShape)
            intersectionShape = intersect(mainPoly, subPolygonShape);
            subPolygons{end+1} = intersectionShape.Vertices; % Store the vertices of the intersecting area
        end
    end
end

% Assign subregions to each UAV in a balanced way
finalClusters = cell(1, numDrones);
for i = 1:numel(subPolygons)
    droneIdx = mod(i - 1, numDrones) + 1;
    finalClusters{droneIdx}{end + 1} = subPolygons{i};
end

% Convert final polygons to geodetic coordinates and plan paths
coverageWidth = 100;

for droneIdx = 1:numDrones
    regions = finalClusters{droneIdx};  % Get the assigned subregions for the current UAV

    coveragePolygons = {};  % Reset for each UAV
    for i = 1:numel(regions)
        currentPolygon = regions{i};
        
        altitude = refHeight * ones(size(currentPolygon, 1), 1);
        localENU = [currentPolygon(:, 1), currentPolygon(:, 2), altitude];
        subArea = enu2lla(localENU, geocenter, "flat");

        if ~isempty(subArea)
            coveragePolygons{end + 1} = subArea(:, 1:2);
        end
    end

    if ~isempty(coveragePolygons)
        fprintf('Creating coverage space for UAV %d with %d polygons\n', droneIdx, numel(coveragePolygons));

        cs = uavCoverageSpace(Polygons=coveragePolygons, ...
                              UseLocalCoordinates=false, ...
                              ReferenceLocation=geocenter, ...
                              ReferenceHeight=refHeight);
        cs.UnitWidth = coverageWidth;

        % Suppress the automatic legend entries by not calling `show(cs, Parent=gax);`
        % show(cs, Parent=gax); 

        cp = uavCoveragePlanner(cs, Solver="MinTraversal"); % Use "MinTraversal" solver

        takeoff = [44.3150, -72.0100, 0];

        [waypoints, soln] = plan(cp, takeoff);

        % Plot the planned path on the map with unique color and label
        geoplot(gax, waypoints(:,1), waypoints(:,2), '-o', 'LineWidth', 1.5, 'Color', colors(droneIdx, :), ...
                'DisplayName', sprintf("UAV %d Path", droneIdx));
        geoplot(gax, takeoff(1), takeoff(2), 'p', 'MarkerSize', 8, 'MarkerFaceColor', colors(droneIdx, :), ...
                'DisplayName', sprintf("UAV %d Takeoff/Landing", droneIdx));
    else
        fprintf('No valid polygons assigned to UAV %d\n', droneIdx);
    end
end

% Manually specify legend entries for only the UAV paths and takeoff/landing points


hold off
