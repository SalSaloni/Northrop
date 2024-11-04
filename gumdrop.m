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
