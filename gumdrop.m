gax = geoaxes(Basemap="satellite");
latlimits = [44.3135 44.3534];
lonlimits = [-72.0227 -71.9544];
geolimits(latlimits,lonlimits);
geocenter = [mean(latlimits) mean(lonlimits) 0];
refHeight = 400;
hold on

interactiveROI = false;
load predefinedROI.mat
helperPlotTakeoffROILanding(gax,takeoffLat,takeoffLon,landLat,landLon,llapoints);

if(interactiveROI)
    [visLimitsLat,visLimitsLon] = geolimits; 
    [takeoffLat,takeoffLon] = helperTakeoffSelectionFcn(gax,visLimitsLat,visLimitsLon);
    [llapoints,xyzpoints] = helperPolygonSelectionFcn(gax,visLimitsLat,visLimitsLon,geocenter);
    [landLat,landLon] = helperLandSelectionFcn(gax,visLimitsLat,visLimitsLon);
end
polygons = coverageDecomposition(xyzpoints(:, 1:2));
polygonAreas = zeros(1, numel(polygons));

for i = 1:numel(polygons)
    polygonAreas(i) = polyarea(polygons{i}(:, 1), polygons{i}(:, 2));
end

averageArea = mean(polygonAreas);
threshold = 2 * averageArea; 

finalPolygons = {};

for i = 1:numel(polygons)
    if polygonAreas(i) > threshold
        newPolygons = coverageDecomposition(polygons{i}); 
        finalPolygons = [finalPolygons, newPolygons];
    else
        finalPolygons{end + 1} = polygons{i};  
    end
end

numDrones = 3; 
numPolygons = numel(finalPolygons); 

function isAdjacent = checkAdjacency(poly1, poly2)
    isAdjacent = false;
    if ~isempty(intersect(poly1, poly2, 'rows'))
        isAdjacent = true;
    end
end

adjacencyGroups = {};

for i = 1:numPolygons
    foundGroup = false;
    for j = 1:numel(adjacencyGroups)
        if any(arrayfun(@(k) checkAdjacency(finalPolygons{i}, adjacencyGroups{j}{k}), 1:numel(adjacencyGroups{j})))
            adjacencyGroups{j}{end + 1} = finalPolygons{i}; 
            foundGroup = true;
            break;
        end
    end
    if ~foundGroup
        adjacencyGroups{end + 1} = {finalPolygons{i}}; 
    end
end

numGroups = numel(adjacencyGroups);
fprintf('Number of groups: %d\n', numGroups);  
groupsPerDrone = floor(numGroups / numDrones);
remainingGroups = mod(numGroups, numDrones); 

assignedAreas = cell(1, numDrones);

for i = 1:numDrones
    assignedCount = groupsPerDrone;
    if i <= remainingGroups 
        assignedCount = assignedCount + 1;  
    end
    startIndex = (i - 1) * groupsPerDrone + min(i - 1, remainingGroups);
    assignedAreas{i} = adjacencyGroups(startIndex + (1:assignedCount)); 
end

for droneIdx = 1:numDrones
    fprintf('Drone %d is assigned the following areas:\n', droneIdx);
    for groupIdx = 1:numel(assignedAreas{droneIdx})
        disp('Group:');
        for areaIdx = 1:numel(assignedAreas{droneIdx}{groupIdx})
            disp(assignedAreas{droneIdx}{groupIdx}{areaIdx});
        end
    end
end

subAreasLLA = {};  
for i = 1:numel(assignedAreas)
    if ~isempty(assignedAreas{i})  
        for j = 1:numel(assignedAreas{i})
            currentPolygon = assignedAreas{i}{j};

            if iscell(currentPolygon)
                currentPolygon = currentPolygon{1};  
            end
            
            altitude = refHeight * ones(size(currentPolygon, 1), 1);
            localENU = [currentPolygon(:, 1), currentPolygon(:, 2), altitude];
            subArea = enu2lla(localENU, geocenter, "flat");
            subAreasLLA{end + 1} = subArea(:, 1:2); 
        end
    end
end

cs = uavCoverageSpace(Polygons=subAreasLLA, ...
                      UnitWidth=100, ...
                      Sidelap=0, ...
                      ReferenceHeight=refHeight, ...
                      UseLocalCoordinates=false, ...
                      ReferenceLocation=geocenter);

ax = cs.show(Parent=gax,LineWidth=1.25);