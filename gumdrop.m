% Main script
gax = geoaxes(Basemap="satellite");
latlimits = [44.3135 44.3534];
lonlimits = [-72.0227 -71.9544];
geolimits(latlimits, lonlimits);
geocenter = [mean(latlimits) mean(lonlimits) 0];
refHeight = 400;
hold on

% Set the number of UAVs
numDrones = 5;  % Adjust the number of UAVs

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
hold off

function helperPlotTakeoffROILanding(gax,tLat,tLon,lLat,lLon,llapoints)
geoplot(gax,tLat,tLon,LineWidth=2,MarkerSize=25,LineStyle="none",Marker=".")
text(gax,tLat+0.0025,tLon,"Takeoff",HorizontalAlignment="center",FontWeight="bold")
geoplot(gax,llapoints(:,1),llapoints(:,2),MarkerSize=25,Marker=".")
text(gax,mean(llapoints(:,1)),mean(llapoints(:,2))+0.006,"ROI",HorizontalAlignment="center",Color="white",FontWeight="bold")
geoplot(gax,lLat,lLon,LineWidth=2,MarkerSize=25,LineStyle="none",Marker=".")
text(gax,lLat+0.0025,lLon,"Landing",HorizontalAlignment="center",FontWeight="bold")
end

function exampleHelperPlotTakeoffLandingLegend(varargin)
takeoff = varargin{1};
landing = varargin{2};

hold on
scatter(takeoff(1),takeoff(2),45,"filled")
scatter(landing(1),landing(2),45,"filled")

% If path is specified, then plot path and add that to legend
if length(varargin) == 3
    path = varargin{3};
    plot(path(:,1),path(:,2))
    legend(["Polygon 1","Polygon 2","Takeoff","Landing","Path"],Location="northwest")
else    
    legend(["Polygon 1","Polygon 2","Takeoff","Landing"],Location="northwest")
end
hold off
end

function [Lat,Lon] = helperLandSelectionFcn(geoaxes,cacheLimitsLat,cacheLimitsLon)
title("Select Landing Position")
[Lat,Lon] = ginput(1);
hold on;
geoplot(geoaxes,Lat, Lon,'r*','MarkerSize',4);
hold off;
geolimits(cacheLimitsLat,cacheLimitsLon);
title("Coverage Space")
end

function [llapoints,xyzpoints] = helperPolygonSelectionFcn(geoaxes,cacheLimitsLat,cacheLimitsLon,geocenter)
title(["Draw coverage space polygon","Click Enter to finish drawing polgon"])
polygonSelectionLoop=true;
polyLats=[];
polyLons=[];
l=[];
if(polygonSelectionLoop)
    while true
        %Interactively define the area of interest.
        [city.Lat,city.Lon] = ginput(1);
        if isempty(city.Lat)
            break % User typed ENTER
        else
            hold on;
            %Cache the Lat and Lons for algorithm.
            polyLats(end+1)=city.Lat;
            polyLons(end+1)=city.Lon;
            geoplot(geoaxes,city.Lat, city.Lon, ...
                'Marker', 'o', ...
                'MarkerEdgeColor', 'k', ...
                'MarkerFaceColor', 'y', ...
                'MarkerSize', 3);
            delete(l);
            l=geoplot([polyLats,polyLats(1)],[polyLons,polyLons(1)],'b');
            hold off;
            geolimits(cacheLimitsLat,cacheLimitsLon);

        end
    end
end

%Format UAV coordinates
llapoints=[[polyLats,polyLats(1)]',[polyLons,polyLons(1)]',...
    zeros(length(polyLats)+1,1)];
%Convert to ENU
xyzpoints= lla2enu(llapoints,geocenter,'flat');
%Trim third coordinate as polygon is 2D.
xyzpoints(:,3)=[];
end

function [Lat,Lon] = helperTakeoffSelectionFcn(geoaxes,cacheLimitsLat,cacheLimitsLon)
title("Select Takeoff Position")
[Lat,Lon] = ginput(1);
hold on;
geoplot(geoaxes,Lat, Lon,'g*','MarkerSize',4);
hold off;
geolimits(cacheLimitsLat,cacheLimitsLon);
end



% Manually specify legend entries for only the UAV paths and takeoff/landing points


