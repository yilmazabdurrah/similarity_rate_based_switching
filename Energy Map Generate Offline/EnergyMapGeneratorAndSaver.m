%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% Energy map from pgm file %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Date    : 28 Feb 2022
% Author  : Abdurrahman Yilmaz
% Version : v01
% Info    : Energy map of the environment is generated and saved
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% IMPORTANT INFO: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% The map can be scaled for fast energy map calculations.
%%% Otherwise, energy computation will last so much. In this code, the
%%% scale constant was 8. Normally, map resolution was 0.04 m, but 0.32 m
%%% is the resolution of scaled map. The energy values of other grids are
%%% calculated via interpolation in the code. To scale the map, gimp app can be
%%% used, linearly scaled the map. A suitable yaml file must be prepared for the developed
%%% scaled map.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mapImage = imread('maps//map.pgm');
MapSettings = ReadYamlFile('maps//map_scaled.yaml');

% Energy map is computed for four separate range settings, select that
% values as laserRange_1 < ... < laserRange_4
laserRange_1 = 5; % m
laserRange_2 = 10; % m
laserRange_3 = 15; % m
laserRange_4 = 20; % m

laserRes = 0.5; % deg
inflationRad = 0.5; %m

%% Load scaled map
imageBW = 1 - double(mapImage)/255;
map = occupancyMap(imageBW,1/MapSettings.resolution);
map.GridLocationInWorld = MapSettings.origin(1,1:2); 
map.OccupiedThreshold = MapSettings.occupied_thresh;
map.FreeThreshold = MapSettings.free_thresh;

%% Compute energy map for scaled map
[row_free,col_free] = find(imageBW <= map.FreeThreshold); % free cells row: map_y, col: map_x
energyList = [row_free,col_free,zeros(length(col_free),4)]; % row, col, energy for 5m, 10m, 15m, 20m ranges

xyMap2World = map.GridLocationInWorld;
yGrid = map.GridSize(1,1);

pixelRange = ceil(laserRange_4*map.Resolution);

for j = 1:1:size(energyList,1)
    xyPixel = [energyList(j,2),energyList(j,1)];
    start_xy = fliplr(xyPixel); % flipped since map is (y,x)
    xy = [xyPixel(1,1), yGrid - xyPixel(1,2) + 1]/map.Resolution + xyMap2World;
    FakeLaserMeas_1 = zeros(length(0:laserRes:360-laserRes),1);
    FakeLaserMeas_2 = zeros(length(0:laserRes:360-laserRes),1);
    FakeLaserMeas_3 = zeros(length(0:laserRes:360-laserRes),1);
    FakeLaserMeas_4 = zeros(length(0:laserRes:360-laserRes),1);
    cnt_ang = 0;
    for ang = 0:laserRes:360-laserRes
        cnt_ang = cnt_ang + 1;
        pixelyx = ceil([pixelRange*sind(ang), pixelRange*cosd(ang)]);
        end_xy = start_xy + pixelyx;
        if start_xy(1,1) <= end_xy(1,1)
            [LineElements] = BresenhamLineFourQuadrant(start_xy,end_xy);
        else
            [LineElements] = BresenhamLineFourQuadrant(end_xy,start_xy);
            LineElements = flipud(LineElements);
        end
        gridMeas_dist_1 = laserRange_1; % m
        gridMeas_dist_2 = laserRange_2; % m
        gridMeas_dist_3 = laserRange_3; % m
        gridMeas_dist_4 = laserRange_4; % m
        for i = 1:1:size(LineElements,1)
            if any(size(imageBW) - LineElements(i,:) < 0) || LineElements(i,1) <= 0 || LineElements(i,2) <= 0 
                break;
            elseif imageBW(LineElements(i,1),LineElements(i,2)) >= map.OccupiedThreshold
                gridMeas_pixel_xy = fliplr(LineElements(i,:));
                gridMeas_pixel_xy(1,2) = yGrid - gridMeas_pixel_xy(1,2) + 1;
                gridMeas_dist = norm(gridMeas_pixel_xy/map.Resolution + map.GridLocationInWorld - xy);
                if gridMeas_dist > laserRange_1
                    gridMeas_dist_1 = laserRange_1;
                else
                    gridMeas_dist_1 = gridMeas_dist;
                end
                if gridMeas_dist > laserRange_2
                    gridMeas_dist_2 = laserRange_2;
                else
                    gridMeas_dist_2 = gridMeas_dist;
                end
                if gridMeas_dist > laserRange_3
                    gridMeas_dist_3 = laserRange_3;
                else
                    gridMeas_dist_3 = gridMeas_dist;
                end
                if gridMeas_dist > laserRange_4
                    gridMeas_dist_4 = laserRange_4;
                else
                    gridMeas_dist_4 = gridMeas_dist;
                end
                break;
            end
        end
        FakeLaserMeas_1(cnt_ang,:) = gridMeas_dist_1;
        FakeLaserMeas_2(cnt_ang,:) = gridMeas_dist_2;
        FakeLaserMeas_3(cnt_ang,:) = gridMeas_dist_3;
        FakeLaserMeas_4(cnt_ang,:) = gridMeas_dist_4;
    end
    if min(FakeLaserMeas_1) < inflationRad
        energyList(j,3:6) = 1;
    else
       energyList(j,3) = (sum(ones(length(FakeLaserMeas_1),1)) - sum(FakeLaserMeas_1)/laserRange_1)/length(FakeLaserMeas_1); 
       energyList(j,4) = (sum(ones(length(FakeLaserMeas_2),1)) - sum(FakeLaserMeas_2)/laserRange_2)/length(FakeLaserMeas_2); 
       energyList(j,5) = (sum(ones(length(FakeLaserMeas_3),1)) - sum(FakeLaserMeas_3)/laserRange_3)/length(FakeLaserMeas_3); 
       energyList(j,6) = (sum(ones(length(FakeLaserMeas_4),1)) - sum(FakeLaserMeas_4)/laserRange_4)/length(FakeLaserMeas_4); 
    end
end

energyList2 = [energyList(:,1)*1000+energyList(:,2),energyList(:,3:6)];
[X,Y] = meshgrid(1:1:size(imageBW,2),1:1:size(imageBW,1));

for y2 = 1:1:size(Y,2)
    for y1 = 1:1:size(Y,1)
        index = find(energyList2(:,1) == (size(Y,1)-y1 + 1)*1000+y2);
        if isempty(index)
            Energy_05(y1,y2) = NaN;
            Energy_10(y1,y2) = NaN;
            Energy_15(y1,y2) = NaN;
            Energy_20(y1,y2) = NaN;
        else
            Energy_05(y1,y2) = energyList2(index,2);
            Energy_10(y1,y2) = energyList2(index,3);
            Energy_15(y1,y2) = energyList2(index,4);
            Energy_20(y1,y2) = energyList2(index,5);
        end
    end
end

save('EnergyGridDatav01')

%% Draw scaled energy map results

figure
colormap(jet)
contourf(X/map.Resolution+xyMap2World(1,1),Y/map.Resolution+xyMap2World(1,2),Energy_05)
colorbar

figure
colormap(jet)
contourf(X/map.Resolution+xyMap2World(1,1),Y/map.Resolution+xyMap2World(1,2),Energy_10)
colorbar

figure
colormap(jet)
contourf(X/map.Resolution+xyMap2World(1,1),Y/map.Resolution+xyMap2World(1,2),Energy_15)
colorbar

figure
colormap(jet)
contourf(X/map.Resolution+xyMap2World(1,1),Y/map.Resolution+xyMap2World(1,2),Energy_20)
colorbar
caxis([0 1])


%% Real energy map computation via interpolation
mapImageReal = imread('maps//121121_kaynak_2.pgm');
MapSettingsReal = ReadYamlFile('maps//121121_kaynak_2.yaml');

imageBWreal = 1 - double(mapImageReal)/255;
mapReal = occupancyMap(imageBWreal,1/MapSettingsReal.resolution);
mapReal.GridLocationInWorld = MapSettingsReal.origin(1,1:2); 
mapReal.OccupiedThreshold = MapSettingsReal.occupied_thresh;
mapReal.FreeThreshold = MapSettingsReal.free_thresh;

[Xreal,Yreal] = meshgrid(1:1:size(imageBWreal,2),1:1:size(imageBWreal,1));

Energy_05_real = interp2(8*X-4,8*Y-4,Energy_05,Xreal,Yreal); % 8 since real map is reduced/scaled 8 times
Energy_10_real = interp2(8*X-4,8*Y-4,Energy_10,Xreal,Yreal);
Energy_15_real = interp2(8*X-4,8*Y-4,Energy_15,Xreal,Yreal);
Energy_20_real = interp2(8*X-4,8*Y-4,Energy_20,Xreal,Yreal);

save('EnergyGridDatav01')

%% Save normal and negated energy map of the environment for laser range 20m case, follow the same procedure to save other range maps if required
Energy_20_real_pgm = Energy_20_real;
Energy_20_real_pgm(isnan(Energy_20_real_pgm))=0;
imwrite(flipud(Energy_20_real_pgm),'energyMap20m.pgm') % normal map
Energy_20_real_pgm = 1 - Energy_20_real_pgm; 
imwrite(flipud(Energy_20_real_pgm),'energyMap20mNegate.pgm') % negated map

%% Plot energy maps saved for laser range 20m case
mapImage = imread('energyMap20m.pgm');
imageBW = 1-double(mapImage)/255;
map = occupancyMap(imageBW);
figure;show(map)

mapImage = imread('energyMap20mNegate.pgm');
imageBW = 1-double(mapImage)/255;
map = occupancyMap(imageBW);
figure;show(map)

%% Draw real energy map results

figure
show(mapReal)
hold on
colormap(jet)
contourf(Xreal/mapReal.Resolution+xyMap2World(1,1),Yreal/mapReal.Resolution+xyMap2World(1,2),Energy_05_real)
colorbar
xlabel('x [m]')
ylabel('y [m]')
title('Energy map for 5m laser max range')

figure
show(mapReal)
hold on
colormap(jet)
contourf(Xreal/mapReal.Resolution+xyMap2World(1,1),Yreal/mapReal.Resolution+xyMap2World(1,2),Energy_10_real)
colorbar
xlabel('x [m]')
ylabel('y [m]')
title('Energy map for 10m laser max range')

figure
show(mapReal)
hold on
colormap(jet)
contourf(Xreal/mapReal.Resolution+xyMap2World(1,1),Yreal/mapReal.Resolution+xyMap2World(1,2),Energy_15_real)
colorbar
xlabel('x [m]')
ylabel('y [m]')
title('Energy map for 15m laser max range')

figure
show(mapReal)
hold on
colormap(jet)
contourf(Xreal/mapReal.Resolution+xyMap2World(1,1),Yreal/mapReal.Resolution+xyMap2World(1,2),Energy_20_real)
colorbar
xlabel('x [m]')
ylabel('y [m]')
title('Energy map for 20m laser max range')
