clear all;
close all;
clc;

load('HK-233_a_09-Sep-2015_ver1_XYZ+_newZ_nb_clean.mat') %load first pointcloud
figure;
plot3(Xclean, Yclean, Zclean, 'k.');%plot the pointcloud

cfirst=[Xclean ; Yclean ; Zclean]; %concatenate the coordinate to one vector
cfirst=cfirst';%transpose to have the format we want to convert in homogeneous coordinates if needed but don't need because can convert to point cloud directly

%hfirst = cart2hom(cfirst); % convert to homogeneous


ptCloudRef = pointCloud(cfirst); % point Cloud convertion

load('HK-233_b_09-Sep-2015_ver1_XYZ+_newZ_nb_clean.mat');
figure;
plot3(Xclean, Yclean, Zclean, 'k.');%plot the pointcloud

csecond=[Xclean ; Yclean ; Zclean]; %concatenate the coordinate to one vector
csecond=csecond';%transpose to have the format we want to convert in homogeneous coordinates if needed but don't need because can convert to point cloud directly

%hfirst = cart2hom(cfirst); % convert to homogeneous


ptCloudCurrent = pointCloud(csecond); % point Cloud convertion

gridSize = 0.05; %Set a grid size
% down sample the point cloud by the grid size, it speed up the process and
% increase the accuracy
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

tform = pcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true); %set the transform


ptCloudAligned = pctransform(ptCloudCurrent,tform); %transform the second point cloud accoring to the transformation setted beforte
mergeSize = 0.001; %set a merging size
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize); %merge the two pointcloud

figure;

% Visualize the merged point clouds
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('Merged Point Cloud')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
drawnow

%%
load('HK-233_c_09-Sep-2015_ver1_XYZ+_newZ_nb_clean.mat')
figure;
plot3(Xclean, Yclean, Zclean, 'k.');%plot the pointcloud

cthird=[Xclean ; Yclean ; Zclean]; %concatenate the coordinate to one vectore
cthird=cthird';%transpose to have the format we want to convert in homogeneous coordinates if needed but don't need because can convert to point cloud directly

%hfirst = cart2hom(cfirst); % convert to homogeneous
ptCloudRef = ptCloudScene;

ptCloudCurrent = pointCloud(cthird); % point Cloud convertion

ptCloud3 = ptCloudCurrent;
gridSize = 0.05; %Set a grid size
% down sample the point cloud by the grid size, it speed up the process and
% increase the accuracy
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);


tform = pcregrigid(moving, fixed, 'Metric' , 'pointToPlane','Extrapolate', true); %set the transform
ptCloudAligned = pctransform(ptCloudCurrent,tform); %transform the second point cloud accoring to the transformation setted beforte

mergeSize = 0.015; %set a merging size
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize); %merge the two pointcloud

figure;

% Visualize the merged point clouds
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('Final Merge')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
drawnow






