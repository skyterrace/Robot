% my data processing for robot vision

clc
clear all
close all

numCluster = 12;
path = 'image_3.28\train\';


idx = 1;
imgSize = [128 128];
zeroImg = zeros([imgSize 3]);
imgTrain = zeroImg;
zeroLabel = zeros(numCluster, 1);
label = zeroLabel;

% % loading data
% 
% for i = 1:numCluster
%     
%     folderInfo = dir([path num2str(i) '\output']);
%     
%     for j = 1:size(folderInfo,1)-2
%         imgTrain(:,:,:, idx) = imresize(double(imread([path num2str(i) '\output\' folderInfo(2+j).name])), imgSize);
%         tmpLabel = zeroLabel;
%         tmpLabel(i) = 1;
%         label(:, idx) = tmpLabel;
%         idx = idx+1;
%     end
%     
% end
% 
% 
% save('data.mat', 'imgTrain', 'label', '-v7.3');


% Test Data

path = 'image_3.28\test\';
imgTest = zeroImg;
labelTest = zeroLabel;

for i = 1:numCluster
    
    folderInfo = dir([path num2str(i) '\output']);
    
    for j = 1:size(folderInfo,1)-2
        imgTest(:,:,:, idx) = imresize(double(imread([path num2str(i) '\output\' folderInfo(2+j).name])), imgSize);
        tmpLabel = zeroLabel;
        tmpLabel(i) = 1;
        labelTest(:, idx) = tmpLabel;
        idx = idx+1;
    end
    
end

save('dataTest.mat', 'imgTest', 'labelTest', '-v7.3');