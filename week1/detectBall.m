% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(Image)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load Samples from disk and use those
load('samples')
Samples = double(Samples);
mu = mean(Samples);
sig = cov(Samples);
thre = 4.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
ImgVector = reshape(Image, [], 3);
res = mahal(double(ImgVector),double(Samples));
res_arr = reshape(res, 120,160) < 5.;
%res_arr = res_arr < 5.;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% Find the biggest connected component
% Compute the location by using the centroid
CC = bwconncomp(res_arr);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
segI = false(size(res_arr));
segI(CC.PixelIdxList{idx}) = true; 
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid

end
