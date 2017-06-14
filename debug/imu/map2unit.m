function [ xNew, yNew ] = map2unit( readings )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


%% Get the mins and maxes for a
xMinR = min(readings(:,1));
yMinR = min(readings(:,2));

xMaxR = max(readings(:,1));
yMaxR = max(readings(:,2));

% Compute the mapping
xNew = (readings(:,1)-xMinR)*(2/(xMaxR-xMinR))-1;
yNew = (readings(:,2)-yMinR)*(2/(yMaxR-yMinR))-1;


end

