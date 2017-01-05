function [ heading ] = getHeading( x, y )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    heading = -1;
    if y > 0
        heading =  270 - atand(x/y);
    elseif y < 0
        heading =  90 - atand(x/y);
    else
        if x < 0
            heading = 360;
        else
            heading = 180;
        end
        
    end


end

