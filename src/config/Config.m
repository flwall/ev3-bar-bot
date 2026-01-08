classdef Config < handle
properties (Access=public)
    minDistance=0.2;
    acceleration=0.4;
    maxRotations=20;
    initialFrontDriveSpeed=30; % initial
    deceleration=3;
    direction=1;
end
end