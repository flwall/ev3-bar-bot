classdef (Abstract) LogicController < handle
    properties (Access = protected)
        config Config
        robotObj
        direction=1
    end

    methods
        function obj=LogicController(config)
            obj.robotObj=RobotObject();
            obj.config=config;
        end

    end

    methods (Abstract)
        initialize(obj)
        robotStep(obj)
        shutdown(obj)
    end
end
