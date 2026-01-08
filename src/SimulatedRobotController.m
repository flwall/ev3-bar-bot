classdef SimulatedRobotController < LogicController
    properties
        view;
        config;
    end
    methods
        function obj = SimulatedRobotController(view)
            obj.config=Config();
            view.config=obj.config;
            obj.view=view;

        end

        function initializeRobot(obj)
            pause(1);
            disp("Initializing robot ...finished");
        end

        function runRobot(obj, dataQueue)
            disp("running the robot");
            while 1
                a = 0; b = 10;
                r=randi([a,b]);

                % if(app.isPaused)
                %     break;
                % end

                if r==1
                    send(dataQueue, 'gyro drifting');
                elseif r==2
                    send(dataQueue, 'Driving');
                elseif r==3
                    send(dataQueue, 'Paused');
                elseif r==4
                        break;
                end

                pause(1);

            end
        end
        function start_gyrosensor(obj)
            disp("gyro started");
        end
        function stop_gyrosensor(obj)
            disp("gyro stopped");
        end

        function shutdown(obj)
            disp("shutting down");
        end

    end
end
