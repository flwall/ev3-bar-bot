classdef RealRobotLogicController < LogicController

    properties
        state=robot_states.INIT;
    end
    methods

        function initialize(obj)

            robotObj=RobotObject();
            % robotObj.robot = legoev3('usb');
            % 
            % %% section init sensors
            % disp("Robot connected");
            % robotObj.touch = touchSensor(robotObj.robot);
            % robotObj.gyrosensor = gyroSensor(robotObj.robot);
            % robotObj.balanceMotor=motor(robotObj.robot, 'A');
            % robotObj.frontDriveMotor = motor(robotObj.robot, 'C');
            % robotObj.backDriveMotor= motor(robotObj.robot, 'D');
            % robotObj.schallsensor = sonicSensor(robotObj.robot);
            obj.robotObj=robotObj;
        end

        function robotStep(obj)
            
            is_running=1;
            

            %% config from UI
            DRIVE_SPEED_FRONT=obj.config.initialFrontDriveSpeed;
            MAX_ROTATIONS=obj.config.maxRotations;
            acceleration=obj.config.acceleration;
            MIN_DISTANCE=obj.config.minDistance;
            DECELERATION=obj.config.deceleration; 
            direction=obj.config.direction;


            %% start other config options
            radius=56/2;
            DRIVE_SPEED_BACK=DRIVE_SPEED_FRONT*1.6666;
            currentSpeedFront=DRIVE_SPEED_FRONT;
            currentSpeedBack=DRIVE_SPEED_BACK;
            ROTATION_DEADZONE=5;
            % obj.direction=1;  %1=forward, -1 =backwards
            
            
            %% section plotting vectors
            frontRotations=[];
            backRotations=[];
            frontSpeeds=[];
            backSpeeds=[];
            
            
            %% section init & drift-check gyrosensor
            try
                obj.start_gyrosensor();
            catch exc
            end
            
            %% section main-running-loop
            if ~is_running
                return;
            end

                switch obj.state
                    case robot_states.INIT
                        distance=0;
                        balanceMotorSpeeds=[];
                        backRotations=[];
                        frontRotations=[];
                        backSpeeds=[];
                        frontSpeeds=[];
                        xs=[];
                        i=1;
                        tic;
                        obj.robotObj.frontDriveMotor.Speed = direction*DRIVE_SPEED_FRONT;
                        obj.robotObj.backDriveMotor.Speed=-direction*DRIVE_SPEED_BACK;
                        currentSpeedFront=DRIVE_SPEED_FRONT;
                        currentSpeedBack=DRIVE_SPEED_BACK;
                        writeStatusLight(obj.robotObj.robot, 'orange', 'solid'); % maybe dont write 
            
                        disp("robot initialized... waiting for start command via touch sensor");
                        obj.state=robot_states.WAIT_FOR_START_COMMAND;
                    case robot_states.WAIT_FOR_START_COMMAND
                        if(readTouch(obj.robotObj.touch)==1)
                            writeStatusLight(obj.robotObj.robot, 'green', 'solid');
                            playTone(obj.robotObj.robot, 500, 1, 10);
            
                            pause(1);
            
                            resetRotation(obj.robotObj.frontDriveMotor);
                            resetRotation(obj.robotObj.backDriveMotor);
                            start(obj.robotObj.frontDriveMotor);
                            start(obj.robotObj.backDriveMotor);
                            disp("started motors");
                            obj.state=robot_states.DRIVING;
                            send(obj.EventQueue, 'Driving');
                            
                        end
                    case robot_states.DRIVING
                        motorRotation = readRotation(obj.robotObj.frontDriveMotor);
                        distance=motorRotation * ((2*pi*radius)/360);
            
                        if(readTouch(obj.robotObj.touch)==1 ||obj.isPaused)
                            obj.state=robot_states.FINAL;
                        end
                        sonicDistance = readDistance(obj.robotObj.schallsensor);
                        if(direction==1 && sonicDistance<MIN_DISTANCE)  % sonic Sensor only relevant if direction forward
                            obj.state=robot_states.BLOCKED;
                        end
                    
                        %start for plotting
                        xs(end+1)=i;
                        balanceMotorSpeeds(end+1)=obj.robotObj.balanceMotor.Speed;
                        frontSpeeds(end+1)=obj.robotObj.frontDriveMotor.Speed;
                        backSpeeds(end+1)=-obj.robotObj.backDriveMotor.Speed;
                        
                        frontRotations(end+1)=readRotation(obj.robotObj.frontDriveMotor)-sum(frontRotations);  % subtract last value because we want not the aggregated rotations
                        backRotations(end+1)=(-readRotation(obj.robotObj.backDriveMotor)-sum(backRotations))/1.6666;
                        
                        i=i+1;
                        %end for plotting
                    
                        if frontRotations(end) < MAX_ROTATIONS
                            currentSpeedFront=double(currentSpeedFront+acceleration);
                            obj.robotObj.frontDriveMotor.Speed=direction*currentSpeedFront;
                        elseif frontRotations > MAX_ROTATIONS+ROTATION_DEADZONE    % some deadzone to avoid flipping between two states
                            currentSpeedFront=double(currentSpeedFront-acceleration);
                            obj.robotObj.frontDriveMotor.Speed=direction*currentSpeedFront;
                        end
                        if backRotations(end) < MAX_ROTATIONS*1.666
                            currentSpeedBack=double(currentSpeedBack+acceleration);
                            obj.robotObj.backDriveMotor.Speed=-direction * currentSpeedBack;
                        elseif backRotations(end) > MAX_ROTATIONS*1.666+ROTATION_DEADZONE % some deadzone to avoid flipping between two states
                            currentSpeedBack=double(currentSpeedBack-acceleration);
                            obj.robotObj.backDriveMotor.Speed=-direction*currentSpeedBack;
                        end
            
                    case robot_states.BLOCKED
                        writeStatusLight(obj.robotObj.robot, 'red', 'pulsing');
                        send(obj.EventQueue, 'Blocked');
                        
                        while readDistance(obj.robotObj.schallsensor)< MIN_DISTANCE && (abs(obj.robotObj.frontDriveMotor.Speed)>DECELERATION && abs(obj.robotObj.backDriveMotor.Speed)>DECELERATION)
                            obj.robotObj.frontDriveMotor.Speed=sign(obj.robotObj.frontDriveMotor.Speed)* (abs(obj.robotObj.frontDriveMotor.Speed)-DECELERATION);
                            obj.robotObj.backDriveMotor.Speed=sign(obj.robotObj.backDriveMotor.Speed) * (abs(obj.robotObj.backDriveMotor.Speed)-DECELERATION);
                            balance_regulation(obj.robotObj.gyrosensor, obj.robotObj.balanceMotor);
                        end
                            obj.robotObj.frontDriveMotor.Speed=0;
                            obj.robotObj.backDriveMotor.Speed=0;
                            
                        
                        sonicDistance = readDistance(obj.robotObj.schallsensor);
                        if(sonicDistance >= MIN_DISTANCE)
                            writeStatusLight(obj.robotObj.robot, 'green', 'solid');
                            currentSpeedFront=DRIVE_SPEED_FRONT;
                            currentSpeedBack=DRIVE_SPEED_BACK;
                            obj.robotObj.frontDriveMotor.Speed=direction*currentSpeedFront;
                            obj.robotObj.backDriveMotor.Speed=-direction*currentSpeedFront;
                            obj.state=robot_states.DRIVING;
                            send(obj.EventQueue, 'Driving');
                        else
                            beep(obj.robotObj.robot, 1);
                        end
            
                        if(readTouch(obj.robotObj.touch)==1||obj.isPaused)
                            obj.state=robot_states.FINAL;
                        end
            
                
                    case robot_states.FINAL
                        close all; % close figures
                        send(obj.EventQueue, 'Paused');
                        writeStatusLight(obj.robotObj.robot, 'red', 'solid');
                        while(abs(obj.robotObj.frontDriveMotor.Speed)>DECELERATION && abs(obj.robotObj.backDriveMotor.Speed)>DECELERATION)
                            obj.robotObj.frontDriveMotor.Speed=sign(obj.robotObj.frontDriveMotor.Speed)* (abs(obj.robotObj.frontDriveMotor.Speed)-DECELERATION);
                            obj.robotObj.backDriveMotor.Speed=sign(obj.robotObj.backDriveMotor.Speed) * (abs(obj.robotObj.backDriveMotor.Speed)-DECELERATION);
                            balance_regulation(obj.robotObj.gyrosensor, obj.robotObj.balanceMotor);
                        end
            
            
                        beep(obj.robotObj.robot, 1);
            
                        direction=-direction; % switch the direction front/backwards
                        send(obj.EventQueue, struct('direction', direction));

                        disp("stopping motors");
                        stop(obj.robotObj.frontDriveMotor);
                        stop(obj.robotObj.backDriveMotor);
                
                        distance=double(distance)/double(1000);  % mm to metres
                        time=toc;
                        velocity=distance/time;
                        msg=['The robot travelled ', num2str(distance), ' metres in ', num2str(time), ' seconds (=', num2str(velocity), ' m/s)'];
                        disp(msg);
                        clearLCD(obj.robotObj.robot);
                        writeLCD(obj.robotObj.robot, msg, 1, 1);
            
                        figure(1);
                        subplot(3,1,1);
                        plot(xs, balanceMotorSpeeds);
                        hold on;
                        grid on;
            
                        subplot(3,1,2);
                        plot(xs, frontRotations);
                        grid on;
                        hold on;
                        plot(xs, backRotations);
                        legend("frontRotations", "backRotations");
            
                        subplot(3,1,3);
                        plot(xs, frontSpeeds);
                        hold on;
                        grid on;
                        plot(xs, backSpeeds);
                        legend("frontSpeeds", "backSpeeds");
            
                        pause(1);
            
                        % is_running=0;
                        obj.state=robot_states.INIT;  % for now, just run the robot forever
            
                    otherwise
                        warning("UNRECOGNIZED robot STATE!");
                        is_running=0;
                end
            
                taste = readButton(obj.robotObj.robot, 'down');
                if(taste==1)
                    disp("Roboter was stopped via down button");
                    is_running=0;
                end
                    
                    balance_regulation(obj.robotObj.gyrosensor, obj.robotObj.balanceMotor);
            end


            obj.shutdown();
        end

        function start_gyrosensor(obj)
            disp("waiting for robot to initialize...");
            resetRotationAngle(obj.robotObj.gyrosensor);
            pause(2);
            if(abs(readRotationAngle(obj.robotObj.gyrosensor))>2)
                beep(obj.robotObj.robot, 3);
                exc=MException('gyrosensor seems to drift!');
                throw(exc);               
            end
            
            resetRotation(obj.robotObj.balanceMotor);
            obj.robotObj.balanceMotor.Speed=0;
            start(obj.robotObj.balanceMotor);
        end
        function stop_gyrosensor(obj)
            stop(obj.robotObj.balanceMotor);
        end
        function shutdown(obj)
            stop(obj.robotObj.balanceMotor);
            stop(obj.robotObj.frontDriveMotor);
            stop(obj.robotObj.backDriveMotor);
        end
    end
end