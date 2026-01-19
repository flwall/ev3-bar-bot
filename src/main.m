clear;
close all;
clc;

% CONFIG PARAMETERS
DRIVE_SPEED_FRONT=10; % 0-100 (should be low because robot automatically accelerates unti MAX_ROTATIONS is reached
DECELERATION=2.2;
MIN_DISTANCE=0.2;  %in meters
MAX_ROTATIONS=10;
acceleration=0.8;  % acceleration from base speed;

% Start program

if exist('robot') ~= true
 robot = legoev3('usb');
end

state=robot_states.INIT;
is_running=1;

%init sensors
disp("Robot connected");
touch = touchSensor(robot);
gyrosensor = gyroSensor(robot);
balanceMotor=motor(robot, 'A');
frontDriveMotor = motor(robot, 'C');
backDriveMotor= motor(robot, 'D');
schallsensor = sonicSensor(robot);


radius=56/2;

TRANSMISSION_RATIO=1.6666;
DRIVE_SPEED_BACK=DRIVE_SPEED_FRONT*TRANSMISSION_RATIO;
currentSpeedFront=DRIVE_SPEED_FRONT;
currentSpeedBack=DRIVE_SPEED_BACK;
ROTATION_DEADZONE=5;
direction=1;  %1=forward, -1 =backwards

frontRotations=[];
backRotations=[];
frontSpeeds=[];

disp("waiting for robot to initialize...");
resetRotationAngle(gyrosensor);
pause(2);
if(abs(readRotationAngle(gyrosensor))>2)
    beep(robot, 3);
    error("gyrosensor seems to drift!!!");                  
end

resetRotation(balanceMotor);
balanceMotor.Speed=0;
start(balanceMotor);




while is_running
    switch state
        case robot_states.INIT
            distance=0;
            balanceMotorSpeeds=[];
            backRotations=[];
            frontRotations=[];
            frontSpeeds=[];
            xs=[];

            frontDriveMotor.Speed = direction*DRIVE_SPEED_FRONT;
            backDriveMotor.Speed=-direction*DRIVE_SPEED_BACK;
            currentSpeedFront=DRIVE_SPEED_FRONT;
            currentSpeedBack=DRIVE_SPEED_BACK;
            writeStatusLight(robot, 'orange', 'solid'); % maybe dont write 

            disp("robot initialized... waiting for start command via touch sensor");
            state=robot_states.WAIT_FOR_START_COMMAND;
            
        case robot_states.WAIT_FOR_START_COMMAND
            if(readTouch(touch)==1)
                writeStatusLight(robot, 'green', 'solid');
                playTone(robot, 500, 1, 10);

                pause(1);

                resetRotation(frontDriveMotor);
                resetRotation(backDriveMotor);
                start(frontDriveMotor);
                start(backDriveMotor);
                disp("started motors");
                tic;
                state=robot_states.DRIVING;
            end
        case robot_states.DRIVING
            motorRotation = readRotation(frontDriveMotor);
            distance=motorRotation * ((2*pi*radius)/360);

            if(readTouch(touch)==1)
                state=robot_states.FINAL;
            end
            sonicDistance = readDistance(schallsensor);
            if(direction==1 && sonicDistance<MIN_DISTANCE)  % sonic Sensor only relevant if direction forward
                state=robot_states.BLOCKED;
            end
        
            %start for plotting
            xs(end+1)=toc;
            frontSpeeds(end+1)=frontDriveMotor.Speed*direction;
            
            balanceMotorSpeeds(end+1)=balanceMotor.Speed;
            frontRotations(end+1)=readRotation(frontDriveMotor)-sum(frontRotations);  % subtract last value because we want not the aggregated rotations
            backRotations(end+1)=(-readRotation(backDriveMotor)-sum(backRotations));
            
            %end for plotting
        
            if frontRotations(end) < MAX_ROTATIONS
                currentSpeedFront=double(currentSpeedFront+acceleration);
                frontDriveMotor.Speed=direction*currentSpeedFront;
            elseif frontRotations > MAX_ROTATIONS+ROTATION_DEADZONE    % some deadzone to avoid flipping between two states
                currentSpeedFront=double(currentSpeedFront-acceleration);
                frontDriveMotor.Speed=direction*currentSpeedFront;
            end
            if backRotations(end) < MAX_ROTATIONS*TRANSMISSION_RATIO
                currentSpeedBack=double(currentSpeedBack+acceleration);
                backDriveMotor.Speed=-direction * currentSpeedBack;
            elseif backRotations(end) > MAX_ROTATIONS*TRANSMISSION_RATIO+ROTATION_DEADZONE % some deadzone to avoid flipping between two states
                currentSpeedBack=double(currentSpeedBack-acceleration);
                backDriveMotor.Speed=-direction*currentSpeedBack;
            end

        case robot_states.BLOCKED
            writeStatusLight(robot, 'red', 'pulsing');
            
            while readDistance(schallsensor)< MIN_DISTANCE && (abs(frontDriveMotor.Speed)>DECELERATION && abs(backDriveMotor.Speed)>DECELERATION)
                frontDriveMotor.Speed=sign(frontDriveMotor.Speed)* (abs(frontDriveMotor.Speed)-DECELERATION);
                backDriveMotor.Speed=sign(backDriveMotor.Speed) * (abs(backDriveMotor.Speed)-DECELERATION);
                frontSpeeds(end+1)=frontDriveMotor.Speed*direction;
                balanceMotorSpeeds(end+1)=balanceMotor.Speed;
                frontRotations(end+1)=readRotation(frontDriveMotor)-sum(frontRotations);  % subtract last value because we want not the aggregated rotations
                backRotations(end+1)=(-readRotation(backDriveMotor)-sum(backRotations));
                
                xs(end+1)=toc;
                balance_regulation(gyrosensor, balanceMotor);
            end
                frontDriveMotor.Speed=0;
                backDriveMotor.Speed=0;
                
            
            sonicDistance = readDistance(schallsensor);
            if(sonicDistance >= MIN_DISTANCE)
                writeStatusLight(robot, 'green', 'solid');
                currentSpeedFront=DRIVE_SPEED_FRONT;
                currentSpeedBack=DRIVE_SPEED_BACK;
                frontDriveMotor.Speed=direction*currentSpeedFront;
                backDriveMotor.Speed=-direction*currentSpeedFront;
                state=robot_states.DRIVING;
            else
                beep(robot, 1);
            end

            if(readTouch(touch)==1)
                state=robot_states.FINAL;
            end

    
        case robot_states.FINAL
            close all; % close figures
            writeStatusLight(robot, 'red', 'solid');
            while(abs(frontDriveMotor.Speed)>DECELERATION && abs(backDriveMotor.Speed)>DECELERATION)
                frontDriveMotor.Speed=sign(frontDriveMotor.Speed)* (abs(frontDriveMotor.Speed)-DECELERATION);
                backDriveMotor.Speed=sign(backDriveMotor.Speed) * (abs(backDriveMotor.Speed)-DECELERATION);
                frontSpeeds(end+1)=frontDriveMotor.Speed*direction;
                balanceMotorSpeeds(end+1)=balanceMotor.Speed;
            frontRotations(end+1)=readRotation(frontDriveMotor)-sum(frontRotations);  % subtract last value because we want not the aggregated rotations
            backRotations(end+1)=(-readRotation(backDriveMotor)-sum(backRotations));
                xs(end+1)=toc;
                balance_regulation(gyrosensor, balanceMotor);
            end
           disp("finished deceleration");
           
           while balanceMotor.Speed ~= 0
                balance_regulation(gyrosensor, balanceMotor);
           end

           beep(robot, 1);

            direction=-direction; % switch the direction front/backwards
            
            disp("stopping motors");
            stop(frontDriveMotor);
            stop(backDriveMotor);
    
            distance=double(distance)/double(10);  % mm to cm
            time=toc;
            velocity=distance/time;
            msg1=['Distance: ', num2str(distance,'%.3f'),' cm'];
            msg2=['Time: ', num2str(time,'%.3f'),' sec'];
            msg3=['Velocity: ', num2str(velocity,'%.3f'), ' cm/s'];
            disp(msg1), disp(msg2), disp(msg3);
            clearLCD(robot);
            writeLCD(robot, msg1, 7, 1);
            writeLCD(robot, msg2, 8, 1);
            writeLCD(robot, msg3, 9, 1);




            figure(1);
            subplot(3,1,1);
            plot(xs, balanceMotorSpeeds);
            title("Gyrosensor Balance");
            xlabel("Zeit [s]");
            ylabel("%");
            hold on;
            grid on;

            subplot(3,1,2);
            plot(xs, frontRotations);
            title("Motor-Rotationen");
            xlabel("Zeit [s]");
            ylabel("Rotationen");
            grid on;
            hold on;
            plot(xs, backRotations);
            legend("Rotationen Frontantrieb", "Rotationen Heckantrieb");

            subplot(3,1,3);
            plot(xs, frontSpeeds);
            title("Fahrzeug Geschwindigkeit");
            xlabel("Zeit [s]");
            ylabel("%");
            hold on;
            grid on;
            legend("Gesamt-Geschwindigkeit");

            pause(1);

            % is_running=0;
            state=robot_states.INIT;  % for now, just run the robot forever

        otherwise
            warning("UNRECOGNIZED robot STATE!");
            is_running=0;
    end

    taste = readButton(robot, 'down');
    if(taste==1)
        disp("Roboter was stopped via down button");
        is_running=0;
    end
        
    balance_regulation(gyrosensor, balanceMotor);

end


stop(balanceMotor);
stop(frontDriveMotor);
stop(backDriveMotor);