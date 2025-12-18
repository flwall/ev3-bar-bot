clear;
close all;
clc;
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

radius=56/2;
DRIVE_SPEED_FRONT=30;
MAX_ROTATIONS=20;
acceleration=0.4;
DRIVE_SPEED_BACK=DRIVE_SPEED_FRONT*1.6666;
currentSpeedFront=DRIVE_SPEED_FRONT;
currentSpeedBack=DRIVE_SPEED_BACK;
ROTATION_DEADZONE=5;

frontRotations=[];
backRotations=[];
frontSpeeds=[];
backSpeeds=[];

while is_running
    switch state
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
            frontDriveMotor.Speed = DRIVE_SPEED_FRONT;
            backDriveMotor.Speed=-DRIVE_SPEED_BACK;
            currentSpeedFront=DRIVE_SPEED_FRONT;
            currentSpeedBack=DRIVE_SPEED_BACK;
            balanceMotor.Speed=0;
            writeStatusLight(robot, 'orange', 'solid'); % maybe dont write 

            disp("robot initialized... waiting for start command via touch sensor");
            state=robot_states.WAIT_FOR_START_COMMAND;
        case robot_states.WAIT_FOR_START_COMMAND
            if(readTouch(touch)==1)
                writeStatusLight(robot, 'green', 'solid');
                playTone(robot, 500, 1, 10);

                resetRotationAngle(gyrosensor);
                resetRotation(balanceMotor);

                resetRotation(frontDriveMotor);
                resetRotation(backDriveMotor);
                start(frontDriveMotor);
                start(backDriveMotor);
                start(balanceMotor);
                disp("started motors");
                state=robot_states.DRIVING;
                pause(1);
            end
        case robot_states.DRIVING
            motorRotation = readRotation(frontDriveMotor);
            distance=motorRotation * ((2*pi*radius)/360);

            if(readTouch(touch)==1)
                state=robot_states.FINAL;
            end

    angle=double(-readRotationAngle(gyrosensor));
    balanceMotorRotationRaw=readRotation(balanceMotor);
    balanceMotorRotation=balanceMotorRotationRaw / double(5.0); % uebersetzung von Zahnrad

   % speedFactor=double(-0.01*double(frontDriveMotor.Speed-20)+1);
    speedFactor=1;
    kP=double(4.6 * speedFactor); % 4.6 is a tested value for speed = 20, so the multiplier there should begin at 1, getting higher for higher speed
    angleSum = double(-kP) * double(balanceMotorRotation-angle);

    balanceMotor.Speed=angleSum;

    %start for plotting
    xs(end+1)=i;
    balanceMotorSpeeds(end+1)=angleSum;
    frontSpeeds(end+1)=frontDriveMotor.Speed;
    backSpeeds(end+1)=-backDriveMotor.Speed;
    
    frontRotations(end+1)=readRotation(frontDriveMotor)-sum(frontRotations);  % subtract last value because we want not the aggregated rotations
    backRotations(end+1)=(-readRotation(backDriveMotor)-sum(backRotations))/1.6666;
    
    i=i+1;
    %end for plotting

    if frontRotations(end) < MAX_ROTATIONS
        currentSpeedFront=double(currentSpeedFront+acceleration);
        frontDriveMotor.Speed=currentSpeedFront;
    elseif frontRotations > MAX_ROTATIONS+ROTATION_DEADZONE    % some deadzone to avoid flipping between two states
        currentSpeedFront=double(currentSpeedFront-acceleration);
        frontDriveMotor.Speed=currentSpeedFront;
    end
    if backRotations(end) < MAX_ROTATIONS*1.666
        currentSpeedBack=double(currentSpeedBack+acceleration);
        backDriveMotor.Speed=-currentSpeedBack;
    elseif backRotations(end) > MAX_ROTATIONS*1.666+ROTATION_DEADZONE % some deadzone to avoid flipping between two states
        currentSpeedBack=double(currentSpeedBack-acceleration);
        backDriveMotor.Speed=-currentSpeedBack;
    end

    
        case robot_states.FINAL
            close all; % close figures
            writeStatusLight(robot, 'red', 'solid');
            beep(robot, 1);
            
            disp("stopping motors");
            stop(balanceMotor);
            stop(frontDriveMotor);
            stop(backDriveMotor);
    
            distance=double(distance)/double(1000);  % mm to metres
            time=toc;
            velocity=distance/time;
            msg=['The robot travelled ', num2str(distance), ' metres in ', num2str(time), ' seconds (=', num2str(velocity), ' m/s)'];
            disp(msg);
            clearLCD(robot);
            writeLCD(robot, msg, 1, 1);

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
            state=robot_states.INIT;  % for now, just run the robot forever

        otherwise
            warning("UNRECOGNIZED robot STATE!");
            is_running=0;
    end


end
