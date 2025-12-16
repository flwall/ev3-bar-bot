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
frontDriveMotor = motor(robot, 'D');
backDriveMotor= motor(robot, 'C');

radius=56/2;
DRIVE_SPEED_FRONT=15;
MAX_SPEED=100;
acceleration=0.4;
SPEED_DIFF=0;
DRIVE_SPEED_BACK=DRIVE_SPEED_FRONT+SPEED_DIFF;
currentSpeed=DRIVE_SPEED_FRONT;


while is_running
    switch state
        case robot_states.INIT
            distance=0;
            balanceMotorSpeeds=[];
            xs=[];
            i=1;
            tic;
            resetRotation(frontDriveMotor);
            resetRotation(backDriveMotor);
            frontDriveMotor.Speed = DRIVE_SPEED_FRONT;
            backDriveMotor.Speed=-DRIVE_SPEED_BACK;
            currentSpeed=DRIVE_SPEED_FRONT;
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
            else
                if frontDriveMotor.Speed < MAX_SPEED
                    currentSpeed=double(currentSpeed+acceleration);
                    frontDriveMotor.Speed=currentSpeed;
                end
                if abs(backDriveMotor.Speed) < MAX_SPEED+SPEED_DIFF;
                    backDriveMotor.Speed=-(currentSpeed+SPEED_DIFF);
                end
            end

    angle=double(-readRotationAngle(gyrosensor));
    balanceMotorRotation=readRotation(balanceMotor) / double(5.0); % uebersetzung von Zahnrad

    speedFactor=double(-0.01*double(frontDriveMotor.Speed-20)+1);
    kP=double(4.6 * speedFactor); % 4.6 is a tested value for speed = 20, so the multiplier there should begin at 1, getting higher for higher speed
    angleSum = double(-kP) * double(balanceMotorRotation-angle);

    balanceMotor.Speed=angleSum;
    xs(end+1)=i;
    balanceMotorSpeeds(end+1)=angleSum;
    i=i+1;
    
        case robot_states.FINAL
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
            plot(xs, balanceMotorSpeeds);
            hold on;
            grid on;

            pause(1);

            % is_running=0;
            state=robot_states.INIT;  % for now, just run the robot forever

        otherwise
            warning("UNRECOGNIZED robot STATE!");
            is_running=0;
    end


end
