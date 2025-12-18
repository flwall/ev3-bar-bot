

backDriveMotor.Speed=50;
resetRotation(backDriveMotor);
start(backDriveMotor);

while 1
    readRotation(backDriveMotor)
    pause(1)
end

