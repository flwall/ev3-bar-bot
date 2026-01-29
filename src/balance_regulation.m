function balance_regulation(gyrosensor, balanceMotor)
    angle=-readRotationAngle(gyrosensor);

    kP=4.5; % was: 7.5
    angleSum = double(-kP * angle);
    
    balanceMotor.Speed=angleSum;
end