function X = norm_do(angle)
    while(angle > 180)
        angle= 360 - angle;
    end
    X = abs(angle);