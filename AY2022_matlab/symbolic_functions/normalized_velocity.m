function [norm_vel factor] = normalized_velocity(vel, speed_max)
    factor = 1;
    
    if abs(vel(1)) > pi/2
        factor = (pi/2)/(abs(vel(1)));
    end
    if abs(vel(2)) > pi/2 && (pi/2)/abs(vel(2)) < factor
        factor = (pi/2)/abs(vel(2));
    end
    if abs(vel(5)) > pi/2 && (pi/2)/abs(vel(5)) < factor
        factor = (pi/2)/abs(vel(5));
    end
    
    norm_vel_int = factor * vel;
    
    norm_vel_int(1) = max(min(norm_vel_int(1),speed_max),-speed_max);
    norm_vel_int(2) = max(min(norm_vel_int(2),speed_max),-speed_max);
    norm_vel_int(3) = -norm_vel_int(2);
    norm_vel_int(2) = vel(2);
    norm_vel_int(3) = vel(3);
    norm_vel_int(5) = max(min(norm_vel_int(5),speed_max),-speed_max);
    
    norm_vel = norm_vel_int;
end