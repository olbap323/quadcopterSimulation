function wd = alpha(omega,omega_des, Km)
omega_max = 1151.9; % this is the equivalent of 11,000 RPM and is near to the max speed of motor
omega_min = 125.6637;
if omega >= omega_max && omega_des >= omega_max,
    wd = 0;
elseif omega <= omega_min && omega_des <= omega_min,
    wd = 0;
else
    wd = (omega_des - omega)*Km;
end

% wd = (omega_des - omega)*Km;
end

