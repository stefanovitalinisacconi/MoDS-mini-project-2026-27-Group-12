function dz = printerFlow(tt,z,q,mem,A,B,C,t_ref,u_ref,nozzle_rhs,Kp,Kd,unhold)

% The flow map computes the derivatives of the 9 physical states.
% Motor voltages depend on the current discrete mode.
% The nozzle voltage also changes according to the active mode.

if q == 1  % Printing
    
    t_print = mem.progress + tt-mem.entry; % Calculate the printing time
    t_print = min(max(t_print,t_ref(1)),t_ref(end)); % Keep printing time within the available reference interval
    motor_u = interp1(t_ref,u_ref,t_print,'linear')'; % Obtain the motor voltages from the Q1.4 reference signals

else % Other modes

    if q==2 || q==8 % Stop and Save / Returning

        target = mem.saved; % Target is the position stored before filament replacement

    else % Other non-printing modes

        target = [0;0]; % Target is the wiping position at the origin

    end

    pos = C*z(1:6); % Compute position from motor angles
    vel = C(:,1:2)*z(3:4); % Compute velocity from motor angular velocities

    u_xy = Kp*(target-pos)-Kd*vel; % PD controller
    motor_u = [1 1;1 -1]*u_xy; % Convert into left/right motor voltages
    motor_u = max(-24,min(24,motor_u)); % Limit motor voltages to [-24,24] V

end

% Nozzle control
if q==5 || q==6 % Heater Off and Loading Filament:

    nozzle_u = 0; % Switch off the nozzle heater

elseif q==7 % Heating

    nozzle_u = 24; % Apply 24 V to heat the nozzle

else % All other modes:

    nozzle_u = unhold; % Maintain the nozzle at printing temperature

end

% Derivatives
dz = [A*z(1:6)+B*motor_u;
    nozzle_rhs(z,nozzle_u)];

end