function [dz,free_acc] = printerFlow(tt,z,q,mem,A,B,C,t_ref,u_ref,r_ref,v_ref,nozzle_rhs,Kp,Kd,unhold)
% computing the derivatives of the 9 physical states

pos = C*z(1:6); % compute position from motor angles
vel = C(:,1:2)*z(3:4); % compute velocity from motor angular velocities

if q == 1  % Printing
    
    t_print = mem.progress + tt-mem.entry; % calculate the printing time
    t_print = min(max(t_print,t_ref(1)),t_ref(end)); % keep printing time within the available reference interval
    motor_u = interp1(t_ref,u_ref,t_print,'linear')'; % obtain the motor voltages from the Q1.4 reference signals
    
    if mem.replaced

        % desired position and velocity
        r_des = interp1(t_ref,r_ref,t_print,'linear')';
        v_des = interp1(t_ref,v_ref,t_print,'linear')';

        u_xy = Kp*(r_des-pos)+Kd*(v_des-vel);  % PD feedback for trajectory tracking

        motor_u = motor_u + [1 1;1 -1]*u_xy;
        motor_u = max(-24,min(24,motor_u));
        
    end  

elseif q == 3 % Moving to Wipe

    % hold the inactive axis at its entry position
    u_xy = Kp*(mem.hold-pos)-Kd*vel;
    % maintain a preload at previously reached stops
    u_xy(mem.contact) = -mem.homeVoltage-Kd*vel(mem.contact);

    if mem.axis == 'H'
        % command motion towards the left stop
        u_xy(1) = -mem.homeVoltage-Kd*vel(1);
    else
        % command motion towards the bottom stop
        u_xy(2) = -mem.homeVoltage-Kd*vel(2);
    end

    % convert cartesian commands into motor voltages
    motor_u = [1 1; 1 -1]*u_xy;
    motor_u = max(-24,min(24,motor_u));

elseif q >= 4 && q <= 7 % Keep the head against both stops

    u_xy = -mem.homeVoltage*ones(2,1);
    motor_u = [1 1;1 -1]*u_xy;

else % Other modes

    target = mem.saved; % Stop and Save / Returning

    u_xy = Kp*(target-pos)-Kd*vel; % PD 
    motor_u = [1 1;1 -1]*u_xy; % convert into left/right motor voltages
    motor_u = max(-24,min(24,motor_u)); % limit motor voltages to [-24,24] V

end

% Nozzle control
if q==5 || q==6 % Heater Off and Loading Filament

    nozzle_u = 0; % switch off the nozzle heater

elseif q==7 % Heating

    nozzle_u = 24; % apply 24 V to heat the nozzle

else % All other modes:

    nozzle_u = unhold; % maintain the nozzle at printing temperature

end

% derivatives
dz = [A*z(1:6)+B*motor_u;
    nozzle_rhs(z,nozzle_u)];

% free Cartesian acceleration, before contact reactions
S = C(:,1:2);
free_acc = S*dz(3:4);

% constrain normal motion at active contacts
acc = free_acc;
acc(mem.contact) = 0;
dz(3:4) = S\acc;

end