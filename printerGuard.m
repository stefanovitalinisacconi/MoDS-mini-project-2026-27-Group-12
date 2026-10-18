function [value,stop,direction] = printerGuard(tt,z,q,mem,C,Tlow,Tref,t_end)
% The guard function defines the transition condition for each mode

if q==1 % Printing

    if ~mem.replaced % First printing phase:
        value = tt-2; % Trigger filament replacement after 2 seconds (arbitrary amount of time)

    else % Second printing phase:

        value = mem.progress+(tt-mem.entry)-t_end; % Stop when total printing time reaches 10 seconds

    end

    direction = 1;     % Detect crossing from negative to positive

elseif q==2 || q==3 || q==8 % Stop And Save/ Moving to wipe / Returning

    if q==3 % Moving to Wipe
        target = [0;0]; % target is the origin

    else % Stop and Save / Returning
        target = mem.saved; % target is the saved position

    end

    % Current position and velocity
    pos = C*z(1:6);
    vel = C(:,1:2)*z(3:4);
    % Both conditions must be satisfied: position error <= 1 mm / velocity <= 1 mm/s
    value = max(norm(pos-target)-1e-3,norm(vel)-1e-3);
    direction = -1;    % Detect crossing from positive to negative

elseif q==4 || q==6 % Wiping / Loading filament

    value = tt-mem.entry-2; % Each operation lasts 2 seconds from mode entry (arbitrary amount of time)
    direction = 1; % Detect crossing from negative to positive

elseif q==5 % Heater off

    value = z(7)-Tlow; % Transition when nozzle temperature decreases to Tlow
    direction = -1; % Detect crossing from positive to negative

elseif q==7 % Heating

    value = z(7)-Tref; % Transition when nozzle temperature reaches Tref
    direction = 1; % Detect crossing from negative to positive

end

stop = 1;
% The integration stops when the event is detected.

end