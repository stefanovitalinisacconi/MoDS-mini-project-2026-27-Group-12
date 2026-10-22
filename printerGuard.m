function [value,stop,direction] = printerGuard(tt,z,q,mem,C,Tlow,Tref,t_end,flow)
% defining transition condition for each mode

if q==1 % Printing

    if ~mem.replaced % first printing phase
        value = tt-2; % trigger filament replacement after 2 seconds (arbitrary amount of time)

    else % second printing phase

        value = mem.progress+(tt-mem.entry)-t_end; % stop when total printing time reaches 10 seconds

    end

    direction = 1;     % detect crossing from negative to positive

elseif q == 3 % Moving to Wipe

    value = tt-mem.nextSample;
    direction = 1; 


elseif q == 2 || q == 8 % Stop and Save / Returning

    target = mem.saved;
    
    pos = C*z(1:6);
    vel = C(:,1:2)*z(3:4);
    % reach the saved position with sufficiently small velocity
    value = max(norm(pos-target)-1e-3,norm(vel)-1e-3);
    direction = -1;    % detect crossing from positive to negative

elseif q==4 || q==6 % Wiping / Loading filament

    value = tt-mem.entry-2; % each operation lasts 2 seconds from mode entry (arbitrary amount of time)
    direction = 1; 

elseif q==5 % Heater off

    value = z(7)-Tlow; % transition when nozzle temperature decreases to Tlow
    direction = -1; 

elseif q==7 % Heating

    value = z(7)-Tref; % transition when nozzle temperature reaches Tref
    direction = 1; 

end

% physical lower stop events
pos = C*z(1:6);
[~,free_acc] = flow(tt,z);

impact = pos;
release = ones(2,1);

for k = 1:2
    if mem.contact(k)
        impact(k) = 1;                  % disable repeated impacts
        release(k) = free_acc(k)-1e-9;   % detect acceleration away from the stop
    end
end

% 1 controller event, 2/3 are left/bottom impact, 4/5 are left/bottom release
value = [value; impact; release];
direction = [direction; -1; -1; 1; 1];
stop = ones(size(value));

end