function [z,q,mem] = printerJump(tt,z,q,mem,C,event_id)
% Controller transitions preserve continuous states.
% Physical impacts reset the normal velocity.

if event_id == 2 || event_id == 3

    axis_id = event_id-1;
    mem.contact(axis_id) = true;

    % Stop the normal velocity at the rigid boundary
    S = C(:,1:2);
    velocity = S*z(3:4);
    velocity(axis_id) = 0;
    z(3:4) = S\velocity;

    return

elseif event_id == 4 || event_id == 5

    % Release contact without resetting continuous states
    axis_id = event_id-3;
    mem.contact(axis_id) = false;

    return
end

if q == 3

    % Use the active homing command and measured motor angular velocities
    omegaTol = 1e-3; % Angular velocity tolerance [rad/s]
    stopped = all(abs(z(3:4)) <= omegaTol);

    if stopped

        if mem.axis == 'H'
            % (H,0)
            if mem.parked == 0
                mem.parked = 1;     % No edge -> Left edge
            elseif mem.parked == 2
                mem.parked = 3;     % Bottom edge -> Origin
            end

        else
            %(V,0)
            if mem.parked == 0
                mem.parked = 2;     % No edge -> Bottom edge
            elseif mem.parked == 1
                mem.parked = 3;     % Left edge -> Origin
            end
        end

        if mem.parked == 3
            q = 4;                 % Moving to Wipe -> Wiping
            mem.entry = tt;

        else
            % Continue towards the remaining edge
            mem.hold = C*z(1:6);

            if mem.parked == 1
                mem.axis = 'V';
            elseif mem.parked == 2
                mem.axis = 'H';
            end
        end
    end

    % Moving observations leave the detector state unchanged
    mem.nextSample = tt+mem.samplePeriod;

    return
end

if q==1 % Printing

    mem.progress = mem.progress+(tt-mem.entry); % Add the duration of the current printing segment to the total printing time

    if ~mem.replaced %First interruption

        mem.saved = C*z(1:6); % Save the print head position
        mem.replaced = true; % filament replacement is being done
        q = 2;  %Stop and Save

    else

        mem.done = true;

    end

elseif q == 2 % Stop and Save -> Moving to Wipe

    q = 3;

    mem.parked = 0;
    mem.axis = mem.firstAxis;
    mem.hold = C*z(1:6);
    mem.nextSample = tt+mem.samplePeriod;

elseif q==8 % returning

    q = 1; % Resume printing from the previous point

else % other modes

    q = q+1;

end

mem.entry = tt; % Store the entry time of the new discrete mode to measure the time spent in each mode

end