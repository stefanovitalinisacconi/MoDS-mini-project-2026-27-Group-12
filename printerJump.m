function [z,q,mem] = printerJump(tt,z,q,mem,C)
% The jump map updates the discrete mode and the controller memory, the physical state z is not modified (z+ = z-)

if q==1 % Printing

    mem.progress = mem.progress+(tt-mem.entry); % Add the duration of the current printing segment to the total printing time

    if ~mem.replaced %First interruption

        mem.saved = C*z(1:6); % Save the print head position
        mem.replaced = true; % filament replacement is being done
        q = 2;  %Stop and Save

    else

        mem.done = true;

    end

elseif q==8 % returning

    q = 1; % Resume printing from the previous point

else % other modes

    q = q+1;

end

mem.entry = tt; % Store the entry time of the new discrete mode, to measure the time spent in each mode

end