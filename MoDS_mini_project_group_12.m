%% Mini project 2026/27 - Modelling of Dynamical Systems 

% Stefano Vitalini Sacconi & Giovanni Masetti

clear; clc; close all;

%% Q1.4 - DRAWING SHAPES

% Parameters
m = 0.3; % Tool head mass [kg]
rm = 0.025; % Motor pulley radius [m]
Jm = 2e-5; % Motor rotational inertia [kg*m^2]
Km = 0.05; % Motor constant [Nm/A] or [V*s/rad]
Rm = 2; % Motor resistance [Ohm]
Lm = 0.01; % Motor inductance [H]
mu_lx = 0.4; % Friction coefficient along x [Ns/m]
mu_ly = 0.5; % Friction coefficient along y [Ns/m]
mu_r = 0.02; % Rolling bearing friction coefficient [Ns/m]
Jeq = Jm + (m*rm^2)/2; % Equivalent rotational inertia
d1 = (rm^2/2)*(mu_lx + mu_ly + 6*mu_r); % Equivalent damping coefficients
d2 = (rm^2/2)*(mu_lx - mu_ly + 2*mu_r); % Equivalent damping coefficients

t_end = 10; % Total simulation time [s]
dt = 0.001; % Time step [s]
t = (0:dt:t_end)';

%% Ellipse trajectory
xc = 0.06; % Ellipse center [m]
yc = 0.06; % Ellipse center [m]
a = 0.03; % Semi-axis along x [m]
b = 0.02; % Semi-axis along y [m]
omega = 2*pi/t_end; % Angular frequency [rad/s]

% Desired position
xd = xc + a*cos(omega*t);
yd = yc + b*sin(omega*t);

% Desired velocity
xd_dot = -a*omega*sin(omega*t);
yd_dot =  b*omega*cos(omega*t);

% Desired acceleration
xd_ddot = -a*omega^2*cos(omega*t);
yd_ddot = -b*omega^2*sin(omega*t);

% Desired jerk (third derivative)
xd_dddot =  a*omega^3*sin(omega*t);
yd_dddot = -b*omega^3*cos(omega*t);

% State space matrices

A = [0  0       1          0         0       0;
     0  0       0          1         0       0;
     0  0     -d1/Jeq   -d2/Jeq    Km/Jeq    0;
     0  0     -d2/Jeq   -d1/Jeq      0     Km/Jeq;
     0  0     -Km/Lm       0      -Rm/Lm     0;
     0  0       0       -Km/Lm       0    -Rm/Lm];

B = [0       0;
     0       0;
     0       0;
     0       0;
     1/Lm    0;
     0       1/Lm];

C = [rm/2   rm/2    0  0  0  0;
     rm/2  -rm/2    0  0  0  0];

% Direct transmission matrix
D = zeros(2,2);

% Create continuous-time state-space model
sys = ss(A,B,C,D);

% Required currents
ix = (Jeq*xd_ddot + (d1+d2)*xd_dot)/(Km*rm);
iy = (Jeq*yd_ddot + (d1-d2)*yd_dot)/(Km*rm);

% Time derivatives of required currents
ix_dot = (Jeq*xd_dddot + (d1+d2)*xd_ddot)/(Km*rm);
iy_dot = (Jeq*yd_dddot + (d1-d2)*yd_ddot)/(Km*rm);

% Required voltages in the decoupled coordinates
ux = Lm*ix_dot + Rm*ix + (Km/rm)*xd_dot;
uy = Lm*iy_dot + Rm*iy + (Km/rm)*yd_dot;

% Convert to left and right motor voltages
ul = ux + uy;
ur = ux - uy;

% Input matrix for simulation
u = [ul, ur];

% Initial state:
% [theta_l, theta_r, omega_l, omega_r, i_l, i_r]
% Initial conditions consistent with desired trajectory
x0 = [(xd(1)+yd(1))/rm;
    (xd(1)-yd(1))/rm;
    (xd_dot(1)+yd_dot(1))/rm;
    (xd_dot(1)-yd_dot(1))/rm;
    ix(1)+iy(1);
    ix(1)-iy(1)];

% Simulate the continuous-time state-space model
[Y, ~, X] = lsim(sys,u,t,x0);

% Simulated tool head positions
x_sim = Y(:,1);
y_sim = Y(:,2);

% Tracking errors
error_x = xd - x_sim;
error_y = yd - y_sim;

% Maximum absolute tracking errors
max_error_x = max(abs(error_x));
max_error_y = max(abs(error_y));

fprintf('Maximum error along x: %.8e m\n', max_error_x);
fprintf('Maximum error along y: %.8e m\n', max_error_y);

%% Ellipse plots
% Desired vs simulated trajectory
figure;
plot(xd,yd,'--','LineWidth',1.5);
hold on;
plot(x_sim,y_sim,'LineWidth',1.5);
grid on;
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('CoreXY - Desired vs Simulated Trajectory');
legend('Desired trajectory','Simulated trajectory');

% Motor input voltages
figure;
plot(t,ul,'LineWidth',1.5);
hold on;
plot(t,ur,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Voltage [V]');
title('Motor Input Voltages');
legend('u_l','u_r');

%% Heart trajectory
xc_h = 0.06;           % Horizontal reference position [m]
yc_h = 0.07;           % Vertical reference position [m]
s = 0.002;           % Scale factor [m]
omega = 2*pi/t_end;  % One complete heart in t_end seconds

% Desired position
xd_h = xc_h + s*(12*sin(omega*t) - 4*sin(3*omega*t));
yd_h = yc_h + s*(13*cos(omega*t) - 5*cos(2*omega*t) - 2*cos(3*omega*t) - cos(4*omega*t));

% Desired velocity
xd_h_dot = s*omega*(12*cos(omega*t) - 12*cos(3*omega*t));
yd_h_dot = s*omega*(-13*sin(omega*t) + 10*sin(2*omega*t) + 6*sin(3*omega*t) + 4*sin(4*omega*t));

% Desired acceleration
xd_h_ddot = s*omega^2*(-12*sin(omega*t) + 36*sin(3*omega*t));
yd_h_ddot = s*omega^2*(-13*cos(omega*t) + 20*cos(2*omega*t) + 18*cos(3*omega*t) + 16*cos(4*omega*t));

% Desired third derivative
xd_h_dddot = s*omega^3*(-12*cos(omega*t) + 108*cos(3*omega*t));
yd_h_dddot = s*omega^3*(13*sin(omega*t) - 40*sin(2*omega*t) - 54*sin(3*omega*t) - 64*sin(4*omega*t));

% Required currents
ix_h = (Jeq*xd_h_ddot + (d1+d2)*xd_h_dot)/(Km*rm);
iy_h = (Jeq*yd_h_ddot + (d1-d2)*yd_h_dot)/(Km*rm);

% Current derivatives
ix_h_dot = (Jeq*xd_h_dddot + (d1+d2)*xd_h_ddot)/(Km*rm);
iy_h_dot = (Jeq*yd_h_dddot + (d1-d2)*yd_h_ddot)/(Km*rm);

% Required voltages in decoupled coordinates
ux_h = Lm*ix_h_dot + Rm*ix_h + (Km/rm)*xd_h_dot;
uy_h = Lm*iy_h_dot + Rm*iy_h + (Km/rm)*yd_h_dot;

% Motor input voltages
ul_h = ux_h + uy_h;
ur_h = ux_h - uy_h;

% Input matrix
u_h = [ul_h, ur_h];

% Initial conditions
x0_h = [(xd_h(1)+yd_h(1))/rm;
    (xd_h(1)-yd_h(1))/rm;
    (xd_h_dot(1)+yd_h_dot(1))/rm;
    (xd_h_dot(1)-yd_h_dot(1))/rm;
    ix_h(1)+iy_h(1);
    ix_h(1)-iy_h(1)];

% Simulation
[Y_h, ~, X_h] = lsim(sys,u_h,t,x0_h);

% Simulated tool head positions
x_sim_h = Y_h(:,1);
y_sim_h = Y_h(:,2);

% Tracking errors
error_x_h = xd_h - x_sim_h;
error_y_h = yd_h - y_sim_h;

fprintf('Heart - Maximum error x: %.8e m\n', max(abs(error_x_h)));
fprintf('Heart - Maximum error y: %.8e m\n', max(abs(error_y_h)));

%% Heart plots
% Desired vs simulated trajectory
figure;
plot(xd_h,yd_h,'--','LineWidth',1.5);
hold on;
plot(x_sim_h,y_sim_h,'LineWidth',1.5);
grid on;
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('CoreXY - Heart Trajectory');
legend('Desired trajectory','Simulated trajectory');

% Motor input voltages
figure;
plot(t,ul_h,'LineWidth',1.5);
hold on;
plot(t,ur_h,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('Voltage [V]');
title('Heart - Motor Input Voltages');
legend('u_l','u_r');

%% Q1.6 

% PLA and heat-exchange parameters
rho_f = 1240;       % Filament density [kg/m^3]
c_f = 1800;         % Specific heat [J/(kg*K)]
k_f = 0.13;         % Thermal conductivity [W/(m*K)]
w = 0.4e-3;         % Filament square cross-section side [m]
h_b = 100;         % Heat transfer coefficient to plate [W/(m^2*K)]
h_e = 30;          % Heat transfer coefficient to air [W/(m^2*K)]
T_print_C = 210;    % COMMON nozzle [C]
T_plate_C = 60;     % Plate temperature [C]
T_air_C = 20;       % Ambient temperature [C]
T_glass_C = 60;     % PLA glass transition temperature [C]

% Discretized filament deposited along the Q1.4 ellipse
s_p = cumtrapz(t,hypot(xd_dot,yd_dot));  % Printed arc length [m]
L_fil = s_p(end);                         % Ellipse perimeter [m]
N = 400;                                 % Number of filament cells
ds = L_fil/N;                            % Cell length [m]
s_c = ((1:N)'-0.5)*ds;                   % Cell centres [m]
t_dep = interp1(s_p,t,s_c);              % Deposition times [s]
tau_cool = rho_f*c_f*w/(h_b+3*h_e);     % Cooling time constant [s]
T_inf = (h_b*T_plate_C+3*h_e*T_air_C)/(h_b+3*h_e);

% Forward Euler on the growing printed domain
t6 = 0:dt:40;
n_sv = round(0.1/dt);
t_sv = t6(1:n_sv:end);
T_sv = nan(N,length(t_sv));
T_cells = T_print_C*ones(N,1);

for k = 1:length(t6)
    active = t_dep<=t6(k);
    if mod(k-1,n_sv)==0
        T_sv(active,(k-1)/n_sv+1) = T_cells(active);
    end
    left = circshift(active,1);
    right = circshift(active,-1);
    if ~active(N)
        left(1)=false;
        right(N)=false;
    end
    heat = k_f/ds^2*(left.*(circshift(T_cells,1)-T_cells) ...
        + right.*(circshift(T_cells,-1)-T_cells)) ...
        + h_b/w*(T_plate_C-T_cells)+3*h_e/w*(T_air_C-T_cells);
    T_cells = T_cells+dt*active.*heat/(rho_f*c_f);
end

T_an = T_inf+(T_print_C-T_inf)*exp(-(t_sv-t_dep)/tau_cool);
fprintf('Filament length = %.1f mm, cooling tau = %.2f s, T_inf = %.1f C\n', ...
    1e3*L_fil,tau_cool,T_inf);
fprintf('Whole layer below T_g at t = %.1f s\n', ...
    t_sv(find(t_sv>=t_end & max(T_sv)<T_glass_C,1)));
fprintf('Nearly uniform (max-min < 1 K) at t = %.1f s\n', ...
    t_sv(find(t_sv>t_end & max(T_sv)-min(T_sv)<1,1)));
fprintf('Conduction changes T by > 1 K over %.1f mm only\n', ...
    1e3*ds*max(sum(abs(T_sv-T_an)>1)));

figure('Position',[100 100 900 650]);
for panel = 1:2
    j = round(5*panel/0.1)+1;
    on = ~isnan(T_sv(:,j));
    subplot(2,2,panel);
    plot(1e3*xd,1e3*yd,':','Color',[0.7 0.7 0.7]); hold on;
    scatter(1e3*interp1(t,xd,t_dep(on)), ...
        1e3*interp1(t,yd,t_dep(on)),12,T_sv(on,j),'filled');
    plot(1e3*(xc+a*cos(omega*5*panel)), ...
        1e3*(yc+b*sin(omega*5*panel)),'kv','MarkerFaceColor','k');
    axis equal; grid on; caxis([T_inf T_print_C]); colormap(jet);
    cb = colorbar; ylabel(cb,'T [^oC]');
    xlabel('x [mm]'); ylabel('y [mm]');
    title(sprintf('layer at t = %g s',5*panel));
end

subplot(2,2,3); hold on;
for tp = [2.5 5 7.5 10 15 20 30]
    j = round(tp/0.1)+1;
    last = find(~isnan(T_sv(:,j)),1,'last');
    plot(1e3*s_c,T_sv(:,j));
    text(1e3*s_c(last)+2,T_sv(last,j)+6,sprintf('%g s',tp));
end
grid on; axis([0 180 0 230]);
xlabel('s [mm]'); ylabel('T [^oC]'); title('temperature along the path');

subplot(2,2,4);
plot(t_sv,T_sv([1 N/2 N],:),t_sv,max(T_sv),'k', ...
    [0 40],[T_inf T_inf],'k--');
grid on; axis([0 40 0 230]);
xlabel('t [s]'); ylabel('T [^oC]'); title('temperature in time');
legend('s = 0','s = L/2','s = L','max over layer','T_\infty');
print(gcf,'q1_6_filament.png','-dpng','-r300');

% Nozzle parameters
Rn = 10;             % Heater resistance [Ohm]
Ln = 0.002;          % Inductance [H]
Cn = 220e-6;         % Capacitance [F]
Ct = 10;             % Thermal capacity [J/K]
Rt = 5;              % Thermal resistance [K/W]
Te = T_air_C+273.15;         
Tref = T_print_C+273.15;     
Tlow = 80+273.15;            % Loading temperature [K]
unhold = sqrt(Rn*(Tref-Te)/Rt);  % Holding voltage [V]

nozzle_rhs = @(zz,un) [(zz(8)^2/Rn-(zz(7)-Te)/Rt)/Ct;
    (zz(9)-zz(8)/Rn)/Cn;
    (un-zz(8))/Ln];

%% Q3.1 Filament replacement

% The printer is modeled as a hybrid system with 8 operating modes.
% continuous dynamics depend on the operating mode and the contact state.
% Events update the mode and memory and contacts.

Kp = 20;       % PD position gain [V/m]
Kd = 4;        % PD velocity gain [V*s/m]

% Desired Cartesian position and velocity from Q1.4
r_ref = [xd(:),yd(:)];
v_ref = [xd_dot(:),yd_dot(:)];

% Initial physical state: 6 motor states and 3 nozzle states
z = [x0; Tref; unhold; unhold/Rn];

q = 1;         % Initial mode = printing
t_start = 0;   % Initial simulation time

% Controller memory
mem.entry = 0;          % Entry time of the current mode
mem.progress = 0;       % Accumulated printing time
mem.replaced = false;   % Filament replacement request handled
mem.done = false;       % Printing completed

% Head-parked detector
mem.parked = 0;          % 0 No edge, 1 Left edge, 2 Bottom edge, 3 Origin

% Homing commands
mem.firstAxis = 'H';     % Select 'H' or 'V' as the first command
mem.axis = mem.firstAxis;
mem.hold = C*z(1:6);     % Position reference for the inactive axis
mem.homeVoltage = 0.4;   % Homing voltage magnitude [V]

% Periodic detector observations
mem.samplePeriod = 0.1;  % Sampling period [s]
mem.nextSample = Inf;    % Enabled when entering Moving to Wipe

% Physical lower-stop contacts
mem.contact = false(2,1); % [left; bottom]

% Store simulation results for plots
t_sim = [];       % Simulation times
z_sim = [];       % Continuous states
mode_sim = [];    % Discrete modes

while ~mem.done

    % FLOW MAP
    flow = @(tt,zz) printerFlow(tt,zz,q,mem,A,B,C,t,u,r_ref,v_ref,nozzle_rhs,Kp,Kd,unhold);

    % GUARD
    guard = @(tt,zz) printerGuard(tt,zz,q,mem,C,Tlow,Tref,t(end),flow);

    % Integrate continuous dynamics until the guard is reached
    [ts,zs,te,ze,ie] = ode15s(flow,[t_start 180],z,odeset('Events',guard));
    
    if isempty(te)
        error('Guard not reached in mode %d',q);
    end

    % INVARIANTS CHECK
    xy = (C*zs(:,1:6)')'; % convert motor angles into cartesian positions

    % print head must remain inside the 20cmx20cm workspace
    valid = all(xy(:)>=-1e-5 & xy(:)<=0.20+1e-5);

    % during the states wiping, heater off, loading and heating, the print head must remain near the wiping location
    if q>=4 && q<=7
        valid = valid && all(hypot(xy(:,1),xy(:,2))<=1e-3+1e-5);
    end
    
    % Temperature constraints in the corresponding modes
    if q==5
        valid = valid && all(zs(:,7)>=Tlow-1e-3);
    elseif q==6
        valid = valid && all(zs(:,7)<=Tlow+1e-3);
    elseif q==7
        valid = valid && all(zs(:,7)<=Tref+1e-3);
    end

    assert(valid,'Invariant violated');

    % Store the continuous trajectory and active discrete mode
    t_sim = [t_sim;ts];
    z_sim = [z_sim;zs];
    mode_sim = [mode_sim;q*ones(length(ts),1)];

    % JUMP MAP
    q_old = q;
    t_start = te(end);
    z = ze(end,:)';

    % Collect events occurring at the final integration time
    event_ids = unique(ie(abs(te-t_start) < 1e-8));

    % Handle physical events before a simultaneous controller event.
    event_ids = [event_ids(event_ids ~= 1); event_ids(event_ids == 1)];

    for event_id = event_ids(:)'

    [z,q,mem] = printerJump(t_start,z,q,mem,C,event_id);

    end

    % Display transitions
    if ~mem.done && q ~= q_old
        fprintf('Mode %d -> %d at t = %.2f s\n',q_old,q,t_start);
    end

end

fprintf('Printing completed at t = %.2f s\n',t_start);

%% Plots
Q = ["Printing","Stop and Save","Moving to Wipe","Wiping","Heater Off","Loading Filament","Heating","Returning"];

% Discrete mode evolution
figure;
stairs(t_sim,mode_sim,'LineWidth',1.5);
yticks(1:8);
yticklabels(Q);
grid on;
xlabel('Time [s]');
ylabel('Discrete mode');
title('Hybrid System - Discrete Modes');

% Nozzle temperature
figure;
plot(t_sim,z_sim(:,7)-273.15,'LineWidth',1.5);
hold on;
yline(T_print_C,'--');
yline(Tlow-273.15,'--');
grid on;
xlabel('Time [s]');
ylabel('Temperature [C]');
title('Hybrid System - Nozzle Temperature');

xy = (C*z_sim(:,1:6)')'; % convert motor states into cartesian positions

% Print-head trajectory during filament replacing process
figure; hold on;
plot(xd,yd,'k--'); % desired trajectory
printing_xy = xy;
printing_xy(mode_sim~=1,:) = NaN;
plot(printing_xy(:,1),printing_xy(:,2),'b','LineWidth',2); % printing trajectory
travel_xy = xy;
travel_xy(~ismember(mode_sim,[2 3 8]),:) = NaN;
plot(travel_xy(:,1),travel_xy(:,2),'k:'); % travel trajectory
plot(0,0,'ko','MarkerFaceColor','k'); % wiping location
plot(mem.saved(1),mem.saved(2),'bs','MarkerFaceColor','b'); % interruption point
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Filament Replacement');
legend('Desired trajectory','Printing','Travel','Wipe location','Interruption');

