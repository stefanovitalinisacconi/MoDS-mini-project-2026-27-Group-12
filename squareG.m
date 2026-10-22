function [value,stop,direction] = squareG(~,z,q,theta_vertices,theta_tol,omega_tol)

target_theta = theta_vertices(:,q+1); % target motor angles

position_error = norm(z(1:2)-target_theta);
angular_speed = norm(z(3:4));

value = max(position_error-theta_tol,angular_speed-omega_tol);

direction = -1;
stop = 1;

end