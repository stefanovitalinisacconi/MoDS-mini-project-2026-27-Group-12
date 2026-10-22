function dz = squareF(~,z,q,theta_vertices,A,B,Ktheta,Komega,unhold,nozzle_rhs)

target_theta = theta_vertices(:,q+1); % target motor angles 

um = Ktheta*(target_theta-z(1:2))-Komega*z(3:4); % PD controller 

dz = [A*z(1:6)+B*um;
    nozzle_rhs(z,unhold)];

end