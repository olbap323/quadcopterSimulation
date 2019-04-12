function h = MeasurmentDynamics(Xhat, g, m, Cf )

h = zeros(6,1);

%%%% Forces and moments produced by motor in body frame
w_sqrd = (Xhat(14:17).^2);
% Forces
F = (Cf * w_sqrd);
Fb = [ 0; 
       0;
      -sum(F) ]; 

  
%%%% Rotation matrices
R_b2f = Rotation_b2f_Q( Xhat(10:13,1) );             % go from body to fixed
R_f2b = R_b2f';                             % go from fixed to body

h(1:3,1) = (1/m)*Fb + g*R_f2b(1:3,3) - cross( Xhat(4:6,1), Xhat(1:3,1) ); 
h(4:6,1) = Xhat(4:6,1);

end