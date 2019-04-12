function a2q = Ang_rate2Q_rate( Q )
%ANG_RATE2Q_RATE Summary of this function goes here
% matrix to convert from angular rate to quaternion rate

e0 = Q(1);
ex = Q(2);
ey = Q(3);
ez = Q(4);

a2q = [-ex -ey -ez;
        e0 -ez  ey;
        ez  e0 -ex;
       -ey  ex  e0];
   
a2q = 0.5*a2q;

end

