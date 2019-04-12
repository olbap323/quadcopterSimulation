function R = Rotation_b2f_Q( Q )
%ROTATION_B2F_Q Summary of this function goes here
%   rotation matrix from body to fixed given a quaternion
e0 = Q(1);
ex = Q(2);
ey = Q(3);
ez = Q(4);
e0sq = Q(1)^2;
exsq = Q(2)^2;
eysq = Q(3)^2;
ezsq = Q(4)^2;

R = [e0sq+exsq-eysq-ezsq  2*(ex*ey-ez*e0)      2*(ex*ez+ey*e0);
     2*(ex*ey+ez*e0)      eysq+e0sq-exsq-ezsq  2*(ey*ez-ex*e0);
     2*(ex*ez-ey*e0)      2*(ey*ez+ex*e0)      ezsq+e0sq-exsq-eysq];
end

