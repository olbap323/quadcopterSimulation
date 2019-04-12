function eAngs = Q2eAng( Q )
%Q2EANG Summary of this function goes here
%   quaternion to euler angles

e0 = Q(1);
ex = Q(2);
ey = Q(3);
ez = Q(4);
e0sq = Q(1)^2;
exsq = Q(2)^2;
eysq = Q(3)^2;
ezsq = Q(4)^2;


eAngs = [ atan2(2*(e0*ex+ey*ez),e0sq+ezsq-exsq-eysq);
          asin(2*(e0*ey-ex*ez));
          atan2(2*(e0*ez+ex*ey),e0sq+exsq-eysq-ezsq)];




end

