function C = MeasurmentJacobian( y, Xhat, m, Cf, g )

% state-vector: X = [u v w p q r xf yf zf e0 ex ey ez w1 w2 w3 w4]'


u = Xhat(1);
v = Xhat(2);
w = Xhat(3);
p = Xhat(4);
q = Xhat(5);
r = Xhat(6);
% xf = Xhat(7);
% yf = Xhat(8);
% zf = Xhat(9);
e0 = Xhat(10);
ex = Xhat(11); 
ey = Xhat(12);
ez = Xhat(13);
w1 = Xhat(14);
w2 = Xhat(15);
w3 = Xhat(16);
w4 = Xhat(17);

C = zeros(length(y), length(Xhat));
%          u  v  w  p  q  r xf yf zf  e0      ex      ey     ez      w1         w2         w3         w4
C(1,:) = [ 0  r -q  0 -w  v  0  0  0 -2*g*ey  2*g*ez -2*g*e0 2*g*ex  0          0          0          0];
C(2,:) = [-r  0  p  w  0 -u  0  0  0  2*g*ex  2*g*e0  2*g*ez 2*g*ey  0          0          0          0];
C(3,:) = [ q -p  0 -v  u  0  0  0  0  2*g*e0 -2*g*ex -2*g*ey 2*g*ez -2*Cf*w1/m -2*Cf*w2/m -2*Cf*w3/m -2*Cf*w4/m];
C(4,4) = 1;
C(5,5) = 1;
C(6,6) = 1;


end