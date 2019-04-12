function A = ModelJacobian( Xhat, wd, g, L, In, m, Km, Cf, Cm )
%MODELJACOBIAN 
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
% w1 = wd(1) - Xhat(14);
% w2 = wd(2) - Xhat(15);
% w3 = wd(3) - Xhat(16);
% w4 = wd(4) - Xhat(17);
w1 = Xhat(14);
w2 = Xhat(15);
w3 = Xhat(16);
w4 = Xhat(17);
Ix = In(1,1);
Iy = In(2,2);
Iz = In(3,3);
A =...
[                         0,                         r,                        -q,                 0,               -w,                 v, 0, 0, 0,                  -2*ey*g,                   2*ez*g,                  -2*e0*g,                   2*ex*g,              0,              0,               0,               0;
                         -r,                         0,                         p,                 w,                0,                -u, 0, 0, 0,                   2*ex*g,                   2*e0*g,                   2*ez*g,                   2*ey*g,              0,              0,               0,               0;
                          q,                        -p,                         0,                -v,                u,                 0, 0, 0, 0,                   2*e0*g,                  -2*ex*g,                  -2*ey*g,                   2*ez*g,   -(2*Cf*w1)/m,   -(2*Cf*w2)/m,    -(2*Cf*w3)/m,    -(2*Cf*w4)/m;
                          0,                         0,                         0,                 0, (r*(Iy - Iz))/Ix,  (q*(Iy - Iz))/Ix, 0, 0, 0,                        0,                        0,                        0,                        0,              0, (2*Cf*L*w2)/Ix,               0, -(2*Cf*L*w4)/Ix;
                          0,                         0,                         0, -(r*(Ix - Iz))/Iy,                0, -(p*(Ix - Iz))/Iy, 0, 0, 0,                        0,                        0,                        0,                        0, (2*Cf*L*w1)/Iy,              0, -(2*Cf*L*w3)/Iy,               0;
                          0,                         0,                         0,  (q*(Ix - Iy))/Iz, (p*(Ix - Iy))/Iz,                 0, 0, 0, 0,                        0,                        0,                        0,                        0,   (2*Cm*w1)/Iz,  -(2*Cm*w2)/Iz,    (2*Cm*w3)/Iz,   -(2*Cm*w4)/Iz;
  e0^2 + ex^2 - ey^2 - ez^2,         2*ex*ey - 2*e0*ez,         2*e0*ey + 2*ex*ez,                 0,                0,                 0, 0, 0, 0, 2*e0*u - 2*ez*v + 2*ey*w, 2*ex*u + 2*ey*v + 2*ez*w, 2*ex*v - 2*ey*u + 2*e0*w, 2*ex*w - 2*e0*v - 2*ez*u,              0,              0,               0,               0;
          2*e0*ez + 2*ex*ey, e0^2 - ex^2 + ey^2 - ez^2,         2*ey*ez - 2*e0*ex,                 0,                0,                 0, 0, 0, 0, 2*ez*u + 2*e0*v - 2*ex*w, 2*ey*u - 2*ex*v - 2*e0*w, 2*ex*u + 2*ey*v + 2*ez*w, 2*e0*u - 2*ez*v + 2*ey*w,              0,              0,               0,               0;
          2*ex*ez - 2*e0*ey,         2*e0*ex + 2*ey*ez, e0^2 - ex^2 - ey^2 + ez^2,                 0,                0,                 0, 0, 0, 0, 2*ex*v - 2*ey*u + 2*e0*w, 2*ez*u + 2*e0*v - 2*ex*w, 2*ez*v - 2*e0*u - 2*ey*w, 2*ex*u + 2*ey*v + 2*ez*w,              0,              0,               0,               0;
                          0,                         0,                         0,             -ex/2,            -ey/2,             -ez/2, 0, 0, 0,                        0,                     -p/2,                     -q/2,                     -r/2,              0,              0,               0,               0;
                          0,                         0,                         0,              e0/2,            -ez/2,              ey/2, 0, 0, 0,                      p/2,                        0,                      r/2,                     -q/2,              0,              0,               0,               0;
                          0,                         0,                         0,              ez/2,             e0/2,             -ex/2, 0, 0, 0,                      q/2,                     -r/2,                        0,                      p/2,              0,              0,               0,               0;
                          0,                         0,                         0,             -ey/2,             ex/2,              e0/2, 0, 0, 0,                      r/2,                      q/2,                     -p/2,                        0,              0,              0,               0,               0;
                          0,                         0,                         0,                 0,                0,                 0, 0, 0, 0,                        0,                        0,                        0,                        0,            -Km,              0,               0,               0;
                          0,                         0,                         0,                 0,                0,                 0, 0, 0, 0,                        0,                        0,                        0,                        0,              0,            -Km,               0,               0;
                          0,                         0,                         0,                 0,                0,                 0, 0, 0, 0,                        0,                        0,                        0,                        0,              0,              0,             -Km,               0;
                          0,                         0,                         0,                 0,                0,                 0, 0, 0, 0,                        0,                        0,                        0,                        0,              0,              0,               0,             -Km];
 


end

