function Q = eAng2Q( eAngs )

a2 = eAngs*0.5;
ph2 = a2(1);
th2 = a2(2);
ps2 = a2(3);


Q = [cos(ph2)*cos(th2)*cos(ps2)+sin(ph2)*sin(th2)*sin(ps2);
     sin(ph2)*cos(th2)*cos(ps2)-cos(ph2)*sin(th2)*sin(ps2);
     cos(ph2)*sin(th2)*cos(ps2)+sin(ph2)*cos(th2)*sin(ps2);
     cos(ph2)*cos(th2)*sin(ps2)-sin(ph2)*sin(th2)*cos(ps2) ];
end