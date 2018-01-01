syms theta1 theta2 theta3 theta4 theta5
syms nx ny nz ox oy oz ax ay az px py pz
syms x

dtor = pi/180;
rtod = 180/pi;
%A0 = [cos(theta1) 0 sin(theta1) 0;sin(theta1) 0 -cos(theta1) 0 ;0 1 0 0 ;0 0 0 1];
A0 = dh(theta1,0,0,-pi/2)
A1 = dh(theta2,0,145,0)
A2 = dh(theta3,0,184,0)
A3 = dh(theta4,0,0,-pi/2)
A4 = dh(theta5,0,0,0)

C = [1, 0,0,0;
      0,1,0,0;
      0, 0,1,70;
      0, 0,0,1];
          
Gk = [1,0 ,0 ,0;
      0 ,1,0 ,0;
      0 , 0,1 ,90;
      0 , 0,0 ,1];
  
  function A = dh(theta,d,a,t)
A = [cos(theta), -sin(theta)*cos(t),   sin(theta)*sin(t), a*cos(theta);
     sin(theta),  cos(theta)*cos(t),  -cos(theta)*sin(t), a*sin(theta);
              0,             sin(t),              cos(t),            d;
              0,                  0,                   0,            1];
end 