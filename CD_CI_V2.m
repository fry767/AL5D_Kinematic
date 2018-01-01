syms theta1 theta2 theta3 theta4 theta5
syms nx ny nz ox oy oz ax ay az px py pz
syms x

dtor = pi/180;
rtod = 180/pi;
%A0 = [cos(theta1) 0 sin(theta1) 0;sin(theta1) 0 -cos(theta1) 0 ;0 1 0 0 ;0 0 0 1];
A1 = [cos(theta1) 0 -sin(theta1) 0;sin(theta1) 0 cos(theta1) 0 ;0 -1 0 0 ;0 0 0 1];
A2 = dh(theta2,0,-145,0);
A3 = dh(theta3,0,-184,0);
A4 = [cos(theta4) 0 sin(theta4) 0;sin(theta4) 0 -cos(theta4) 0 ;0 1 0 0 ;0 0 0 1];
A5 = dh(theta5,0,0,0);


C = [1, 0,0,0;
      0,1,0,0;
      0, 0,1,70;
      0, 0,0,1];
          
GK = [1,0 ,0 ,0;
      0 ,1,0 ,0;
      0 , 0,1 ,90;
      0 , 0,0 ,1];

ct5gk = C*A1*A2*A3*A4*A5*GK;

X = [nx ox ax px; ny oy ay py; nz oz az pz;0 0 0 1];

cxgk = X*GK^-1*A5^-1;
t5 = C*A1*A2*A3*A4;
simplify(vpa(cxgk),10)
simplify(vpa(t5,10))

 th1 = 90* dtor;
 th2 =90* dtor;
 th3 = 90* dtor;
 th4 =  0* dtor;
 th5 =  0 *dtor;
 CD = symfun(C*A1*A2*A3*A4*A5*GK,[theta1,theta2,theta3,theta4,theta5]);
 r = eval(CD(th1,th2,th3,th4,th5))
nx = r(1,1);
ny = r(2,1); 
nz = r(3,1);
ox = r(1,2);
oy = r(2,2);
oz = r(3,2);
ax = r(1,3);
ay = r(2,3);
az = r(3,3);
px = r(1,4);
py = r(2,4);
pz = r(3,4);

%theta1
pty1 = py-90*ay;
ptx1 = px-90*ax;
t1 = atan2(pty1,ptx1);
t11 = atan2(-pty1,-ptx1);
t111 = atan2(ay,ax);
t1111= atan2(-ay,-ax);
if((360 - t1 ) > 180 && t1 < 0)
    t1 = t11;
end
if(py < 0 && px > 0)
    t1 = t11;
end

if(ptx1 ==0 && pty1==0)
    if(ax >0 && ay >0)
        t1 = t111; 
    end
    if(ax <0 && ay >0)
        t1 = t111; 
    end
    if(ax <0 && ay <0)
        t1 = t1111; 
    end
    if(ax >0 && ay <0)
        t1 = t1111; 
    end
end
%theta5
t5 = atan2(-oz,nz);
t55 = atan2(oz,-nz);
if(t5 < 0 )
    t5 = t55;
end
phi5 = atan2(ox,-nx);
phi55 = atan2(oy,-ny);
r5 = ox^2 + nx^2;
r55 = oy^2 + ny^2;
if(sin(t1)^2 <= r5 &&(ox ~= 0 || nx~=0))
    t555 = phi5 - atan2(-1*sin(t1),sqrt(r5 - (sin(t1))^2));
    t5555 = phi5 - atan2(-1*sin(t1),-sqrt(r5 - (sin(t1))^2));
    if(oz ==0 && nz == 0)
        t5 = t555
    end
end
if((cos(t1)^2 <= r5) &&(oy ~= 0 || ny~=0))
    t55555 = phi55 - atan2(cos(t1),sqrt(r55 - (cos(t1))^2));
    t555555 = phi55 - atan2(cos(t1),-sqrt(r55 - (cos(t1))^2));
    if(oz ==0 && nz == 0)
        t5 = t55555
    end
end

%theta2 - theta3
PX1 = (px - 90*ax)*cos(t1) + (py-90*ay)*sin(t1);
PX11 = (px - 90*ax)*cos(t11) + (py-90*ay)*sin(t11);
PX111 = (px - 90*ax)*cos(t111) + (py-90*ay)*sin(t111);
PX1111 = (px - 90*ax)*cos(t1111) + (py-90*ay)*sin(t1111);
PY = 90*az-pz+70;
L1 = -145;
L2 = -184;

C31 = (PX1^2 + PY^2 - L1^2 - L2^2)/(2*L1*L2);
C311 = (PX11^2 + PY^2 - L1^2 - L2^2)/(2*L1*L2);
C3111 = (PX111^2 + PY^2 - L1^2 - L2^2)/(2*L1*L2);
C31111 = (PX1111^2 + PY^2 - L1^2 - L2^2)/(2*L1*L2);
if(C31^2 <=1)
    t3 = atan2(sqrt(1-C31^2),C31);
    t33 = atan2(-sqrt(1-C31^2),C31);
end
if(C311^2 <= 1)
    t333 = atan2(sqrt(1-C311^2),C311);
    t3333 = atan2(-sqrt(1-C311^2),C311);
end
if (C3111^2 <= 1)
    t33333 = atan2(sqrt(1-C3111^2),C3111);
    t333333 = atan2(-sqrt(1-C3111^2),C3111);
end
if(C31111^2<=1)
    t3333333 = atan2(sqrt(1-C31111^2),C31111);
    t33333333 = atan2(-sqrt(1-C31111^2),C31111);
end

K1 = L1+L2*cos(t3);
K11= L1+L2*cos(t33);
K2 = L2*sin(t3);
K22 = L2*sin(t33);
phi1 = atan2(K1,K2);
phi11 = atan2(K11,K22);

R1 = K1^2 + K2^2;
R11 = K11^2 + K22^2;
t2 = phi1 - atan2(PX1,sqrt(R1 - PX1^2));
t22 = phi1 - atan2(PX1,-sqrt(R1 - PX1^2));
t222 = phi11 - atan2(PX1,sqrt(R11 - PX1^2));
t2222 = phi11 - atan2(PX1,-sqrt(R11 - PX1^2));
if(pz >= 0)
    t2 = t22;
end
   
%theta4
%cos
k1 = oz*cos(t2)*sin(t3)*sin(t5) - 1.0*nz*cos(t3)*cos(t5)*sin(t2) - 1.0*nz*cos(t2)*cos(t5)*sin(t3) + oz*cos(t3)*sin(t2)*sin(t5) + nx*cos(t1)*cos(t2)*cos(t3)*cos(t5) + ny*cos(t2)*cos(t3)*cos(t5)*sin(t1) - 1.0*ox*cos(t1)*cos(t2)*cos(t3)*sin(t5) - 1.0*nx*cos(t1)*cos(t5)*sin(t2)*sin(t3) - 1.0*oy*cos(t2)*cos(t3)*sin(t1)*sin(t5) - 1.0*ny*cos(t5)*sin(t1)*sin(t2)*sin(t3) + ox*cos(t1)*sin(t2)*sin(t3)*sin(t5) + oy*sin(t1)*sin(t2)*sin(t3)*sin(t5);
k3 = oz*cos(t2)*cos(t3)*sin(t5) - nz*cos(t2)*cos(t3)*cos(t5) + nz*cos(t5)*sin(t2)*sin(t3) - oz*sin(t2)*sin(t3)*sin(t5) - nx*cos(t1)*cos(t2)*cos(t5)*sin(t3) - nx*cos(t1)*cos(t3)*cos(t5)*sin(t2) - ny*cos(t2)*cos(t5)*sin(t1)*sin(t3) - ny*cos(t3)*cos(t5)*sin(t1)*sin(t2) + ox*cos(t1)*cos(t2)*sin(t3)*sin(t5) + ox*cos(t1)*cos(t3)*sin(t2)*sin(t5) + oy*cos(t2)*sin(t1)*sin(t3)*sin(t5) + oy*cos(t3)*sin(t1)*sin(t2)*sin(t5);
%sin
k2 = ax*cos(t1)*cos(t2)*cos(t3) - 1.0*az*cos(t3)*sin(t2) - 1.0*az*cos(t2)*sin(t3) + ay*cos(t2)*cos(t3)*sin(t1) - 1.0*ax*cos(t1)*sin(t2)*sin(t3) - 1.0*ay*sin(t1)*sin(t2)*sin(t3);
k4 = az*sin(t2)*sin(t3) - az*cos(t2)*cos(t3) - ax*cos(t1)*cos(t2)*sin(t3) - ax*cos(t1)*cos(t3)*sin(t2) - ay*cos(t2)*sin(t1)*sin(t3) - ay*cos(t3)*sin(t1)*sin(t2);
t4 = atan2(k2,k1);
t44 = atan2(k4,k1);
t444 = atan2(k2,-k3);
t4444 = atan2(k4,-k3);
%Resultat final
td1 = [rtod*t1,rtod*t11,rtod*t111,rtod*t1111]
td2 = [rtod*t2,rtod*t22,rtod*t222,rtod*t2222]
td3 = [rtod * t3,rtod*t33,rtod*t333,rtod*t3333,rtod*t33333,rtod*t333333,rtod*t3333333,rtod*t33333333]
td4 = [rtod * t4,rtod*t44,rtod*t444,rtod*t4444]
td5 = [t5 * rtod,t55*rtod,t555*rtod,t5555*rtod,t55555*rtod,t555555*rtod]



function A = dh(theta,d,a,t)
A = [cos(theta), -sin(theta)*cos(t),   sin(theta)*sin(t), a*cos(theta);
     sin(theta),  cos(theta)*cos(t),  -cos(theta)*sin(t), a*sin(theta);
              0,             sin(t),              cos(t),            d;
              0,                  0,                   0,            1];
end 