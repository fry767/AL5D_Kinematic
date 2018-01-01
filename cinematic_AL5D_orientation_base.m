syms t1 t2 t3 t4
syms nx ny nz
syms ox oy oz
syms ax ay az
syms px py pz
syms nx1 ny1 nz1
syms ox1 oy1 oz1
syms ax1 ay1 az1
syms px1 py1 pz1
D_to_R = pi/180;
L1 = 145;
L2 = 184;


l1 = [cos(t1) 0 sin(t1) 0;sin(t1) 0 -cos(t1) 0;0 1 0 0;0 0 0 1];
l2 = [cos(t2+pi) -sin(t2+pi) 0 -145*cos(t2+pi);sin(t2+pi) cos(t2+pi) 0 -145*sin(t2+pi);0 0 1 0;0 0 0 1];
l3 = [cos(t3) -sin(t3) 0 -184*cos(t3);sin(t3) cos(t3) 0 -184*sin(t3);0 0 1 0;0 0 0 1];
l4 = [cos(t4) 0 -sin(t4) 0;sin(t4) 0 cos(t4) 0;0 -1 0 0;0 0 0 1];
c = [-1 0 0 0;0 -1 0 0;0 0 1 70;0 0 0 1];
gk = [1 0 0 0;0 1 0 0;0 0 1 90; 0 0 0 1];
x = [nx ox ax px; ny oy ay py; nz oz az pz;0 0 0 1];

t1234 = l1*l2*l3*l4;
cxgk = (c^(-1)) * x * gk^-1 ;
simplify(trn);
simplify (cxgk);
t234 = l2*l3*l4;
l1cxgk = (l1^(-1)) * (c^(-1)) * x * (gk^(-1)) ;
simplify(t234);
simplify (l1cxgk);

cxgkl4 = (c^(-1)) * x * (gk^(-1))*(l4^(-1)) ;
t123 = l1*l2*l3;
simplify (cxgkl4)
simplify(t123)

th1 =90*D_to_R;
th2 = 180*D_to_R;
th3 =90*D_to_R;
th4 = 139*D_to_R;
dk = symfun(c*l1*l2*l3*l4*gk,[t1,t2,t3,t4]);
r = eval(dk(th1,th2,th3,th4))

 rnx = r(1,1);
 rny = r(2,1);
 rnz = r(3,1);
 rox = r(1,2);
 roy = r(2,2);
 roz = r(3,2);
 rax = r(1,3);
 ray = r(2,3);
 raz = r(3,3);
 rpx = r(1,4);
 rpy = r(2,4);
 rpz = r(3,4);
 
% cinematique inverse
% theta1
theta1 = atan2(rox,-roy);

%theta3
px3 = 90*rax*cos(theta1) - rpx*cos(theta1) + 90*ray*sin(theta1) - rpy*sin(theta1)
py3 = rpz - 90*raz - 70
V = (px3^2 + py3^2 - L1^2 - L2^2)/(2*L1*L2) 
theta3 = [atan2(sqrt(1-(V)^2),V) atan2(-sqrt(1-V^2),V)];

%theta2
phi2 = [atan2(L1+L2*cos(theta3(1)),L2*sin(theta3(1))) atan2(L1+L2*cos(theta3(2)),L2*sin(theta3(2)))]
r2 = (L1+L2*cos(theta3(1)))^2+(L2*sin(theta3(1)))^2  
theta2 = [phi2(1)-atan2(px3,sqrt(r2-(px3^2))) phi2(1)-atan2(px3,-sqrt(r2-(px3^2))) phi2(2)-atan2(px3,sqrt(r2-(px3^2))) phi2(2)-atan2(px3,-sqrt(r2-(px3^2))) ];
%theta 4
phi4 = atan2(rnz,raz)
r42 = raz^2 + rnz^2
rt1 = theta1*180/pi;

if (py3<=0)
    if(theta2(2)*180/pi < -179.99999999)
        rt2 = 360 + theta2(2)*180/pi
    else
        rt2 = theta2(2)*180/pi   
    end
    (sin(theta2(2)+theta3(1)))^2
    (sin(theta2(2)+theta3(1)))^2
    theta4 = [phi4 - atan2(-sin(theta2(2)+theta3(1)),sqrt(r42 - (sin(theta2(2)+theta3(1)))^2)) phi4 - atan2(-sin(theta2(2)+theta3(1)),-sqrt(r42 - (sin(theta2(2)+theta3(1)))^2))];
else
   if(theta2(2)*180/pi < -179.999999999)
        rt2 = 360 + theta2(2)*180/pi
    else
        rt2 = theta2(2)*180/pi   
    end
    theta4 = [phi4 - atan2(-sin(theta2(1)+theta3(1)),sqrt(r42 - (sin(theta2(1)+theta3(1)))^2)) phi4 - atan2(-sin(theta2(1)+theta3(1)),-sqrt(r42 - (sin(theta2(1)+theta3(1)))^2))];
end
rt3 = theta3 * 180/pi
rt4 = theta4 * 180/pi


