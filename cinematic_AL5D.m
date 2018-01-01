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
l1 = [cos(t1) 0 sin(t1) 0;sin(t1) 0 -cos(t1) 0;0 1 0 0;0 0 0 1];
l2 = [cos(t2+pi) -sin(t2+pi) 0 -145*cos(t2+pi);sin(t2+pi) cos(t2+pi) 0 -145*sin(t2+pi);0 0 1 0;0 0 0 1];
l3 = [cos(t3) -sin(t3) 0 -184*cos(t3);sin(t3) cos(t3) 0 -184*sin(t3);0 0 1 0;0 0 0 1];
l4 = [cos(t4) 0 -sin(t4) 0;sin(t4) 0 cos(t4) 0;0 -1 0 0;0 0 0 1];
c = [-1 0 0 0;0 -1 0 0;0 0 1 70;0 0 0 1];
gk = [1 0 0 0;0 1 0 0;0 0 1 90; 0 0 0 1];
x = [nx ox ax px; ny oy ay py; nz oz az pz;0 0 0 1];


trn = l1*l2*l3*l4 * gk;
cxgk = (c^(-1)) * x ;
simplify(trn);


tr234gk = l2*l3*l4*gk;
cx1 = l1^(-1)*c^(-1) * x;
cx1s = simplify(cx1);
tr3s = simplify(tr234gk);


 tr234 = l2*l3*l4;
 cx1gk = l1^(-1)*c^(-1) * x * gk^(-1);
 cx1gks = simplify(cx1gk);
 tr234s = simplify(tr234);

  tr123 = l1*l2*l3;
  cxgk4 = c^(-1) * x * gk^(-1 )*l4^(-1);
% cxgk2 = l2^(-1)*l1^(-1)*c^(-1) * x * gk^(-1);
% tr2 = l3*l4;
 cxgk4s = simplify(cxgk4);
 tr123s = simplify(tr123);
% cxgk2 = l1^(-1)*c^(-1) * x * gk^(-1)*l4^(-1);
% tr2 = l2*l3;
% simplify(cxgk2)
% simplify(tr2)
th1 = 45*D_to_R;
th2 = 0*D_to_R;
th3 =90*D_to_R;
th4 = 180*D_to_R;
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
%Inverse kinematics
%theta1
theta1 = symfun([atan2(py,px) atan2(-py,-px)],[px,py]);
rt1 = eval(theta1(rpx,rpy));
rst1 = rt1 * 180/pi
%theta 3
if rst1(1)<0
    px3 = 90*rax*cos(rt1(2)) - rpx*cos(rt1(2)) + 90*ray*sin(rt1(2)) - rpy*sin(rt1(2));
else
    px3 = 90*rax*cos(rt1(1)) - rpx*cos(rt1(1)) + 90*ray*sin(rt1(1)) - rpy*sin(rt1(1));

end
py3 = rpz-90*raz-70;
px3m = 90*rax*cos(rt1(2)) - rpx*cos(rt1(2)) + 90*ray*sin(rt1(2)) - rpy*sin(rt1(2));
 

L1 = 145;
L2 = 184;

V = [(px3*px3 + py3*py3 - L1*L1 - L2*L2)/(2*L1*L2) (px3m*px3m + py3*py3 - L1*L1 - L2*L2)/(2*L1*L2)];
theta3 = [atan2(sqrt(1-(V(1))^2),V(1)) atan2(-sqrt(1-V(1)^2),V(1)) atan2(sqrt(1-V(2)*V(2)),V(2)) atan2(-sqrt(1-V(2)*V(2)),V(2))];
rt3 = theta3*180/pi
%theta2

phi2 = [atan2(L1+L2*cos(theta3(1)),L2*sin(theta3(1))) atan2(L1+L2*cos(theta3(2)),L2*sin(theta3(2)))];
r2 = (L1+L2*cos(theta3(1)))^2+(L2*sin(theta3(1)))^2  ;

%theta2 = [phi2(1)-atan2(px3,sqrt(r2-px3^2)) phi2(1)-atan2(px3,sqrt(r2-px3m^2)) phi2(2)-atan2(px3,sqrt(r2-px3^2)) phi2(2)-atan2(px3,sqrt(r2-px3m^2)) phi2(1)-atan2(px3,-sqrt(r2-px3^2)) phi2(1)-atan2(px3,-sqrt(r2-px3m^2)) phi2(2)-atan2(px3,-sqrt(r2-px3^2)) phi2(2)-atan2(px3,-sqrt(r2-px3m^2))];
theta2 = [phi2(1)-atan2(px3,sqrt(r2-px3^2)) phi2(1)-atan2(px3,-sqrt(r2-px3^2)) ];
rt2 = theta2*180/pi
%theta4
phi4 = atan2(rnz,raz);
r42 = raz^2 + rnz^2;
theta4 = phi4 - atan2(-sin(theta2(1)+theta3(1)),sqrt(r42 - (sin(theta2(1)+theta3(1)))^2));
rt4 = theta4 * 180/pi