% Entrée : les quatre variables articulaires du robot
% Sortie : x, y, z, w, p, r de F_outil par rapport au F_R

function pose = Cinematique_Directe(theta1, theta2,theta3,theta4,theta5)

% Conversion de degrés en radians (car Matlab fonctionne en radians)
theta1 = theta1 *pi/180;
theta2 = theta2 *pi/180;
theta3 = theta3 *pi/180;
theta4 = theta4 *pi/180;
theta5 = theta5 *pi/180;
% Calcul des matrices "A" avec la fonction dh (Denavit Hartenberg)

A1 = dh(theta1,0,0,-pi/2);
A2 = dh(theta2,0,-145,0);
A3 = dh(theta3,0,-184,0);
A4 = dh(theta4,0,0,pi/2);
A5 = dh(theta5,0,0,0);

% Matrice de transformation C
C =  [ 1, 0, 0, 0;
        0, 1, 0, 0;
        0, 0, 1,  70;
        0, 0, 0,   1];
   
% Matrice de transformation Gk
Gk = [  1, 0, 0, 0;
        0, 1, 0,    0;
        0, 0, 1,  90;
        0, 0, 0,   1];

ca1 = C*A1;
ca2 = C*A1*A2;
ca3 = C*A1*A2*A3;
ca4 = C*A1*A2*A3*A4;
ca5 = C*A1*A2*A3*A4*A5;
cgk = C*A1*A2*A3*A4*A5*Gk;
x = [ca1(1,4),ca2(1,4),ca3(1,4),ca4(1,4),ca5(1,4),cgk(1,4)];
y = [ca1(2,4),ca2(2,4),ca3(2,4),ca4(2,4),ca5(2,4),cgk(2,4)];
z = [ca1(3,4),ca2(3,4),ca3(3,4),ca4(3,4),ca5(3,4),cgk(3,4)];
figure
plot3(x,y,z)

% Matrice homogène qui définit la pose du référentiel F_outil
% par rapport au référentiel F_R
H = C*A1*A2*A3*A4*A5*Gk;
%H = C*A1*A2*A3*A4*A5*Gk
% Calcul de r, p, w (convention XYZ d'angles d'Euler)
 if abs(H(3,1)) == 1
    p = -H(3,1)*90;
    w = 0; % la valeur de w peut être arbitraire, mais on choisit w = 0
    r = atan2(-H(3,1)*H(2,3),H(2,2))*180/pi;
 else
    % il y a deux solutions dans la plage [-180°,180°]
    p = atan2(-H(3,1),sqrt(H(1,1)^2+H(2,1)^2))*180/pi;
    cp = cos(p*pi/180);
    r = atan2(H(2,1)/cp,H(1,1)/cp)*180/pi;
    w = atan2(H(3,2)/cp,H(3,3)/cp)*180/pi;
 end
 
 % Résultat
 pose.x = H(1,4);
 pose.y = H(2,4);
 pose.z = H(3,4);
 pose.w = w;
 pose.p = p;
 pose.r = r;
 
% Matrice A
function A = dh(theta,d,a,t)
A = [cos(theta), -sin(theta)*cos(t),   sin(theta)*sin(t), a*cos(theta);
     sin(theta),  cos(theta)*cos(t),  -cos(theta)*sin(t), a*sin(theta);
              0,             sin(t),              cos(t),            d;
              0,                  0,                   0,            1];
