function varargout = modelAL5D_IK(varargin)
% MODELAL5D_IK MATLAB code for modelAL5D_IK.fig
%      MODELAL5D_IK, by itself, creates a new MODELAL5D_IK or raises the existing
%      singleton*.
%
%      H = MODELAL5D_IK returns the handle to a new MODELAL5D_IK or the handle to
%      the existing singleton*.
%
%      MODELAL5D_IK('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MODELAL5D_IK.M with the given input arguments.
%
%      MODELAL5D_IK('Property','Value',...) creates a new MODELAL5D_IK or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before modelAL5D_IK_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to modelAL5D_IK_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help modelAL5D_IK

% Last Modified by GUIDE v2.5 27-Dec-2017 15:02:32

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @modelAL5D_IK_OpeningFcn, ...
                   'gui_OutputFcn',  @modelAL5D_IK_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before modelAL5D_IK is made visible.
function modelAL5D_IK_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to modelAL5D_IK (see VARARGIN)

% Choose default command line output for modelAL5D_IK
handles.output = hObject;
handles.x = 0;
handles.y = 184;
handles.z = 125;

handles.w = 180;
handles.p = 0;
handles.r = -90;


% Update handles structure
guidata(hObject, handles);

set(handles.slider1,'Value',handles.x);
set(handles.text2,'String',handles.x);
set(handles.slider2,'Value',handles.y);
set(handles.text3,'String',handles.y);
set(handles.slider3,'Value',handles.z);
set(handles.text4,'String',handles.z);
set(handles.slider4,'Value',handles.w);
set(handles.text5,'String',handles.w);
set(handles.slider5,'Value',handles.p);
set(handles.text6,'String',handles.p);
set(handles.slider6,'Value',handles.r);
set(handles.text7,'String',handles.r);
update_plot(handles.x,handles.y,handles.z,handles.w,handles.p,handles.r)
% UIWAIT makes modelAL5D_IK wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = modelAL5D_IK_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
 handles = guidata(hObject);
 sliderValue = get(handles.slider1,'Value');
 set(handles.text2,'String',num2str(sliderValue));
 handles.x = sliderValue;
 update_plot(handles.x,handles.y,handles.z,handles.w,handles.p,handles.r)
 guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
 handles = guidata(hObject);
 sliderValue = get(handles.slider2,'Value');
 set(handles.text3,'String',num2str(sliderValue));
 handles.y = sliderValue;
 update_plot(handles.x,handles.y,handles.z,handles.w,handles.p,handles.r)
 guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
 handles = guidata(hObject);
 sliderValue = get(handles.slider3,'Value');
 set(handles.text4,'String',num2str(sliderValue));
 handles.z = sliderValue;
 update_plot(handles.x,handles.y,handles.z,handles.w,handles.p,handles.r)
 guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
 handles = guidata(hObject);
 sliderValue = get(handles.slider4,'Value');
 set(handles.text5,'String',num2str(sliderValue));
 handles.w = sliderValue;
 update_plot(handles.x,handles.y,handles.z,handles.w,handles.p,handles.r)
 guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
 handles = guidata(hObject);
 sliderValue = get(handles.slider5,'Value');
 set(handles.text6,'String',num2str(sliderValue));
 handles.p = sliderValue;
 update_plot(handles.x,handles.y,handles.z,handles.w,handles.p,handles.r)
 guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
 handles = guidata(hObject);
 sliderValue = get(handles.slider6,'Value');
 set(handles.text7,'String',num2str(sliderValue));
 handles.r = sliderValue;
 update_plot(handles.x,handles.y,handles.z,handles.w,handles.p,handles.r)
 guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function A = dh(theta,d,a,t)
A = [cos(theta), -sin(theta)*cos(t),   sin(theta)*sin(t), a*cos(theta);
     sin(theta),  cos(theta)*cos(t),  -cos(theta)*sin(t), a*sin(theta);
              0,             sin(t),              cos(t),            d;
              0,                  0,                   0,            1];

function update_plot(x,y,z,w,p,r)
   rtod = 180/pi;
% Matrice homogène qui définit la pose du référentiel F_outil
% par rapport au référentiel F_R
x = round(x,10);
y = round(y,10);
z = round(z,10);
w = w*pi/180;
p = p*pi/180;
r = r*pi/180;
H = [ cos(r)*cos(p), cos(r)*sin(p)*sin(w)-sin(r)*cos(w), cos(r)*sin(p)*cos(w)+sin(r)*sin(w), x;
      sin(r)*cos(p), sin(r)*sin(p)*sin(w)+cos(r)*cos(w), sin(r)*sin(p)*cos(w)-cos(r)*sin(w), y;
            -sin(p),                      cos(p)*sin(w),                      cos(p)*cos(w), z;
                  0,                                  0,                                 0,  1]

% Décomposition de la matrice H
nx = round(H(1,1),10);
ny = round(H(2,1),10);
nz = round(H(3,1),10);
ox = round(H(1,2),10);
oy = round(H(2,2),10);
oz = round(H(3,2),10);
ax = round(H(1,3),10);
ay = round(H(2,3),10);
az = round(H(3,3),10);
px = round(H(1,4),10);
py = round(H(2,4),10);
pz = round(H(3,4),10);


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
        t5 = t555;
    end
end
if((cos(t1)^2 <= r5) &&(oy ~= 0 || ny~=0))
    t55555 = phi55 - atan2(cos(t1),sqrt(r55 - (cos(t1))^2));
    t555555 = phi55 - atan2(cos(t1),-sqrt(r55 - (cos(t1))^2));
    if(oz ==0 && nz == 0)
        t5 = t55555;
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
tf = [t1 * rtod,t2*rtod,t3*rtod,t4*rtod,t5*rtod]
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
        
    dtor = pi/180;
    
    A1 = dh(t1,0,0,-pi/2);
    A2 = dh(t2,0,-145,0);
    A3 = dh(t3,0,-184,0);
    A4 = dh(t4,0,0,pi/2);
    A5 = dh(t5,0,0,0);
    ca1 = C*A1;
    ca2 = C*A1*A2;
    ca3 = C*A1*A2*A3;
    ca4 = C*A1*A2*A3*A4;
    ca5 = C*A1*A2*A3*A4*A5;
    cgk = C*A1*A2*A3*A4*A5*Gk;
    x = [ca1(1,4),ca2(1,4),ca3(1,4),ca4(1,4),ca5(1,4),cgk(1,4)];
    y = [ca1(2,4),ca2(2,4),ca3(2,4),ca4(2,4),ca5(2,4),cgk(2,4)];
    z = [ca1(3,4),ca2(3,4),ca3(3,4),ca4(3,4),ca5(3,4),cgk(3,4)];
    plot3(x,y,z)
    view(-40,20)
    xlim([-300,500])
    ylim([-300,500])
    zlim([-300,500])

  
