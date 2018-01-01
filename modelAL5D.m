function varargout = modelAL5D(varargin)
% MODELAL5D MATLAB code for modelAL5D.fig
%      MODELAL5D, by itself, creates a new MODELAL5D or raises the existing
%      singleton*.
%
%      H = MODELAL5D returns the handle to a new MODELAL5D or the handle to
%      the existing singleton*.
%
%      MODELAL5D('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MODELAL5D.M with the given input arguments.
%
%      MODELAL5D('Property','Value',...) creates a new MODELAL5D or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before modelAL5D_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to modelAL5D_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help modelAL5D

% Last Modified by GUIDE v2.5 26-Dec-2017 13:30:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @modelAL5D_OpeningFcn, ...
                   'gui_OutputFcn',  @modelAL5D_OutputFcn, ...
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
end

% --- Executes just before modelAL5D is made visible.
function modelAL5D_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to modelAL5D (see VARARGIN)

% Choose default command line output for modelAL5D
handles.output = hObject;
handles.theta1 = 90;
handles.theta2 = 90;
handles.theta3 = 90;
handles.theta4 = 0;
handles.theta5 = 0;


% Update handles structure
guidata(hObject, handles);
set(handles.slider1,'Value',handles.theta1);
set(handles.texttheta1,'String',handles.theta1);
set(handles.slider2,'Value',handles.theta2);
set(handles.texttheta2,'String',handles.theta2);
set(handles.slider3,'Value',handles.theta3);
set(handles.texttheta3,'String',handles.theta3);
set(handles.slider4,'Value',handles.theta4);
set(handles.texttheta4,'String',handles.theta4);
set(handles.slider5,'Value',handles.theta5);
set(handles.texttheta5,'String',handles.theta5);
update_plot(handles.theta1,handles.theta2,handles.theta3,handles.theta4,handles.theta5)


% UIWAIT makes modelAL5D wait for user response (see UIRESUME)
% uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = modelAL5D_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end

% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 handles = guidata(hObject);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
 sliderValue = get(handles.slider1,'Value');
 set(handles.texttheta1,'String',num2str(sliderValue));
 handles.theta1 = sliderValue;
 update_plot(handles.theta1,handles.theta2,handles.theta3,handles.theta4,handles.theta5)
 guidata(hObject,handles);
end
% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
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
set(handles.texttheta2,'String',num2str(sliderValue));
handles.theta2 = sliderValue;
update_plot(handles.theta1,handles.theta2,handles.theta3,handles.theta4,handles.theta5)
 guidata(hObject,handles);
end
% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
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
set(handles.texttheta3,'String',num2str(sliderValue)); 
handles.theta3 = sliderValue;
update_plot(handles.theta1,handles.theta2,handles.theta3,handles.theta4,handles.theta5)
 guidata(hObject,handles);
end

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
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
set(handles.texttheta4,'String',num2str(sliderValue));
 handles.theta4 = sliderValue;
 update_plot(handles.theta1,handles.theta2,handles.theta3,handles.theta4,handles.theta5)
  guidata(hObject,handles);
end

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
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
set(handles.texttheta5,'String',num2str(sliderValue));
 handles.theta5 = sliderValue;
 update_plot(handles.theta1,handles.theta2,handles.theta3,handles.theta4,handles.theta5)
  guidata(hObject,handles);
end

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
end
% Matrice A
function A = dh(theta,d,a,t)
A = [cos(theta), -sin(theta)*cos(t),   sin(theta)*sin(t), a*cos(theta);
     sin(theta),  cos(theta)*cos(t),  -cos(theta)*sin(t), a*sin(theta);
              0,             sin(t),              cos(t),            d;
              0,                  0,                   0,            1];
end
function update_plot(t1,t2,t3,t4,t5)
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
    
    t1 = t1 * dtor;
    t2 = t2 * dtor;
    t3 = t3 * dtor;
    t4 = t4 * dtor;
    t5 = t5 * dtor;
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
   
end
