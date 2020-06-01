function varargout = Teach_GUIv1(varargin)
% TEACH_GUIV1 MATLAB code for Teach_GUIv1.fig
%      TEACH_GUIV1, by itself, creates a new TEACH_GUIV1 or raises the existing
%      singleton*.
%
%      H = TEACH_GUIV1 returns the handle to a new TEACH_GUIV1 or the handle to
%      the existing singleton*.
%
%      TEACH_GUIV1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEACH_GUIV1.M with the given input arguments.
%
%      TEACH_GUIV1('Property','Value',...) creates a new TEACH_GUIV1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Teach_GUIv1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Teach_GUIv1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Teach_GUIv1

% Last Modified by GUIDE v2.5 23-May-2020 13:35:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Teach_GUIv1_OpeningFcn, ...
                   'gui_OutputFcn',  @Teach_GUIv1_OutputFcn, ...
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


% --- Executes just before Teach_GUIv1 is made visible.
function Teach_GUIv1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Teach_GUIv1 (see VARARGIN)

handles.kinova=Assignment2Functions;


camlight;
% Choose default command line output for Teach_GUIv1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Teach_GUIv1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Teach_GUIv1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in exit_Btn.
function exit_Btn_Callback(hObject, eventdata, handles)
% hObject    handle to exit_Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close Teach_GUIv1
Start_GUI


% --- Executes on slider movement.
function q1slider_Callback(hObject, eventdata, handles)
% hObject    handle to q1slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kinovaPos=handles.kinova.GetKinovaPos;              %gives xyz martrix (3x1)
setX=set(handles.xValue,'String',kinovaPos(1,1));   %
setY=set(handles.yValue,'String',kinovaPos(2,1));
setz=set(handles.zValue,'String',kinovaPos(3,1));

q1=get(hObject,'Value');                %take the value of the slider
qNew=set(handles.q1edit,'String',q1);   %display the value of slider into the text box
handles.kinova.UpdateJoint(1,q1);       %Updating the joint q1 with animation




% --- Executes during object creation, after setting all properties.
function q1slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q2slider_Callback(hObject, eventdata, handles)
% hObject    handle to q2slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kinovaPos=handles.kinova.GetKinovaPos;              %gives xyz martrix (3x1)
setX=set(handles.xValue,'String',kinovaPos(1,1));   %
setY=set(handles.yValue,'String',kinovaPos(2,1));
setz=set(handles.zValue,'String',kinovaPos(3,1));

q2=get(hObject,'Value');                %take the value of the slider
qNew=set(handles.q2edit,'String',q2);   %display the value of slider into the text box
handles.kinova.UpdateJoint(2,q2);       %Updating the joint q1 with animation

% --- Executes during object creation, after setting all properties.
function q2slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q2slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q3slider_Callback(hObject, eventdata, handles)
% hObject    handle to q3slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kinovaPos=handles.kinova.GetKinovaPos;              %gives xyz martrix (3x1)
setX=set(handles.xValue,'String',kinovaPos(1,1));   %
setY=set(handles.yValue,'String',kinovaPos(2,1));
setz=set(handles.zValue,'String',kinovaPos(3,1));

q3=get(hObject,'Value');                %take the value of the slider
qNew=set(handles.q3edit,'String',q3);   %display the value of slider into the text box
handles.kinova.UpdateJoint(3,q3);       %Updating the joint q1 with animation

% --- Executes during object creation, after setting all properties.
function q3slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q3slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q4slider_Callback(hObject, eventdata, handles)
% hObject    handle to q4slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kinovaPos=handles.kinova.GetKinovaPos;              %gives xyz martrix (3x1)
setX=set(handles.xValue,'String',kinovaPos(1,1));   %
setY=set(handles.yValue,'String',kinovaPos(2,1));
setz=set(handles.zValue,'String',kinovaPos(3,1));

q4=get(hObject,'Value');                %take the value of the slider
qNew=set(handles.q4edit,'String',q4);   %display the value of slider into the text box
handles.kinova.UpdateJoint(4,q4);       %Updating the joint q1 with animation

% --- Executes during object creation, after setting all properties.
function q4slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q4slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q5lider_Callback(hObject, eventdata, handles)
% hObject    handle to q5lider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kinovaPos=handles.kinova.GetKinovaPos;              %gives xyz martrix (3x1)
setX=set(handles.xValue,'String',kinovaPos(1,1));   %
setY=set(handles.yValue,'String',kinovaPos(2,1));
setz=set(handles.zValue,'String',kinovaPos(3,1));

q5=get(hObject,'Value');                %take the value of the slider
qNew=set(handles.q5edit,'String',q5);   %display the value of slider into the text box
handles.kinova.UpdateJoint(5,q5);       %Updating the joint q1 with animation

% --- Executes during object creation, after setting all properties.
function q5lider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q5lider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q6slider_Callback(hObject, eventdata, handles)
% hObject    handle to q6slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kinovaPos=handles.kinova.GetKinovaPos;              %gives xyz martrix (3x1)
setX=set(handles.xValue,'String',kinovaPos(1,1));   %
setY=set(handles.yValue,'String',kinovaPos(2,1));
setz=set(handles.zValue,'String',kinovaPos(3,1));

q6=get(hObject,'Value');                %take the value of the slider
qNew=set(handles.q6edit,'String',q6);   %display the value of slider into the text box
handles.kinova.UpdateJoint(6,q6);       %Updating the joint q1 with animation

% --- Executes during object creation, after setting all properties.
function q6slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q6slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q7slider_Callback(hObject, eventdata, handles)
% hObject    handle to q7slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
kinovaPos=handles.kinova.GetKinovaPos;              %gives xyz martrix (3x1)
setX=set(handles.xValue,'String',kinovaPos(1,1));   %
setY=set(handles.yValue,'String',kinovaPos(2,1));
setz=set(handles.zValue,'String',kinovaPos(3,1));

q7=get(hObject,'Value');                %take the value of the slider
qNew=set(handles.q7edit,'String',q7);   %display the value of slider into the text box
handles.kinova.UpdateJoint(7,q7);       %Updating the joint q1 with animation

% --- Executes during object creation, after setting all properties.
function q7slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q7slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function q1_value_Callback(hObject, eventdata, handles)
% hObject    handle to q1_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1_value as text
%        str2double(get(hObject,'String')) returns contents of q1_value as a double


% --- Executes during object creation, after setting all properties.
function q1_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q2_value_Callback(hObject, eventdata, handles)
% hObject    handle to q2_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q2_value as text
%        str2double(get(hObject,'String')) returns contents of q2_value as a double


% --- Executes during object creation, after setting all properties.
function q2_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q2_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q3_value_Callback(hObject, eventdata, handles)
% hObject    handle to q3_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q3_value as text
%        str2double(get(hObject,'String')) returns contents of q3_value as a double


% --- Executes during object creation, after setting all properties.
function q3_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q3_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q4_value_Callback(hObject, eventdata, handles)
% hObject    handle to q4_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q4_value as text
%        str2double(get(hObject,'String')) returns contents of q4_value as a double


% --- Executes during object creation, after setting all properties.
function q4_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q4_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q5_value_Callback(hObject, eventdata, handles)
% hObject    handle to q5_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q5_value as text
%        str2double(get(hObject,'String')) returns contents of q5_value as a double


% --- Executes during object creation, after setting all properties.
function q5_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q5_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q6_value_Callback(hObject, eventdata, handles)
% hObject    handle to q6_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q6_value as text
%        str2double(get(hObject,'String')) returns contents of q6_value as a double


% --- Executes during object creation, after setting all properties.
function q6_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q6_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q7_value_Callback(hObject, eventdata, handles)
% hObject    handle to q7_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q7_value as text
%        str2double(get(hObject,'String')) returns contents of q7_value as a double


% --- Executes during object creation, after setting all properties.
function q7_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q7_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xValue_Callback(hObject, eventdata, handles)
% hObject    handle to xValue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xValue as text
%        str2double(get(hObject,'String')) returns contents of xValue as a double
xValue=str2num(get(hObject,'String'));
minxReach=-1.3;
maxxReach=1.3;
if isempty(xValue)
    errordlg('Missing X value', 'Run Error');
    set(hObject,'String',0.0);
elseif xValue<minxReach || xValue > maxxReach
    errordlg('Please enter a suitable value','Value Error');
    set(hObject,'String',0.0);
end
% --- Executes during object creation, after setting all properties.
function xValue_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xValue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yValue_Callback(hObject, eventdata, handles)
% hObject    handle to yValue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yValue as text
%        str2double(get(hObject,'String')) returns contents of yValue as a double
yValue=str2num(get(hObject,'String'));
minyReach=-1;
maxyReach=1.3;
if isempty(yValue)
    errordlg('Missing Y value', 'Run Error');
    set(hObject,'String',0.0);
elseif yValue<minyReach || yValue > maxyReach
    errordlg('Please enter a suitable value','Value Error');
    set(hObject,'String',0.0);
end

% --- Executes during object creation, after setting all properties.
function yValue_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yValue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function zValue_Callback(hObject, eventdata, handles)
% hObject    handle to zValue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zValue as text
%        str2double(get(hObject,'String')) returns contents of zValue as a double
zValue=str2num(get(hObject,'String'));
minzReach=0.1; 
maxzReach=1;
if isempty(zValue)
    errordlg('Missing Z value', 'Run Error');
    set(hObject,'String',0.0);
elseif zValue<minzReach || zValue > maxzReach
    errordlg('Please enter a suitable value','Value Error');
    set(hObject,'String',0.0);
end

% --- Executes during object creation, after setting all properties.
function zValue_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zValue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q1edit_Callback(hObject, eventdata, handles)
% hObject    handle to q1edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1edit as text
%        str2double(get(hObject,'String')) returns contents of q1edit as a double


% --- Executes during object creation, after setting all properties.
function q1edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q2edit_Callback(hObject, eventdata, handles)
% hObject    handle to q2edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q2edit as text
%        str2double(get(hObject,'String')) returns contents of q2edit as a double


% --- Executes during object creation, after setting all properties.
function q2edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q2edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q3edit_Callback(hObject, eventdata, handles)
% hObject    handle to q3edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q3edit as text
%        str2double(get(hObject,'String')) returns contents of q3edit as a double


% --- Executes during object creation, after setting all properties.
function q3edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q3edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q4edit_Callback(hObject, eventdata, handles)
% hObject    handle to q4edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q4edit as text
%        str2double(get(hObject,'String')) returns contents of q4edit as a double


% --- Executes during object creation, after setting all properties.
function q4edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q4edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q5edit_Callback(hObject, eventdata, handles)
% hObject    handle to q5edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q5edit as text
%        str2double(get(hObject,'String')) returns contents of q5edit as a double


% --- Executes during object creation, after setting all properties.
function q5edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q5edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q6edit_Callback(hObject, eventdata, handles)
% hObject    handle to q6edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q6edit as text
%        str2double(get(hObject,'String')) returns contents of q6edit as a double


% --- Executes during object creation, after setting all properties.
function q6edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q6edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q7edit_Callback(hObject, eventdata, handles)
% hObject    handle to q7edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q7edit as text
%        str2double(get(hObject,'String')) returns contents of q7edit as a double


% --- Executes during object creation, after setting all properties.
function q7edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q7edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in run_btn.
function run_btn_Callback(hObject, eventdata, handles)
% hObject    handle to run_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%get the value from the box

getX=str2double(get(handles.xValue,'String'))
getY=str2double(get(handles.yValue,'String'))
getZ=str2double(get(handles.zValue,'String'))

inputCoord=[getX,getY,getZ]
handles.kinova.TeachMove(inputCoord)
