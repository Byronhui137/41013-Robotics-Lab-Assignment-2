function varargout = CoffeeMaking_GUIv1(varargin)
% COFFEEMAKING_GUIV1 MATLAB code for CoffeeMaking_GUIv1.fig
%      COFFEEMAKING_GUIV1, by itself, creates a new COFFEEMAKING_GUIV1 or raises the existing
%      singleton*.
%
%      H = COFFEEMAKING_GUIV1 returns the handle to a new COFFEEMAKING_GUIV1 or the handle to
%      the existing singleton*.
%
%      COFFEEMAKING_GUIV1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in COFFEEMAKING_GUIV1.M with the given input arguments.
%
%      COFFEEMAKING_GUIV1('Property','Value',...) creates a new COFFEEMAKING_GUIV1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CoffeeMaking_GUIv1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CoffeeMaking_GUIv1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CoffeeMaking_GUIv1

% Last Modified by GUIDE v2.5 29-May-2020 21:12:07

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CoffeeMaking_GUIv1_OpeningFcn, ...
                   'gui_OutputFcn',  @CoffeeMaking_GUIv1_OutputFcn, ...
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


% --- Executes just before CoffeeMaking_GUIv1 is made visible.
function CoffeeMaking_GUIv1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CoffeeMaking_GUIv1 (see VARARGIN)

handles.kinova=Assignment2Functions;
handles.kinova.HumanHand();
camlight;
% Choose default command line output for CoffeeMaking_GUIv1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CoffeeMaking_GUIv1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CoffeeMaking_GUIv1_OutputFcn(hObject, eventdata, handles) 
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


% --- Executes on button press in robotCollision_btn.
function robotCollision_btn_Callback(hObject, eventdata, handles)
% hObject    handle to robotCollision_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in outsideCollision_btn.
function outsideCollision_btn_Callback(hObject, eventdata, handles)
% hObject    handle to outsideCollision_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in eStop_btn.
function eStop_btn_Callback(hObject, eventdata, handles)
% hObject    handle to eStop_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in exit_Btn.
function exit_Btn_Callback(hObject, eventdata, handles)
% hObject    handle to exit_Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close CoffeeMaking_GUIv1
Start_GUI

% --- Executes on slider movement.
function handSlider_Callback(hObject, eventdata, handles)
% hObject    handle to handSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.kinova.handlocation(1,4)=get(hObject,'Value'); %get pos from the slider
updatePoints=[handles.kinova.handlocation*[handles.kinova.handVerts,ones(handles.kinova.handvCount,1)]']'; %update the verteies and faces
handles.kinova.handMesh_h.Vertices=updatePoints(:,1:3); %move the hand 

% --- Executes during object creation, after setting all properties.
function handSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to handSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
