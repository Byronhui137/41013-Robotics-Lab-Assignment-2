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

% Last Modified by GUIDE v2.5 22-May-2020 11:50:10

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
handles.environment=Environment();
handles.kinova=Kinova();
handles.kinova.model.teach;

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
