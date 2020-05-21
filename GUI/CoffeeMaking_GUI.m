function varargout = CoffeeMaking_GUI(varargin)
% COFFEEMAKING_GUI MATLAB code for CoffeeMaking_GUI.fig
%      COFFEEMAKING_GUI, by itself, creates a new COFFEEMAKING_GUI or raises the existing
%      singleton*.
%
%      H = COFFEEMAKING_GUI returns the handle to a new COFFEEMAKING_GUI or the handle to
%      the existing singleton*.
%
%      COFFEEMAKING_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in COFFEEMAKING_GUI.M with the given input arguments.
%
%      COFFEEMAKING_GUI('Property','Value',...) creates a new COFFEEMAKING_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CoffeeMaking_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CoffeeMaking_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CoffeeMaking_GUI

% Last Modified by GUIDE v2.5 19-May-2020 17:18:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CoffeeMaking_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @CoffeeMaking_GUI_OutputFcn, ...
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


% --- Executes just before CoffeeMaking_GUI is made visible.
function CoffeeMaking_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CoffeeMaking_GUI (see VARARGIN)
handles.environment=Environment();
handles.kinova=Kinova();

camlight;
% set(findall(hObject, '-property','Units'),'Units','Normalized')
% Choose default command line output for CoffeeMaking_GUI
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CoffeeMaking_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CoffeeMaking_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in start_Simulation.
function start_Simulation_Callback(hObject, eventdata, handles)
% hObject    handle to start_Simulation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in robot_collision.
function robot_collision_Callback(hObject, eventdata, handles)
% hObject    handle to robot_collision (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in estop_Btn.
function estop_Btn_Callback(hObject, eventdata, handles)
% hObject    handle to estop_Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in outside_collision.
function outside_collision_Callback(hObject, eventdata, handles)
% hObject    handle to outside_collision (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in exit_Btn.
function exit_Btn_Callback(hObject, eventdata, handles)
% hObject    handle to exit_Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse press over figure background.
function figure1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
