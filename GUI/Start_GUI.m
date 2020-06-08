function varargout = Start_GUI(varargin)
% START_GUI MATLAB code for Start_GUI.fig
%      START_GUI, by itself, creates a new START_GUI or raises the existing
%      singleton*.
%
%      H = START_GUI returns the handle to a new START_GUI or the handle to
%      the existing singleton*.
%
%      START_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in START_GUI.M with the given input arguments.
%
%      START_GUI('Property','Value',...) creates a new START_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Start_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Start_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Start_GUI

% Last Modified by GUIDE v2.5 07-Jun-2020 15:17:02

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Start_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Start_GUI_OutputFcn, ...
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


% --- Executes just before Start_GUI is made visible.
function Start_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Start_GUI (see VARARGIN)

% Choose default command line output for Start_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Start_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Start_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in make_coffee.
function make_coffee_Callback(hObject, eventdata, handles)
% hObject    handle to make_coffee (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CoffeeMaking_GUI
close Start_GUI


% --- Executes on button press in teach_button.
function teach_button_Callback(hObject, eventdata, handles)
% hObject    handle to teach_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Teach_GUIv1
close Start_GUI

% --- Executes on button press in exit_button.
function exit_button_Callback(hObject, eventdata, handles)
% hObject    handle to exit_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close Start_GUI


% --- Executes on button press in remote_btn.
function remote_btn_Callback(hObject, eventdata, handles)
% hObject    handle to remote_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
remotecontrol
