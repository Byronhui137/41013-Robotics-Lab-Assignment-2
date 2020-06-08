function varargout = remotecontrol(varargin)
% REMOTECONTROL MATLAB code for remotecontrol.fig
%      REMOTECONTROL, by itself, creates a new REMOTECONTROL or raises the existing
%      singleton*.
%
%      H = REMOTECONTROL returns the handle to a new REMOTECONTROL or the handle to
%      the existing singleton*.
%
%      REMOTECONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in REMOTECONTROL.M with the given input arguments.
%
%      REMOTECONTROL('Property','Value',...) creates a new REMOTECONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before remotecontrol_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to remotecontrol_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help remotecontrol

% Last Modified by GUIDE v2.5 08-Jun-2020 11:50:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @remotecontrol_OpeningFcn, ...
                   'gui_OutputFcn',  @remotecontrol_OutputFcn, ...
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


% --- Executes just before remotecontrol is made visible.
function remotecontrol_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to remotecontrol (see VARARGIN)
handles.kinova=Assignment2Functions;
handles.kinova.HumanHand();
camlight;

% Choose default command line output for remotecontrol
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes remotecontrol wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = remotecontrol_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in start_btn.
function start_btn_Callback(hObject, eventdata, handles)
% hObject    handle to start_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.kinova.Joystick(); %runs joystick function


% --- Executes on button press in exit_btn.
function exit_btn_Callback(hObject, eventdata, handles)
% hObject    handle to exit_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close remotecontrol
Start_GUI
