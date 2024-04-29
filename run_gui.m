function varargout = run_gui(varargin)
% RUN_GUI MATLAB code for run_gui.fig
%      RUN_GUI, by itself, creates a new RUN_GUI or raises the existing
%      singleton*.
%
%      H = RUN_GUI returns the handle to a new RUN_GUI or the handle to
%      the existing singleton*.
%
%      RUN_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RUN_GUI.M with the given input arguments.
%
%      RUN_GUI('Property','Value',...) creates a new RUN_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before run_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to run_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help run_gui

% Last Modified by GUIDE v2.5 29-Apr-2024 17:01:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @run_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @run_gui_OutputFcn, ...
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


% --- Executes just before run_gui is made visible.
function run_gui_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to run_gui (see VARARGIN)
    clc;
    global prev_pos;
    prev_pos = [0 0 1.315 0 0 0]';
    global prev_angles;
    prev_angles = zeros(1,7);
    robot = make_robot();
    axes(handles.axes1);
    scatter3(0, 0, 0);
    robot.plot(zeros(1,7));
    cla(handles.axes2);
    cla(handles.axes3);
    cla(handles.axes4);
    cla(handles.axes5);

    % Choose default command line output for run_gui
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

    % UIWAIT makes run_gui wait for user response (see UIRESUME)
    % uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = run_gui_OutputFcn(hObject, eventdata, handles) 
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

    % Extract Pose
    X = str2double(get(handles.editX,"String"));
    Y = str2double(get(handles.editY,"String"));
    Z = str2double(get(handles.editZ,"String"));
    roll = str2double(get(handles.editRoll,"String"));
    pitch = str2double(get(handles.editPitch,"String"));
    yaw = str2double(get(handles.editYaw,"String"));

    % Extract Forces
    F_x = str2double(get(handles.ForceX,"String"));
    F_y = str2double(get(handles.ForceY,"String"));
    F_z = str2double(get(handles.ForceZ,"String"));
    Force = [F_x, F_y, F_z];

    % Clear the Plots
    % clc;
    cla(handles.axes2);
    cla(handles.axes3);
    cla(handles.axes4);
    cla(handles.axes5);

    % Get current pose
    global prev_pos;
    init_point = prev_pos;
    global prev_angles;
    currentangle = prev_angles;

    % Solve RNE
    path = [init_point [X Y Z roll pitch yaw]'];
    [flag, angles, t_acc, jointpos, jointvel, jointacc, tau_acc] = point2point(path, currentangle, Force);

    if flag == false
        h = warndlg('Unable to Solve Inverse Kinematics!! Please Provide Another Position.','Inverse Kinematics Error');
        return;
    end

    robot = make_robot();

    % scatter3(path(1,2), path(2,2), path(3,2), 'filled');
    % Plot the results
    l = size(t_acc,2);
    for ii = 1:50:l
        axes(handles.axes1);
        % scatter3(path(1,2), path(2,2), path(3,2), 'filled');
        robot.plot(jointpos(:,ii)');

        axes(handles.axes2);
        hold on;
        plot(t_acc(1:50:ii), jointpos(1,1:50:ii), 'Linewidth', 2, 'Color','r');
        plot(t_acc(1:50:ii), jointpos(2,1:50:ii), 'Linewidth', 2, 'Color','g');
        plot(t_acc(1:50:ii), jointpos(3,1:50:ii), 'Linewidth', 2, 'Color','b');
        plot(t_acc(1:50:ii), jointpos(4,1:50:ii), 'Linewidth', 2, 'Color','c');
        plot(t_acc(1:50:ii), jointpos(5,1:50:ii), 'Linewidth', 2, 'Color','m');
        plot(t_acc(1:50:ii), jointpos(6,1:50:ii), 'Linewidth', 2, 'Color',[0.5,0.5,0.5]);
        plot(t_acc(1:50:ii), jointpos(7,1:50:ii), 'Linewidth', 2, 'Color','k');
        % hold off;
        title('Position Profiles');
        xlabel('Time [s]'), ylabel('Torque [Nm]');
        xlim([t_acc(1), t_acc(end)]);
        min_jointpos = min(jointpos,[],'all'); % Find the minimum value in jointpos
        max_jointpos = max(jointpos,[],'all'); % Find the maximum value in jointpos
        ylim([min_jointpos, max_jointpos]);
        legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'});
        hold off;

        axes(handles.axes3);
        hold on;
        plot(t_acc(1:50:ii), jointvel(1,1:50:ii), 'Linewidth', 2, 'Color','r');
        plot(t_acc(1:50:ii), jointvel(2,1:50:ii), 'Linewidth', 2, 'Color','g');
        plot(t_acc(1:50:ii), jointvel(3,1:50:ii), 'Linewidth', 2, 'Color','b');
        plot(t_acc(1:50:ii), jointvel(4,1:50:ii), 'Linewidth', 2, 'Color','c');
        plot(t_acc(1:50:ii), jointvel(5,1:50:ii), 'Linewidth', 2, 'Color','m');
        plot(t_acc(1:50:ii), jointvel(6,1:50:ii), 'Linewidth', 2, 'Color',[0.5,0.5,0.5]);
        plot(t_acc(1:50:ii), jointvel(7,1:50:ii), 'Linewidth', 2, 'Color','k');
        % hold off;
        title('Velocity Profiles');
        xlabel('Time [s]'), ylabel('Torque [Nm]');
        xlim([t_acc(1), t_acc(end)]);
        min_jointpos = min(jointvel,[],'all'); % Find the minimum value in jointpos
        max_jointpos = max(jointvel,[],'all'); % Find the maximum value in jointpos
        ylim([min_jointpos, max_jointpos]);
        legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'});
        hold off;

        axes(handles.axes5);
        hold on;
        plot(t_acc(1:50:ii), tau_acc(1,1:50:ii), 'Linewidth', 2, 'Color','r');
        plot(t_acc(1:50:ii), tau_acc(2,1:50:ii), 'Linewidth', 2, 'Color','g');
        plot(t_acc(1:50:ii), tau_acc(3,1:50:ii), 'Linewidth', 2, 'Color','b');
        plot(t_acc(1:50:ii), tau_acc(4,1:50:ii), 'Linewidth', 2, 'Color','c');
        plot(t_acc(1:50:ii), tau_acc(5,1:50:ii), 'Linewidth', 2, 'Color','m');
        plot(t_acc(1:50:ii), tau_acc(6,1:50:ii), 'Linewidth', 2, 'Color',[0.5,0.5,0.5]);
        plot(t_acc(1:50:ii), tau_acc(7,1:50:ii), 'Linewidth', 2, 'Color','k');
        % hold off;
        title('Torque Profiles');
        xlabel('Time [s]'), ylabel('Torque [Nm]');
        xlim([t_acc(1), t_acc(end)]);
        min_jointpos = min(tau_acc,[],'all'); % Find the minimum value in jointpos
        max_jointpos = max(tau_acc,[],'all'); % Find the maximum value in jointpos
        ylim([min_jointpos, max_jointpos]);
        legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'});
        hold off;

        axes(handles.axes4);
        hold on;
        plot(t_acc(1:50:ii), jointacc(1,1:50:ii), 'Linewidth', 2, 'Color','r');
        plot(t_acc(1:50:ii), jointacc(2,1:50:ii), 'Linewidth', 2, 'Color','g');
        plot(t_acc(1:50:ii), jointacc(3,1:50:ii), 'Linewidth', 2, 'Color','b');
        plot(t_acc(1:50:ii), jointacc(4,1:50:ii), 'Linewidth', 2, 'Color','c');
        plot(t_acc(1:50:ii), jointacc(5,1:50:ii), 'Linewidth', 2, 'Color','m');
        plot(t_acc(1:50:ii), jointacc(6,1:50:ii), 'Linewidth', 2, 'Color',[0.5,0.5,0.5]);
        plot(t_acc(1:50:ii), jointacc(7,1:50:ii), 'Linewidth', 2, 'Color','k');
        % hold off;
        title('Acceleration Profiles');
        xlabel('Time [s]'), ylabel('Torque [Nm]');
        xlim([t_acc(1), t_acc(end)]);
        min_jointpos = min(jointacc,[],'all'); % Find the minimum value in jointpos
        max_jointpos = max(jointacc,[],'all'); % Find the maximum value in jointpos
        ylim([min_jointpos, max_jointpos]);
        legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7'});
        hold off;

        drawnow; % This command forces MATLAB to draw the plots immediately
        pause(0.01);
    end
    axes(handles.axes1);
    % Save the updated pose
    prev_pos = [X Y Z roll pitch yaw]';
    prev_angles = angles;


function editX_Callback(hObject, eventdata, handles)
    % hObject    handle to editX (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of editX as text
    %        str2double(get(hObject,'String')) returns contents of editX as a double


% --- Executes during object creation, after setting all properties.
function editX_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to editX (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function editY_Callback(hObject, eventdata, handles)
    % hObject    handle to editY (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of editY as text
    %        str2double(get(hObject,'String')) returns contents of editY as a double


% --- Executes during object creation, after setting all properties.
function editY_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to editY (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function editZ_Callback(hObject, eventdata, handles)
    % hObject    handle to editZ (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of editZ as text
    %        str2double(get(hObject,'String')) returns contents of editZ as a double


% --- Executes during object creation, after setting all properties.
function editZ_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to editZ (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function editRoll_Callback(hObject, eventdata, handles)
    % hObject    handle to editRoll (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of editRoll as text
    %        str2double(get(hObject,'String')) returns contents of editRoll as a double


% --- Executes during object creation, after setting all properties.
function editRoll_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to editRoll (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function editPitch_Callback(hObject, eventdata, handles)
    % hObject    handle to editPitch (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of editPitch as text
    %        str2double(get(hObject,'String')) returns contents of editPitch as a double


% --- Executes during object creation, after setting all properties.
function editPitch_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to editPitch (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function editYaw_Callback(hObject, eventdata, handles)
    % hObject    handle to editYaw (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of editYaw as text
    %        str2double(get(hObject,'String')) returns contents of editYaw as a double


% --- Executes during object creation, after setting all properties.
function editYaw_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to editYaw (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function ForceX_Callback(hObject, eventdata, handles)
    % hObject    handle to ForceX (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of ForceX as text
    %        str2double(get(hObject,'String')) returns contents of ForceX as a double


% --- Executes during object creation, after setting all properties.
function ForceX_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to ForceX (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function ForceY_Callback(hObject, eventdata, handles)
    % hObject    handle to ForceY (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of ForceY as text
    %        str2double(get(hObject,'String')) returns contents of ForceY as a double


% --- Executes during object creation, after setting all properties.
function ForceY_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to ForceY (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end



function ForceZ_Callback(hObject, eventdata, handles)
    % hObject    handle to ForceZ (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of ForceZ as text
    %        str2double(get(hObject,'String')) returns contents of ForceZ as a double


% --- Executes during object creation, after setting all properties.
function ForceZ_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to ForceZ (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
