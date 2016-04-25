function varargout = showrobotpos(varargin)
% SHOWROBOTPOS MATLAB code for showrobotpos.fig
%      SHOWROBOTPOS, by itself, creates a new SHOWROBOTPOS or raises the existing
%      singleton*.
%
%      H = SHOWROBOTPOS returns the handle to a new SHOWROBOTPOS or the handle to
%      the existing singleton*.
%
%      SHOWROBOTPOS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SHOWROBOTPOS.M with the given input arguments.
%
%      SHOWROBOTPOS('Property','Value',...) creates a new SHOWROBOTPOS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before showrobotpos_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to showrobotpos_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help showrobotpos

% Last Modified by GUIDE v2.5 19-Apr-2016 11:24:54

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @showrobotpos_OpeningFcn, ...
                   'gui_OutputFcn',  @showrobotpos_OutputFcn, ...
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


% --- Executes just before showrobotpos is made visible.
function showrobotpos_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to showrobotpos (see VARARGIN)

% Choose default command line output for showrobotpos
handles.output = hObject;

handles.showrobotfigure = figure(2);
axes;
handles.lastAER = [0, 90, 0];
handles.lastjointpos = [0, 0, -90, 0, 0, 0];
handles.lastp = [1755.33, 0, 2255];
% Update handles structure
guidata(hObject, handles);


% UIWAIT makes showrobotpos wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = showrobotpos_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function joint1_slider_Callback(hObject, eventdata, handles)
joint(1) = get(handles.joint1_slider, 'value');
joint(2) = get(handles.joint2_slider, 'value');
joint(3) = get(handles.joint3_slider, 'value');
joint(4) = get(handles.joint4_slider, 'value');
joint(5) = get(handles.joint5_slider, 'value');
joint(6) = get(handles.joint6_slider, 'value');

showjointpos(handles, hObject, joint);
joint = joint * pi / 180;

drawrobotpos(joint, handles.showrobotfigure);


function showjointpos(handles, hObject, joint)

handles.lastjointpos = joint;

set(handles.joint1pos_edit, 'string', num2str(joint(1), '%6.8f'));
set(handles.joint2pos_edit, 'string', num2str(joint(2), '%6.8f'));
set(handles.joint3pos_edit, 'string', num2str(joint(3), '%6.8f'));
set(handles.joint4pos_edit, 'string', num2str(joint(4), '%6.8f'));
set(handles.joint5pos_edit, 'string', num2str(joint(5), '%6.8f'));
set(handles.joint6pos_edit, 'string', num2str(joint(6), '%6.8f'));

jointrad = joint * pi / 180;
g = forwardkinamicsDH(jointrad);
[enlerangle, p] = rotatemat2enlerangle(g);
if norm(enlerangle(1, :) - handles.lastAER) < norm(enlerangle(2, :) - handles.lastAER)
    handles.lastAER = enlerangle(1, :);

else
    handles.lastAER = enlerangle(2, :);
end
handles.lastp = p;
set(handles.baseX_edit, 'string', num2str(p(1), '%6.8f'));
set(handles.baseY_edit, 'string', num2str(p(2), '%6.8f'));
set(handles.baseZ_edit, 'string', num2str(p(3), '%6.8f'));

set(handles.baseA_edit, 'string', num2str(handles.lastAER(1), '%6.8f'));
set(handles.baseE_edit, 'string', num2str(handles.lastAER(2), '%6.8f'));
set(handles.baseR_edit, 'string', num2str(handles.lastAER(3), '%6.8f'));

guidata(hObject, handles);


function joint1_slider_CreateFcn(hObject, eventdata, handles)

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function joint2_slider_Callback(hObject, eventdata, handles)
joint(1) = get(handles.joint1_slider, 'value');
joint(2) = get(handles.joint2_slider, 'value');
joint(3) = get(handles.joint3_slider, 'value');
joint(4) = get(handles.joint4_slider, 'value');
joint(5) = get(handles.joint5_slider, 'value');
joint(6) = get(handles.joint6_slider, 'value');

showjointpos(handles, hObject, joint);

joint = joint * pi / 180;
drawrobotpos(joint, handles.showrobotfigure);

function joint2_slider_CreateFcn(hObject, eventdata, handles)

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function joint3_slider_Callback(hObject, eventdata, handles)
joint(1) = get(handles.joint1_slider, 'value');
joint(2) = get(handles.joint2_slider, 'value');
joint(3) = get(handles.joint3_slider, 'value');
joint(4) = get(handles.joint4_slider, 'value');
joint(5) = get(handles.joint5_slider, 'value');
joint(6) = get(handles.joint6_slider, 'value');

showjointpos(handles, hObject, joint);

joint = joint * pi / 180;
drawrobotpos(joint, handles.showrobotfigure);

function joint3_slider_CreateFcn(hObject, eventdata, handles)

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function joint4_slider_Callback(hObject, eventdata, handles)
joint(1) = get(handles.joint1_slider, 'value');
joint(2) = get(handles.joint2_slider, 'value');
joint(3) = get(handles.joint3_slider, 'value');
joint(4) = get(handles.joint4_slider, 'value');
joint(5) = get(handles.joint5_slider, 'value');
joint(6) = get(handles.joint6_slider, 'value');

showjointpos(handles, hObject, joint);

joint = joint * pi / 180;
drawrobotpos(joint, handles.showrobotfigure);

function joint4_slider_CreateFcn(hObject, eventdata, handles)

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function joint5_slider_Callback(hObject, eventdata, handles)
joint(1) = get(handles.joint1_slider, 'value');
joint(2) = get(handles.joint2_slider, 'value');
joint(3) = get(handles.joint3_slider, 'value');
joint(4) = get(handles.joint4_slider, 'value');
joint(5) = get(handles.joint5_slider, 'value');
joint(6) = get(handles.joint6_slider, 'value');

showjointpos(handles, hObject, joint);

joint = joint * pi / 180;
drawrobotpos(joint, handles.showrobotfigure);

function joint5_slider_CreateFcn(hObject, eventdata, handles)

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function joint6_slider_Callback(hObject, eventdata, handles)
joint(1) = get(handles.joint1_slider, 'value');
joint(2) = get(handles.joint2_slider, 'value');
joint(3) = get(handles.joint3_slider, 'value');
joint(4) = get(handles.joint4_slider, 'value');
joint(5) = get(handles.joint5_slider, 'value');
joint(6) = get(handles.joint6_slider, 'value');

showjointpos(handles, hObject, joint);

joint = joint * pi / 180;
drawrobotpos(joint, handles.showrobotfigure);

function joint6_slider_CreateFcn(hObject, eventdata, handles)

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function recordposition_pushbutton_Callback(hObject, eventdata, handles)
if isfield(handles, 'savedpos') == 1
    n = size(handles.savedpos, 1);
    handles.savedpos(n + 1, 1:6) = handles.lastjointpos;
    handles.savedpos(n + 1, 7:9) = handles.lastp;
    handles.savedpos(n + 1, 10:12) = handles.lastAER;
else
    handles.savedpos(1, 1:6) = handles.lastjointpos;
    handles.savedpos(1, 7:9) = handles.lastp;
    handles.savedpos(1, 10:12) = handles.lastAER;
end
guidata(hObject, handles);

function movetojointpos_pushbutton_Callback(hObject, eventdata, handles)
joint(1) = str2num(get(handles.joint1pos_edit, 'string'));
joint(2) = str2num(get(handles.joint2pos_edit, 'string'));
joint(3) = str2num(get(handles.joint3pos_edit, 'string'));
joint(4) = str2num(get(handles.joint4pos_edit, 'string'));
joint(5) = str2num(get(handles.joint5pos_edit, 'string'));
joint(6) = str2num(get(handles.joint6pos_edit, 'string'));

set(handles.joint1_slider, 'value', joint(1));
set(handles.joint2_slider, 'value', joint(2));
set(handles.joint3_slider, 'value', joint(3));
set(handles.joint4_slider, 'value', joint(4));
set(handles.joint5_slider, 'value', joint(5));
set(handles.joint6_slider, 'value', joint(6));

showjointpos(handles, hObject, joint);

joint = joint * pi / 180;
drawrobotpos(joint, handles.showrobotfigure);



function joint1pos_edit_Callback(hObject, eventdata, handles)


function joint1pos_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function joint2pos_edit_Callback(hObject, eventdata, handles)


function joint2pos_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function joint3pos_edit_Callback(hObject, eventdata, handles)


function joint3pos_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function joint4pos_edit_Callback(hObject, eventdata, handles)


function joint4pos_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function joint5pos_edit_Callback(hObject, eventdata, handles)


function joint5pos_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function joint6pos_edit_Callback(hObject, eventdata, handles)


function joint6pos_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function usercoorX_edit_Callback(hObject, eventdata, handles)


function usercoorX_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit9_Callback(hObject, eventdata, handles)


function edit9_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit10_Callback(hObject, eventdata, handles)


function edit10_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function usercoorY_edit_Callback(hObject, eventdata, handles)


function usercoorY_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function usercoorZ_edit_Callback(hObject, eventdata, handles)


function usercoorZ_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function usercoorA_edit_Callback(hObject, eventdata, handles)


function usercoorA_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function usercoorE_edit_Callback(hObject, eventdata, handles)


function usercoorE_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function usercoorR_edit_Callback(hObject, eventdata, handles)


function usercoorR_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function movetobasepos_pushbutton_Callback(hObject, eventdata, handles)
global Tt;
global Tu;
p(1) = str2num(get(handles.baseX_edit, 'string'));
p(2) = str2num(get(handles.baseY_edit, 'string'));
p(3) = str2num(get(handles.baseZ_edit, 'string'));

AER(1) = str2num(get(handles.baseA_edit, 'string'));
AER(2) = str2num(get(handles.baseE_edit, 'string'));
AER(3) = str2num(get(handles.baseR_edit, 'string'));

g = enlerangle2rotatemat(p, AER);

jp = inversekinamicsDH2(Tu * g / Tt);
jp = jp * 180 / pi;
maxdis = 100000;
for i = 1:size(jp, 1)
%     jp(i, :) - handles.lastjointpos
    temp = norm(jp(i, :) - handles.lastjointpos);
    if temp < maxdis
        maxindex = i;
        maxdis = temp;
    end
end

joint = jp(maxindex, :);

set(handles.joint1_slider, 'value', joint(1));
set(handles.joint2_slider, 'value', joint(2));
set(handles.joint3_slider, 'value', joint(3));
set(handles.joint4_slider, 'value', joint(4));
set(handles.joint5_slider, 'value', joint(5));
set(handles.joint6_slider, 'value', joint(6));

showjointpos(handles, hObject, joint);

handles.lastjointpos = joint;

joint = joint * pi / 180;
drawrobotpos(joint, handles.showrobotfigure);


guidata(hObject, handles);


function backtozero_pushbutton_Callback(hObject, eventdata, handles)
joint = [0, 0, -90, 0, 0, 0];
set(handles.joint1_slider, 'value', joint(1));
set(handles.joint2_slider, 'value', joint(2));
set(handles.joint3_slider, 'value', joint(3));
set(handles.joint4_slider, 'value', joint(4));
set(handles.joint5_slider, 'value', joint(5));
set(handles.joint6_slider, 'value', joint(6));

showjointpos(handles, hObject, joint);

handles.lastjointpos = joint;

joint = joint * pi / 180;
drawrobotpos(joint, handles.showrobotfigure);
guidata(hObject, handles);


function clearrecordpos_pushbutton_Callback(hObject, eventdata, handles)
if isfield(handles, 'savedpos') == 1
    handles = rmfield(handles, 'savedpos');
end
guidata(hObject, handles);


function savepathfile_pushbutton_Callback(hObject, eventdata, handles)
if isfield(handles, 'savedpos') == 1
    % ����ļ��д�����ڣ���������������    
    dirc = ['..\Data\Output\' datestr(now, 29) ];
    if exist([dirc '\robotpath'], 'dir') == 0
        mkdir(dirc, 'robotpath');
    end
    
    % ��ҵ�ǰ��ڵ�ļ�����ӱ�ʹ�������?    fnum = 1;
    fdir = [dirc '\robotpath\teachpath' datestr(now, 29) '-' num2str(fnum, 0) '.mat'];
    while(exist(fdir, 'file') ~= 0)
        fnum = fnum + 1;
        fdir = [dirc '\robotpath\teachpath' datestr(now, 29) '-' num2str(fnum, 0) '.mat'];
    end
    
    % ��������������Ŀ¼��ļ��
    [filename, pahtname, filerindex] = uiputfile({'*.mat'}, '����Խ����',  fdir);
    savepos = handles.savedpos;
    save([pahtname filename], 'savepos');
end


function savepathfile_pushbutton_CreateFcn(hObject, eventdata, handles)

% ������������������û�������б���ȷ��·�����Ϸ���
function usercoordset_pushbutton_Callback(hObject, eventdata, handles)
global Tu;
p = zeros(3, 1); AER = zeros(3, 1);
p(1) = str2num(get(handles.usercoorX_edit, 'string'));
p(2) = str2num(get(handles.usercoorY_edit, 'string'));
p(3) = str2num(get(handles.usercoorZ_edit, 'string'));

AER(1) = str2num(get(handles.usercoorA_edit, 'string'));
AER(2) = str2num(get(handles.usercoorE_edit, 'string'));
AER(3) = str2num(get(handles.usercoorR_edit, 'string'));

handles.Tu = enlerangle2rotatemat(p, AER);
Tu = handles.Tu;
guidata(hObject, handles);


function toolcoordX_edit_Callback(hObject, eventdata, handles)


function toolcoordX_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolcoordY_edit_Callback(hObject, eventdata, handles)


function toolcoordY_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolcoordZ_edit_Callback(hObject, eventdata, handles)


function toolcoordZ_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolcoordA_edit_Callback(hObject, eventdata, handles)


function toolcoordA_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolcoordE_edit_Callback(hObject, eventdata, handles)


function toolcoordE_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolcoordR_edit_Callback(hObject, eventdata, handles)


function toolcoordR_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% 
function toolcoordset_pushbutton_Callback(hObject, eventdata, handles)
global Tt;

p = zeros(3, 1); AER = zeros(3, 1);
p(1) = str2num(get(handles.toolcoordX_edit, 'string'));
p(2) = str2num(get(handles.toolcoordY_edit, 'string'));
p(3) = str2num(get(handles.toolcoordZ_edit, 'string'));

AER(1) = str2num(get(handles.toolcoordA_edit, 'string'));
AER(2) = str2num(get(handles.toolcoordE_edit, 'string'));
AER(3) = str2num(get(handles.toolcoordR_edit, 'string'));

handles.Tt = enlerangle2rotatemat(p, AER);
Tt = handles.Tt;
guidata(hObject, handles);


function toolposX_edit_Callback(hObject, eventdata, handles)


function toolposX_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolposY_edit_Callback(hObject, eventdata, handles)


function toolposY_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolposZ_edit_Callback(hObject, eventdata, handles)


function toolposZ_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolposA_edit_Callback(hObject, eventdata, handles)


function toolposA_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolposE_edit_Callback(hObject, eventdata, handles)


function toolposE_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolposR_edit_Callback(hObject, eventdata, handles)


function toolposR_edit_CreateFcn(hObject, eventdata, handles)

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function toolmovetouser_pushbutton_Callback(hObject, eventdata, handles)
p = zeros(3, 1); AER = zeros(3, 1);
p(1) = str2num(get(handles.toolposX_edit, 'string'));
p(2) = str2num(get(handles.toolposY_edit, 'string'));
p(3) = str2num(get(handles.toolposZ_edit, 'string'));

AER(1) = str2num(get(handles.toolposA_edit, 'string'));
AER(2) = str2num(get(handles.toolposE_edit, 'string'));
AER(3) = str2num(get(handles.toolposR_edit, 'string'));

toolposuser = enlerangle2rotatemat(p, AER);
g = handles.Tu * toolposuser * inv(handles.Tt);
jp = inversekinamicsDH2(g);
jp = jp * 180 / pi;
maxdis = 100000;
for i = 1:size(jp, 1)
%     jp(i, :) - handles.lastjointpos
    temp = norm(jp(i, :) - handles.lastjointpos);
    if temp < maxdis
        maxindex = i;
        maxdis = temp;
    end
end

joint = jp(maxindex, :);

set(handles.joint1_slider, 'value', joint(1));
set(handles.joint2_slider, 'value', joint(2));
set(handles.joint3_slider, 'value', joint(3));
set(handles.joint4_slider, 'value', joint(4));
set(handles.joint5_slider, 'value', joint(5));
set(handles.joint6_slider, 'value', joint(6));

showjointpos(handles, hObject, joint);

handles.lastjointpos = joint;

joint = joint * pi / 180;
drawrobotpos(joint, handles.showrobotfigure);


guidata(hObject, handles);

function baseR_edit_CreateFcn(hObject, eventdata, handles)
function baseE_edit_CreateFcn(hObject, eventdata, handles)
function baseA_edit_CreateFcn(hObject, eventdata, handles)
function baseX_edit_CreateFcn(hObject, eventdata, handles)
function baseY_edit_CreateFcn(hObject, eventdata, handles)
function baseZ_edit_CreateFcn(hObject, eventdata, handles)



function baseX_edit_Callback(hObject, eventdata, handles)
% hObject    handle to baseX_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of baseX_edit as text
%        str2double(get(hObject,'String')) returns contents of baseX_edit as a double
