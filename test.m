L1 = Link('d',0.053,'a',0,'alpha',pi/2,'offset', deg2rad(-85),'qlim',[deg2rad(-150),deg2rad(150)]);
L2 = Link('d',0,'a',0,'alpha',-pi/2,'offset', 0,'qlim',[deg2rad(-105),deg2rad(105)]);
L3 = Link('d',0.128,'a',0,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-150),deg2rad(150)]);
L4 = Link('d',0,'a',0.065,'alpha',-pi/2,'offset', pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
L5 = Link('d',0,'a',0.068,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-105),deg2rad(105)]);
L6 = Link('d',0,'a',0,'alpha',-pi/2,'offset', -pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
L7 = Link('d',0.17,'a',0,'alpha',0,'offset', -pi/2,'qlim',[deg2rad(-150),deg2rad(150)]);


cyton = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','Left');

workspace = [-0.4 0.4 -0.4 0.4 0 0.6];                                      % Set the size of the workspace when drawing the robot
scale = 0.25;
q = [0,0,0,0,0,0,0];
%Arm positions
cyton.base = transl(0,0,0);


%plot arm on graph

clf
cyton.plot(q,'workspace',workspace,'scale',scale,'trail','g.','fps',50);
cyton.teach;
view(3);
%%
clc
clear all
clf
set(0,'DefaultFigureWindowStyle','docked')
getCyton = Cyton;
getCyton.GetCytonRobot();
getCyton.PlotAndColourRobot

scale =0.1;
q = [0,0,0,0,0,0,0];           % joint config for max reach of arm joint4 at -90deg
getCyton.model.plotopt = {'nojoints', 'noname', 'noshadow','nowrist','workspace',getCyton.workspace};
getCyton.model.plot(q,'scale',scale,'fps',50);

hold on
mount=robotBase;
mount.UpdatePos(transl(0,0,-0.02));

qCyton=[0, 0, 0, 0, 0, 0, 0];
qPosition= transl(2,2,0);
qLocation=getCyton.model.ikcon(qPosition);
steps= 35;


s=lspb(0,1,steps);
qMatrix= nan(steps,7);



for i=1:steps
    qMatrix(i,:)=(1-s(i))*qCyton + s(i)*qLocation;
    getCyton.model.animate(qMatrix(i,:));
end