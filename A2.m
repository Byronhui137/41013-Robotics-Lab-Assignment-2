%% hello
L1 = Link('d',(0.1564 + 0.1284) ,'a',0,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-360),deg2rad(360)]);
L2 = Link('d', (0.0054 + 0.0064) ,'a',0,'alpha',-pi/2,'offset', 0.0054,'qlim',[deg2rad(-128.9),deg2rad(128.9)]);
L3 = Link('d', (0.2104 + 0.2104) ,'a',0,'alpha',-pi/2,'offset', 0.0064,'qlim',[deg2rad(0),deg2rad(0)]);
L4 = Link('d', (0.0064 + 0.0064) ,'a',0,'alpha',pi/2,'offset',0.0064,'qlim',[deg2rad(-147.8),deg2rad(147.8)]);
L5 = Link('d', (0.2084 + 0.1059),'a',0,'alpha',pi/2,'offset', 0.0064,'qlim',0);
L6 = Link('d',0,'a',0,'alpha',-pi/2,'offset', 0,'qlim',[deg2rad(-120.3),deg2rad(120.3)]);
L7 = Link('d', (0.1059 + 0.0615) ,'a',0,'alpha',0,'offset', -pi/2,'qlim',[deg2rad(-150),deg2rad(150)]);


cyton = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','Left');

workspace = [-1.5 1.5 -1.5 1.5 -1.5 1.5];                                      % Set the size of the workspace when drawing the robot
scale = 0.25;
q = [0,0,0,0,0,0,0];
%Arm positions
cyton.base = transl(0,0,0);


%plot arm on graph
clf
cyton.plot(q,'workspace',workspace,'scale',scale,'trail','g.','fps',50);
cyton.teach;

hold on;

%%
set(0,'DefaultFigureWindowStyle','docked')
clf
getKinova = Kinova;
getKinova.GetKinovaRobot();
getKinova.PlotAndColourRobot;
getKinova.KinovaLocation(transl(0,0.45,-0.02));

% cytonTrans=[0,0.45,0];
% getCyton.CytonLocation(transl(cytonTrans));

scale =0;
q = [0,0,0,0,0,0,0];           % joint config for max reach of arm joint4 at -90deg
getKinova.model.plotopt = {'nojoints', 'noname', 'noshadow','nowrist','workspace',getKinova.workspace};
getKinova.model.plot(q,'scale',scale,'fps',50);


hold on
mount=robotBase;
mount.UpdatePos(transl(0,0.45,-0.02));


hold on
Barrier=cabinateBarrier;
Barrier.UpdatePos(transl(0,0,-0.13));

cup1=Cup;
cup1.UpdatePos(transl(0,0.779359914877542,-0.01));

cm=CoffeeMachine;

stackCups=NineCup;
stackCups.UpdatePos(transl(-0.5,0.5,-0.045));

%Light Curtains
x1=1.189;
for i=1:15
    plot3([x1 x1],[1.44 1.44],[1.377 -1.623],'Color','c')
    plot3([x1 x1],[-1.56 -1.56],[1.377 -1.623],'Color','c')
    x1=x1-0.2;
end
    
%%
qCyton=[0, 0, 0, 0, 0, 0, 0];
qDelivery=[0, pi/2, 0, pi/2, 0, pi/2,pi];
qPosition= transl(-0.05,1.1,-0.1);
qLocation=getKinova.model.ikcon(qPosition);
steps= 100;


s=lspb(0,1,steps);
qMatrix= nan(steps,7);



for i=1:steps
    qMatrix(i,:)=(1-s(i))*qCyton + s(i)*qDelivery;
    getKinova.model.animate(qMatrix(i,:));
end
 cytoneff=getKinova.model.fkine(qMatrix(i,:));
