
% L1 = Link('d',(0.1564 + 0.1284) ,'a',0,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-360),deg2rad(360)]);
% L2 = Link('d', (0.0054 + 0.0064) ,'a',0,'alpha',-pi/2,'offset', 0.0054,'qlim',[deg2rad(-128.9),deg2rad(128.9)]);
% L3 = Link('d', (0.2104 + 0.2104) ,'a',0,'alpha',-pi/2,'offset', 0.0064,'qlim',[deg2rad(0),deg2rad(0)]);
% L4 = Link('d', (0.0064 + 0.0064) ,'a',0,'alpha',pi/2,'offset',0.0064,'qlim',[deg2rad(-147.8),deg2rad(147.8)]);
% L5 = Link('d', (0.2084 + 0.1059),'a',0,'alpha',pi/2,'offset', 0.0064,'qlim',0);
% L6 = Link('d',0,'a',0,'alpha',-pi/2,'offset', 0,'qlim',[deg2rad(-120.3),deg2rad(120.3)]);
% L7 = Link('d', (0.1059 + 0.0615) ,'a',0,'alpha',0,'offset', -pi/2,'qlim',[deg2rad(-150),deg2rad(150)]);

L1 = Link('d',(0.1564 + 0.1284) ,'a',0,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-360),deg2rad(360)]);
L2 = Link('d', (0.0054 + 0.0064) ,'a',0,'alpha',-pi/2,'offset', 0.0054,'qlim',[deg2rad(-128.9),deg2rad(128.9)]);
L3 = Link('d', (0.2104 + 0.2104) ,'a',0,'alpha',-pi/2,'offset', 0.0064,'qlim',[deg2rad(-360),deg2rad(360)]);
L4 = Link('d', (0.0064 + 0.0064) ,'a',0,'alpha',pi/2,'offset',0.0064,'qlim',[deg2rad(-147.8),deg2rad(147.8)]);
L5 = Link('d', (0.2084 + 0.1059),'a',0,'alpha',pi/2,'offset', 0.0064,'qlim',[deg2rad(-360),deg2rad(360)]);
L6 = Link('d',0,'a',0,'alpha',-pi/2,'offset', 0,'qlim',[deg2rad(-120.3),deg2rad(120.3)]);
L7 = Link('d', (0.1059 + 0.0615) ,'a',0,'alpha',0,'offset', -pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);

Kinova = SerialLink([ L1 L2 L3 L4 L5 L6 L7],'name','Kinova');


workspace = [-1.5 1.5 -1.5 1.5 -1.5 1.5];                                      % Set the size of the workspace when drawing the robot
scale = 0.25;
q = [0,0,0,0,0,0,0];
q2=[0,0,pi/2,0,0,0,0];


%Arm positions
Kinova.base = transl(0,0,0);

%plot arm on graph
clf
Kinova.plot(q,'workspace',workspace,'scale',scale,'trail','g.','fps',50);
Kinova.teach;

hold on;

%%
clf
getKinova = Kinova();
getKinova.PlotAndColourRobot;
getKinova.KinovaLocation(transl(0,0.45,-0.1));

% cytonTrans=[0,0.45,0];
% getCyton.CytonLocation(transl(cytonTrans));
hold on
scale =0;

q = [0,0,0,0,0,0,0];           % joint config for max reach of arm joint4 at -90deg



%%
hold on
% mount=robotBase;
% mount.UpdatePos(transl(0,0.45,-0.02));


hold on
Barrier=cabinateBarrier;
Barrier.UpdatePos(transl(0,0,0.20));

cup1=Cup;
cup1.UpdatePos(transl(0,-0.779359914877542,-0.045));

cm=CoffeeMachine;
cm.UpdatePos(trotz(pi/2));

stackCups=NineCup;
stackCups.UpdatePos(transl(-0.5,-0.5,-0.045));

%Light Curtains
x1=1.334;
y1=1.279;
for i=1:14
    plot3([x1 x1],[1.279 1.279],[1.241 -0.05866],'Color','c');
    plot3([x1 x1],[-1.221 -1.221],[1.241 -0.05866],'Color','c');
    x1=x1-0.2;
    
end
for i=1:13
    plot3([-1.286 -1.286],[y1 y1],[1.241 -0.05866],'Color','c');
    plot3([1.334 1.334],[y1 y1],[1.241 -0.05866],'Color','c');
    y1=y1-0.2;
end

%%
qCyton=[0, 0, 0, 0, 0, 0, 0, 0];
qDelivery=[0,pi/2,pi/2, pi/2, pi/2, pi/2,pi/2,pi/2];
qPosition= transl(-0.05,1.2,-0.1);
qLocation=getKinova.model.ikcon(qPosition);

steps= 100;
s=lspb(0,1,steps);
qMatrix= nan(steps,8);



for i=1:steps
    qMatrix(i,:)=(1-s(i))*qCyton + s(i)*qDelivery;
    getKinova.model.animate(qMatrix(i,:));
end

%%
hold on
camlight
kinova=Assignment2Functions;
kinova.HumanHand;
axis equal;


%%
clf
a2=Assignment2Functions;
a2.HumanHand();
camlight;
%%
kinova=Kinova();
