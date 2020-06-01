%% Final
% Create environment
clear all
clc
clf
camlight
enviro=Environment();
kinova=Kinova();
kinova.GetCup(enviro.cup);

pause();
qStart = [0,0,0,pi/2,0,pi/2,pi];
trCup = transl(0.6,0,-0.2) * trotx(-pi) * troty(deg2rad(80)) * trotz(-pi);
trCoffeeMachine = transl(-0.115,0.68,-0.2) * trotx(1.5*pi) * troty(-2*pi) * trotz(-1.5*pi);

qCup = kinova.model.ikcon(trCup);
qCoffeeMachine = kinova.model.ikcon(trCoffeeMachine);
qDropOff = qCoffeeMachine + [deg2rad(180) 0 0 0 0 0 0];

% Make Coffee
kinova.Move(qStart,qCup,60,0);        
kinova.Move(qCup,qCoffeeMachine,60,1);    
kinova.Move(qCoffeeMachine,qDropOff,60,2);   

%% Testing
% Create environment
clear all
clc
clf
camlight
enviro=Environment();
kinova=Kinova();
kinova.GetCup(enviro.cup);

% q=[0,0,0,0,0,0,0];
% scale=0.1;
% kinova.model.plotopt = {'nojoints', 'noname', 'noshadow','nowrist','workspace',kinova.workspace};
% kinova.model.plot(q,'scale',scale,'fps',50);

%%
xOffset = kinova.endEffector(1,4);
yOffset = kinova.endEffector(2,4);
zOffset = kinova.endEffector(3,4);
delete(kinova.cup.mesh);
% enviro=Environment();
%cupWCoffee=Eniroment();
% enviro.CupWCoffee();
enviro.GetCupWCoffee();

%%
clc;
xOffset = kinova.endEffector(1,4);
yOffset = kinova.endEffector(2,4);
zOffset = kinova.endEffector(3,4);
delete(kinova.cup.mesh);
cupWCoffee=enviro.CupWCoffee(xOffset,yOffset,zOffset);
kinova.GetCupWCoffee(cupWCoffee);

%%
kinova.GetCupWCoffee(enviro.cupWCoffee);

%%
% Standard Kinova Poses
qStart = [0,0,0,pi/2,0,pi/2,pi];

trCup = transl(0.6,0,-0.2) * trotx(-pi) * troty(deg2rad(80)) * trotz(-pi);
trCoffeeMachine = transl(-0.115,0.68,-0.2) * trotx(1.5*pi) * troty(-2*pi) * trotz(-1.5*pi);
% trDropOff = transl(0,-0.8,-0.18) * trotx(1.5*pi) * troty(-2*pi) * trotz(1.5*pi);
% trDropOff = transl(0,-0.8,-0.18) * trotx(-1.5*pi) * troty(pi/2) * trotz(-pi/2);
% trDropOff = transl(-0.3,0.5,0.1) * trotx(pi) * troty(2) * trotz(0);
% trDropOff = transl(0,-0.8,-0.18) * trotx(pi/2) * trotz(-pi/2);

qCup = kinova.model.ikcon(trCup);
qCoffeeMachine = kinova.model.ikcon(trCoffeeMachine);
% qDropOff = kinova.model.ikcon(trDropOff);
qDropOff = qCoffeeMachine + [deg2rad(180) 0 0 0 0 0 0];

% Make Coffee
% kinova.Move(qStart,qCup,60,0);        %%
% kinova.Move(qCup,qCoffeeMachine,60,1);      %%      
kinova.Move(qCoffeeMachine,qDropOff,60,2);     %%

% kinova.Move(qCoffeeMachine,qDropOff,30,1);     %%

% % for testing
% kinova.Move(qCoffeeMachine,qCoffeeMachine,2,1); 
% kinova.Move(qDropOff,qDropOff,2,2);

% trCoffeeMachine2 = transl(0.7,0,-0.1) ;
% qCoffeeMachine2 = kinova.model.ikcon(trCoffeeMachine2);
% kinova.Move(qCup,qCoffeeMachine2,10,1);
% kinova.Move(qCoffeeMachine2,qCoffeeMachine2,2,1);

kinova.model.teach;

% -90 / 0 / 90 // -63 === +180
% 90 / 0 / -90 // 109 

%%

% Below stuff is for testing

%%
clear all
clc
clf

enviro=Environment();
q=[0,0,0,0,0,0,0];
scale=0.1;
%%
% q = [0,-1.3963,0,1.9199,0,1.5708,1.5708]
q = [0,-0.3963,0,1.3199,0,-1.9708,1.5708]
tr = kinova.model.fkine(q);
q = kinova.model.ikcon(tr)

%%
q = [0,-70,0,110,0,80,90];
tr = kinova.model.fkine(q);
tr2rpy(tr)
%%
qKinova=[0, 0, 0, 0, 0, 0, 0];
qDelivery=[0, pi/2, 0, pi/2, 0, pi/2,pi];
qPosition= transl(0,-1,-0.19);
qLocation=kinova.model.ikcon(qPosition);
steps= 20;


s=lspb(0,1,steps);
qMatrix= nan(steps,7);



for i=1:steps
    qMatrix(i,:)=(1-s(i))*qKinova + s(i)*qLocation;
    kinova.model.animate(qMatrix(i,:));
end
cytoneff=kinova.model.fkine(qMatrix(i,:));
%%

function move2point(abc, x,y,z)
    tr = transl(x,y,z);
    q = abc.ikcon(tr);
    abc.Move(q,q,5);
    abc.teach;
end

%%
