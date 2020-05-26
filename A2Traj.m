% Create environment
clear all
clc
clf

enviro=Environment();
kinova=Kinova();
kinova.GetObject(enviro.cup);

% q=[0,0,0,0,0,0,0];
% scale=0.1;
% kinova.model.plotopt = {'nojoints', 'noname', 'noshadow','nowrist','workspace',kinova.workspace};
% kinova.model.plot(q,'scale',scale,'fps',50);

%%
% Standard Kinova Poses
qStart = [0,0,0,pi/2,0,pi/2,pi];

trCup = transl(0.6,0,-0.2) * trotx(-pi) * troty(deg2rad(80)) * trotz(-pi);
% trCoffeeMachine = transl(-0.1,0.7,-0.18) * trotx(-pi/2) * troty(1.5*pi) * trotz(-pi/2);
% trCoffeeMachine = transl(-0.08,0.7,-0.18) * trotx(-pi/2) * troty(0) * trotz(pi/2);
% trCoffeeMachine = transl(0.05,0.2,0) * trotx(1.5*pi) * troty(-2*pi) * trotz(-1.5*pi);

trCoffeeMachine = transl(-0.115,0.68,-0.2) * trotx(1.5*pi) * troty(-2*pi) * trotz(-1.5*pi);


% trDropOff = transl(0,-1,-0.18) * trotx(-pi/2) * troty(pi) * trotz(pi/2);
% trDropOff = transl(0,-1,-0.18) * trotx(-pi/2) * troty(0) * trotz(pi/2);
% trDropOff = transl(0,-0.8,-0.18) * trotx(1.5*pi) * troty(pi) * trotz(1.5*pi);
trDropOff = transl(0,-0.8,-0.18) * trotx(1.5*pi) * troty(-2*pi) * trotz(1.5*pi);

qCup = kinova.model.ikcon(trCup);
qCoffeeMachine = kinova.model.ikcon(trCoffeeMachine);
qDropOff = kinova.model.ikcon(trDropOff);

% Make Coffee
% kinova.Move(qStart,qCup,60,0);        %%
kinova.Move(qCup,qCoffeeMachine,60,1);      %%      
% kinova.Move(qCoffeeMachine,qDropOff,60,1;     %%

% % for testing
% kinova.Move(qCoffeeMachine,qCoffeeMachine,2,1); 
% kinova.Move(qDropOff,qDropOff,2);

% trCoffeeMachine2 = transl(0.7,0,-0.1) ;
% qCoffeeMachine2 = kinova.model.ikcon(trCoffeeMachine2);
% kinova.Move(qCup,qCoffeeMachine2,10,1);
% kinova.Move(qCoffeeMachine2,qCoffeeMachine2,2,1);

kinova.model.teach;


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
