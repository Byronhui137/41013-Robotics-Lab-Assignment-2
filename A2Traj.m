% Create environment
clear all
clc
clf

environment=Environment();
kinova=Kinova();

%%
% Standard Kinova Poses
qStart = [0,0,0,pi/2,0,pi/2,pi/2];

% move2point(kinova.model, 0.3,0,0);
% move2point(kinova.model, 0.75,0,-0.1);

% tr = transl(0.3,0,0) * troty(-pi);
% q = kinova.model.ikcon(tr);
% kinova.Move(qStart,q,20);

tr = transl(0.6,0,0);
q2 = kinova.model.ikcon(tr);
kinova.Move(q,q2,20);

kinova.model.teach;


%%
% q = [0,-1.3963,0,1.9199,0,1.5708,1.5708]
q = [0,-0.3963,0,1.3199,0,-1.9708,1.5708]
tr = kinova.model.fkine(q);
q = kinova.model.ikcon(tr)
%%

function move2point(abc, x,y,z)
    tr = transl(x,y,z);
    q = abc.ikcon(tr);
    abc.Move(q,q,5);
    abc.teach;
end