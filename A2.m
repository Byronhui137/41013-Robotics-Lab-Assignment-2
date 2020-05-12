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

cytonTrans=[0,0.45,0];
getCyton.CytonLocation(transl(cytonTrans));

scale =0;
q = [0,0,0,0,0,0,0];           % joint config for max reach of arm joint4 at -90deg
getCyton.model.plotopt = {'nojoints', 'noname', 'noshadow','nowrist','workspace',getCyton.workspace};
getCyton.model.plot(q,'scale',scale,'fps',50);

hold on
mount=robotBase;
mount.UpdatePos(transl(0,0.45,-0.02));
hold on
Barrier=cabinateBarrier;
Barrier.UpdatePos(transl(0,0,-0.13));

cup1=Cup;
cup1.UpdatePos(transl(0,0.779359914877542,-0.01));

cm=CoffeeMachine;
cm.UpdatePos(transl(0,-0.1,0.11));

%%
qCyton=[0, 0, 0, 0, 0, 0, 0];
qDelivery=[0, -pi/2, 0, 0, 0, 0, 0];
qPosition= transl(-0.05,1.1,-0.1);
qLocation=getCyton.model.ikcon(qPosition);
steps= 100;


s=lspb(0,1,steps);
qMatrix= nan(steps,7);



for i=1:steps
    qMatrix(i,:)=(1-s(i))*qCyton + s(i)*qLocation;
    getCyton.model.animate(qMatrix(i,:));
end
 cytoneff=getCyton.model.fkine(qMatrix(i,:));
%% Robotics
% clf
close all;
set(0,'DefaultFigureWindowStyle','docked')
% 2.1: Make a 3DOF model
L1 = Link('d',0.053,'a',0,'alpha',pi/2,'offset', deg2rad(-85),'qlim',[deg2rad(-150),deg2rad(150)]);
L2 = Link('d',0,'a',0,'alpha',-pi/2,'offset', 0,'qlim',[deg2rad(-105),deg2rad(105)]);
L3 = Link('d',0.128,'a',0,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-150),deg2rad(150)]);
L4 = Link('d',0,'a',0.065,'alpha',-pi/2,'offset', pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
L5 = Link('d',0,'a',0.068,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-105),deg2rad(105)]);
L6 = Link('d',0,'a',0,'alpha',-pi/2,'offset', -pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
L7 = Link('d',0.17,'a',0,'alpha',0,'offset', -pi/2,'qlim',[deg2rad(-150),deg2rad(150)]);      
robot = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','myRobot');                     
q = zeros(1,7);                                                     % Create a vector of initial joint angles        
scale = 0.1;
workspace = [-2 2 -2 2 -2 2];                                       % Set the size of the workspace when drawing the robot
robot.plot(q,'workspace',workspace,'scale',scale, 'fps' ,50);                  % Plot the robot
hold on        
% 2.2 and 2.3
% centerpnt = [0,1,0.11];
% side = 1.5;
% plotOptions.plotFaces = true;
% [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
% axis equal
% camlight
plotOptions.plotFaces = true;
[faces, vertex, data] = plyread('CoffeeMach.ply','tri');
            
            v_count = size(vertex,1);
            mid_point = sum(vertex)/v_count;
            verts = vertex - repmat(mid_point, v_count, 1);
            
%             pose = eye(4);
            
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            mesh = trisurf(faces, verts(:,1)+0, verts(:, 2)+-0.2, verts(:, 3)+0.11...
                ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');


% [vertex, faces, faceNormals] = plyread('CoffeeMach.ply','tri');
%             
%             v_count = size(faces,1);
%             mid_point = sum(faces)/v_count;
%             verts = faces - repmat(mid_point, v_count, 1);
%             
%             pose = eye(4);
%             
%             vertexColours = [faceNormals.vertex.red, faceNormals.vertex.green, faceNormals.vertex.blue] / 255;
%             mesh = trisurf(vertex, verts(:,1), verts(:, 2), verts(:, 3)...
%                 ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
axis equal
 % 2.4: Get the transform of every joint (i.e. start and end of every link)
tr = zeros(4,4,robot.n+1);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% 2.5: Go through each link and also each triangle face
for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = verts(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,verts(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            display('Intersection');
        end
    end    
end

% 2.6: Go through until there are no step sizes larger than 1 degree
q1 = [0, -pi/2, 0, 0, 0, 0, 0];
q2 = [0, pi/2, 0, 0, 0, 0, 0];
steps = 30;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

% 2.7
result = true(steps,1);
for i = 1: steps
    result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
    robot.animate(qMatrix(i,:));
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,verts,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = verts(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,verts(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end
display('hi');