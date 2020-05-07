clc
clear all
clf

getCyton = Cyton;
hold on
[f,v,data] = plyread('RobotBase.ply','tri');
% Get vertex count
VertexCount = size(v,1);
% Move center point to origin
midPoint = sum(v)/VertexCount;
Verts = v - repmat(midPoint,VertexCount,1);
% Create a transform to describe the location (at the origin, since it's centered
%             pose = eye(4);
% model offsets
zOffset = 0.1;
yOffset = 0;
xOffset = 0;
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% Then plot the trisurf
Mesh_h = trisurf(f,Verts(:,1)+xOffset,Verts(:,2)+yOffset, Verts(:,3)+zOffset ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');