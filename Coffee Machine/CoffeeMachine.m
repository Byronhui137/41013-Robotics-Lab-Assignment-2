classdef CoffeeMachine < handle
    properties
       pose;
        v_count;
        mid_point;
        verts;
        mesh;
        
    end
    
   methods%% Class for UR3 robot simulation
        function self = CoffeeMachine
            [f, v, data] = plyread('CoffeeMach.ply','tri');
            
            self.v_count = size(v,1);
            self.mid_point = sum(v)/self.v_count;
            self.verts = v - repmat(self.mid_point, self.v_count, 1);
            
            self.pose = eye(4);
            
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            mesh = trisurf(f, verts(:,1), verts(:, 2), verts(:, 3)...
                ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            hold on
        end
        function UpdatePos(self, Pos)
            self.pose = Pos;
            updatedPoints = [Pos * [self.verts,ones(self.v_count,1)]']';
            self.mesh.Vertices = updatedPoints(:,1:3);
        end
   end
end