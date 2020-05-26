classdef Environment
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
%         vCount
%         verts
%         mesh
        cup
        coffeeMachine
        nineCups
        cabinate        
    end
    
    methods(Static)
        function self=Environment()
            hold on;            
            self.cup=self.Cup();
            self.coffeeMachine=self.CoffeeMachine();
            self.nineCups=self.NineCups();
            self.cabinate=self.Cabinate();           
        end
                
       %%         
            function cup = Cup(self)
                [f, v, data] = plyread('cup.ply','tri');
                
%                 self.cup.vCount = size(v,1);
%                 midPoint = sum(v)/self.cup.vCount;
%                 self.cup.verts = v - repmat(midPoint, self.cup.vCount, 1);
%                 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%                 self.cup.mesh = trisurf(f, self.cup.verts(:,1), self.cup.verts(:, 2)-1, self.cup.verts(:, 3)-0.18...
%                     ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');

                cup.vCount = size(v,1);
                midPoint = sum(v)/cup.vCount;
                cup.verts = v - repmat(midPoint, cup.vCount, 1);
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                cup.mesh = trisurf(f, cup.verts(:,1)+0.65, cup.verts(:, 2)-0.012, cup.verts(:, 3)-0.22...
                    ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
                
            end
            %%
            function coffeeMachine = CoffeeMachine(self)
                [f, v, data] = plyread('CoffeeMach.ply','tri');
                
                coffeeMachine.vCount = size(v,1);
                midPoint = sum(v)/coffeeMachine.vCount;
                coffeeMachine.verts = v - repmat(midPoint,coffeeMachine.vCount, 1);
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                coffeeMachine.mesh = trisurf(f, coffeeMachine.verts(:,1), coffeeMachine.verts(:, 2)+0.8, coffeeMachine.verts(:, 3)-0.1...
                    ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            end
            %%
            function nineCups = NineCups(self)
                [f, v, data] = plyread('9cups.ply','tri');
                
                nineCups.vCount = size(v,1);
                midPoint = sum(v)/nineCups.vCount;
                nineCups.verts = v - repmat(midPoint,nineCups.vCount, 1);
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                nineCups.mesh = trisurf(f, nineCups.verts(:,1)+0.75, nineCups.verts(:, 2), nineCups.verts(:, 3)-0.22...
                    ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            end
            %%
            function cabinate = Cabinate(self)
                [f, v, data] = plyread('cabinate.ply','tri');
                
                cabinate.vCount = size(v,1);
                midPoint = sum(v)/cabinate.vCount;
                cabinate.verts = v - repmat(midPoint,cabinate.vCount, 1);
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                cabinate.mesh = trisurf(f, cabinate.verts(:,1), cabinate.verts(:, 2), cabinate.verts(:, 3)...
                    ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            end
        
    end
end

