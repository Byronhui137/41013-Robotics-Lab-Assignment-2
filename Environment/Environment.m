classdef Environment
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cup
        coffeeMachine
        nineCups
        cabinate
        cupWCoffee
        dropOff
        humanhand
    end
    
    methods(Static)
        function self=Environment()
            hold on;
            self.cup=self.Cup();
            self.coffeeMachine=self.CoffeeMachine();
            self.nineCups=self.NineCups();
            self.cabinate=self.Cabinate();
            self.dropOff = self.DropOff();
          
            
            
        end
        %%
        function GetCupWCoffee(self)
            hold on;
            self.CupWCoffee();
        end
     
        %%
        function cup = Cup(self)
            [f, v, data] = plyread('cup.ply','tri');
            
            cup.vCount = size(v,1);
            midPoint = sum(v)/cup.vCount;
            cup.verts = v - repmat(midPoint, cup.vCount, 1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            cup.mesh = trisurf(f, cup.verts(:,1)+0.65, cup.verts(:, 2)-0.012, cup.verts(:, 3)-0.22...
                ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            
        end
        %%
         function cupWCoffee = CupWCoffee(xOffset,yOffset,zOffset)
            [f, v, data] = plyread('cupwithcoffee.ply','tri');
            
            cupWCoffee.vCount = size(v,1);
            midPoint = sum(v)/cupWCoffee.vCount;
            cupWCoffee.verts = v - repmat(midPoint, cupWCoffee.vCount, 1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            cupWCoffee.mesh = trisurf(f, cupWCoffee.verts(:, 1)+xOffset, cupWCoffee.verts(:, 2)+yOffset, cupWCoffee.verts(:, 3)+zOffset...
                ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            
        end
        %%
        function coffeeMachine = CoffeeMachine(self)
            [coffeeMachine.f, coffeeMachine.v, coffeeMachine.data] = plyread('CoffeeMachlow.ply','tri');
            
            coffeeMachine.vCount = size(coffeeMachine.v,1);
            midPoint = sum(coffeeMachine.v)/coffeeMachine.vCount;
            coffeeMachine.verts = coffeeMachine.v - repmat(midPoint,coffeeMachine.vCount, 1);
            vertexColours = [coffeeMachine.data.vertex.red, coffeeMachine.data.vertex.green, coffeeMachine.data.vertex.blue] / 255;
            coffeeMachine.mesh = trisurf(coffeeMachine.f, coffeeMachine.verts(:,1), coffeeMachine.verts(:, 2)+0.8, coffeeMachine.verts(:, 3)-0.1....
                ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat'); %=(y- to +0.8) z -0.1 to 0 
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
        %%        
%         function cupWCoffee = CupWCoffee(xOffset,yOffset,zOffset)
%             [f, v, data] = plyread('cupwithcoffee.ply','tri');
%             
%             cupWCoffee.vCount = size(v,1);
%             midPoint = sum(v)/cupWCoffee.vCount;
%             cupWCoffee.verts = v - repmat(midPoint, cupWCoffee.vCount, 1);
%             vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%             cupWCoffee.mesh = trisurf(f, cupWCoffee.verts(:, 1)+xOffset, cupWCoffee.verts(:, 2)+yOffset, cupWCoffee.verts(:, 3)+zOffset...
%                 ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
%             
%         end              
        %%
        function dropOff = DropOff(self)
            [f, v, data] = plyread('dropoff.ply','tri');
            
            dropOff.vCount = size(v,1);
            midPoint = sum(v)/dropOff.vCount;
            dropOff.verts = v - repmat(midPoint,dropOff.vCount, 1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            dropOff.mesh = trisurf(f, dropOff.verts(:,1)+0.17, dropOff.verts(:, 2)-0.9, dropOff.verts(:, 3)-0.25...
                ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
        end
        
        %%
    end
end


