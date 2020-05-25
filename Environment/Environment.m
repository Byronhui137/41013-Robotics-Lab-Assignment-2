classdef Environment
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods(Static)
        function self=Environment()
            hold on;
            self.cup();
            self.coffeeMachine();
            self.nineCups();
            self.cabinate();
            
        end
        
        %%
            function cup()
                [f, v, data] = plyread('cup.ply','tri');
                
                v_count = size(v,1);
                mid_point = sum(v)/v_count;
                verts = v - repmat(mid_point, v_count, 1);
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                mesh = trisurf(f, verts(:,1), verts(:, 2)-1, verts(:, 3)-0.18...
                    ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
                
            end
            %%
            function coffeeMachine()
                [f, v, data] = plyread('CoffeeMach.ply','tri');
                
                v_count = size(v,1);
                mid_point = sum(v)/v_count;
                verts = v - repmat(mid_point,v_count, 1);
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                mesh = trisurf(f, verts(:,1), verts(:, 2)+0.8, verts(:, 3)-0.1...
                    ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            end
            %%
            function nineCups()
                [f, v, data] = plyread('9cups.ply','tri');
                
                v_count = size(v,1);
                mid_point = sum(v)/v_count;
                verts = v - repmat(mid_point,v_count, 1);
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                mesh = trisurf(f, verts(:,1)+0.75, verts(:, 2), verts(:, 3)-0.22...
                    ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            end
            %%
            function cabinate()
                [f, v, data] = plyread('cabinate.ply','tri');
                
                v_count = size(v,1);
                mid_point = sum(v)/v_count;
                verts = v - repmat(mid_point,v_count, 1);
                vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                mesh = trisurf(f, verts(:,1), verts(:, 2), verts(:, 3)...
                    ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            end
        
    end
end

