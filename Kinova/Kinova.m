classdef Kinova < handle
    properties
        model;
        pose;
        %          workspace = [-0.4 0.4 -0.4 0.4 -0.2 0.6];
        %          workspace = [-200 200 -200 200 -200 200];
        workspace = [-3 3 -3 3 -3 3];
        %> If we have a tool model which will replace the final links model, combined ply file of the tool model and the final link models
        toolModelFilename = []; % Available are: 'DabPrintNozzleTool.ply';
        toolParametersFilename = []; % Available are: 'DabPrintNozzleToolParameters.mat';
        cup
        cupWCoffee
        coffeeMachine
        vCount
        verts
        mesh
        qMatrix
        endEffector;
        
    end
    
    methods
         function self = Kinova()
             self.GetKinovaRobot();
             self.model.base = transl(0,0,-0.27);
             self.PlotAndColourRobot();
             
         end
        %% GetKinova
        function GetKinovaRobot(self)
            pause(0.001);
            name = ['Kinova_',datestr(now,'yyyymmddTHHMMSSFFF')];

            L1 = Link('d',(0.1564 + 0.1284) ,'a',0,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L2 = Link('d', (0.0054 + 0.0064) ,'a',0,'alpha',-pi/2,'offset', 0.0054,'qlim',[deg2rad(-128.9),deg2rad(128.9)]);
            L3 = Link('d', (0.2104 + 0.2104) ,'a',0,'alpha',-pi/2,'offset', 0.0064,'qlim',[deg2rad(-360),deg2rad(360)]);
            L4 = Link('d', (0.0064 + 0.0064) ,'a',0,'alpha',pi/2,'offset',0.0064,'qlim',[deg2rad(-147.8),deg2rad(147.8)]);
            L5 = Link('d', (0.2084 + 0.1059),'a',0,'alpha',pi/2,'offset', 0.0064,'qlim',[deg2rad(-360),deg2rad(360)]);
            L6 = Link('d',0,'a',0,'alpha',-pi/2,'offset', 0,'qlim',[deg2rad(-120.3),deg2rad(120.3)]);
            L7 = Link('d', (0.1059 + 0.0615) ,'a',0,'alpha',0,'offset', -pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);

            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
            %         self.model.base = transl(0,0,-0.07);
            %         q0 = [0 0 0 0 0 0 0]
            %         self.model.plotopt={'nojoints','noname','nowrist'};
            %         self.model.teach(q0,'scale',0.5)
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)
            for linkIndex = 1:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Kinova',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            %             if isempty(findobj(get(gca,'Children'),'Type','Light'))
            camlight
            %             end
            self.model.delay = 0;
            for linkIndex = 1:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        %%
        function KinovaLocation(self,transform)
            self.model.base = transform;
        end
        %%
        function Move(self, qStart, qEnd, steps, objectUpdateOption)
            % This function uses the trapezoidal method for trajectory
            % generation, which is more efficient than the quintic method.
            
            if objectUpdateOption == 2 
                xOffset = self.endEffector(1,4);
                yOffset = self.endEffector(2,4);
                zOffset = self.endEffector(3,4);
                delete(self.cup.mesh);
                enviro=Environment();
                self.cupWCoffee=enviro.CupWCoffee(xOffset,yOffset,zOffset);
                self.GetCupWCoffee(self.cupWCoffee);   
            end
            
            % Trapezoidal Trajectory Generation
            % Create the scalar function
            s = lspb(0,1,steps);   
            
            % Create memory allocation for joint state matrix variables
            self.qMatrix = nan(steps,7);
            
            % Generate interpolated joint angles
            for i = 1:steps
                self.qMatrix(i,:) = (1 - s(i)) * qStart + s(i) * qEnd;   
            end

%             qMatrix = jtraj(qStart, qEnd, steps);

            % Animate the trajectory movements of both end effectors
            for i = 1:1:steps
                self.model.animate(self.qMatrix(i,:));
                
                
                % Depending on the required action for an object, the end
                % effector of the object will be moved/updated
                % i.e.  case 0  : no parts move
                %       case 1  : move coffee cup
                %       case 2  : replace empty coffee cup with filled
                %                 coffee cup
                
                switch objectUpdateOption
                    case 0
                        % do nothing
                    case 1
                        % move cup with xyzrpy offsets
                        self.UpdateCup(i,-0.02,0,0.1,0,-pi/2,0);
                    case 2
                        % replace empty cup with filled
                        % write code to get base location of current empty
                        % cup, delete empty cup, insert filled cup using
                        % saved base location                     
                        
                        self.UpdateCupWCoffee(i,-0.02,0,0.1,0,-pi/2,0);

                end
                
                
                % Transformations of end effectors
                % calculated at each step
                t = self.model.fkine(self.qMatrix(i,:));
                
                drawnow();
            end
            
        end
        %%
        function GetCup(self, object)
            % This function is used to get the cup model from the
            % environment class
            self.cup = object;
        end 
        
        function GetCupWCoffee(self, object)
            % This function is used to get the cup model from the
            % environment class
            self.cupWCoffee = object;
        end 
        
        function GetCM(self, object)
            % This function is used to get the cup model from the
            % environment class
            self.coffeeMachine = object;
        end 
        
        %%
        
        function UpdateCup(self, i, xOffset, yOffset, zOffset, rollOffset, pitchOffset, yawOffset)           
            self.endEffector = self.model.fkine(self.qMatrix(i,:)) * ...
                          transl(xOffset,yOffset,zOffset) * ...
                          trotx(rollOffset) * ...
                          troty(pitchOffset) * ...
                          trotz(yawOffset);
            updatedPoints = (self.endEffector * [self.cup.verts,ones(self.cup.vCount,1)]')';
            self.cup.mesh.Vertices = updatedPoints(:,1:3);
            
        end
        
        function UpdateCupWCoffee(self, i, xOffset, yOffset, zOffset, rollOffset, pitchOffset, yawOffset)           
            self.endEffector = self.model.fkine(self.qMatrix(i,:)) * ...
                          transl(xOffset,yOffset,zOffset) * ...
                          trotx(rollOffset) * ...
                          troty(pitchOffset) * ...
                          trotz(yawOffset);
            updatedPoints = (self.endEffector * [self.cupWCoffee.verts,ones(self.cupWCoffee.vCount,1)]')';
            self.cupWCoffee.mesh.Vertices = updatedPoints(:,1:3);
            
        end
            
    end
        
        %%
        
end