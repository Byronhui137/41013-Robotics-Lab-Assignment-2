classdef Assignment2Functions <handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Property1
        kinova;
        enviro;
        cup;
        coffeeMachine;
        nineCups;
        cabinate;
        handlocation;
        handvCount;
        handVerts;
        handMesh_h;
        updatePoints;
       
       
    end
    
    methods
        function self=Assignment2Functions()
            
            self.kinova=Kinova();
            self.enviro=Environment();
            self.handlocation=transl(2.5,0,0);
            
            
        end
        %%
        function UpdateJoint(self,jointNumber,link)
            qNew=self.kinova.model.getpos();        %take current robot joint array
            qNew(1,jointNumber)=deg2rad(link);     %joint number 1-7 make them into rads
            % add code for collision (maybe)
            self.kinova.model.animate(qNew);        %animate the new joint angle
            drawnow();
        end
        %%
        function TeachMove(self,qInput)
            qStart=self.kinova.model.getpos()  %take current location
            qEnd=transl(qInput)                        %translate to the next input location from teach
            qNew=self.kinova.model.ikcon(qEnd)
            
            steps= 100;
            s=lspb(0,1,steps);
            qMatrix= nan(steps,7);
            
            for i=1:steps
                qMatrix(i,:)=(1-s(i))*qStart + s(i)*qNew;
                self.kinova.model.animate(qMatrix(i,:));
                drawnow();
            end
        end
        %%
        function currentPos=GetKinovaPos(self);
            qCurrent=self.kinova.model.getpos(); %returns the joint coordinates set
            currentPos=self.kinova.model.fkine(qCurrent);  %get the current Kinova end effector pos
            currentPos=currentPos(1:3,4); %update the xyz
        end
        %%
        function HumanHand(self)
            
            [f, v, data] = plyread('hand.ply','tri');
            
            self.handvCount = size(v,1);
            self.handVerts=v;
            %midPoint = sum(v)/self.handvCount;
            handPos=self.handlocation;
            %self.verts = v - repmat(midPoint,self.handvCount, 1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            self.handMesh_h = trisurf(f, self.handVerts(:,1), self.handVerts(:, 2), self.handVerts(:, 3)...
                ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            updatePoints=[handPos*[self.handVerts,ones(self.handvCount,1)]']';
            self.handMesh_h.Vertices=updatePoints(:,1:3);
        end
          %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function testDeleteObject(self)
            xOffset = self.endEffector(1,4);
            yOffset = self.endEffector(2,4);
            zOffset = self.endEffector(3,4);
            delete(self.cup.mesh);
            enviro=Environment();
            cupWCoffee=enviro.CupWCoffee(xOffset,yOffset,zOffset);
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
    
end

