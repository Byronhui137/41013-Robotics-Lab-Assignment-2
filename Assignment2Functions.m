classdef Assignment2Functions <handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
        kinova;
        enviro;
        cup;
        cupWCoffee;
        coffeeMachine;
        nineCups;
        cabinate;
        handlocation;
        handvCount;
        handVerts;
        handFaces;
        handFaceNormals;
        handMesh_h;
        updatePoints;
        f;
        v;
        data;
        
        currentHandPose
        
        lightcurtains;
        qMatrix;
        
        laserMatStart;
        laserMatEnd;
        
        estopTrigger;
        handCollisionTrigger;
        
        
    end
    
    methods
        function self=Assignment2Functions() %automatically calls these functions and variables when initiated
            
            self.kinova=Kinova();
            self.enviro=Environment();
            self.lightcurtains=self.LightCurtains();
            self.handlocation=transl(2.5,0,0);
            self.handCollisionTrigger = 0;
            self.estopTrigger=0;
            
            
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
            
            [self.f, self.v, self.data] = plyread('hand.ply','tri');
            
            self.handvCount = size(self.v,1);
            self.handFaces=self.f;
            self.handVerts=self.v;
            %midPoint = sum(v)/self.handvCount;
            handPos=self.handlocation;
            %self.verts = v - repmat(midPoint,self.handvCount, 1);
            vertexColours = [self.data.vertex.red, self.data.vertex.green, self.data.vertex.blue] / 255;
            
            self.handMesh_h = trisurf(self.f, self.handVerts(:,1), self.handVerts(:, 2), self.handVerts(:, 3)...
                ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            self.updatePoints=[handPos*[self.handVerts,ones(self.handvCount,1)]']';
            self.handMesh_h.Vertices=self.updatePoints(:,1:3);
            self.handFaceNormals = zeros(size(self.f,1),3);
            for faceIndex = 1:size(self.f,1)
                v1 = self.v(self.f(faceIndex,1)',:);
                v2 = self.v(self.f(faceIndex,2)',:);
                v3 = self.v(self.f(faceIndex,3)',:);
                self.handFaceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
            
            
        end
        
        
        %%
        function lightcurtains=LightCurtains(self)
            lightcurtains.x1=1.334;
            lightcurtains.y1=1.279;
            for i=1:14
                plot3([lightcurtains.x1 lightcurtains.x1],[1.279 1.279],[1.068 -0.2587],'Color','c'); %% Front
                plot3([lightcurtains.x1 lightcurtains.x1],[-1.221 -1.221],[1.068 -0.2587],'Color','c');%% Back
                lightcurtains.x1=lightcurtains.x1-0.2;
                
            end
            for i=1:13
                plot3([-1.286 -1.286],[lightcurtains.y1 lightcurtains.y1],[1.068 -0.2587],'Color','c'); %% Left
                plot3([1.334 1.334],[lightcurtains.y1 lightcurtains.y1],[1.068 -0.2587],'Color','c')%% Right (position of Hand)
                lightcurtains.y1=lightcurtains.y1-0.2;
                
                self.laserMatStart(i,:)= [1.334, lightcurtains.y1,1.068]; %getting array from the Right light curtains top
                self.laserMatEnd(i,:)=[1.334,lightcurtains.y1,-0.2587]; %getting array from the Right light curtains bottom
                
            end
            
        end
        
        %%
        
        
        %%
        function Move(self, qStart, qEnd, steps, objectUpdateOption)
            % This function uses the trapezoidal method for trajectory
            % generation, which is more efficient than the quintic method.
            
            % replace empty cup with filled
            % write code to get base location of current empty
            % cup, delete empty cup, insert filled cup using
            % saved base location
            if objectUpdateOption == 2
                xOffset = self.kinova.endEffector(1,4);
                yOffset = self.kinova.endEffector(2,4);
                zOffset = self.kinova.endEffector(3,4);
                delete(self.enviro.cup.mesh);
                self.enviro.cupWCoffee=self.enviro.CupWCoffee(xOffset,yOffset,zOffset);
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
                if self.estopTrigger==1
                    pause();
                end
                
                if self.handCollisionTrigger==1
                    self.currentHandPose=self.handlocation(1:3,4)';
%                     isObjectCollision = ObjectCollisionCheck(self.laserMatStart,self.laserMatEnd,self.handVerts,self.handFaces,self.handFaceNormals);
                    isObjectCollision = ObjectCollisionCheck(self.laserMatStart,self.laserMatEnd,self.handVerts,self.handFaces,self.handFaceNormals);
                    if isObjectCollision==1
                        pause();
                        while isObjectCollision==1
                            isObjectCollision = ObjectCollisionCheck(self.laserMatStart,self.laserMatEnd,self.handMesh_h.Vertices,self.handMesh_h.Faces,self.handMesh_h.FaceNormals);
                            self.currentHandPose=self.handlocation(1:3,4)';
                        end
                    end
                    
                end
                self.kinova.model.animate(self.qMatrix(i,:));
                
                
                % Depending on the required action for an object, the end
                % effector of the object will be moved/updated
                % i.e.  case 0  : no parts move
                %       case 1  : move coffee cup
                %       case 2  : replace empty coffee cup with filled
                %                 coffee cup
                
                if(objectUpdateOption == 1)
                    % move cup with xyzrpy offsets
                    self.UpdateCup(i,-0.02,0,0.1,0,-pi/2,0);
                    
                elseif(objectUpdateOption == 2 | objectUpdateOption == 3)
                    % move cupWCoffee with xyzrpy offsets
                    self.UpdateCupWCoffee(i,-0.02,0,0.1,0,-pi/2,0);
                end
                
                % Transformations of end effectors
                % calculated at each step
                t = self.kinova.model.fkine(self.qMatrix(i,:));
                
                drawnow();
            end
            
        end
        
        %%
        function UpdateCup(self, i, xOffset, yOffset, zOffset, rollOffset, pitchOffset, yawOffset)
            self.kinova.endEffector = self.kinova.model.fkine(self.qMatrix(i,:)) * ...
                transl(xOffset,yOffset,zOffset) * ...
                trotx(rollOffset) * ...
                troty(pitchOffset) * ...
                trotz(yawOffset);
            updatedPoints = (self.kinova.endEffector * [self.enviro.cup.verts,ones(self.enviro.cup.vCount,1)]')';
            self.enviro.cup.mesh.Vertices = updatedPoints(:,1:3);
            
        end
        
        function UpdateCupWCoffee(self, i, xOffset, yOffset, zOffset, rollOffset, pitchOffset, yawOffset)
            self.kinova.endEffector = self.kinova.model.fkine(self.qMatrix(i,:)) * ...
                transl(xOffset,yOffset,zOffset) * ...
                trotx(rollOffset) * ...
                troty(pitchOffset) * ...
                trotz(yawOffset);
            updatedPoints = (self.kinova.endEffector * [self.enviro.cupWCoffee.verts,ones(self.enviro.cupWCoffee.vCount,1)]')';
            self.enviro.cupWCoffee.mesh.Vertices = updatedPoints(:,1:3);
            
        end
        
        %%
        function Simulation(self)
            qStart = [0,0,0,pi/2,0,pi/2,pi];
            trCup = transl(0.6,0,-0.2) * trotx(-pi) * troty(deg2rad(80)) * trotz(-pi);
            trCoffeeMachine = transl(-0.115,0.68,-0.2) * trotx(1.5*pi) * troty(-2*pi) * trotz(-1.5*pi);
            
            qCup = self.kinova.model.ikcon(trCup);
            qCoffeeMachine = self.kinova.model.ikcon(trCoffeeMachine);
            qDropOff = qCoffeeMachine + [deg2rad(180) 0 0 0 0 0 0];
            qServe = qDropOff + [-deg2rad(14) deg2rad(6.5) deg2rad(21.6) deg2rad(41.4) -deg2rad(19) -deg2rad(36.1) -deg2rad(15)];
            
            % Make Coffee
            self.Move(qStart,qCup,60,0);
            self.Move(qCup,qCoffeeMachine,60,1);
            self.Move(qCoffeeMachine,qDropOff,60,2);
            self.Move(qDropOff,qServe,40,3);
            self.Move(qServe,qStart,60,0);
            
            
            
            
            
            
        end
        
    end
end

