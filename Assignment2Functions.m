classdef Assignment2Functions <handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
        
        %%%%%%%
        
        kinova;
        enviro;
        cup;
        cupWCoffee;
        coffeeMachine;
        nineCups;
        cabinate;
        
        %%%%%%%%
        
        handlocation;
        handvCount;
        handVerts;
        handFaces;
        handFaceNormals;
        handMesh_h;
        updateHandPos;
        handCollisionTrigger;
        
        isObjectCollision;
        %%%%%%
        
        
        qMatrix;
        kinovaqMatrix
        %%%%%
        lightcurtains;
        laserMatStart;
        laserMatEnd;
        %%%%%%%%%
        estopTrigger;
        robotcollisionTrigger
        restTrigger;
        
        povs
        
        
    end
    
    methods
        function self=Assignment2Functions() %automatically calls these functions and variables when initiated
            
            self.kinova=Kinova();   %call robotic arm
            self.enviro=Environment(); %call Environment
            self.lightcurtains=self.LightCurtains(); %call light Curtains
            self.handlocation=transl(2.5,0,0); %set hand location 
            
            %Some triggers used to stop simulation
            self.handCollisionTrigger = 1;  
            self.estopTrigger=0;
            self.robotcollisionTrigger=0
            
            
            
        end
        %%
        %This function used in teach to get and update joint angles
        function UpdateJoint(self,jointNumber,link)
            qNew=self.kinova.model.getpos();        %take current robot joint array q=[0,0,0,0,0,0,0]
            qNew(1,jointNumber)=deg2rad(link);     %joint number 1-7 make them into rads           
            self.kinova.model.animate(qNew);        %animate the new joint angle
            drawnow();
        end
        %%
        %this function moves the robotic arm when you set xyz points and
        %hit run 
        function TeachMove(self,qInput)
            qStart=self.kinova.model.getpos()  %take current location
            qEnd=transl(qInput)                %translate to the next input location from teach
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
        %This function displays the hand in the figure
        function HumanHand(self)
            
            [f, v, data] = plyread('hand.ply','tri');
            
            self.handvCount = size(v,1);
            %taking Face and Vertex values to calculate facenormals
            self.handFaces=f;   
            self.handVerts=v;
            
            %midPoint = sum(v)/self.handvCount;
            handPos=self.handlocation; %get hand location (transl(2.5,0,0));
            %self.verts = v - repmat(midPoint,self.handvCount, 1);
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            self.handMesh_h = trisurf(f, self.handVerts(:,1), self.handVerts(:, 2), self.handVerts(:, 3)...
                ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            self.updateHandPos=[handPos*[self.handVerts,ones(self.handvCount,1)]']';
            self.handMesh_h.Vertices=self.updateHandPos(:,1:3);
            
            %getting hand FaceNormals
            self.handFaceNormals = zeros(size(f,1),3);
            for faceIndex = 1:size(f,1)
                v1 = v(f(faceIndex,1)',:);
                v2 = v(f(faceIndex,2)',:);
                v3 = v(f(faceIndex,3)',:);
                self.handFaceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
            
            
            
        end
        
        
        
        
        
        
        
        %%
        %this function draws the lines for light curtains
        function lightcurtains=LightCurtains(self)
            lightcurtains.x1=1.334;
            lightcurtains.y1=1.279;
            for i=1:14
                plot3([lightcurtains.x1 lightcurtains.x1],[1.279 1.279],[1.068 -0.2587],'Color','c'); %% Front xyz[Starting Point, Ending point]
                plot3([lightcurtains.x1 lightcurtains.x1],[-1.221 -1.221],[1.068 -0.2587],'Color','c');%% Back xyz[Starting Point, Ending point]
                lightcurtains.x1=lightcurtains.x1-0.2;
                
            end
            for i=1:13
                plot3([-1.286 -1.286],[lightcurtains.y1 lightcurtains.y1],[1.068 -0.2587],'Color','c'); %% Left xyz[Starting Point, Ending point]
                plot3([1.334 1.334],[lightcurtains.y1 lightcurtains.y1],[1.068 -0.2587],'Color','c')%% Right (position of Hand) xyz[Starting Point, Ending point]
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
                %when the Emergency Button is pressed, Trigger is ==1 then
                %pause the simulation
                
                
                if self.estopTrigger==1
                    pause();
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
        %%
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
        
        
        %%
        function RobotArmCollision(self)
            q = zeros(1,7);
            tr = zeros(4,4,self.kinova.model.n+1);
            tr(:,:,1) = self.kinova.model.base;
            L = self.kinova.model.links;
            for i = 1 : self.kinova.model.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            
            %             for i = 1 : size(tr,3)-1
            %                 for faceIndex = 1:size(self.enviro.machineF ,1)
            %                     vertOnPlane = self.enviro.machineV(self.enviro.machineF(faceIndex,1)',:);
            %                     [intersectP,self.check] = LinePlaneIntersection(self.enviro.machineFaceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
            %
            %                     if self.check == 1 && IsIntersectionPointInsideTriangle(intersectP,self.enviro.machineV(self.enviro.machineF(faceIndex,:)',:))
            %                         plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            %                         display('Intersection');
            %                         self.hitflag=1;
            %                     end
            %                 end
            %
            %             end
            
            steps = 100;
            s=lspb(0,1,steps);
            q1 = [0,0,0,0,0,0,0,];
            q2 = [-pi/2,pi/2,0,0,0,0,0];
            
            self.kinovaqMatrix = nan(steps,7);
            result = true(steps,1);
            
            for i=1:steps
                self.kinovaqMatrix(i,:)=(1-s(i))*q1 + s(i)*q2;
                result(i) = self.IsCollision(self.kinova,self.kinovaqMatrix(i,:),self.enviro.coffeeMachine.mesh.Faces,self.enviro.coffeeMachine.mesh.Vertices,self.enviro.coffeeMachine.mesh.FaceNormals,false);
                self.kinova.model.animate(self.kinovaqMatrix(i,:));
                drawnow();
            end
        end
        
        
        %% IsIntersectionPointInsideTriangle
        % Given a point which is known to be on the same plane as the triangle
        % determine if the point is
        % inside (result == 1) or
        % outside a triangle (result ==0 )
        function result = IsIntersectionPointInsideTriangle(self,intersectP,triangleVerts)
            
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
        % This is based upon Lab 5 exercises
        % Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
        % and triangle obstacles in the environment (faces,vertex,faceNormals)
        function result = IsCollision(self,robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
            if nargin < 6
                returnOnceFound = true;
            end
            result = false;
            
            
            for qIndex = 1:size(qMatrix,1)
                %     q = qMatrix(qIndex,:);
                
                % Get the transform of every joint (i.e. start and end of every link)
                tr = self.GetLinkPoses(qMatrix(qIndex,:), robot);
                
                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = self.LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                        if check == 1 && self.IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                            display('bang');
                            result = true;
                            display('Press Key to Continue');
                            pause();
                            
                            if returnOnceFound
                                return
                            end
                        end
                    end
                end
            end
        end
        
        
        
        %% GetLinkPoses
        function [ transforms ] = GetLinkPoses(self, q, kinova)
            %q - robot joint angles
            %robot -  seriallink robot model
            %transforms - list of transforms
            
            links = kinova.model.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = kinova.model.base;
            
            for i = 1:length(links)
                L = links(1,i);
                
                current_transform = transforms(:,:, i);
                
                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                
                transforms(:,:,i + 1) = current_transform;
            end
        end
        
        %%
        function [intersectionPoint,check] = LinePlaneIntersection(self,planeNormal,pointOnPlane,point1OnLine,point2OnLine)
            
            intersectionPoint = [0 0 0];
            u = point2OnLine - point1OnLine;
            w = point1OnLine - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);
            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end
            
            %compute the intersection parameter
            sI = N / D;
            intersectionPoint = point1OnLine + sI.*u;
            
            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end
        
        %%
        function HandReset(self)
            handPos=[1,0,0,2.5;0,1,0,0;0,0,1,0;0,0,0,1]; %tell the system where hand should be when reset
            self.updateHandPos=[handPos*[self.handVerts,ones(self.handvCount,1)]']'; 
            self.handMesh_h.Vertices=self.updateHandPos(:,1:3);
            drawnow();
        end
        %%
        % Make arm go back to original position q= [0,0,0,0,0,0,0]
        function ArmReset(self)
            qReset=zeros(1,7);  
            self.kinova.model.animate(qReset);
        end
        %%
        function Joystick(self)
            id = 2; % Note: may need to be changed if multiple joysticks present
            joy = vrjoystick(id);
            caps(joy) % display joystick information
            
            q = zeros(1,7);                  % Set initial robot configuration 'q'
            
%              HF = figure(1);         % Initialise figure to display robot
%             self.kinova.model.plot(q);          % Plot robot in initial configuration
            self.kinova.model.delay = 0.001;    % Set smaller delay when animating
%              set(HF,'Position',[0.1 0.1 0.8 0.8]);
            
            duration = 300;  % Set duration of the simulation (seconds)
            dt = 0.15;      % Set time step for simulation (seconds)
            
            n = 0;  % Initialise step count to zero
            tic;    % recording simulation start time
           
            while( toc < duration)
               
                n=n+1; % increment step count
                
                % read joystick
                [axes, buttons, self.povs] = read(joy);
                
                % -------------------------------------------------------------
                % YOUR CODE GOES HERE
                % 1 - turn joystick input into an end-effector velocity command
                Kv = 0.2; % linear velocity gain
                Kw = 1.0; % angular velocity gain
                
                vx = Kv*axes(1);
                vy = Kv*axes(2);
                vz = Kv*(buttons(5)-buttons(7));
                
                wx = Kw*axes(4);
                wy = Kw*axes(3);
                wz = Kw*(buttons(6)-buttons(8));
                
                dx = [vx;vy;vz;wx;wy;wz]; % combined velocity vector
                
                % 2 - use J inverse to calculate joint velocity
                J = self.kinova.model.jacob0(q);
                dq = pinv(J)*dx;
                
                % 3 - apply joint velocity to step robot joint angles
                q = q + dq'*dt;
                
                % -------------------------------------------------------------
                
                % Update plot
                self.kinova.model.animate(q);
                
                % wait until loop time elapsed
                if (toc > dt*n)
                    warning('Loop %i took too much time - consider increating dt',n);
                end
                while (toc < dt*n); % wait until loop time (dt) has elapsed
                end
            end
        end
        
        
    end
end

