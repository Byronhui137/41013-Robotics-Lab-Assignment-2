classdef Assignment2Functions <handle
    % This is the primary class that integrates all sections of the
    % simulation together, which is then called within their respective GUI
    % functions (i.e. coffee making, collision detections, teach)
    
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
        
        %% Class Initialise
        % This function is automatically called upon first
        % initialisation of an instance of this class, where key
        % functions are called to create the environment and initiate
        % variables
        
        function self=Assignment2Functions() 
            
            self.kinova=Kinova();                                           % call Robot (Kinova) Class
            self.enviro=Environment();                                      % call Environment Class
            self.lightcurtains=self.LightCurtains();                        % call light curtains
            self.handlocation=transl(2.5,0,0);                              % set hand location 
            
            % Some triggers used to stop simulation
            self.handCollisionTrigger = 1;  
            self.estopTrigger=0;
            self.robotcollisionTrigger=0;           
            
        end
        
        %% COFFEE MAKING
        
        %% Simulation
        % This is the main function that is called in the CoffeeMaking
        % GUI. It's main objective is to move to the initial empty
        % coffee cup, then pick it up to carry it to the coffee
        % machine. From here, the coffee is made where it will then be
        % dropped off at the drop off location to the customer
        
        function Simulation(self)
            
            % Set the transforms of the key positions needed to make coffee
            qStart              = [0,0,0,pi/2,0,pi/2,pi];
            trStart             = self.kinova.model.fkine(qStart);
            trCup               = transl(0.6,0,-0.2) * troty(pi/2);
            trCoffeeMachine1    = transl(0.6,0,-0.1) * troty(pi/2);
            trCoffeeMachine2    = transl(-0.115,0.68,-0.2) * trotx(-pi/2) * troty(0) * trotz(pi/2);
            trDropOff           = transl(0.17,-0.9,-0.22) * trotx(pi/2) * trotz(-pi/2);
            trEnd               = transl(0.3,0,0.6) * trotz(pi/2);
            
            % Create assumed/guessed poses when attempting to reach each
            % position (optimisation)
            %specifies a starting pose instead of zeroes
            guess1 = [0,0,0,pi/2,0,pi/2,pi]; 
            guess2 = [pi,-pi/2,0,pi,0,pi/2,pi];
            guess3 = [2*pi,-pi/2,-pi/2,pi/2,pi,0,0];
            guess4 = [2*pi,0,0,pi/2,2*pi,pi/2,-pi];
                        
            % Start making coffee
            % The move function takes the desired transform and the assumed
            % pose, as well as a speed and desired object update outcome
            display('Begin Coffee Making...');
            self.Move(trStart,guess1,20,0);
            self.Move(trCup,guess1,30,0);
            
            display('Retrieved Cup');
            self.Move(trCoffeeMachine1,guess1,5,1);
            self.Move(trCoffeeMachine2,guess2,20,1);
            
            display('Making Coffee...');
            self.Move(trDropOff,guess3,60,2);
            
            display('Coffee Delivered. Enjoy!');
            self.Move(trEnd,guess4,30,0);
            
        end
        
        %% Move
        % This function aims to navigate from one point to another

        % trEnd   transform of the end pose
        % qGuess  an initial guess for a pose
        % steps   to determine speed of the movement
        % objectUpdateOption   moves different objects based on
        % options:
        %           0  arm movement only, no objects move
        %           1  empty cup moves with the arm
        %           2  cup with coffee moves with the arm
        
        function Move(self, trEnd, qGuess, steps, objectUpdateOption)
            
            % Replaces empty cup with coffee filled cup
            if objectUpdateOption == 2
                xOffset = self.kinova.endEffector(1,4);                     % extracts x location of empty cup
                yOffset = self.kinova.endEffector(2,4);                     % extracts y location of empty cup
                zOffset = self.kinova.endEffector(3,4);                     % extracts z location of empty cup
                delete(self.enviro.cup.mesh);                               % delete empty cup
                pause(3);                                                   % act as  simulated coffee making 
                self.enviro.cupWCoffee=self.enviro.CupWCoffee(xOffset,yOffset,zOffset); % insert coffee-filled cup
            end
            
            s = lspb(0,1,steps);                                            % trapezoidal trajectory scalar
            self.qMatrix = zeros(steps,7);                                  % initialise array for joint angles
            
            % Generate trajectory start and end poses
            qStart = self.kinova.model.getpos();                            % use current position as start pose
            
            qEnd = self.kinova.model.ikcon(trEnd,qGuess);                   % use inverse kinematics to use end 
                                                                            % transform and guessed pose to optimise end pose accuracy
            
            for i = 1:steps
                
                % Stop making coffee if e-stop is triggered
                if (self.estopTrigger==1)
                    pause();
                end
                
                % Interpolate joint angles using the trapezoidal method
                self.qMatrix(i,:) = (1 - s(i)) * qStart + s(i) * qEnd;
                
                % Animate
                self.kinova.model.animate(self.qMatrix(i,:));
                
                if(objectUpdateOption == 1)
                    % Empty cup moves with the robot's end effector with 
                    % xyz rpy offsets for realistic simulation
                    self.UpdateCup(i,-0.02,0,0.1,0,-pi/2,0);
                    
                elseif(objectUpdateOption == 2)
                    % Coffee-filled cup moves with the robot's end effector with 
                    % xyz rpy offsets for realistic simulation
                    self.UpdateCupWCoffee(i,-0.02,0,0.1,0,-pi/2,0);
                end
                
                drawnow();
                
            end            
            
        end
        
        %% UpdateCup
        % This function updates the location of the empty cup with 
        % xyz rpy offsets
        
        function UpdateCup(self, i, xOffset, yOffset, zOffset, rollOffset, pitchOffset, yawOffset)
            
            self.kinova.endEffector = self.kinova.model.fkine(self.qMatrix(i,:)) * ...
                transl(xOffset,yOffset,zOffset) * ...
                trotx(rollOffset) * ...
                troty(pitchOffset) * ...
                trotz(yawOffset);
            updatedPoints = (self.kinova.endEffector * [self.enviro.cup.verts,ones(self.enviro.cup.vCount,1)]')';
            self.enviro.cup.mesh.Vertices = updatedPoints(:,1:3);
            
        end
        
        %% UpdateCupWCoffee
        % This function updates the location of the coffee-filled cup with 
        % xyz rpy offsets
        
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
        %%%%%%%%%%%%%%%%%%%%%%%%%%%TEACH FUNCTION%%%%%%%%%%%%%%%%%%%%%%%%
        %% GetKinovaPos
        % This function is used in the Teach Sliders to get the
        % kinova's current position
        
        function currentPos=GetKinovaPos(self)
            
            qCurrent=self.kinova.model.getpos();                            % returns the joint coordinates set
            currentPos=self.kinova.model.fkine(qCurrent);                   % get the current Kinova end effector pos
            currentPos=currentPos(1:3,4);                                   % update the xyz
            
        end
        %% UpdateJoint
        % This function is used to get and update joint angles of the
        % Kinova robot
        % Used in teach with sliders to get UpdateJoint
        
        function TeachUpdateJoint(self,jointNumber,link)
            
            qJoint=self.kinova.model.getpos();                                % take current robot joint array q=[0,0,0,0,0,0,0]
            qJoint(1,jointNumber)=deg2rad(link);                              % joint number 1-7 make them into rads           
            self.kinova.model.animate(qJoint);                                % animate the new joint angle
            drawnow();
            
        end
        
        %% TeachMove
        % This function moves the robotic arm when you set xyz points 
        % and hit run
         
        function TeachMove(self,qInput)
            
            qStart=self.kinova.model.getpos();                              % take current location
            qEnd=transl(qInput);                                            % translate to the next input location from teach
            qNew=self.kinova.model.ikcon(qEnd);
            
            steps= 100;
            s=lspb(0,1,steps);
            self.qMatrix= nan(steps,7);
            
            for i=1:steps
                self.qMatrix(i,:)=(1-s(i))*qStart + s(i)*qNew;
                self.kinova.model.animate(self.qMatrix(i,:));
                drawnow();
            end
            
        end
        

        
        %% %%%%%%%%%%%%%%%%%%%%%%OTHER OBJECTS%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% HumanHand
        % This function plots the hand into the figure
            
        function HumanHand(self)
            
            [f, v, data] = plyread('hand.ply','tri');
            
            self.handvCount = size(v,1);
            
            % taking Face and Vertex values to calculate facenormals
            self.handFaces=f;   
            self.handVerts=v;
            
            % get hand location (transl(2.5,0,0));
            handPos=self.handlocation;                                      
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            self.handMesh_h = trisurf(f, self.handVerts(:,1), self.handVerts(:, 2), self.handVerts(:, 3)...
                ,'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            self.updateHandPos=[handPos*[self.handVerts,ones(self.handvCount,1)]']';
            self.handMesh_h.Vertices=self.updateHandPos(:,1:3);
            
            % getting hand FaceNormals
            self.handFaceNormals = zeros(size(f,1),3);
            for faceIndex = 1:size(f,1)
                v1 = v(f(faceIndex,1)',:);
                v2 = v(f(faceIndex,2)',:);
                v3 = v(f(faceIndex,3)',:);
                self.handFaceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
            
        end
        
        %% LightCurtains
        % This function draws each line for the light curtains
        
        function lightcurtains=LightCurtains(self)
            
            lightcurtains.x1=1.334;
            lightcurtains.y1=1.279;
            
            for i=1:14
                plot3([lightcurtains.x1 lightcurtains.x1],[1.279 1.279],[1.068 -0.2587],'Color','c');       %% Front xyz[Starting Point, Ending point]
                plot3([lightcurtains.x1 lightcurtains.x1],[-1.221 -1.221],[1.068 -0.2587],'Color','c');     %% Back xyz[Starting Point, Ending point]
                lightcurtains.x1=lightcurtains.x1-0.2; % making gaps between the lightcurtains
                
            end
            
            for i=1:13
                plot3([-1.286 -1.286],[lightcurtains.y1 lightcurtains.y1],[1.068 -0.2587],'Color','c');     %% Left xyz[Starting Point, Ending point]
                plot3([1.334 1.334],[lightcurtains.y1 lightcurtains.y1],[1.068 -0.2587],'Color','c')        %% Right (position of Hand) xyz[Starting Point, Ending point]
                lightcurtains.y1=lightcurtains.y1-0.2;
                
                self.laserMatStart(i,:)= [1.334, lightcurtains.y1,1.068];                                   % getting array from the Right light curtains top
                self.laserMatEnd(i,:)=[1.334,lightcurtains.y1,-0.2587];                                     % getting array from the Right light curtains bottom 
                
            end
            
        end
                
        
        %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% COLLISION DETECTION%%%%%%%%%%%%%%%%
        
        %% HandReset
        % Resets position of the hand
        
        function HandReset(self)
            handPos=[1,0,0,2.5;0,1,0,0;0,0,1,0;0,0,0,1];                    %tell the system where hand should be when reset
            self.updateHandPos=[handPos*[self.handVerts,ones(self.handvCount,1)]']'; 
            self.handMesh_h.Vertices=self.updateHandPos(:,1:3);
            drawnow();
        end
        
        %% Resets position of the robot arm
        % Make arm go back to original position q= [0,0,0,0,0,0,0]
        
        function ArmReset(self)
            qReset=zeros(1,7);  
            self.kinova.model.animate(qReset);
        end
        
        %% RobotArmCollision
        % This function attempts to move from one pose to another
        % though simulates a collision where the robot's trajectory
        % detects this and retreats from the safety symbol
        
        function RobotArmCollision(self)
            
            q = zeros(1,7);                                                 % initialise array for joint angles
            tr = zeros(4,4,self.kinova.model.n+1);                          % initialise array for transform
            tr(:,:,1) = self.kinova.model.base;                             % set transform with robot's base position
            L = self.kinova.model.links;                                    % kinova links
            
            % Iterates through all links of the robot model and creates 
            % a homogenous transform
            for i = 1 : self.kinova.model.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
            
            steps = 70;                                                     % sets speed
            s=lspb(0,1,steps);                                              % trapezoidal trajectory scalar
            q1 = [0,0,0,0,0,0,0,];                                          % start pose
            q2 = [-pi/2,pi/2,0,0,0,0,0];                                    % end pose
            
            self.kinovaqMatrix = nan(steps,7);                              % initialise matrix
            result = true(steps,1);
            
            % Interpolate joint angles using the trapezoidal method
            % Check for collisions in each step
            for i=1:steps
                self.kinovaqMatrix(i,:)=(1-s(i))*q1 + s(i)*q2;
                result(i) = self.IsCollision(self.kinova,self.kinovaqMatrix(i,:), ...
                                              self.enviro.coffeeMachine.mesh.Faces, ...
                                              self.enviro.coffeeMachine.mesh.Vertices, ...
                                              self.enviro.coffeeMachine.mesh.FaceNormals, ...
                                              false);
            end
            
            % Go through each step and animate movement
            % If collision previously detected, retreat from collision
            % point
            for i=1:steps
                if (result(i) == 1)
                    firstCollision = i-2;                                   % first collision point index, -2 used for offset
                    poseNum = (firstCollision-2);                           % end position point index
                    steps = round(steps/4);
                    s=lspb(0,1,steps);
                    q1 = self.kinovaqMatrix(firstCollision,:);
                    q2 = self.kinovaqMatrix(poseNum,:);
                    retreatMatrix = nan(steps,7);
                    
                    % Create retreat matrix then animate
                    for j=1:steps
                        retreatMatrix(j,:) = (1-s(j))*q1 + s(j)*q2;
                    end
                    for k=1:steps
                        self.kinova.model.animate(retreatMatrix(k,:));
                        drawnow();
                    end
                    break
                else
                    % Normal animation going towards coffee machine
                    self.kinova.model.animate(self.kinovaqMatrix(i,:)); %inital movement
                    drawnow();
                end
            end
            
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
                            result = true;
                            
                            if returnOnceFound
                                return
                            end
                        end
                    end
                end
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
                        
        %% GetLinkPoses
        function [ transforms ] = GetLinkPoses(self, q, kinova)
            % q - robot joint angles
            % robot -  seriallink robot model
            % transforms - list of transforms
            
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
        %%%%%%%%%%%%%%%%%%%%% ADDITIONAL HARDWARE%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% Joystick
        % Used to control end effector of the robot
        
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

