classdef Assignment2Functions
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
        kinova;
        enviro;
    end
    
    methods
        function self=Assignment2Functions()
            self.kinova=Kinova();
            self.enviro=Environment();
            
        end
        
        function UpdateJoint(self,jointNumber,link)
            qNew=self.kinova.model.getpos();
            qNew(1,jointNumber)=deg2rad(angle);
            
            
        end
        function TeachMove(self,qInput)
            qStart=self.kinova.model.getpos();  %take current location
            qEnd=qInput;                        %translate to the next input location from teach
            qNew=self.kinova.model.ikcon(qEnd)
            
            steps= 100;
            s=lspb(0,1,steps);
            qMatrix= nan(steps,7);
            
            for i=1:steps
                qMatrix(i,:)=(1-s(i))*qStart + s(i)*qNew;
                self.kinova.model.animate(qMatrix(i,:));
            end
        end
        function currentPos=GetKinovaPos(self); 
            qCurrent=self.kinova.model.getpos(); %returns the joint coordinates set
            currentPos=self.kinova.model.fkine(qCurrent);  %get the current Kinova end effector pos
            currentPos=currentPos(1:3,4); %update the xyz
        end 
    end
end

