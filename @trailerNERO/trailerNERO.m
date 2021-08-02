classdef trailerNERO < handle
    % In a methods block, set the method attributes
    % and add the function signature
    properties
        
        % Properties or Parameters
        pCAD   % Pioneer 3DX 3D image
        pPar   % Parameters
        pID    % Identification
        
        % Control variables
        pPos   % Posture
        pSC    % Signals
        pFlag  % Flags
        
        % Navigation Data and Communication
        pData % Flight Data
        pCom  % Communication
        
    end
    
    methods
        function car = trailerNERO(ID)
            if nargin < 1
                ID = 1;
            end
            car.pID = ID;
                
            iParameters(car);
            iControlVariables(car);
            
            iFlags(car);
            mCADload(car);
            mCADmake(car);
      
        end
        
        % ==================================================
        iControlVariables(car);
        iParameters(car);
        iFlags(car);       
        
        % ==================================================
        % Pioneer 3DX 3D Image
        mCADload(car);
        mCADmake(car);
        mCADplot(car,char,p3dx);
        mCADdel(car);

        mCADplot2D(car,visible);
        mCADcolor(car,color);
        
        % ==================================================
        % Pose definition, based on kinematic or dynamic model
        sKinematicModel(car,p3dx);
        sInvKinematicModel(car,dXr);
        sDynamicModel(car);
        
        % ==================================================
        % Robot functions
        rSetPose(car,Xo);
        
        % Data request
        rGetSensorData(car,p3dx);
        
        % Command
        rSendControlSignals(car,p3dx);
        
        % ==================================================
      
    end
end