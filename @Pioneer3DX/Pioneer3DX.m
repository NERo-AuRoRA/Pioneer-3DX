classdef Pioneer3DX < handle
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
        function p3dx = Pioneer3DX(ID)
            if nargin < 1
                ID = 1;
            end
            p3dx.pID = ID;
                
            iControlVariables(p3dx);
            iParameters(p3dx);
            iFlags(p3dx);
            mCADload(p3dx);
            mCADmake(p3dx);
      
        end
        
        % ==================================================
        iControlVariables(p3dx);
        iParameters(p3dx);
        iFlags(p3dx);       
        
        % ==================================================
        % Pioneer 3DX 3D Image
        mCADload(p3dx);
        mCADmake(p3dx);
        mCADplot(p3dx);
        mCADdel(p3dx);

        mCADplot2D(p3dx,visible);
        mCADcolor(p3dx,color);
        
        % ==================================================
        % Pose definition, based on kinematic or dynamic model
        sKinematicModel(p3dx);
        sInvKinematicModel(p3dx,dXr);
        sDynamicModel(p3dx);
        
        % ==================================================
        % Robot functions
        rSetPose(p3dx,Xo);
        
        % Data request
        rGetSensorData(p3dx);
        
        % Command
        rSendControlSignals(p3dx);
        
        % ==================================================
      
    end
end