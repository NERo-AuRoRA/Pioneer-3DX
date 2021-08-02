function iControlVariables(car)

% Pionner P3DX
% ========================================================================
% Robot pose
car.pPos.X    = zeros(12,1); % Current pose (point of control)
car.pPos.Xa   = zeros(12,1); % Past pose

car.pPos.Xc   = zeros(12,1); % Current pose (center of the robot)

car.pPos.Xd   = zeros(12,1); % Desired pose

car.pPos.Xr   = zeros(12,1); % Reference pose
car.pPos.Xra  = zeros(12,1); % Past reference pose

% First time derivative 
car.pPos.dX   = zeros(12,1); % Current pose
car.pPos.dXd  = zeros(12,1); % Desired pose
car.pPos.dXr  = zeros(12,1); % Reference pose

% Pose error
car.pPos.Xtil = car.pPos.Xd - car.pPos.X; 

% ========================================================================
% Sensor data
car.pPos.Xso  = zeros(12,1); % Initial sensor data
car.pPos.Xs   = zeros(12,1); % Current sensor data 
car.pPos.Xsa  = zeros(12,1); % Past sensor data
car.pPos.dXs  = zeros(12,1); % First time derivative of sensor data


% ========================================================================
% Signals of Control 
% Linear and Angular Velocity
car.pSC.U   = [0;0]; % Current
car.pSC.Ua  = [0;0]; % Past
car.pSC.Ud  = [0;0]; % Desired
car.pSC.Uda = [0;0]; % Past desired
car.pSC.Ur  = [0;0]; % Reference
car.pSC.Kinematics_control = 0;

% Linear and Angular Acceleration
car.pSC.dU   = [0;0]; % Current
car.pSC.dUd  = [0;0]; % Desired