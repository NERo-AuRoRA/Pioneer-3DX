function sKinematicModel(car,p3dx)

% Cinemática extendida
% Determine the robot pose, based on the control signal
%      +-----------+      .
% U -> | Kinematic |  ->  X
%      | Model     |      
%      +-----------+      
%

K = [ cos(car.pPos.X(6)) sin(car.pPos.X(6))  p3dx.pPar.b*sin(car.pPar.alpha); ...
     -sin(car.pPos.X(6)) cos(car.pPos.X(6)) -p3dx.pPar.b*cos(car.pPar.alpha); ...
              0                  0                         1                   ];

% Current position
car.pPos.X([1 2 6]) = car.pPos.X([1 2 6]) + K*[car.pSC.U(1); 0; car.pSC.U(2)]*p3dx.pPar.Ts;

% first-time derivative of the current position
car.pPos.X([7 8 12]) = K*[car.pSC.U(1); 0; car.pSC.U(2)];

% Angle limitation per quadrant
for ii = 4:6
    if abs(car.pPos.X(ii)) > pi
        if car.pPos.X(ii) < 0
            car.pPos.X(ii) = car.pPos.X(ii) + 2*pi;
        else
            car.pPos.X(ii) = car.pPos.X(ii) - 2*pi;
        end
    end
end

% Pose of the robot's center
car.pPos.Xc([1 2 6]) = car.pPos.X([1 2 6]) - ...
    [cos(car.pPos.X(6)) -sin(car.pPos.X(6)) 0; sin(car.pPos.X(6)) cos(car.pPos.X(6)) 0; 0 0 1]*...
    [car.pPar.a*cos(car.pPar.alpha); car.pPar.a*sin(car.pPar.alpha); 0];
