function rGetSensorData(car,p3dx)

% Store past position
car.pPos.Xa = car.pPos.X;


% Simulation       
% Robot center position

% car.pPos.X = [p3dx.pPos.Xc(1:3) - [0.25+0.385; 0; 0]; p3dx.pPos.Xc(4:12)];

% car.pPos.Xc = car.pPos.X;

% car.pPos.Xc([1 2 6]) = car.pPos.X([1 2 6]) - [car.pPar.a*cos(car.pPos.X(6)); car.pPar.a*sin(car.pPos.X(6)); 0];


end