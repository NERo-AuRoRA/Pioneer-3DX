function iParameters(car)

car.pPar.Model = 'SemiTrailer'; % Carretinha model

% Dynamic Model Parameters 
car.pPar.g = 9.8;    % [kg.m/s^2] Gravitational acceleration

% [kg] 
car.pPar.m = 0.429; %0.442;  

% [m and rad] 
car.pPar.a = 0.0;  % point of control
car.pPar.alpha = 0; % angle of control

% [Identified Parameters]

car.pPar.theta = [0.5338; 0.2168; -0.0134; 0.9560; -0.0843; 1.0590];


%%% distâncias LR(L0)  e  LT (L1) -> Aberto
car.pPar.L0 = 0.3/1.6; % [m]
car.pPar.L1 = 0.455/2;


%%% dimensoes do trailer aberto  (Fazer alguma estrutura para alterar isso automaticamente!)
car.pPar.ret_lg = 0.375;
car.pPar.ret_lp = 0.463;

%%% dimensoes do trailer fechado
% car.pPar.ret_lg = 0.375;
% car.pPar.ret_lp = 0.180;




end