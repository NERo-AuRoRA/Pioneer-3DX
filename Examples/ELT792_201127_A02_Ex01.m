% Inicialização
% Controle de Posição sem Orientação

clearvars
close all
clc

% Rotina para buscar pasta raiz
addpath(genpath(pwd))

P = Pioneer3DX;
umax = 0.35;
wmax = 0.44;

% Cenário
figure(1)
P.mCADplot
axis([-2 3 -2 3 0 1])
view(40,75)
grid on
drawnow

%% Tempo de simulação
tmax = 30;
tc = tic;
tp = tic;
t = tic;

% Laço de simulação
while toc(t) < tmax
    % Estrutura de controle
    if toc(tc) > P.pPar.Ts
        tc = tic;
        P.rGetSensorData;
        
        % Tarefa de posicionamento
        P.pPos.Xd(1) = 2;
        P.pPos.Xd(2) = 2;
        
        % Aplicar controlador
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        rho = norm(P.pPos.Xtil(1:2));
        % rho = sqrt(P.pPos.Xtil(1)^2 + P.pPos.Xtil(2)^2); % Não elegante
        theta = atan2(P.pPos.Xtil(2),P.pPos.Xtil(1));
        alpha = theta - P.pPos.X(6);
        
        % Arquivo Controladores_Navegacao.PDF
        % Eq (2.5)
        P.pSC.Ud(1) = umax*tanh(rho)*cos(alpha);
        % Eq (2.8)
        P.pSC.Ud(2) = wmax*alpha+umax*tanh(rho)/rho*sin(alpha)*cos(alpha);
        
        % Enviar sinais de controle
        P.rSendControlSignals;               
        
    end   
    % Estrutura de plotagem
    if toc(tp) > P.pPar.Ts*2      
        tp = tic;
        tic
        P.mCADplot
        drawnow
        toc
    end
end

disp('oi')
