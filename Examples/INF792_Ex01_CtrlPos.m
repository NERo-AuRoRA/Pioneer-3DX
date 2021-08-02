%%%% Pioneer P3-Dx: Controle de posição sem orientação final %%%%

clc, clear  
% close all

% Close all the open connections
try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz - Look for root directory
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2020';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% Solução para encontrar qualquer AuRoRa

PastaAtual = pwd;
% Encontra o AuRoRa
beginAuRoRa = strfind(PastaAtual,'\AuRoRA');
% Depois do \AuRoRA
endAuRoRaPath = strfind(PastaAtual(beginAuRoRa+6:end),'\');

% Se já não estiver no AuRoRa:
if ~isempty(endAuRoRaPath)
    % Encontra a pasta raiz do AuRoRa
    PastaRaiz = PastaAtual(1:beginAuRoRa+6+endAuRoRaPath(1)-1);
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
end


%% Classes initialization - Definindo o Robô
% Criando uma variável para representar o Robô

tic
P = Pioneer3DX;
toc 

P.pPar.a = 0;
P.pPar.alpha = 0*pi/3;

% Tempo de esperar para início do experimento/simulação
% clc;
fprintf('\nInício..............\n\n')
pause(1)

%% Definindo a Figura que irá rodar a simulação
% P.mPlotInit;
f1 = figure('Name','Simulação: Robótica Móvel (Pioneer P3-Dx)','NumberTitle','off');
% f1.Position = [435 2 930 682];
f1.Position = [1 2 930 682];
figure(f1);

ax = gca;
ax.FontSize = 12;
xlabel({'$$x$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$y$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
zlabel({'$$z$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
view(3)
view(45,30)
grid on
hold on
grid minor
light;
axis([-3 3 -3 3 0 2])
set(gca,'Box','on');

%% P.mCADplot();
t = tic;
P.mCADplot;
drawnow

disp([num2str(toc(t)) 's para 1º plot'])

P.mCADcolor([0.5 0.5 0.5]);

pause()

%% Initial Position
% Xo = input('Digite a posição inicial do robô ([x y z psi]): ');
Xo = [0 0 0 0];

P.rSetPose(Xo);         % define pose do robô

%% Variables initialization
% Xa = P.pPos.X(1:6);    % postura anterior
data = [];
Rastro.Xd = [];
Rastro.X = [];

% Temporização

tsim = 220;
tap = 0.100;     % taxa de atualização do pioneer
t = tic;
tc = tic;
tp = tic;

%% Simulation

% Parâmetros de controle
umax = 0.35;
wmax = 0.44;

while toc(t) < tsim
    
    if toc(tc) > tap
        % Inicio da realimentação
        tc = tic;
        
        % Data aquisition
        P.rGetSensorData;

        % Percorrendo os 4 quadrantes:
        if toc(t) > 200
            P.pPos.Xd(1:2) = [0 0]';
        elseif toc(t) > 150
            P.pPos.Xd(1:2) = [-2 -2]';
        elseif toc(t) > 100
            P.pPos.Xd(1:2) = [2 -2]';
        elseif toc(t) > 50
            P.pPos.Xd(1:2) = [-2 2]';
        else
            P.pPos.Xd(1:2) = [2 2]';
        end
        
        % -----------------------------------------------------
        % Erro Instantâneo:
        P.pPos.Xtil = (P.pPos.Xd - P.pPos.X); 

        % Modelagem cinemática em coordenada polares:
        P.pPos.rho = norm(P.pPos.Xtil(1:2)); 
        P.pPos.theta = atan2(P.pPos.Xtil(2),P.pPos.Xtil(1));
        P.pPos.alpha = P.pPos.theta - P.pPos.X(6);
        
        if abs(P.pPos.alpha) > pi
            if P.pPos.alpha > 0
                P.pPos.alpha = -2*pi + P.pPos.alpha;
            else 
                P.pPos.alpha  = 2*pi + P.pPos.alpha;
            end
        end

        % Control (Arquivo 'Controladores_Navegacao.PDF'):
        
        if P.pPos.rho > 0.003
        % o erro mínimo de posição adotado (\roh_min = 30mm), a fim de evitar
        % indeterminações no sistema de controle
            
            % Eq (2.5) -> u
            P.pSC.Ud(1) = umax*tanh(P.pPos.rho)*cos(P.pPos.alpha);
            % Eq (2.8) -> w
            P.pSC.Ud(2) = wmax*P.pPos.alpha + umax*(tanh(P.pPos.rho)/P.pPos.rho)*sin(P.pPos.alpha)*cos(P.pPos.alpha);
               
        else
            P.pSC.Ud(1:2) = 0;
        end
        
        % -----------------------------------------------------
        % Enviar sinais de controle para o robô
        P.rSendControlSignals;
                        
        % salva variáveis para plotar no gráfico
        Rastro.Xd = [Rastro.Xd; P.pPos.Xd(1:2)'];  % formação desejada
        Rastro.X  = [Rastro.X; P.pPos.X(1:2)'];    % formação real
                
        data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' P.pPos.rho P.pPos.alpha P.pPos.theta toc(t)];
        
    end
    % --------------------------------------------------------------- %
         
        
    %% Desenha o robô
    
    if toc(tp) > tap
        tp = tic;
        try
            delete(h);
            P.mCADdel;
        end
        hold on
        P.mCADplot;
        h(1) = plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'xk','MarkerSize',8.5,'LineWidth',1);
        h(2) = plot(Rastro.X(:,1),Rastro.X(:,2),'g','LineWidth',1);
        axis([-3 3 -3 3])
        grid on
        hold off
        drawnow
    end   
    
end

%%  Stop robot
% Zera velocidades do robô
P.pSC.Ud = [0 ; 0];
P.rSendControlSignals;
% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
pause(2)

%% Figuras
% Rota descrita pelo robô:
legend([h(1) h(2)],{'$\textbf{x}_d$','$\textbf{x}$'},'FontSize',20,'interpreter','latex','Position',[0.82 0.52 0.084 0.10])

%% Sinais de Controle (Fazer um lado do eixo y com velocidade linear e o outro com angular)
figure();
ax = gca;
ax.FontSize = 12;
yyaxis left
plot(data(:,end),data(:,25),'--k','LineWidth',1.5);
ax = gca;
ax.FontSize = 12;
ylabel({'Velocidade Linear [m/s]'},'FontSize',16,'FontWeight','bold','interpreter','latex');
yyaxis right 
plot(data(:,end),data(:,26),'-.b','LineWidth',1.5);
ylabel({'Velocidade Angular [rad/s]'},'FontSize',16,'FontWeight','bold','interpreter','latex');
xlabel({'$$t_{simu}$$ [s]'},'FontSize',16,'FontWeight','bold','interpreter','latex');

legend({'$u$','$\omega$'},'FontSize',18,'interpreter','latex','location','south')

hAx = gca;                       
set(hAx.YAxis,{'Color'},{'k'})   
grid on
axis equal
axis tight

%% Erros de posição:
figure();
plot(data(:,end),(data(:,1)-data(:,13)),'--k',data(:,end),(data(:,2)-data(:,14)),'-.k','LineWidth',1.5);
ax = gca;
ax.FontSize = 12;
xlabel({'$$t_{simu}$$ [s]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'Erro de Posi\c{c}{\~a}o [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
legend({'$\tilde{x}$','$\tilde{y}$'},'FontSize',20,'interpreter','latex','Position',[0.83 0.87 0.091 0.092])
grid on
axis tight

%%         
%data = [data; P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)];
            %  (1:12)    (13:24)    (25:26)   (27:28)
%% Erros de orientação:
figure();
plot(data(:,end),(data(:,6)-data(:,18)),'-k','LineWidth',1.5);
ax = gca;
ax.FontSize = 12;
xlabel({'$$t_{simu}$$ [s]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'Erro de Orienta\c{c}{\~a}o [rad]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
legend({'$\tilde{\beta}$'},'FontSize',20,'interpreter','latex','box','off')
grid on
axis tight

%% Gráfico para Rho e alpha:

figure();
subplot(311,'Position',[0.128214285714286 0.670693274231353 0.775 0.229306725768648])
plot(data(:,end),data(:,29),'-k','LineWidth',1.5);
ax = gca;
ax.FontSize = 12;
xticklabels({})
ylabel({'$$\rho$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
grid on
axis tight

subplot(312,'Position',[0.13 0.407142857142857 0.775 0.242857142857143])
plot(data(:,end),data(:,30),'-.k','LineWidth',1.5);
ax = gca;
ax.FontSize = 12;
xticklabels({})
ylabel({'$$\alpha$$ [rad]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
grid on
axis tight

subplot(313,'Position',[0.128214285714286 0.154761904761905 0.775 0.232878151260504])
plot(data(:,end),data(:,31),'--k','LineWidth',1.5);
ax = gca;
ax.FontSize = 12;
xlabel({'$$t_{simu}$$ [s]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$\beta$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
grid on
axis tight
