%%%% Pioneer P3-Dx: Rastreamento de Trajetória %%%%

clc, clear  
% close all

% Close all the open connections
try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz - Look for root directory
addpath(genpath(pwd))

%% Gerando o vídeo: 
% %-----------------------------------------------------------------------
% set(gca,'nextplot','replacechildren'); 
% v = VideoWriter('Traj_Circ.avi','Uncompressed AVI');
% % v.Quality = 95;
% v.FrameRate = 10; % 30 (default)
% open(v);
% %-----------------------------------------------------------------------

%% Classes initialization - Definindo o Robô
% Criando uma variável para representar o Robô
P = Pioneer3DX;
P.pPar.a = 0.05; % X Xc
P.pPar.alpha = pi/3;

carretinha(1) = trailerNERO;

% Tempo de esperar para início do experimento/simulação
clc;
fprintf('\nInício..............\n\n')
pause(1)

%% Definindo a Figura que irá rodar a simulação
% P.mPlotInit;
f1 = figure('Name','Simulação: Robótica Móvel (Pioneer P3-Dx)','NumberTitle','off');
% f1.Position = [435 2 930 682];
f1.Position = [1 2 930 682];
%     f1.Position = [1367 50 930 634]; % Quando uso segunda tela em Sete Lagoas!
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
axis([-1.5 1.5 -1.5 1.5 0 1.5])
set(gca,'Box','on');
axis tight

%% P.mCADplot();
t = tic;
P.mCADplot;
carretinha(1).mCADplot('closed',P);
drawnow

disp([num2str(toc(t)) 's para 1º plot'])

pause()

%% Teste para mudar a cor do Pioneer
% P(1) = Pioneer3DX;
% P(1).pPar.a = 0.1;
% P(2) = Pioneer3DX;
% P(2).pPar.a = 0.1;
% P(3) = Pioneer3DX;
% P(3).pPar.a = 0.1;
% P(4) = Pioneer3DX;
% P(4).pPar.a = 0.1;
% 
% Xo = [0 0 0 0];
% P.rSetPose(Xo);         % define pose do robô
% P(1).rSetPose(Xo + [0 1 0 0]);
% P(2).rSetPose(Xo + [0 -1 0 0]);
% P(3).rSetPose(Xo + [1 0 0 0]);
% P(4).rSetPose(Xo + [-1 0 0 0]);
% 
% P(1).mCADplot3D;
% P(2).mCADplot3D;
% P(3).mCADplot3D;
% P(4).mCADplot3D;
% 
% % Robot Appearance
% P.mCADcolor([0; 0.4470; 0.5410]);
% P(1).mCADcolor([0.4660 0.6740 0.1880]);
% P(2).mCADcolor([0.6350 0.0780 0.1840]);
% P(3).mCADcolor([0 0.4470 0.7410]);
% P(4).mCADcolor([0.9290 0.6940 0.1250]);
% 
% % P.pCAD.i3D.FaceAlpha = 0.5;
% 
% pause

%% Disposição dos dados
x_inc = 3;
y_inc = 3;
z_inc = 4;
t_display = tic;

tt_T = text(x_inc/1.5,y_inc/1.05,z_inc*1.05,['$$tempo\Rightarrow$$ ',num2str(toc(t_display),'%3.2f'),' [s]',],'FontWeight','bold','FontSize',10);
tt_T(1).Color = 'black'; %[0 0.4470 0.7410] -> azul claro; [0.4660 0.6740 0.1880] -> verde; [0.8500 0.3250 0.0980] -> laranja
tt_T(1).FontSize = 12;
tt_T(1).Interpreter = 'latex';

tt_X = text([x_inc x_inc x_inc]*0.65,[y_inc y_inc y_inc],1.05*[z_inc*0.9 z_inc*0.825 z_inc*0.75], {{['$$\textbf{x} =$$ ',num2str(P.pPos.X(1),'%3.2f'),' [m]']},{['$$\textbf{y} =$$ ',num2str(P.pPos.X(2),'%3.2f'),' [m]']},{['$$\textbf{z} =$$ ',num2str(P.pPos.X(3),'%3.2f'),' [mm]']}},'FontWeight','bold','FontSize',12);
tt_TH = text([x_inc x_inc x_inc]*0.65,[y_inc y_inc y_inc],1.05*[z_inc*0.675 z_inc*0.6 z_inc*0.525], {{['$   \phi =$ ',num2str(P.pPos.X(4)*180/pi,'%3.2f'), ' $[^\circ]$']},{['$ \theta =$ ',num2str(P.pPos.X(5)*180/pi,'%3.2f'), ' $[^\circ]$']},{['$   \psi =$ ', num2str((P.pPos.X(6))*180/pi,'%3.2f'), ' $[^\circ]$']}},'FontWeight','bold','FontSize',12);

tt_X(1).Interpreter = 'latex';
tt_X(2).Interpreter = 'latex';
tt_X(3).Interpreter = 'latex';
tt_TH(1).Interpreter = 'latex';
tt_TH(2).Interpreter = 'latex';
tt_TH(3).Interpreter = 'latex';

%% Tempo Real
pause(1)
%% Initial Position
% Xo = input('Digite a posição inicial do robô ([x y z psi]): ');
Xo = [0 0 0 0];

P.rSetPose(Xo);         % define pose do robô

%% Variables initialization
% Xa = P.pPos.X(1:6);    % postura anterior
data = [];
Rastro.Xd = [];
Rastro.X = [];

%% Trajectory variables
a = 1;         % distância em x
b = 1;         % distância em y

w = 0.1;

nvoltas = 2;
tsim = 2*pi*nvoltas/w;

%% Simulation

% Temporização
tap = 0.1;     % taxa de atualização do pioneer
t = tic;
tc = tic;
tp = tic;

while toc(t) < tsim
    
    if toc(tc) > tap
        
        tc = tic;
        
%         if toc(t) > 40
%             P.pPos.Xd(1:2) = [0; 2];
%             P.pPar.a = 0.5;
%             P.pPar.alpha = pi/4;
%             P.rSetPose
%         elseif toc(t) > 20
%             P.pPos.Xd(1:2) = [2; 1];
%             P.pPar.a = 0;
%             P.rSetPose
%         else
%             P.pPos.Xd(1:2) = [1; 2];
%         end
        
        % Trajectory
        % Lemniscata (8')
        ta = toc(t);
        P.pPos.Xd(1)  = a*sin(w*ta);       % posição x
        P.pPos.Xd(2)  = b*sin(2*w*ta);     % posição y
        P.pPos.Xd(7)  = a*w*cos(w*ta);     % velocidade em x
        P.pPos.Xd(8)  = 2*b*w*cos(2*w*ta); % velocidade em y
        
        % Data aquisition
        P.rGetSensorData;
        
        % salva variáveis para plotar no gráfico
        Rastro.Xd = [Rastro.Xd; P.pPos.Xd(1:2)'];  % formação desejada
        Rastro.X  = [Rastro.X; P.pPos.X(1:2)'];    % formação real
        
        % Control
        
        P = cKinematicController(P);
%         P = cDynamicController(P);
%         P = cKinematicControllerExtended(P);

        data = [data; P.pPos.Xd(1:2)' P.pPos.X(1:2)' P.pSC.Ud' P.pSC.U' toc(t)];
        
        % Send control to robot
        P.rSendControlSignals;
        

    % --------------------------------------------------------------- %
         
        
    % Desenha o robô
    
    if toc(tp) > 2*tap
    
%         tt_T.String = ['$tempo\Rightarrow $',num2str(toc(t_display),'%3.2f'),' [s]'];
% 
%         tt_X(1).String = {['$$\textbf{x} =$$ ',num2str(P.pPos.X(1),'%3.2f'),' [m]']};
%         tt_X(2).String = {['$$\textbf{y} =$$ ',num2str(P.pPos.X(2),'%3.2f'),' [m]']};
%         tt_X(3).String = {['$$\textbf{z} =$$ ',num2str(P.pPos.X(3),'%3.2f'),' [m]']};
% 
%         tt_TH(1).String = {['$   \phi =$ ',num2str(P.pPos.X(4)*180/pi,'%3.2f'), ' $[^\circ]$']};
%         tt_TH(2).String = {['$   \theta =$ ',num2str(P.pPos.X(5)*180/pi,'%3.2f'), ' $[^\circ]$']};
%         tt_TH(3).String = {['$   \psi =$ ',num2str(P.pPos.X(6)*180/pi,'%3.2f'), ' $[^\circ]$']};
        
        tp = tic;
        try
            delete(h);
            P.mCADdel;
        catch
        end
        hold on
        %P.mCADplot(1,'r')
        %P.mCADplot2D('r')
        P.mCADplot;
        carretinha(1).mCADplot('closed',P);
        
        h(1) = plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'k');
        h(2) = plot(Rastro.X(:,1),Rastro.X(:,2),'g');
        axis([-1.5 1.5 -1.5 1.5 0 1.5])

        grid on
        hold off
        drawnow
    end   
    
    end
%     % Pega o frame do vídeo
%     frame = getframe(gca);
%     writeVideo(v,frame);

end
%%  Stop robot
% Zera velocidades do robô
P.pSC.Ud = [0 ; 0];
P.rSendControlSignals;
% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

pause(2)

axis equal
set(gca,'Box','on')
% close(v); % Fecha o vídeo

figure
subplot(211)
plot(data(:,end),data(:,1),'b',data(:,end),data(:,3),'r')
subplot(212)
plot(data(:,end),data(:,2),'b',data(:,end),data(:,4),'r')


%% 

clc, clear  
% close all


% Definindo a Figura que irá rodar a simulação
% P.mPlotInit;
f1 = figure('Name','Simulação: Robótica Móvel (Pioneer P3-Dx)','NumberTitle','off');
% f1.Position = [435 2 930 682];
% f1.Position = [1 2 930 682];
f1.Position = [1370 -48 751 682];


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
% axis([-3 3 -3 3 0 2])
set(gca,'Box','on');
axis tight




p3dx(1) = Pioneer3DX;    % Pioneer (Robô)
carretinha(1) = trailerNERO;



p3dx(1).mCADplot;

% Xc = [x y z phi theta psi dx dy dz ...];
% carretinha(1).rSetPose([p3dx(1).pPos.Xc(1:3)'-[0.25+0.385 0 0] p3dx(1).pPos.Xc(6)] + [0 0 0 0]);

carretinha(1).mCADplot('closed',p3dx(1));
% carretinha(1).mCADplot('open');
% carretinha(1).mCADplot;

% carretinha(1).mCADcolor([0.4660 0.6740 0.1880]);

% p3dx(2).mCADcolor([0.6350 0.0780 0.1840]);
% p3dx(3).mCADcolor([0 0.4470 0.7410]);

% light;


