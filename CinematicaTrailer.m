

clc
clearvars
% close all

%% Rotina para buscar pasta raiz - Look for root directory
addpath(genpath(pwd))


P = Pioneer3DX;
trailer = trailerNERO;


%% Definindo a Figura que irá rodar a simulação

f1 = figure('Name','Simulação: Robótica Móvel (Pioneer P3-Dx)','NumberTitle','off');
f1.Position = [9 2 930 682];

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
% axis([-1.5 1.5 -1.5 1.5 0 1.5])
axis([-2.5 2.5 -2 2 0 1])
set(gca,'Box','on');

hold on
P.mCADplot;

trailer.mCADplot('open',P(1));
hold off

pause(6)

clc
disp('Início.......')



% Variáveis do Robô
X_robo = zeros(4,1); % posicao atual do robo (P.pPos.X)
X_trailer1 = zeros(4,1); % posicao atual do trailer1 (trailer.pPos.X)
Xd_trajetoria = zeros(4,1); % posicoes desejadas da trajetoria
Vd_trajetoria = zeros(4,1); % velocidades desejadas da trajetoria
Xtil = zeros(4,1); % Erros robo
Xtil_trailer1 = zeros(4,1); % Erros trailer 1
velocidade = zeros(4,1); % Velocidades controle
U_robo = zeros(4,1); % Comandos do Robô
U_trailer1 = zeros(4,1); % Comandos do trailer1
dX_robo = zeros(4,1); % Variacoes posicoes do robo entre interacoes
dX_trailer1 = zeros(4,1);% Variacoes posicoes do trailer1 entre interacoes


% Cinemática estendida do pioneer
alpha = pi;
sat_v = 0.35; %velocidade do pioneer para percorrer a trajetória
sat_w = 100*pi/180;
b = 0.25;

% Variaveis para plot corpo
raio = 0.15;
circ = 0:0.5:2*pi;

% Variaveis de Ganho
K_1 = diag([0.8 0.8]);
K_2 = diag([0.8 0.8]);


% Variáveis de histórico
t_hist = [];
X1_hist = [];
dX1_hist = [];
Xd1_hist = [];
dXd1_hist = [];
Xtil_hist = [];
X_robo_hist = [];
U1_hist = [];
Xd_trajetoria_hist = [];
dX_robo_hist = [];
dX_trailer1_hist = [];
X_trailer1_hist = [];
Xtil_trailer1_hist = [];
theta_1_hist = [];


%%% distâncias LR(L0)  e  LT (L1)
L0 = 0.275;  % Ok (robô - eixoRotação)
L1 = 0.34;   % Ok (eixoRotação - centroRodasTrailer)

%%% dimensoes do trailer fechado
%ret_lg = 0.375;
%ret_lp = 0.180;

%dimensoes do trailer aberto
ret_lg = 0.375 + 0.025;
ret_lp = 0.463;


% Variáveis de Tempo
t = tic; % Temporizador de experimento
t_sim = tic; % Temporizador de simulação
t_der = tic; % Temporizador de derivada
T_MAX = 60;
T_CONTROL = 1/10;
t_control = tic;


% Variaveis da trajetoria
rX = 2;           % [m]
rY = 1.5;           % [m]
T = T_MAX;             % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;


while toc(t) < T_MAX
    if toc(t_control) > T_CONTROL
        
        t_control = tic;
        t_atual = toc(t);
        
         %%% ---- Trajetoria desejada ------
         
         %%%% circulo
%         Xd_trajetoria([1 2]) = [-rX*sin(ww*t_atual + phase) - (L0+L1); -rY*cos(ww*t_atual + phase)];
%         Vd_trajetoria([1 2]) = [-ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];

        %%%% lemniscatta
        Xd_trajetoria([1 2]) = [rX*sin(ww*t_atual + phase); rY*sin(2*ww*t_atual + phase)];
        Vd_trajetoria([1 2]) = [ww*rX*cos(ww*t_atual + phase);2*ww*rY*cos(2*ww*t_atual + phase)];
        
        
        %%% ---- Calculo posicao no mundo do trailer 1 ------
        x_t1 = X_robo(1) - L0*cos(X_robo(4)) - (L1+b)*cos(X_trailer1(4));
        y_t1 = X_robo(2) - L0*sin(X_robo(4)) - (L1+b)*sin(X_trailer1(4));
        
        %----
        P.pPos.X(1:2) = [x_t1; y_t1];
        
        x_t1_c = X_robo(1) - L0*cos(X_robo(4)) - (L1)*cos(X_trailer1(4));
        y_t1_c = X_robo(2) - L0*sin(X_robo(4)) - (L1)*sin(X_trailer1(4));
        

        X_trailer1([1 2]) = ([x_t1 y_t1]);
        X_trailer1_centro = ([x_t1_c;y_t1_c]);  %%% plot
      
        %----
        traler.pPos.X(1:2) = [x_t1_c; y_t1_c];
        traler.pPos.X(6)   = X_trailer1(4);
        
        
        Xtil = Xd_trajetoria - X_robo;
        Xtil_trailer1 = Xd_trajetoria - X_trailer1;
                
        %%% Velocidades - lei de controle
        v = Vd_trajetoria([1 2]) + K_1*tanh(K_2*Xtil([1 2]));
        vx = v([1]);
        vy = v([2]);
        v_fi = (-vx/(b*cos(alpha)))*sin(X_robo([4])) +  (vy/(b*cos(alpha)))*cos(X_robo([4]));
        
        velocidade([1 2 4]) = [vx vy -v_fi];
                
        %% Cinemática
        % Matriz de cinemática estendida do trailer
        K_inv = [ cos(X_trailer1([4])), sin(X_trailer1([4])), b*sin(alpha); ...
                 -sin(X_trailer1([4])), cos(X_trailer1([4])), -b*cos(alpha); ...
                  0, 0, 1];
        
        %U_trailer1 = K_inv*velocidade([1 2 4]);
        %U_trailer1(2) = 0;
        
        theta_1 = X_robo(4)-X_trailer1(4);
        
        %v0_c = U_trailer1(1)*cos(theta_1) + (U_trailer1(3))*(L1)*sin(theta_1);
        %w0_c = U_trailer1(1)*sin(theta_1)/L0 - (U_trailer1(3))*(L1)*cos(theta_1)/L0;
                
        %U_robo([1 2 3]) = [v0_c 0 w0_c];
        
        %Teste-------------
        
        K_inv_robo = [ cos(X_robo([4])), sin(X_robo([4])), 0; ...
            -sin(X_robo([4])), cos(X_robo([4])), 0; ...
            0, 0, 1];
        
        
        
        U_robo = K_inv_robo*velocidade([1 2 4]);
        U_robo(2) = 0;
        
        %Saturação
        if abs(U_robo(1)) > sat_v
            U_robo(1) = sat_v*sign(U_robo(1));
        end
        
        if abs(U_robo(3)) > sat_w
            U_robo(3) = sat_w*sign(U_robo(3));
        end
        
        % Matriz de cinemática estendida do robo
        %K_inv_robo = [ cos(X_robo([4])), sin(X_robo([4])), 0; ...
        %    -sin(X_robo([4])), cos(X_robo([4])), 0; ...
        %    0, 0, 1];
        
        v1 = U_robo(1)*cos(theta_1) + (U_robo(3))*(L0)*sin(theta_1);
        w1 = U_robo(1)*sin(theta_1)/L1 - (U_robo(3))*(L0)*cos(theta_1)/L1;
        U_trailer1([1 2 3]) = [v1 0 w1];
        
        %% Simulação
        % Velocidade do robô na referencia do mundo
        dX_robo([1 2 4]) = K_inv_robo\U_robo([1 2 3]);
        dX_trailer1([1 2 4]) = K_inv\U_trailer1([1 2 3]);
        dX_trailer1(3) = 0;
        dX_robo(3) = 0;
        
        % Cálculo de posição na referência do mundo
        X_robo = X_robo + dX_robo*toc(t_sim);
        X_trailer1 = X_trailer1 + dX_trailer1*toc(t_sim);
        t_sim = tic;
              
        t_hist = [t_hist toc(t)];
        Xd_trajetoria_hist = [Xd_trajetoria_hist Xd_trajetoria(1:2)];
        dX_robo_hist = [dX_robo_hist dX_robo];
        dX_trailer1_hist = [dX_trailer1_hist dX_trailer1];
        X_robo_hist = [X_robo_hist X_robo];
        X_trailer1_hist = [X_trailer1_hist X_trailer1];
        Xtil_hist = [Xtil_hist Xtil];
        X_trailer1_hist = [X_trailer1_hist X_trailer1];
        Xtil_trailer1_hist = [Xtil_trailer1_hist Xtil_trailer1];
        theta_1_hist =[theta_1_hist theta_1];
        
        %%
        
        P.pPos.Xc(1:2) = [X_robo(1) X_robo(2)]';
        P.pPos.Xc(6) = X_robo(4);
        trailer.pPos.Xc(1:2) = [X_trailer1_centro(1) X_trailer1_centro(2)];
        trailer.pPos.Xc(6) = X_trailer1(4);
        
        
        Corpo1 = [raio*cos(circ);raio*sin(circ)] + X_robo(1:2);
        Corpo_frente = X_robo(1:2) + [(raio+0.025)*cos(X_robo(4));(raio+0.025)*sin(X_robo(4))];
        
        offset = 0.035;
        Corpo2 = [cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[-ret_lg/2+offset ret_lg/2+offset ret_lg/2+offset -ret_lg/2+offset -ret_lg/2+offset;-ret_lp/2 -ret_lp/2 ret_lp/2 ret_lp/2 -ret_lp/2] + X_trailer1_centro(1:2);
        haste_L0 = L0*[cos(X_robo(4)) -sin(X_robo(4));sin(X_robo(4)) cos(X_robo(4))]*[-1;0] + X_robo(1:2);
        haste_L1 = L1*[cos(X_trailer1(4)) -sin(X_trailer1(4));sin(X_trailer1(4)) cos(X_trailer1(4))]*[1;0] + X_trailer1_centro(1:2);
       
        
        
        try
            for i = 1:length(h)
                delete(h(i))
            end
        catch    
        end
        
        hold on
        P.mCADplot;
        if toc(t) > T_MAX/2
            trailer.mCADplot('open',P(1));
        else
            trailer.mCADplot('closed',P(1));
        end
        hold off
        
        hold on
        h(1) = plot(Xd_trajetoria(1),Xd_trajetoria(2),'b');
        
        grid on
        h(2) = plot(Xd_trajetoria_hist(1,:),Xd_trajetoria_hist(2,:),'b-');
%         plot(X_robo_hist(1,:),X_robo_hist(2,:),'k','LineWidth',2)
        h(3) = plot(Corpo1(1,:),Corpo1(2,:),'k','LineWidth',2);
        h(4) = plot(X_trailer1_hist(1,:),X_trailer1_hist(2,:),'r--','LineWidth',2);
        h(5) = plot(Corpo2(1,:),Corpo2(2,:),'k','LineWidth',2);
        h(6) = plot([X_robo(1) Corpo_frente(1)],[X_robo(2) Corpo_frente(2)],'k','LineWidth',2);
        h(7) = plot([X_robo(1) haste_L0(1)],[X_robo(2) haste_L0(2)],'k','LineWidth',2);
        h(8) = plot([haste_L0(1)],[haste_L0(2)],'ko');
        h(9) = plot([X_trailer1_centro(1) haste_L1(1)],[X_trailer1_centro(2) haste_L1(2)],'k','LineWidth',2);
%         axis([-3.5 1.5 -0.5 4.5])  %%% circulo raio 1,5

        axis([-2.5 2.5 -2 2 0 1])

%         axis([-3 1.5 -2.5 2.5])  %%% lemniscata raio 1,5
        hold off
                
%         hold on
%         grid on
%         plot(t_hist(1,:),Xtil_trailer1_hist(1,:))
%         plot(t_hist(1,:),Xtil_trailer1_hist(2,:))
%         legend('erro_x','erro_y')
%         axis([0 T_MAX -0.5 0.5])
%         hold off

%         hold on
%         grid on
%         plot(t_hist(1,:),theta_1_hist(1,:)*180/pi)
%         legend('\theta_1')
%         axis([0 T_MAX -1 50])
%         hold off

        drawnow   
        
        
    end
    
end
