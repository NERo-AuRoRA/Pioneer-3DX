% Discretização de sistemas
clearvars
close all
clc


% Rotina para buscar pasta raiz
addpath(genpath(pwd))

% Modelo cinemático do robô uniciclo
% \dot{x} = u*\cos{\psi}
% \dot{y} = u*\sin{\psi}
% \dot{\psi} = \omega
% 
% % Posição inicial
% x = 0;
% y = 0;
% psi = 0;

% Velocidades
% u = 0;
% w = 1;

P = Pioneer3DX;


P.mCADplot;
drawnow
axis([-2 2 -2 2])

tf = 10;
%ta = 0.1; % Tempo de amostragem

disp('ini')
pause

t = tic;
tc = tic;

k = 1;
while toc(t) < tf
    %if toc(tc) > ta
    if toc(tc) > P.pPar.Ts
        % Permissão para execução
        tc = tic;
        
        % Discretização do modelo
        % \dot{x(t)} = u*\cos{\psi}
        % (x(t) - x(t-dt))/dt = u(t)*cos(psi(t))
        % x(t) = dt*(u(t)*cos(psi(t))) + x(t-dt)
        
        % \dot{y} = u*\sin{\psi}
        % \dot{\psi} = \omega
        
        %xd = 1;
        %yd = 1;
        P.pPos.Xd([1 2]) = [1 1]';
        
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
        umax = 0.44;
        wmax = 0.35;
       
        
        rho = norm(P.pPos.Xtil([1 2]));
        theta = atan2(P.pPos.Xtil(2),P.pPos.Xtil(1));
        alpha = theta - P.pPos.X(6);
        
        
        % Sinais de controle
        u = umax*tanh(rho)*cos(alpha); % Velocidade Linear
        w = wmax*alpha + umax*tanh(rho)/rho*sin(alpha)*cos(alpha); % Velocidade Angular
        
        P.pSC.Ud = [u ; w];
                
%         % Modelo discreto do P3DX
%         x = x + ta*u*cos(psi);
%         y = y + ta*u*sin(psi);
%         psi = psi + ta*w;
%         
%         % Análise angular, pois o intervalo é $p \in (-\pi,\pi]$
%         if abs(psi) > pi
%             if psi > 0
%                 psi = -2*pi + psi;
%             else
%                 psi =  2*pi + psi;
%             end
%         end

        P.rSendControlSignals;
        P.rGetSensorData;
        
        % X(k,:) = [x y psi toc(t)];
        X(k,:) = [P.pPos.X([1 2 6])' toc(t)];
        k = k + 1;
        
        P.mCADplot;
        try
            delete(h)
        end
        hold on
        h = plot(X(:,1),X(:,2));
        hold off
        drawnow
        

        
    end
end


% k = 1;
% for t = 0:ta:tf
%     % Discretização do modelo
%     % \dot{x(t)} = u*\cos{\psi}
%     % (x(t) - x(t-dt))/dt = u(t)*cos(psi(t))
%     % x(t) = dt*(u(t)*cos(psi(t))) + x(t-dt)
%
%     % \dot{y} = u*\sin{\psi}
%     % \dot{\psi} = \omega
%
%     xd = 1;
%     yd = 1;
%     umax = 0.44;
%     wmax = 0.35;
%
%     rho = sqrt((xd-x)^2+(yd-y)^2);
%     theta = atan2(yd-y,xd-x);
%     alpha = theta - psi;
%
%     % Sinais de controle
%     u = umax*tanh(rho)*cos(alpha); % Velocidade Linear
%     w = wmax*alpha + umax*tanh(rho)/rho*sin(alpha)*cos(alpha); % Velocidade Angular
%
%     % Modelo discreto do P3DX
%     x = x + ta*u*cos(psi);
%     y = y + ta*u*sin(psi);
%     psi = psi + ta*w;
%
%     % Análise angular, pois o intervalo é $p \in (-\pi,\pi]$
%     if abs(psi) > pi
%         if psi > 0
%             psi = -2*pi + psi;
%         else
%             psi =  2*pi + psi;
%         end
%     end
%
%     X(k,:) = [x y psi t];
%     k = k + 1;
% end
disp('fim')

figure
subplot(2,2,1),plot(X(:,end),X(:,1))
xlabel('Tempos [s]','interpreter','latex')
ylabel('$$x$$ [m]','interpreter','latex')

subplot(2,2,2),plot(X(:,end),X(:,2))
xlabel('Tempos [s]','interpreter','latex')
ylabel('$$y$$ [m]','interpreter','latex')

subplot(2,2,3),plot(X(:,end),X(:,3))
xlabel('Tempos [s]','interpreter','latex')
ylabel('$$\psi$$ [rad]','interpreter','latex')

subplot(2,2,4),plot(X(:,1),X(:,2))
xlabel('$$x$$ [m]','interpreter','latex')
ylabel('$$y$$ [m]','interpreter','latex')
axis equal















