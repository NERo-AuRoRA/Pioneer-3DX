function rSendControlSignals(car,p3dx)

% Simulation Mode
car.pSC.U = car.pSC.Ud;
car.sKinematicModel(p3dx); % MODIFY IT BY THE DYNAMIC MODEL

end


