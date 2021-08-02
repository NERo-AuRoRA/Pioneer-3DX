function mCADplot(obj,char,p3dx)
% Plot Pioneer 3D CAD model on its current position
% drone.pPos.Xc = [x y z psi theta phi dx dy dz dpsi dtheta dphi]^T

if nargin < 2
    char = 'open';
end

if obj.pCAD.flagCreated == 0
    mCADmake(obj)
    mCADplot(obj,char,p3dx)
    
else
    
    if strcmp(char,'closed')
        model = 2;
        obj.pCAD.i3D{1,2}.Visible = 'on';
        obj.pCAD.i3D{1,1}.Visible = 'off';
        obj.pCAD.i3D{1,1}.FaceAlpha = 0;
        
    elseif strcmp(char,'open')
        model = 1;
        obj.pCAD.i3D{1,1}.Visible = 'on';
        obj.pCAD.i3D{1,2}.Visible = 'off';
        obj.pCAD.i3D{1,2}.FaceAlpha = 0;
    end
    
    obj.pCAD.i3D{1,model}.FaceAlpha = 1;
    
    
    % Update reboque pose
    RotX = [1 0 0; 0 cos(p3dx.pPos.Xc(4)) -sin(p3dx.pPos.Xc(4)); 0 sin(p3dx.pPos.Xc(4)) cos(p3dx.pPos.Xc(4))];
    RotY = [cos(p3dx.pPos.Xc(5)) 0 sin(p3dx.pPos.Xc(5)); 0 1 0; -sin(p3dx.pPos.Xc(5)) 0 cos(p3dx.pPos.Xc(5))];
    RotZ = [cos(p3dx.pPos.Xc(6)) -sin(p3dx.pPos.Xc(6)) 0; sin(p3dx.pPos.Xc(6)) cos(p3dx.pPos.Xc(6)) 0; 0 0 1];
    
    Rot = RotZ*RotY*RotX;
    Hp3dx = [Rot p3dx.pPos.Xc(1:3); 0 0 0 1]; % Pioneer's Homogeneous Transformation Matrix
    
    Hr = Hp3dx*[eye(3) -[+0.021+0.2525; 0; 0]; 0 0 0 1];
    
    vertices = Hr*[obj.pCAD.obj{1,3}.v; ones(1,size(obj.pCAD.obj{1,3}.v,2))];
    obj.pCAD.i3D{1,3}.Vertices = vertices(1:3,:)';
    

    
    %%% Rotational matrix (trailer)
    % Rotação da peça
    
    RotX = [1 0 0; 0 cos(obj.pPos.Xc(4)) -sin(obj.pPos.Xc(4)); 0 sin(obj.pPos.Xc(4)) cos(obj.pPos.Xc(4))];
    RotY = [cos(obj.pPos.Xc(5)) 0 sin(obj.pPos.Xc(5)); 0 1 0; -sin(obj.pPos.Xc(5)) 0 cos(obj.pPos.Xc(5))];
    RotZ = [cos(obj.pPos.Xc(6)-p3dx.pPos.Xc(6)) -sin(obj.pPos.Xc(6)-p3dx.pPos.Xc(6)) 0;
            sin(obj.pPos.Xc(6)-p3dx.pPos.Xc(6))  cos(obj.pPos.Xc(6)-p3dx.pPos.Xc(6)) 0; 
                            0                                    0                   1];
    
    Rot = RotZ*RotY*RotX;
%     H = [Rot [obj.pPos.Xc(1:3)-[0.3611; 0; 0]]; 0 0 0 1];
%     H = [Rot -[0.3611; 0; 0]; 0 0 0 1];
    H = [Rot -[0; 0; 0]; 0 0 0 1];
    
    vertices = Hr*H*[obj.pCAD.obj{model}.v; ones(1,size(obj.pCAD.obj{model}.v,2))];
    %vertices = H*[obj.pCAD.obj{model}.v; ones(1,size(obj.pCAD.obj{model}.v,2))];
    obj.pCAD.i3D{1,model}.Vertices = vertices(1:3,:)';
    
   
    
end

end

% =========================================================================
function mCADmake(obj)

for i = 1:3
    
    hold on
    obj.pCAD.i3D{i} = patch('Vertices',obj.pCAD.obj{1,i}.v','Faces',obj.pCAD.obj{1,i}.f3');
    axis('image');
    hold off
    

    for ii = 1:length(obj.pCAD.obj{i}.umat3)
        mtlnum = obj.pCAD.obj{i}.umat3(ii);
        for jj=1:length(obj.pCAD.mtl{i})
            if strcmp(obj.pCAD.mtl{i}(jj).name,obj.pCAD.obj{i}.usemtl(mtlnum-1))
                break;
            end
        end
        fvcd3(ii,:) = obj.pCAD.mtl{i}(jj).Kd';
        %fvcd(ii,:) = rand(1,3);
    end

    obj.pCAD.i3D{i}.FaceVertexCData = fvcd3;
    obj.pCAD.i3D{i}.EdgeColor = 'none';
    
    obj.pCAD.i3D{i}.FaceAlpha = 1;
    obj.pCAD.i3D{i}.Visible = 'on';
    
    if i ~= 3
        obj.pCAD.i3D{i}.FaceAlpha = 0;
        obj.pCAD.i3D{i}.Visible = 'off';
        obj.pCAD.i3D{i}.FaceColor = 'flat';
    else
        obj.pCAD.i3D{i}.FaceColor = [0.698039 0.698039 0.698039];
    end
   
    % light;

end



obj.pCAD.flagCreated = 1;

end



% backUp

% if nargin < 2
%     char = 'open';
% end
% 
% if obj.pCAD.flagCreated == 0
%     mCADmake(obj)
%     mCADplot(obj,char,p3dx)
%     
% else
%     
%     if strcmp(char,'closed')
%         model = 2;
%         obj.pCAD.i3D(2).Visible = 'on';
%         obj.pCAD.i3D(1).Visible = 'off';
%         obj.pCAD.i3D(1).FaceAlpha = 0;
%         
%     elseif strcmp(char,'open')
%         model = 1;
%         obj.pCAD.i3D(1).Visible = 'on';
%         obj.pCAD.i3D(2).Visible = 'off';
%         obj.pCAD.i3D(2).FaceAlpha = 0;
%     end
%     
%     obj.pCAD.i3D(model).FaceAlpha = 1;
%     
% 
%     
%     % Update reboque pose
%     RotX = [1 0 0; 0 cos(p3dx.pPos.Xc(4)) -sin(p3dx.pPos.Xc(4)); 0 sin(p3dx.pPos.Xc(4)) cos(p3dx.pPos.Xc(4))];
%     RotY = [cos(p3dx.pPos.Xc(5)) 0 sin(p3dx.pPos.Xc(5)); 0 1 0; -sin(p3dx.pPos.Xc(5)) 0 cos(p3dx.pPos.Xc(5))];
%     RotZ = [cos(p3dx.pPos.Xc(6)) -sin(p3dx.pPos.Xc(6)) 0; sin(p3dx.pPos.Xc(6)) cos(p3dx.pPos.Xc(6)) 0; 0 0 1];
%     
%     Rot = RotZ*RotY*RotX;
%     Hp3dx = [Rot p3dx.pPos.Xc(1:3); 0 0 0 1]; % Pioneer's Homogeneous Transformation Matrix
%     
%     Hr = Hp3dx*[eye(3) -[+0.021+0.2525; 0; 0]; 0 0 0 1];
%     
%     vertices = Hr*[obj.pCAD.obj{3}.v; ones(1,size(obj.pCAD.obj{3}.v,2))];
%     obj.pCAD.i3D(3).Vertices = vertices(1:3,:)';
% 
%     
%     %%% Rotational matrix (trailer)
%     RotX = [1 0 0; 0 cos(obj.pPos.Xc(4)) -sin(obj.pPos.Xc(4)); 0 sin(obj.pPos.Xc(4)) cos(obj.pPos.Xc(4))];
%     RotY = [cos(obj.pPos.Xc(5)) 0 sin(obj.pPos.Xc(5)); 0 1 0; -sin(obj.pPos.Xc(5)) 0 cos(obj.pPos.Xc(5))];
%     RotZ = [cos(obj.pPos.Xc(6)) -sin(obj.pPos.Xc(6)) 0; sin(obj.pPos.Xc(6)) cos(obj.pPos.Xc(6)) 0; 0 0 1];
%     
%     Rot = RotZ*RotY*RotX;
%     H = [Rot [obj.pPos.Xc(1:3)-[0.3611; 0; 0]]; 0 0 0 1];
%     
%     vertices = Hr*H*[obj.pCAD.obj{model}.v; ones(1,size(obj.pCAD.obj{model}.v,2))];
%     %vertices = H*[obj.pCAD.obj{model}.v; ones(1,size(obj.pCAD.obj{model}.v,2))];
%     obj.pCAD.i3D(model).Vertices = vertices(1:3,:)';
%     
%    
%     
% end
% 
% end
% 
% % =========================================================================
% function mCADmake(obj)
% 
% for i = 1:3
%     
%     obj.pCAD.i3D(i) = patch('Vertices',obj.pCAD.obj{i}.v','Faces',obj.pCAD.obj{i}.f3');
% 
%     for ii = 1:length(obj.pCAD.obj{i}.umat3)
%         mtlnum = obj.pCAD.obj{i}.umat3(ii);
%         for jj=1:length(obj.pCAD.mtl{i})
%             if strcmp(obj.pCAD.mtl{i}(jj).name,obj.pCAD.obj{i}.usemtl(mtlnum-1))
%                 break;
%             end
%         end
%         fvcd3(ii,:) = obj.pCAD.mtl{i}(jj).Kd';
%         %fvcd(ii,:) = rand(1,3);
%     end
% 
%     obj.pCAD.i3D(i).FaceVertexCData = fvcd3;
%     obj.pCAD.i3D(i).FaceColor = 'flat';
%     obj.pCAD.i3D(i).EdgeColor = 'none';
%     obj.pCAD.i3D(i).FaceAlpha = 0;
%     obj.pCAD.i3D(i).Visible = 'off';
%     
%     if i == 3
%         obj.pCAD.i3D(i).FaceAlpha = 1;
%         obj.pCAD.i3D(i).Visible = 'on';
%     end
%     
%     % light;
%     
% end
%     