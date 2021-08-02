function mCADcolor(p3dx,color)
% Modify drone color

% colocar for para pegar as duas peças ('open' e 'closed')

if nargin > 1
    
    for i = 1:2
        p3dx.pCAD.mtl{i}(10).Kd = color';  % 10  -> lateral plats
        %p3dx.pCAD.mtl{i}(8).Kd = color';   % 8   -> central plat

        for ii = 1:length(p3dx.pCAD.obj{i}.umat3)
            mtlnum = p3dx.pCAD.obj{i}.umat3(ii);
            for jj=1:length(p3dx.pCAD.mtl{i})
                if strcmp(p3dx.pCAD.mtl{i}(jj).name,p3dx.pCAD.obj{i}.usemtl(mtlnum-1))
                    break;
                end
            end
            fvcd3(ii,:) = p3dx.pCAD.mtl{i}(jj).Kd';
        end

        p3dx.pCAD.i3D{i}.FaceVertexCData  = fvcd3;

    end
end

end