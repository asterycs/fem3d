% Forked from Antti Hannukainen's simple_assembly.m

function [K,F] = step_assembly(mesh,bilin,linf,G,x)

Ndof = size(mesh.p,2);

[Ax,Ay,bx,by,detA,Px,Py] = affine_tri(mesh);

[X,W] = inttri(3);

gX{1} = bsxfun(@plus,Ax*X,bx);
gX{2} = bsxfun(@plus,Ay*X,by);

L{1} = 1-X(1,:)-X(2,:);
L{2} = X(1,:);
L{3} = X(2,:);

n_quadrature_points = size(X,2);
n_triangles = size(mesh.t,2);

dL{1} = [ -ones(1,n_quadrature_points); -ones(1,n_quadrature_points) ];
dL{2} = [  ones(1,n_quadrature_points); zeros(1,n_quadrature_points) ];
dL{3} = [ zeros(1,n_quadrature_points);  ones(1,n_quadrature_points) ];


% define arrays for matrix entries.
ffind = [];
ff = zeros(3,n_triangles);

kk = zeros(9,n_triangles);

mind = 1;

[U,dU,~] = eval2Dtri(mesh,x,X);
dUNorm2 = sum((dU{1}.^2+dU{2}.^2).^2*W.*abs(detA));

for i=1:3
    Li = repmat(L{i},n_triangles,1);
    
    dLi{1} = Px*dL{i};
    dLi{2} = Py*dL{i};  
    
    ff(i,:) = linf(Li,dLi,gX)*W.*abs(detA) - G(U,Li,dU,dLi,gX,dUNorm2)*W.*abs(detA);
    
    for j=1:3
        Lj = repmat(L{j},n_triangles,1);
        
        dLj{1} = Px*dL{j};
        dLj{2} = Py*dL{j};                       

        kk(mind,:) = bilin(Lj,Li,dLj,dLi,gX,dU,dUNorm2)*W.*abs(detA);
        mind = mind+1;
    end
end

iind = [1 1 1 2 2 2 3 3 3];
jind = [1 2 3 1 2 3 1 2 3];

K = sparse(mesh.t(iind,:),mesh.t(jind,:),kk,Ndof,Ndof);

F = sparse(mesh.t,ones(size(mesh.t)),ff,Ndof,1);

