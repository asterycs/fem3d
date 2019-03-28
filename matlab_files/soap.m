clear all; close all;

model = createpde;
geometryFromEdges(model,@circleg);
mMesh = generateMesh(model,'Hmax',0.1,'GeometricOrder','linear');
[p,e,t] = meshToPet(mMesh);
t = t(1:3,:);

% Häck-ish...
mesh.t = t;
mesh.p = p;
trimesh(mesh.t',mesh.p(1,:)', mesh.p(2,:)')

% Find the boundary
Xb = unique([e(1,:) e(2,:)]);
tmp = false(size(p,2),1);
tmp(Xb) = 1;
iind = ~tmp;

x = zeros( size(mesh.p,2),1);

% Some funky boundary condition
x(~iind) = sin(4*mesh.p(1,~iind));


% Set up
gamma = 0.001;
maxiter = 10000;
min_epsilon = 1e-10;
epsnorms = zeros(maxiter,1);
energies = zeros(maxiter,1);
[~, ~, ~, ~, detA, ~, ~] = affine_tri(mesh);

a = @(x) 1./sqrt(1+x);
da = @(x) -1/2*(1+x).^(-3/2);
bilin_eps_wrapped = @(E,V,dE,dV,gX,dU,dUNorm2) bilin_eps_soap(E,V,dE,dV,gX,dU,dUNorm2,a,da);
G_wrapped = @(U,V,dU,dV,gX,dUNorm2) G(U,V,dU,dV,gX,dUNorm2,a,da);
L = @(V,dV,gX)(0.*V);
    
for i=1:maxiter
    
    [A,b] = step_assembly(mesh,bilin_eps_wrapped,L,G_wrapped,x);

    epsilon = zeros(size(mesh.p,2), 1);
    epsilon(iind) = A(iind,iind)\b(iind);
    
    % Evaluate ||epsilon|| on every step
    [E_l2,~] = compute_l2norm(mesh, epsilon);
    epsnorms(i) = E_l2;
    
    % Evaluate energy
    J = energy(G_wrapped,L,mesh,x);
    energies(i) = J;
    
    if E_l2 < min_epsilon
        figure(5);
        close(5);
        break;
    end
    
    figure(5);
    clf;
    [U,dU,gX] = eval2Dtri(mesh,x,[0 1 0 ; 0 0 1]);
    patch(gX{1}',gX{2}',U',U','FaceColor','None','EdgeColor','r');
    view(20,20);
    drawnow;
    
    x = x + gamma*epsilon;
end

[U,dU,gX] = eval2Dtri(mesh,x,[0 1 0 ; 0 0 1]);
figure;
hold on;
patch(gX{1}',gX{2}',U',U','FaceColor','None','EdgeColor','r');
view(20,20);
movegui('northwest');

figure;
semilogy(epsnorms);
grid on;
xlabel('Iteration (1)');
ylabel('||\epsilon|| (1)');
movegui('north');

figure;
semilogy(energies);
grid on;
xlabel('Iteration (1)');
ylabel('J (1)');
movegui('northeast');