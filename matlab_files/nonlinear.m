clear all; close all;

mesh = make_rect_mesh(3);

% Compute initial guess
bilin = @(U,V,dU,dV,gX)(dU{1}.*dV{1} + dU{2}.*dV{2});
linf = @(V,dV,gX)(2^(3/2)*pi*sin(pi*gX{1}).*sin(pi*gX{2}).*V);
[A,b] = simple_assembly(mesh,bilin,linf);

be = find( mesh.e2t(2,:) == 0);
bind = mesh.edges(:,be);
bind = unique(bind(:));
iind = setdiff(1:size(mesh.p,2),bind);

x = zeros( size(mesh.p,2),1);
x(iind) = A(iind,iind)\b(iind);
x(iind) = x(iind) + 0.1*rand(size(iind,1),1);

% Set up
gamma = 1;
maxiter = 10000;
epsnorms = zeros(maxiter,1);
energies = zeros(maxiter,1);
[~, ~, ~, ~, detA, ~, ~] = affine_tri(mesh);

% a = @(x) 1./(1+exp(-x));
% da = @(x) exp(-x)./(exp(-x)+1).^2;
a = @(x) x.^2;
da = @(x) 2*x;
bilin_eps_wrapped = @(E,V,dE,dV,gX,dU,dUNorm2) bilin_eps(E,V,dE,dV,gX,dU,dUNorm2,a,da);
G = @(U,V,dU,dV,gX,dUNorm2) G_eps(U,V,dU,dV,gX,dUNorm2,a,da);

J = @(u,du,gX,dUNorm2,detA,W) 1/2*sum(G(u,u,du,du,gX,dUNorm2)*W.*abs(detA))-sum(linf(u,du,gX)*W.*abs(detA));
    
for i=1:maxiter
    
    [A,b] = step_assembly(mesh,bilin_eps_wrapped,linf,G,x);

    epsilon = zeros(size(mesh.p,2), 1);
    epsilon(iind) = A(iind,iind)\b(iind);
    
    % Evaluate ||epsilon|| on every step
    xip = [0.5 0.5 0; 0 0.5 0.5];
    W = ones(3,1)*1/6;
    [epsilon_eval, depsilon_eval, epsilon_gx] = eval2Dtri(mesh, epsilon, xip);
    epsilon_l2norm = sqrt(sum(epsilon_eval.^2*W.*abs(detA)));
    epsnorms(i) = epsilon_l2norm;
    
    % Evaluate energy energy
    dEpsilonNorm2 = sum((depsilon_eval{1}.^2+depsilon_eval{2}.^2).^2*W.*abs(detA));
    energy = J(epsilon_eval, depsilon_eval, epsilon_gx, dEpsilonNorm2, detA,W);
    energies(i) = energy;
    
    x = x + gamma*epsilon;
end

[U,dU,gX] = eval2Dtri(mesh,x,[0 1 0 ; 0 0 1]);
figure;
hold on;
patch(gX{1}',gX{2}',U',U','FaceColor','None');
[X,Y] = meshgrid(linspace(0,1,100));
surf(X,Y,2*sin(pi*X).*sin(pi*Y),'FaceColor','None');

figure;
plot(epsnorms);
grid on;
xlabel('Iteration (1)');
ylabel('||\epsilon|| (1)');

figure;
plot(energies);
grid on;
xlabel('Iteration (1)');
ylabel('J (1)');