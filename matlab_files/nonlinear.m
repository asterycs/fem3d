clear all; close all;

mesh = make_rect_mesh(3);

% Compute initial guess
bilin = @(U,V,dU,dV,gX)(dU{1}.*dV{1} + dU{2}.*dV{2} );
linf = @(V,dV,gX)(sin(pi*gX{1}).*sin(pi*gX{2}).*V);
[A,b] = simple_assembly(mesh,bilin,linf);

be = find( mesh.e2t(2,:) == 0);
bind = mesh.edges(:,be);
bind = unique(bind(:));
iind = setdiff(1:size(mesh.p,2),bind);

x = zeros( size(mesh.p,2),1);
x(iind) = A(iind,iind)\b(iind);

% Set up
gamma = 1;
maxiter = 1000;
epsnorms = zeros(maxiter,1);
energies = zeros(maxiter,1);
[~, ~, ~, ~, detA, ~, ~] = affine_tri(mesh);

% a = @(x) 1./(1+exp(-x));
% da = @(x) exp(-x)./(exp(-x)+1).^2;
a = @(x) 1+x.^2;
da = @(x) 2*x;
bilin_eps_wrapped = @(E,V,dE,dV,gX,dU,dUNorm2) bilin_eps(E,V,dE,dV,gX,dU,dUNorm2,a,da);
G = @(U,V,dU,dV,gX,dUNorm2) G_eps(U,V,dU,dV,gX,dUNorm2,a,da);

J = @(u,du,gX,detA) 1/2*sum(G(u,u,du,du,gX)*W.*abs(detA))-sum(linf(u,du,gX)*W.*abs(detA));

for i=1:maxiter
    
    [A,b] = step_assembly(mesh,bilin_eps_wrapped,linf,G,x);

    epsilon = zeros(size(mesh.p,2), 1);
    epsilon(iind) = A(iind,iind)\b(iind);
    
    % Evaluate ||epsilon|| on every step
    [epsilon_evaluated, depsilon_evaluated, epsilon_gx] = eval2Dtri(mesh, epsilon, [0.5 0.5 0; 0 0.5 0.5]);
    W = ones(3,1)*1/6;
    epsilon_l2norm = sqrt(sum(epsilon_evaluated.^2*W.*abs(detA)));
    epsnorms(i) = epsilon_l2norm;
    
    % And energy
    energy = J(epsilon_evaluated, depsilon_evaluated, epsilon_gx, detA);
    energies(end+1) = energy;
    
    x = x + gamma*epsilon;
end

[U,dU,gX] = eval2Dtri(mesh,x,[0 1 0 ; 0 0 1]);
patch(gX{1}',gX{2}',U',U');

figure;
plot(epsnorms);
grid on;
xlabel('Iteration (1)');
ylabel('||\epsilon|| (1)');