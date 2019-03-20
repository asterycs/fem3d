clear all; close all;

mesh = make_rect_mesh(3);

% Compute initial guess
bilin_laplacian = @(U,V,dU,dV,gX)(dU{1}.*dV{1} + dU{2}.*dV{2});
L = @(V,dV,gX)(sqrt(8)*pi*sin(pi*gX{1}).*sin(pi*gX{2}).*V);
[A,b] = simple_assembly(mesh,bilin_laplacian,L);

be = find(mesh.e2t(2,:) == 0);
bind = mesh.edges(:,be);
bind = unique(bind(:));
iind = setdiff(1:size(mesh.p,2),bind);

x = zeros( size(mesh.p,2),1);
x(iind) = A(iind,iind)\b(iind);

% ignore initial guess (for testing)
%x(iind) = ones(size(iind));

% Set up
gamma = 1;
maxiter = 1000;
epsnorms = zeros(maxiter,1);
energies = zeros(maxiter,1);
[~, ~, ~, ~, detA, ~, ~] = affine_tri(mesh);

% a = @(x) 1./(1+exp(-x));
% da = @(x) exp(-x)./(exp(-x)+1).^2;
a = @(x) x.^2;
da = @(x) 2*x;
bilin_eps_wrapped = @(E,V,dE,dV,gX,dU,dUNorm2) bilin_eps(E,V,dE,dV,gX,dU,dUNorm2,a,da);
G_wrapped = @(U,V,dU,dV,gX,dUNorm2) G(U,V,dU,dV,gX,dUNorm2,a,da);
    
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
    
    x = x + gamma*epsilon;
end

[U,dU,gX] = eval2Dtri(mesh,x,[0 1 0 ; 0 0 1]);
figure;
hold on;
patch(gX{1}',gX{2}',U',U','FaceColor','None');
[X,Y] = meshgrid(linspace(0,1,100));
surf(X,Y,sqrt(2)/pi*sin(pi*X).*sin(pi*Y),'FaceColor','None');
view(20,20);
movegui('northwest');

figure;
semilogy(epsnorms);
grid on;
xlabel('Iteration (1)');
ylabel('||\epsilon|| (1)');
movegui('north');

figure;
semilogx(energies);
grid on;
xlabel('Iteration (1)');
ylabel('J (1)');
movegui('northeast');