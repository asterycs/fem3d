function J = energy(a,L,mesh,x)
% Compute energy according to -1/2a(u,u)+L(u)

[~, ~, ~, ~, detA, ~, ~] = affine_tri(mesh);
xip = [0.5 0.5 0; 0 0.5 0.5];
W = ones(3,1)*1/6;

[U, dU, gX] = eval2Dtri(mesh, x, xip);

dUNorm2 = sum((dU{1}.^2+dU{2}.^2).^2*W.*abs(detA));

J = 1/2*sum(a(U,U,dU,dU,gX,dUNorm2)*W.*abs(detA))-sum(L(U,dU,gX)*W.*abs(detA));

end

