function [U_l2, dU_l2] = compute_l2norm(mesh,x)

[~, ~, ~, ~, detA, ~, ~] = affine_tri(mesh);

xip = [0.5 0.5 0; 0 0.5 0.5];
W = ones(3,1)*1/6;

[U, dU, gX] = eval2Dtri(mesh, x, xip);
U_l2 = sqrt(sum(U.^2*W.*abs(detA)));

dU_l2 = sqrt(sum((dU{1}.^2+dU{2}.^2).^2*W.*abs(detA)));

end

