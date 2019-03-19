function Y = G_eps(U,V,dU,dV,gX,dUNorm2,a,da)

Y = (a(dUNorm2).*(dU{1}.*dV{1} + dU{2}.*dV{2}));

end

