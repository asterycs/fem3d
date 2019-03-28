function Y = bilin_eps_soap(E,V,dE,dV,gX,dU,dUNorm2,a,da)

B = cell(4,1);
B{1} = a(dUNorm2) + 2*da(dUNorm2).*dU{1}.*dU{1};
B{2} =              2*da(dUNorm2).*dU{1}.*dU{2};
B{3} =              2*da(dUNorm2).*dU{2}.*dU{1};
B{4} = a(dUNorm2) + 2*da(dUNorm2).*dU{2}.*dU{2};

Y = (B{1}.*dE{1} + B{2}.*dE{2}) + (B{3}.*dE{1} + B{4}.*dE{2});

end

