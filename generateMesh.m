clear all;
close all;

model = createpde;
importGeometry(model,'cube.stl');
mesh = generateMesh(model,'Hmin',0.5,'GeometricOrder','linear');
[p,e,t] = meshToPet(mesh);

t = t(1:4,:);
writettg(p,t,'cube.ttg');
figure;
tetramesh(t',p');
