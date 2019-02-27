clear all;
close all;

model = createpde;
importGeometry(model,'cube.stl');
mesh = generateMesh(model,'Hmin',0.5,'GeometricOrder','linear');
[p,e,t] = meshToPet(mesh);

t = t(1:4,:);
TR=triangulation(t',p');

faces = freeBoundary(TR)';
boundry_nodes =unique(faces);
boundry_edges=[faces(1,:)',faces(2,:)';faces(1,:)',faces(3,:)';faces(2,:)',faces(3,:)'];
hold on
for i = 1:size(boundry_edges,1)
	p1 = p(:,boundry_edges(i,1));
	p2 = p(:,boundry_edges(i,2));
	px1 = p1(1) ;
	py1 = p1(2) ;
	pz1 = p1(3) ;
	px2 = p2(1) ;
	py2 = p2(2) ;
	pz2 = p2(3) ;
	plot3([px1,px2],[py1,py2],[pz1,pz2])
end
tetramesh(t',p');
writettg(p,t,'cube.ttg');
csvwrite('edges.data',boundry_edges)
