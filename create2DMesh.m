

function create2DMesh(elem_size)
	
	model = createpde;
    geometryFromEdges(model, @lshapeg);
	mesh = generateMesh(model,'Hmax',elem_size);
	[p,e,t] = meshToPet(mesh); 

	t = t(1:3,:);

	tr = triangulation(t',p');
	faces = freeBoundary(tr)';	

	boundary_nodes = sort(unique(faces));

	trimesh(t',p(1,:)', p(2,:)');
    
    %figure;
    %scatter(p(1,boundary_nodes),p(2,boundary_nodes))

    %[~, name, ~] = fileparts(filename)

	writettg(p,t,boundary_nodes',"lshape.t2g");
end