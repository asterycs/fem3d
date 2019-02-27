
% Function that creates FEM meshes out of stl files.
% Currently only two options for the stl file
% cube and bracket

% createMesh(geometry,element_size) 
% Default values are provided
% No return value, writes two output files:
% .data containing the node idices for the boundry edges (N_boundry_edges x 2)
% .ttg containing p and t matrix for the mesh
function createMesh(varargin)
	% Set default values
	Defaults ={"cube",0.5};
	Defaults(1:nargin) = varargin;

	name = Defaults{1};
	elem_size=Defaults{2};
	
	% Load the stl file and create the mesh
	model = createpde;
	switch name
		case 'cube'
			importGeometry(model,'cube.stl');
		case 'bracket'
			importGeometry(model,'BracketTwoHoles.stl');
		otherwise
			warning('Unsuported geometry option. Exiting')
			return
	end
	mesh = generateMesh(model,'Hmin',elem_size,'GeometricOrder','linear');
	[p,e,t] = meshToPet(mesh);
	
	% Remove uneeded information from the t matrix (only one material parameter)
	t = t(1:4,:);

	% Extract the boundry edges from the triangulation
	TR=triangulation(t',p');
	faces = freeBoundary(TR)';	
	boundry_edges=[faces(1,:)',faces(2,:)';faces(1,:)',faces(3,:)';faces(2,:)',faces(3,:)'];

	% Boundry nodes (Not used at the moment, all the information is contained in the edges)
	%boundry_nodes =unique(faces);


	% Plotting functions for quick verification that the modell looks like it should
%	hold on
%	for i = 1:size(boundry_edges,1)
%		p1 = p(:,boundry_edges(i,1));
%		p2 = p(:,boundry_edges(i,2));
%		px1 = p1(1) ;
%		py1 = p1(2) ;
%		pz1 = p1(3) ;
%		px2 = p2(1) ;
%		py2 = p2(2) ;
%		pz2 = p2(3) ;
%		plot3([px1,px2],[py1,py2],[pz1,pz2])
%	end
%	tetramesh(t',p');

	% Write the output files
	writettg(p,t,name+".ttg");
	csvwrite(name+"_edges.data",boundry_edges)
	return
end
