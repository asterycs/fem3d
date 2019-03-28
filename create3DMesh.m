
% Function that creates FEM meshes out of stl files.
% Currently only two options for the stl file
% cube and bracket

% createMesh(geometry,element_size) 
% Default values are provided
% No return value, writes two output files:
% .data containing the node idices for the boundary edges (N_boundary_edges x 2)
% .ttg containing p and t matrix for the mesh
function create3DMesh(varargin)
	% Set default values
	Defaults ={"cube.stl",0.5};
	Defaults(1:nargin) = varargin;

	filename = Defaults{1};
	elem_size = Defaults{2};
    
    if ~isfile(filename)
        warning('File not found. Exiting')
        return
    end
	
	% Load the stl file and create the mesh
	model = createpde;    
    importGeometry(model,filename);

	mesh = generateMesh(model,'Hmax',elem_size,'GeometricOrder','linear');
	[p,e,t] = meshToPet(mesh);
	
    % Scale p
    p = p/(max(max(p))/3);    

	% Remove uneeded information from the t matrix (only one material parameter)
	t = t(1:4,:);

	% Extract the boundary edges from the triangulation
	TR=triangulation(t',p');
	faces = freeBoundary(TR)';	
	boundary_edges=[faces(1,:)',faces(2,:)';faces(1,:)',faces(3,:)';faces(2,:)',faces(3,:)'];

	%boundary nodes (Not used at the moment, all the information is contained in the edges)
	boundary_nodes =sort(unique(faces));


	% Plotting functions for quick verification that the modell looks like it should
%	hold on
%	for i = 1:size(boundary_edges,1)
%		p1 = p(:,boundary_edges(i,1));
%		p2 = p(:,boundary_edges(i,2));
%		px1 = p1(1) ;
%		py1 = p1(2) ;
%		pz1 = p1(3) ;
%		px2 = p2(1) ;
%		py2 = p2(2) ;
%		pz2 = p2(3) ;
%		plot3([px1,px2],[py1,py2],[pz1,pz2])
%	end
	tetramesh(t',p');
%	scatter3(p(1,boundary_nodes),p(2,boundary_nodes),p(3,boundary_nodes))

    [~, name, ~] = fileparts(filename)

	% Write the output file
	writettg(p,t,boundary_nodes',name+".t3g");
	return
end
