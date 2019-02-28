function writettg(vs,ts,bs,filename)

fid=fopen(filename,'w');

assert(size(vs,1) == 3)
assert(size(ts,1) == 4)
assert(size(bs,1) == 1)

vertices = size(vs,2);
tetras = size(ts,2);
boundary_nodes = size(bs,2);

fprintf(fid, ['d ' num2str(3) '\n']);
fprintf(fid, ['v ' num2str(vertices) '\n']);
fprintf(fid, ['t ' num2str(tetras) '\n']);
fprintf(fid, ['b ' num2str(boundary_nodes) '\n' ]);

for v=vs
    x = num2str(v(1));
    y = num2str(v(2));
    z = num2str(v(3));
    
    fprintf(fid, [x ' ' y ' ' z '\n']);
end

for t=ts
    v1 = num2str(t(1)-1);
    v2 = num2str(t(2)-1);
    v3 = num2str(t(3)-1);
    v4 = num2str(t(4)-1);
    
    fprintf(fid, [v1 ' ' v2 ' ' v3 ' ' v4 '\n']);
end

for b=bs
	idx=num2str(b-1);
	fprintf(fid,[idx '\n']);
end

fclose(fid);

end

