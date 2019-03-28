function writettg(vs,ts,bs,filename)

if (size(vs,1) == 3)
    assert(size(vs,1) == 3)
    assert(size(ts,1) == 4)
    assert(size(bs,1) == 1)
    dims = 3;
else
    assert(size(vs,1) == 2)
    assert(size(ts,1) == 3)
    assert(size(bs,1) == 1)
    dims = 2;
end

vertices = size(vs,2);
elems = size(ts,2);
boundary_nodes = size(bs,2);

fid=fopen(filename,'w');

fprintf(fid, ['d ' num2str(dims) '\n']);
fprintf(fid, ['v ' num2str(vertices) '\n']);
fprintf(fid, ['e ' num2str(elems) '\n']);
fprintf(fid, ['b ' num2str(boundary_nodes) '\n' ]);

for v=vs
    x = num2str(v(1));
    y = num2str(v(2));
    
    if (dims == 3)
        z = num2str(v(3));
        fprintf(fid, [x ' ' y ' ' z '\n']);
    else
        fprintf(fid, [x ' ' y '\n']);
    end
end

for t=ts
    v1 = num2str(t(1)-1);
    v2 = num2str(t(2)-1);
    v3 = num2str(t(3)-1);
    
    if (dims == 3)
        v4 = num2str(t(4)-1);
        fprintf(fid, [v1 ' ' v2 ' ' v3 ' ' v4 '\n']);
    else
        fprintf(fid, [v1 ' ' v2 ' ' v3 '\n']);
    end
end

for b=bs
	idx=num2str(b-1);
	fprintf(fid,[idx '\n']);
end

fclose(fid);

end

