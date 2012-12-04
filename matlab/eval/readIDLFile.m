function anno = readIDLFile( annofile, fbase)

ids = readIDL(annofile);
anno = struct('fname', {}, 'rts', {}, 'binfo', {});

for i = 1:length(ids)
    fnum = sscanf(ids(i).img, 'left/image_%08d_0.png', 1) - fbase + 1;
    anno(fnum).fname = ids(i).img;
    anno(fnum).rts = correctBBox(ids(i).bb);
    anno(fnum).binfo = 1;
end

end
