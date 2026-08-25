function acfile = ensure_litewing_ac3d()
% ENSURE_LITEWING_AC3D Writes matlab/models/litewing.ac if missing.
root = fileparts(mfilename('fullpath'));
acfile = fullfile(root, '..', 'models', 'litewing.ac');
if exist(acfile, 'file')
    return;
end
fid = fopen(acfile, 'w');
if fid < 0
    error('Could not write %s', acfile);
end
fprintf(fid, 'AC3Db\n');
fprintf(fid, 'MATERIAL "body" rgb 0.22 0.28 0.35  amb 0.4 0.4 0.4  emis 0 0 0  spec 0.4 0.4 0.4  shi 20  trans 0\n');
fprintf(fid, 'MATERIAL "arm" rgb 0.75 0.78 0.85  amb 0.35 0.35 0.35  emis 0 0 0  spec 0.3 0.3 0.3  shi 10  trans 0\n');
fprintf(fid, 'MATERIAL "propb" rgb 0.20 0.70 1.00  amb 0.3 0.3 0.3  emis 0 0 0  spec 0.2 0.2 0.2  shi 10  trans 0.15\n');
fprintf(fid, 'MATERIAL "propo" rgb 1.00 0.50 0.20  amb 0.3 0.3 0.3  emis 0 0 0  spec 0.2 0.2 0.2  shi 10  trans 0.15\n');
fprintf(fid, 'OBJECT world\nkids 9\n');
write_box(fid, 'fuselage', 0, 0, 0, 0.08, 0.05, 0.02, 0);
s = 0.12;
write_box(fid, 'arm1',  s/2,  s/2, 0, 0.12, 0.012, 0.008, 1);
write_box(fid, 'arm2',  s/2, -s/2, 0, 0.12, 0.012, 0.008, 1);
write_box(fid, 'arm3', -s/2, -s/2, 0, 0.12, 0.012, 0.008, 1);
write_box(fid, 'arm4', -s/2,  s/2, 0, 0.12, 0.012, 0.008, 1);
write_disc(fid, 'prop1',  s,  s, 0.012, 0.05, 2);
write_disc(fid, 'prop2',  s, -s, 0.012, 0.05, 3);
write_disc(fid, 'prop3', -s, -s, 0.012, 0.05, 2);
write_disc(fid, 'prop4', -s,  s, 0.012, 0.05, 3);
fclose(fid);
end

function write_box(fid, name, cx, cy, cz, lx, ly, lz, mat)
hx = lx/2; hy = ly/2; hz = lz/2;
v = [
    cx-hx, cy-hy, cz-hz
    cx+hx, cy-hy, cz-hz
    cx+hx, cy+hy, cz-hz
    cx-hx, cy+hy, cz-hz
    cx-hx, cy-hy, cz+hz
    cx+hx, cy-hy, cz+hz
    cx+hx, cy+hy, cz+hz
    cx-hx, cy+hy, cz+hz
];
faces = [1 2 3 4; 5 8 7 6; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1];
fprintf(fid, 'OBJECT poly\nname "%s"\nloc 0 0 0\nnumvert 8\n', name);
for i = 1:8
    fprintf(fid, '%g %g %g\n', v(i,1), v(i,2), v(i,3));
end
fprintf(fid, 'numsurf 6\n');
for i = 1:6
    fprintf(fid, 'SURF 0x10\nmat %d\nrefs 4\n', mat);
    for k = 1:4
        fprintf(fid, '%d 0 0\n', faces(i,k)-1);
    end
end
fprintf(fid, 'kids 0\n');
end

function write_disc(fid, name, cx, cy, cz, r, mat)
n = 12;
fprintf(fid, 'OBJECT poly\nname "%s"\nloc 0 0 0\nnumvert %d\n', name, n);
th = linspace(0, 2*pi, n+1); th(end) = [];
for i = 1:n
    fprintf(fid, '%g %g %g\n', cx + r*cos(th(i)), cy + r*sin(th(i)), cz);
end
fprintf(fid, 'numsurf 1\nSURF 0x10\nmat %d\nrefs %d\n', mat, n);
for i = 0:n-1
    fprintf(fid, '%d 0 0\n', i);
end
fprintf(fid, 'kids 0\n');
end
