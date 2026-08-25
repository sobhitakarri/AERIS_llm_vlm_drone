function litewing_aero_viz(p, eul)
% LITEWING_AERO_VIZ Aerospace Toolbox 3D animation (Aero.Animation).
% Camera/axes are locked to the 3 m indoor workspace so the craft is visible.
persistent anim idx use_aero

if nargin < 2
    return;
end

if isempty(use_aero)
    use_aero = (license('test', 'Aerospace_Toolbox') ~= 0) && exist('Aero.Animation', 'class');
end

if ~use_aero
    litewing_simulink_viz(p, eul);
    return;
end

if isempty(anim)
    try
        anim = Aero.Animation;
        anim.FramesPerSecond = 20;
        acfile = local_litewing_ac3d();
        idx = anim.createBody(acfile, 'Ac3d');
        anim.show();
        fit_indoor_aero_camera(anim, p);
        fprintf('[Aero] Indoor 3D window opened (axes ±1.6 m).\n');
    catch ME
        warning('Aero.Animation failed (%s). Using MATLAB mesh.', ME.message);
        use_aero = false;
        anim = [];
        litewing_simulink_viz(p, eul);
        return;
    end
end

try
    move(anim.Bodies{idx}, p(:).', eul(:).');
catch
    try
        anim.Bodies{idx}.move(p(:).', eul(:).');
    catch
        litewing_simulink_viz(p, eul);
        return;
    end
end
fit_indoor_aero_camera(anim, p);
end

function fit_indoor_aero_camera(anim, p)
try
    anim.Camera.Position = [2.1 1.7 1.4];
    anim.Camera.Target = [p(1) p(2) max(p(3), 0.2)];
    anim.Camera.ViewAngle = 38;
catch
end
figs = findall(0, 'Type', 'figure');
for i = 1:numel(figs)
    nm = lower(char(string(get(figs(i), 'Name'))));
    if ~(contains(nm, 'aero') || contains(nm, 'animation'))
        continue;
    end
    ax = findall(figs(i), 'Type', 'axes');
    if isempty(ax)
        continue;
    end
    set(ax(1), ...
        'XLim', [-1.6 1.6], 'YLim', [-1.6 1.6], 'ZLim', [0 1.7], ...
        'DataAspectRatio', [1 1 1], 'Projection', 'perspective');
    view(ax(1), 38, 22);
    grid(ax(1), 'on');
    xlabel(ax(1), 'X (m)'); ylabel(ax(1), 'Y (m)'); zlabel(ax(1), 'Z (m)');
end
end

function acfile = local_litewing_ac3d()
root = fileparts(mfilename('fullpath'));
acfile = fullfile(root, '..', 'models', 'litewing.ac');
if exist(acfile, 'file')
    return;
end
write_litewing_ac3d(acfile);
end

function write_litewing_ac3d(acfile)
% Minimal AC3D X-quad, units = meters (visual scale ~0.16 m).
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
