function groups = objectMesh(o, T) %#ok<INUSD>
% OBJECTMESH  Procedural mesh for one scene object.
%   Geometry is built in the object's own frame: origin at the centre of
%   its footprint on the surface it stands on, z up. o.size_m is the
%   bounding box [x y z]. Returns one group per material.

s = o.size_m(:).';
col = o.color(:).';
groups = struct('mesh', {}, 'texture', {}, 'shininess', {}, 'metallic', {}, 'specular', {}, 'transparency', {});

switch lower(o.type)
    case 'box'
        m = aeris.MeshBuilder;
        if strcmpi(o.material, 'cardboard')
            m.box(s, [0 0 s(3)/2], [1 1 1]);
            groups = add(groups, m, 'cardboard.png', 0.08, 0, 0.05);
        else
            m.box(s, [0 0 s(3)/2], col);
            groups = add(groups, m, '', 0.55, 0, 0.5);
        end

    case 'table'
        top = 0.04; leg = 0.05;
        m = aeris.MeshBuilder;
        m.box([s(1) s(2) top], [0 0 s(3)-top/2], [1 1 1], [], 2);
        groups = add(groups, m, 'wood.png', 0.35, 0, 0.3);
        f = aeris.MeshBuilder;
        steel = [0.20 0.21 0.23];
        for sx = [-1 1]
            for sy = [-1 1]
                f.box([leg leg s(3)-top], [sx*(s(1)/2-leg) sy*(s(2)/2-leg) (s(3)-top)/2], steel);
            end
        end
        f.box([s(1)-2*leg 0.03 0.06], [0  (s(2)/2-leg) s(3)-top-0.05], steel);
        f.box([s(1)-2*leg 0.03 0.06], [0 -(s(2)/2-leg) s(3)-top-0.05], steel);
        f.box([s(1)-2*leg s(2)-2*leg 0.02], [0 0 0.18], [0.30 0.31 0.33]);
        groups = add(groups, f, '', 0.5, 0.6, 0.5);

    case 'bottle'
        r = s(1)/2; h = s(3);
        prof = [0 0; 0 r*0.92; 0.02*h r; 0.56*h r; 0.70*h r*0.72; 0.80*h r*0.36; 0.93*h r*0.36; 0.93*h r*0.42; h r*0.42; h 0];
        m = aeris.MeshBuilder;
        m.revolve(prof, [0 0 0], col);
        groups = add(groups, m, '', 0.95, 0.15, 0.9);
        c = aeris.MeshBuilder;
        c.cyl(r*0.9, 0.035*h, [0 0 h*0.985], [0.9 0.9 0.9]);
        c.cyl(r*2.02, 0.26*h, [0 0 0.30*h], [0.96 0.96 0.92]);   % paper label
        groups = add(groups, c, '', 0.2, 0, 0.2);

    case 'laptop'
        m = aeris.MeshBuilder;
        body = [0.62 0.63 0.65];
        m.box([s(1) s(2) 0.016], [0 0 0.008], body);
        m.box([s(1)*0.86 s(2)*0.45 0.002], [0 0.03 0.017], [0.10 0.10 0.11]);                 % keyboard
        m.box([s(1) 0.012 s(3)], [0 s(2)/2 s(3)/2], body, [-12 0 0]);
        groups = add(groups, m, '', 0.6, 0.7, 0.6);
        scr = aeris.MeshBuilder;
        R = aeris.MeshBuilder.rotm([-12 0 0]);
        p = @(x, z) ([x -0.0065 z] * R.') + [0 s(2)/2 s(3)/2];
        hw = s(1)*0.46; hh = s(3)*0.44;
        scr.quad(p(hw, -hh), p(-hw, -hh), p(-hw, hh), p(hw, hh), [1 1 1], [0 1; 1 1; 1 0; 0 0]);
        groups = add(groups, scr, 'screen.png', 0.9, 0, 0.9);

    case 'drum'
        r = s(1)/2; h = s(3);
        prof = [0 0; 0 r; 0.03*h r*1.03; 0.06*h r; 0.33*h r; 0.345*h r*1.03; 0.36*h r; 0.64*h r; 0.655*h r*1.03; 0.67*h r; 0.94*h r; 0.97*h r*1.03; h r; h r*0.94; 0.97*h r*0.94; 0.97*h 0];
        m = aeris.MeshBuilder;
        m.revolve(prof, [0 0 0], col, [], 36);
        m.cyl(0.06, 0.02, [r*0.55 0 0.975*h], [0.75 0.75 0.77]);
        groups = add(groups, m, '', 0.7, 0.75, 0.7);

    case 'cone'
        r = s(1)/2; h = s(3);
        orange = [0.95 0.32 0.04];
        prof = [0.03*h 0; 0.03*h r*0.72; 0.10*h r*0.66; h r*0.10; h 0];
        m = aeris.MeshBuilder;
        m.revolve(prof, [0 0 0], orange, [], 28);
        m.box([s(1) s(2) 0.03*h], [0 0 0.015*h], [0.08 0.08 0.08]);
        band = @(z0, z1) [z0 0; z0 r*(0.66 - 0.56*(z0-0.10*h)/(0.90*h))+0.002; z1 r*(0.66 - 0.56*(z1-0.10*h)/(0.90*h))+0.002; z1 0];
        m.revolve(band(0.42*h, 0.56*h), [0 0 0], [0.97 0.97 0.97], [], 28);
        m.revolve(band(0.68*h, 0.78*h), [0 0 0], [0.97 0.97 0.97], [], 28);
        groups = add(groups, m, '', 0.3, 0, 0.3);

    case 'extinguisher'
        r = s(1)/2; h = s(3);
        prof = [0 0; 0 r*0.95; 0.02*h r; 0.72*h r; 0.80*h r*0.75; 0.84*h r*0.35; 0.90*h r*0.35; 0.90*h 0];
        m = aeris.MeshBuilder;
        m.revolve(prof, [0 0 0], [0.78 0.04 0.04]);
        m.box([r*1.5 r*0.35 0.035*h], [r*0.45 0 0.93*h], [0.06 0.06 0.06]);
        m.box([r*1.3 r*0.30 0.02*h], [r*0.40 0 0.985*h], [0.06 0.06 0.06], [0 -18 0]);
        m.cyl(r*0.22, 0.55*h, [-r*0.75 0 0.62*h], [0.05 0.05 0.05], [0 12 0], 10);
        m.cyl(r*2.02, 0.20*h, [0 0 0.48*h], [0.95 0.93 0.86]);
        groups = add(groups, m, '', 0.8, 0.2, 0.8);

    case 'shelf'
        post = 0.04; nShelf = 4;
        steel = [0.24 0.27 0.33];
        m = aeris.MeshBuilder;
        for sx = [-1 1]
            for sy = [-1 1]
                m.box([post post s(3)], [sx*(s(1)/2-post/2) sy*(s(2)/2-post/2) s(3)/2], steel);
            end
        end
        zs = linspace(0.12, s(3)-0.03, nShelf);
        for k = 1:nShelf
            m.box([s(1) s(2) 0.025], [0 0 zs(k)], [0.55 0.56 0.58]);
        end
        % Stock on the shelves: bins and cartons with deterministic variety.
        palette = [0.10 0.35 0.65; 0.80 0.12 0.10; 0.15 0.50 0.25; 0.90 0.60 0.10; 0.35 0.35 0.38];
        for k = 1:nShelf-1
            n = 4;
            for j = 1:n
                wj = s(1)/n * (0.55 + 0.08*mod(k*3+j, 4));
                hj = (zs(k+1)-zs(k)) * (0.45 + 0.1*mod(k+j*2, 4));
                m.box([wj s(2)*0.7 hj], [(-0.5 + (j-0.5)/n)*s(1) 0 zs(k)+0.0125+hj/2], palette(mod(k*2+j, 5)+1, :));
            end
        end
        groups = add(groups, m, '', 0.35, 0.2, 0.35);

    case 'chair'
        m = aeris.MeshBuilder;
        fabric = [0.10 0.12 0.16]; steel = [0.55 0.56 0.58];
        seatZ = 0.46 * s(3)/0.92;
        m.box([s(1)*0.9 s(2)*0.9 0.07], [0 0 seatZ], fabric);
        m.box([s(1)*0.85 0.06 s(3)-seatZ-0.12], [0 -s(2)*0.42 seatZ+0.10+(s(3)-seatZ-0.12)/2], fabric, [8 0 0]);
        m.cyl(0.05, seatZ-0.08, [0 0 (seatZ-0.08)/2+0.06], steel, [], 12);
        for k = 0:4
            a = k*72;
            m.box([s(1)*0.52 0.035 0.03], [cosd(a)*s(1)*0.26 sind(a)*s(1)*0.26 0.06], steel, [0 0 a]);
            m.cyl(0.05, 0.03, [cosd(a)*s(1)*0.50 sind(a)*s(1)*0.50 0.025], [0.05 0.05 0.05], [90 0 a], 10);
        end
        groups = add(groups, m, '', 0.3, 0.1, 0.3);

    case 'plant'
        r = s(1)/2; h = s(3);
        m = aeris.MeshBuilder;
        potH = 0.30*h;
        m.revolve([0 0; 0 r*0.42; potH r*0.58; potH r*0.50; potH*0.92 r*0.50; potH*0.92 0], [0 0 0], [0.62 0.30 0.20]);
        m.cyl(0.025, 0.35*h, [0 0 potH+0.17*h], [0.30 0.22 0.12], [], 8);
        % Foliage: overlapping flattened lobes on a golden-angle spiral.
        nLeaf = 26;
        for k = 1:nLeaf
            a = k * 137.5; t = k / nLeaf;
            rad = r * (0.25 + 0.65*sqrt(t));
            zc = potH + 0.20*h + 0.48*h*(1 - 0.75*t) + 0.04*h*sind(k*71);
            g = [0.08+0.10*mod(k, 3)/2, 0.36+0.20*mod(k, 4)/3, 0.10];
            m.sphere(0.34*r*(1.2 - 0.4*t)*2, [cosd(a)*rad*0.8 sind(a)*rad*0.8 zc], g, [1.0 0.75 0.45]);
        end
        groups = add(groups, m, '', 0.25, 0, 0.2);

    case 'crate'
        m = aeris.MeshBuilder;
        t = 0.015;
        m.box([s(1) s(2) t], [0 0 t/2], col);
        m.box([s(1) t s(3)], [0  s(2)/2-t/2 s(3)/2], col);
        m.box([s(1) t s(3)], [0 -s(2)/2+t/2 s(3)/2], col);
        m.box([t s(2) s(3)], [ s(1)/2-t/2 0 s(3)/2], col);
        m.box([t s(2) s(3)], [-s(1)/2+t/2 0 s(3)/2], col);
        m.box([s(1)+0.01 s(2)+0.01 0.02], [0 0 s(3)-0.01], col*0.8);
        m.box([s(1)-2*t s(2)-2*t 0.01], [0 0 s(3)-0.03], [0.05 0.05 0.05]);
        groups = add(groups, m, '', 0.5, 0, 0.5);

    case 'partition'
        m = aeris.MeshBuilder;
        m.box([s(1) s(2) s(3)-0.10], [0 0 0.10+(s(3)-0.10)/2], [0.36 0.42 0.50]);
        m.box([0.06 0.40 0.03], [ s(1)/2-0.10 0 0.015], [0.2 0.2 0.22]);
        m.box([0.06 0.40 0.03], [-s(1)/2+0.10 0 0.015], [0.2 0.2 0.22]);
        m.box([0.04 0.04 0.10], [ s(1)/2-0.10 0 0.05], [0.2 0.2 0.22]);
        m.box([0.04 0.04 0.10], [-s(1)/2+0.10 0 0.05], [0.2 0.2 0.22]);
        m.box([s(1)+0.02 s(2)+0.02 0.03], [0 0 s(3)-0.015], [0.7 0.7 0.72]);
        groups = add(groups, m, '', 0.12, 0, 0.1);

    otherwise
        m = aeris.MeshBuilder;
        m.box(s, [0 0 s(3)/2], col);
        groups = add(groups, m, '', 0.4, 0, 0.4);
end
end

function groups = add(groups, mesh, texture, shininess, metallic, specular)
groups(end+1) = struct('mesh', mesh, 'texture', texture, 'shininess', shininess, ...
    'metallic', metallic, 'specular', specular, 'transparency', 0);
end
