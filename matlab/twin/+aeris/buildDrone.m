function drone = buildDrone(world, parent, T, name, self)
% BUILDDRONE  Visual model of the airframe, sized from T.air.
%   Creates the movable actor <name> (body frame: x forward, y left, z up,
%   origin at the centre of mass) with children <name>_Rotor1..4 that are
%   spun from Simulink. Pass an existing actor as `self` to draw onto it
%   (the Simulation 3D Actor block owns its actor).

if nargin < 4 || isempty(name), name = 'Drone'; end
if nargin < 5, self = []; end
air = T.air;
L = air.L; R = air.R; hull = air.hull;
k = L / 0.045;                                  % scale relative to the LiteWing

mb = aeris.MeshBuilder;
pcb = [0.03 0.05 0.04]; black = [0.03 0.03 0.035];
mb.box([0.036 0.030 0.0016]*k, [0 0 0], pcb);
mb.box([0.018 0.014 0.004]*k, [0.004*k 0 0.003*k], [0.12 0.12 0.13]);          % MCU shield
mb.box([0.030 0.018 0.007]*k, [-0.002*k 0 -0.0045*k], [0.10 0.25 0.70]);       % battery
mb.box([0.004 0.010 0.004]*k, [0.019*k 0 0.001*k], [0.05 0.05 0.05]);          % camera/ToF nose
for i = 1:4
    p = air.rotor_xy(i, :);
    ang = atan2d(p(2), p(1));
    mb.box([L 0.0075*k 0.0016*k], [p/2 0], pcb, [0 0 ang]);                    % arm
    mb.cyl(0.0085*k, 0.020*k, [p -0.004*k], [0.72 0.73 0.75], [], 14);         % motor can
    mb.cyl(0.004*k, hull(3)*0.5, [p -hull(3)*0.25-0.004*k], black, [], 8);     % landing leg
    led = [0.9 0.1 0.1]; if p(1) < 0, led = [0.1 0.9 0.2]; end
    mb.box([0.003 0.003 0.002]*k, [p*0.55 0.002*k], led);
end
home = [T.scene.home.pos(:).' T.z_rest];
if isempty(self)
    drone = mb.emit(world, parent, name, Movable=true, Shininess=0.4, Specular=0.4, Translation=home);
else
    drone = self;
    mb.fill(drone, struct('Movable', true, 'Shininess', 0.4, 'Specular', 0.4, 'Translation', home));
end

for i = 1:4
    p = air.rotor_xy(i, :);
    blade = [0.05 0.05 0.05]; if p(1) > 0, blade = [0.95 0.45 0.05]; end       % front props orange
    rb = aeris.MeshBuilder;
    rb.box([2*R 0.16*R 0.0012*k], [0 0 0], blade, [8 0 0]);
    rb.cyl(0.16*R, 0.006*k, [0 0 0], [0.1 0.1 0.1], [], 10);
    rb.emit(world, drone, sprintf('%s_Rotor%d', name, i), Movable=true, Shininess=0.5, ...
        Translation=[p 0.010*k], Shadows=false);
    disc = aeris.MeshBuilder;
    disc.cyl(2*R, 0.0006*k, [0 0 0], blade, [], 28);
    disc.emit(world, drone, sprintf('%s_Disc%d', name, i), Movable=true, Transparency=0.82, ...
        Translation=[p 0.010*k], Shadows=false, Shininess=0.1);
end
end
