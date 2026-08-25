function litewing_simulink_viz(p, eul)
% LITEWING_SIMULINK_VIZ Persistent 3D X-quad mesh driven by Simulink.
persistent fig h
if nargin < 1
    return;
end
if isempty(fig) || ~isvalid(fig)
    fig = figure('Name', 'LiteWing Simulink 3D Body', 'NumberTitle', 'off', ...
        'Color', [0.08 0.09 0.12], 'Position', [80 80 720 560]);
    ax = axes('Parent', fig, 'Color', [0.12 0.14 0.18], ...
        'XColor', [0.8 0.84 0.9], 'YColor', [0.8 0.84 0.9], 'ZColor', [0.8 0.84 0.9]);
    hold(ax, 'on'); grid(ax, 'on');
    xlim(ax, [-1.6 1.6]); ylim(ax, [-1.6 1.6]); zlim(ax, [0 1.7]);
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
    title(ax, 'Simulink 6-DOF  |  MATLAB mesh (no Multibody)', 'Color', 'w');
    view(ax, 35, 22);
    sq = [0.5 0.5; 0.5 -0.5; -0.5 -0.5; -0.5 0.5; 0.5 0.5];
    plot3(ax, sq(:,1), sq(:,2), ones(5,1), '--', 'Color', [0.45 0.55 0.35]);
    h = hgtransform('Parent', ax);
    s = 0.16;
    plot3([-s s], [s -s], [0 0], 'Parent', h, 'Color', [0.78 0.82 0.88], 'LineWidth', 2.8);
    plot3([-s s], [-s s], [0 0], 'Parent', h, 'Color', [0.78 0.82 0.88], 'LineWidth', 2.8);
    th = linspace(0, 2*pi, 28);
    r = 0.055;
    motors = [s s; s -s; -s -s; -s s];
    cols = [0.25 0.75 1.00; 1.00 0.55 0.20; 0.25 0.75 1.00; 1.00 0.55 0.20];
    for i = 1:4
        plot3(r*cos(th)+motors(i,1), r*sin(th)+motors(i,2), zeros(size(th)), ...
            'Parent', h, 'Color', cols(i,:), 'LineWidth', 1.6);
    end
    fill3([-0.04 0.04 0.04 -0.04], [-0.03 -0.03 0.03 0.03], [0 0 0 0], [0.18 0.22 0.28], ...
        'Parent', h, 'EdgeColor', [0.9 0.9 0.95]);
end
T = makehgtform('translate', [p(1) p(2) p(3)], ...
    'zrotate', eul(3), 'yrotate', eul(2), 'xrotate', eul(1));
set(h, 'Matrix', T);
drawnow limitrate;
end
