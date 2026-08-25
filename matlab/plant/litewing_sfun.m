function litewing_sfun(block)
% LITEWING_SFUN Level-2 MATLAB S-Function: cascaded PID + Newton-Euler 6-DOF.
% Input:  simulation time (s) from Clock
% Output: [x y z roll_deg pitch_deg yaw_deg vx vy vz]
setup(block);
end

function setup(block)
block.NumInputPorts  = 1;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions        = 1;
block.InputPort(1).DatatypeID        = 0;
block.InputPort(1).Complexity        = 'Real';
block.InputPort(1).DirectFeedthrough = false;

block.OutputPort(1).Dimensions = 9;
block.OutputPort(1).DatatypeID = 0;
block.OutputPort(1).Complexity = 'Real';

block.NumDialogPrms     = 0;
block.SampleTimes       = [0.01 0];
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitConditions);
block.RegBlockMethod('Outputs', @Output);
block.RegBlockMethod('Update', @Update);
end

function DoPostPropSetup(block)
block.NumDworks = 1;
block.Dwork(1).Name            = 'plant';
block.Dwork(1).Dimensions      = 15;
block.Dwork(1).DatatypeID      = 0;
block.Dwork(1).Complexity      = 'Real';
block.Dwork(1).UsedAsDiscState = true;
end

function InitConditions(block)
root = fileparts(mfilename('fullpath'));
addpath(root);
addpath(fullfile(root, '..', 'scripts'));
addpath(fullfile(root, '..', 'controllers'));
init_params;
sfun_params(evalin('base', 'params'));
plant = init_litewing_plant();
block.Dwork(1).Data = pack_plant(plant);
end

function Output(block)
x = block.Dwork(1).Data;
block.OutputPort(1).Data = [
    x(1); x(2); x(3);
    x(7)*180/pi; x(8)*180/pi; x(9)*180/pi;
    x(4); x(5); x(6)
];
end

function Update(block)
t = block.InputPort(1).Data;
ref = litewing_square_ref(t);
target = ref(1:3);
yaw = ref(4);
params = sfun_params();
plant = unpack_plant(block.Dwork(1).Data);
dt_inner = 0.002;
for k = 1:5
    plant = litewing_closed_loop_step(plant, target, yaw, params, dt_inner);
end
block.Dwork(1).Data = pack_plant(plant);
end

function x = pack_plant(p)
x = [p.p; p.v; p.eul; p.omega; p.integ_z; p.integ_xy(:)];
end

function p = unpack_plant(x)
p = init_litewing_plant();
p.p = x(1:3);
p.v = x(4:6);
p.eul = x(7:9);
p.omega = x(10:12);
p.integ_z = x(13);
p.integ_xy = x(14:15);
end

function p = sfun_params(newp)
% Level-2 MATLAB S-Functions have no block.UserData on R2025a.
persistent cached
if nargin
    cached = newp;
end
p = cached;
end
