function u = pendulumController(X,A,P,grid)
% u = pendulumController(X,A,P,grid)
%
% This function computes the optimal control to use at each point in the
% state space for the pendulum.
%
% INPUTS:
%   X = [2 x n] = matrix of states to query the controller
%   A = action matrix produced by MDP_pendulum.m
%   P = policy matrix produced by MDP_pendulum.m
%   pendulum = struct with pendulum information.
%
% OUTPUTS:
%   u = [1 x n] = torque vector produced by the controller
%

thBnd = grid.th.bnd;
thCount = grid.th.nGrid;

wBnd = grid.w.bnd;
wCount = grid.w.nGrid;

sBnd = [thBnd;wBnd];
sN = [thCount;wCount];

if exist('barycentricWeights_mex','file') == 3
    [weight, index] = barycentricWeights_mex(X,sBnd,sN);
else
    [weight, index] = barycentricWeights(X,sBnd,sN);
end
if exist('barycentricInterpolate_mex','file') == 3
    u = barycentricInterpolate_mex(A(P)',index,weight)';
else
    u = barycentricInterpolate(A(P)',index,weight)';
end

end