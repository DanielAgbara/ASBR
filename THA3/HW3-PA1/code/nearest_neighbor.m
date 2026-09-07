function [s_m, dist, idx] = nearest_neighbor(m, S)
% 
% Inputs:
%           m: point in moving model set (3x1)
%           S: fixed/statac scene set (3xn)
% Returns:
%           s_m: corresponding point in set S that attains 
%                the shortest distance to a given point m

euclidean_dist = sum((S-m).^2, 1);
[dist, idx] = min(euclidean_dist);
s_m = S(:, idx);
end