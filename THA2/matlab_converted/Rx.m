function R = Rx(a)
% Rotation matrix about x-axis by angle a (radians).

c = cos(a);
s = sin(a);
R = [1 0 0; 0 c -s; 0 s c];
end
