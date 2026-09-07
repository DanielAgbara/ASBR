function R = Ry(a)
% Rotation matrix about y-axis by angle a (radians).

c = cos(a);
s = sin(a);
R = [c 0 s; 0 1 0; -s 0 c];
end
