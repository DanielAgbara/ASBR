function w = unskew(hat_w)
% so(3) vee operator.

w = [hat_w(3,2); hat_w(1,3); hat_w(2,1)];
end
