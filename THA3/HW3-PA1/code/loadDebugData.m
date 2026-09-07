function [data, file_name] = loadDebugData(test_num)

test_list = ["a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k"];
path_main = string(fullfile(fileparts(mfilename('fullpath')), '..', 'data')) + filesep;
if test_num < 8
    file_name = "pa1-debug-" + test_list(test_num) + "-";
else
    file_name = "pa1-unknown-" + test_list(test_num) + "-";
end

data_name = ["calbody.txt", "calreadings.txt", "empivot.txt", "optpivot.txt", "output1.txt"];
path = path_main + file_name + data_name;

%% calbody
nums = readHeader(path(1), 3);
Nd = nums(1); Na = nums(2); Nc = nums(3);

raw = readmatrix(path(1), 'NumHeaderLines', 1);

data.calbody.Nd = Nd;
data.calbody.Na = Na;
data.calbody.Nc = Nc;
d = raw(1:Nd, :);
a = raw(Nd+1 : Nd+Na, :);
c = raw(Nd+Na+1 : Nd+Na+Nc, :);
data.calbody.d = permute(d, [2 1 3]);
data.calbody.a = permute(a, [2 1 3]);
data.calbody.c = permute(c, [2 1 3]);

%% calreadings
nums = readHeader(path(2), 4);
ND = nums(1); NA = nums(2); NC = nums(3); Nf = nums(4);

raw = readmatrix(path(2), 'NumHeaderLines', 1);

D = zeros(ND, 3, Nf);
A = zeros(NA, 3, Nf);
C = zeros(NC, 3, Nf);

idx = 1;
for k = 1:Nf
    D(:,:,k) = raw(idx : idx+ND-1, :); idx = idx + ND;
    A(:,:,k) = raw(idx : idx+NA-1, :); idx = idx + NA;
    C(:,:,k) = raw(idx : idx+NC-1, :); idx = idx + NC;
end
 
data.calreadings.ND = ND;
data.calreadings.NA = NA;
data.calreadings.NC = NC;
data.calreadings.Nf = Nf;
data.calreadings.D = permute(D, [2 1 3]);
data.calreadings.A = permute(A, [2 1 3]);
data.calreadings.C = permute(C, [2 1 3]);

%% empivot
nums = readHeader(path(3), 2);
NG = nums(1); Nf = nums(2);

raw = readmatrix(path(3), 'NumHeaderLines', 1);

G = zeros(NG, 3, Nf);
idx = 1;
for k = 1:Nf
    G(:,:,k) = raw(idx : idx+NG-1, :);
    idx = idx + NG;
end

data.empivot.NG = NG;
data.empivot.Nf = Nf;
data.empivot.G = permute(G, [2 1 3]);

%% optpivot
nums = readHeader(path(4), 3);
ND = nums(1); NH = nums(2); Nf = nums(3);

raw = readmatrix(path(4), 'NumHeaderLines', 1);

D = zeros(ND, 3, Nf);
H = zeros(NH, 3, Nf);

idx = 1;
for k = 1:Nf
    D(:,:,k) = raw(idx : idx+ND-1, :); idx = idx + ND;
    H(:,:,k) = raw(idx : idx+NH-1, :); idx = idx + NH;
end

data.optpivot.ND = ND;
data.optpivot.NH = NH;
data.optpivot.Nf = Nf;
data.optpivot.D = permute(D, [2 1 3]);
data.optpivot.H = permute(H, [2 1 3]);

%% output
fid = fopen(path(5), 'r');

nums = [];
for i = 1:3
    line = fgetl(fid);
    nums_line = sscanf(line, '%f, %f, %f');
    nums = [nums; nums_line];
end
fclose(fid);

NC = nums(1); Nf = nums(2);
em_x = nums(3); em_y = nums(4); em_z = nums(5);
opt_x = nums(6); opt_y = nums(7); opt_z = nums(8);

raw = readmatrix(path(5), 'NumHeaderLines', 3);

C = zeros(NC, 3, Nf);
idx = 1;
for k = 1:Nf
    C(:,:,k) = raw(idx : idx+NC-1, :);
    idx = idx + NC;
end

data.output.NC = NC;
data.output.Nf = Nf;
data.output.em_x = em_x;
data.output.em_y = em_y;
data.output.em_z = em_z;
data.output.opt_x = opt_x;
data.output.opt_y = opt_y;
data.output.opt_z = opt_z;
data.output.C = permute(C, [2 1 3]);
end


function nums = readHeader(filename, n)
fid = fopen(filename, 'r');
header_line = fgetl(fid);
fclose(fid);

fmt = repmat('%d,', 1, n-1);
fmt = fmt + "%d";
nums = sscanf(header_line, fmt);
end