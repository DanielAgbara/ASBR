function saveOutputData(Nc, Nf, em_post, opt_post, C_expected, file_name)

% Save to the current output directory; never overwrite reference datasets.
path = file_name + "output1.txt";

fileID = fopen(path, "w");
if fileID < 0
    error("Unable to open file: %s", path);
end

fprintf(fileID, "%d, %d, %s\n", Nc, Nf, file_name + "output1.txt");
fprintf(fileID, "%.2f, %.2f, %.2f\n", em_post(1), em_post(2), em_post(3));
fprintf(fileID, "%.2f, %.2f, %.2f\n", opt_post(1), opt_post(2), opt_post(3));

for i = 1:Nf
    for j = 1:Nc
        fprintf(fileID, "%.2f, %.2f, %.2f\n", ...
            C_expected(1,j,i), ...
            C_expected(2,j,i), ...
            C_expected(3,j,i));
    end
end

fclose(fileID);

end
