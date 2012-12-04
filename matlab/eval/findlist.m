function idx = findlist(list, name)
idx = [];
for i = 1:length(list)
    if(strcmp(list{i}, name))
        idx = i;
        return;
    end
end
end