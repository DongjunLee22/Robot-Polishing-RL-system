% paths
cur_cd = cd; folder_name = strsplit(cd,'\'); folder_name = folder_name{length(folder_name)};
data_cd = [extractBefore(cur_cd,'\Main') '\Data'];
fun_cd = [extractBefore(cur_cd,'\Main') '\Functions'];
path(path,data_cd); path(path,fun_cd);
