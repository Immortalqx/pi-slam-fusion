import sys
import os

write_item = ['Success', 'MeanInliers']

## 读取文件数据，取出我们想要的数据
def HandleData(file_list, dir_name):
    handle_output = {}
    
    for file in file_list:
        this_file = dir_name + '/' + file
        
        with open(this_file, 'r') as f:
            handle_output.setdefault(file, {})
            for l in f:
                for item in write_item:
                    if l.find(item) != -1:
                        handle_output[file][item] = l[l.find(item):]

    return handle_output
    
## handle_dir_name = "./Hash_Sift_multiH_opt_testInit_demo_(2)"
    
## 读取命令行参数
## 获取要处理的文件夹名字
cmd_para = sys.argv[1:]
handle_dir_name = cmd_para[0]

## 处理文件夹数据
file_list = os.listdir(handle_dir_name)
file_list.sort()
handle_output = HandleData(file_list, handle_dir_name)

write_file = handle_dir_name + '/output.txt'
with open(write_file, 'w') as f:
    for key1 in handle_output:
        if handle_output[key1] and key1 != 'output.txt':
            write_string = key1 + ':\n'
            for key2 in handle_output[key1]:
                write_string = write_string + handle_output[key1][key2]
            f.write(write_string)
            print(write_string)


