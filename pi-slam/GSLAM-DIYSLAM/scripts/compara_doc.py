import sys

def ReadTxt(txt_name):
    dic = {}
    with open(txt_name, 'r') as f:
        for line in f:
            split = line.split(':Success: ')
            dic[split[0]] = split[1]
    return dic
            
## 读取命令行参数
## 获取要处理的文件夹名字
cmd_para = sys.argv[1:]
    
# left_txt = "./Hash_Sift_multiH_opt_testInit_demo_(2)/output.txt"
# right_txt = "./Hash_Sift_multiH_svd_testInit_demo_/output.txt"

left_txt = cmd_para[0]
right_txt = cmd_para[1]

left_name_right_index = left_txt.rfind('/')
left_name_left_index = left_txt[0:left_name_right_index].rfind('/')
right_name_right_index = right_txt.rfind('/')
right_name_left_index = right_txt[0:right_name_right_index].rfind('/')

left_name = left_txt[left_name_left_index+1:left_name_right_index]
right_name = right_txt[right_name_left_index+1:right_name_right_index]

output_txt = left_name + "+" +  right_name + "+" + "output.txt"

left_dic = ReadTxt(left_txt)
right_dic = ReadTxt(right_txt)
output_dic = {}

for key in left_dic:
    if key in right_dic.keys():
        all_num = int(left_dic[key].split('/')[1])
        left_num = int(left_dic[key].split('/')[0])
        right_num = int(right_dic[key].split('/')[0])
    else:
        continue
    
    output_dic[key] = str(left_num - right_num) + "/" + str(all_num)
    
with open(output_txt, 'w') as f:
    for key in output_dic: 
        write_string = left_dic[key] + right_dic[key] + key + ':' + output_dic[key] + '\n'
        f.write(write_string)
        print(write_string)
