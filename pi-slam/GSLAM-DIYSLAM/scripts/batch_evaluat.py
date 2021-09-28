# 目标
# 0. 核心目标，ＤＩＹＳＬＡＭ是模块化的设计，如何能够客观量化的评估各个模块的在所有数据集上的表现
# 1. python pipe 如何将ｓｔｄｉｎ，ｓｔｄｏｕｔ，ｓｔｄｅｒｒ重定向，将ＤＩＹＳＬＡＭ的输出到一个指定的文件，例如“eval_innno_chengdu.txt"
# 2.　ｐｙｔｈｏｎ执行ＤＩＹＳＬＡＭ，命令"eval_file=xxxxx" ，在ＤＩＹＳＬＡＭ的程序里，将特殊标记的输出，写入到这个文件
import sys
import os
import subprocess

## gslam的配置文件
gslam_file = "/home/liu/my_progs/slam/DIYSLAM/Default.cfg"
## gslam配置文件中的方法
gslam_file_feature = ['Map', 'FeatureDetector', 'Matcher', 'Initializer', 'Tracker', 'Mapper']
## 输入的参数
feature_cmd_para = []

## 创造文件夹，文件夹名字是使用的slam方法
## 使用同一种方法，能创造带有数字的文件夹
## 文件夹下面是数据集的读出的输出
def CreateDir(file_path):
    num = 2
    index = file_path.rfind('_')
    file = file_path[0:index]   

    if index != len(file_path) - 1:
        if index == len(file_path) - 2:
            num = file_path[-1]
        else:
            num = file_path[index + 1 : -1] + file_path[-1]
        num = int(num) + 1
        
    if os.path.exists(file_path):
        return CreateDir(file + '_' + str(num))
    else:
        os.makedirs(file_path)
        return file_path

## 返回文件夹名字
def ReadConfigFile(config_file, file_feature, cmd_feature):
    dir_name = str()
    
    with open(config_file, 'r') as f:
        for line in f:
            line = line.split('\n')[0]
            line_split = line.split('?=')
        
            if len(line_split) == 2:
                
                feature = line_split[0]
                feature_data = line_split[1]
                
                if feature in cmd_feature.keys():
                    dir_name += cmd_feature[feature] + '_'
                elif feature in file_feature:
                    dir_name += feature_data + '_'
    return dir_name
        
## 读取数据集文件
def ReadDatasetFile(dataset_file):
    base_dir = str()
    file = []
    with open(dataset_file, 'r') as f:
        for l in f:
            if l == '\n':
                continue
            elif l[0:2] == '##': 
                if l[2:-1] == 'basedir':
                    l = f.readline()
                    base_dir = l.split('\n')[0]
            elif len(base_dir) != 0:
                dataset = l[1:-1]
                file.append(base_dir + dataset)
    return file
                
## 处理cmd输入的命令
def HandlePara(cmd_para):
    para_dict = dict()
    for para in cmd_para:
        feature = para.split('=')[0]
        solution = para.split('=')[1]
        para_dict[feature] = solution
    return para_dict
    
## 读取命令行参数
cmd_para = sys.argv[1:]
para_dict = {}
if len(cmd_para) != 0:
    para_dict = HandlePara(cmd_para)

## 获得数据集文件
dataset_file = ReadDatasetFile('dataset.txt')
## 获取文件夹名字
dir_name = ReadConfigFile(gslam_file, gslam_file_feature, para_dict)
## 创造文件夹名字
create_dir = CreateDir(dir_name)
    
for dataset in dataset_file:
    file_name = dataset.split('/')[-1].split('.')[0]
    file = open(create_dir + '/' + file_name, 'w')
    
    ##　处理命令行参数
    cmd = "gslam"
    cmd += " conf=" + gslam_file + " Dataset=" + dataset

    for value in cmd_para:
        cmd += " " + value
    para = cmd.split(' ')
    
    print(para)
    ## 创造子进程运行gslam程序
    popen = subprocess.Popen(para, stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=True)

    while True:
        stderr_buff = popen.stderr.readline().decode('gbk')
        file.write(stderr_buff)
        
        if stderr_buff.find('SLAM released') != -1:
            popen.terminate()

        if stderr_buff == '' and popen.poll() != None:
            file.close()
            break

