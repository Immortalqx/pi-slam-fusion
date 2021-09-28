# PI-SLAM-FUSION

## 1. 简介

pi-slam-fusion(暂时先用这个名称)，是基于pi-slam和map2dfusion的无人机建图策略。

## 2. 编译

根据Map2DFusion和pi-slam的README文件安装好环境依赖，然后运行如下指令进行编译：

```
mkdir build
cd build
cmake ..
make
```

也可以使用脚本`./build.sh`进行编译，具体用法通过指令`./build.sh -h`查询。

