# Log

**目前的想法：**

目前已经成功的让pi-slam和map2dfusion合并到一起运行了，不过还有几个问题：

- [x] 屏蔽掉PTL中的svar，整个项目统一使用更新的svar；

    > 可以的，使用老的PIL的部分的代码修改一下，把include头文件改成最新的svar 

- [ ] 让map2dfusion完全从pi-slam中获取数据；

  比如，让map2dfusion使用pi-slam已经读取的图片，而不是自己再从数据集中读取同一张图片；

- [ ] 找出map2dfusion里面的问题，想办法解决掉；

- [ ] 修改pi-slam-fusion的接口，让pi-slam-fusion可以在仿真环境中使用（和陶志坤沟通）；

    > 很好！你们沟通好，看看怎么把虚拟的数据接入进来

### 3月19日

**安排：**

- [x] 检查pi-slam与map2dfusion是否有共用的参数；

  - [x] “Act”

    map2dfusion的“Act“修改为"Map.Act"

  - [ ] “GPS.origin”

    暂时不打算修改，这个好像没有影响

  - [ ] “Plane”

    同上，应该没有影响

  - [ ] "Camera.Paraments"

    同上

- [ ] 让map2dfusion完全从pi-slam中获取数据；

- [ ] 修改pi-slam-fusion的接口，让pi-slam-fusion可以在仿真环境中使用（和陶志坤沟通）；


**问题：**



### 3月18日

**安排：**

- [x] 熟悉一下之前已经做的工作
- [x] 屏蔽掉PTL中的svar，整个项目统一使用更新的svar

**问题：**

- 发现统一svar其实挺简单的，没有之前想的那么麻烦，就是再把之前改过的东西改回来比较费工夫。
- pi-slam和map2dfusion共用了一个叫做“Act”的参数，导致map2dfusion运行不起来，**后面需要检查一下这两个线程是否还有共用的参数！！！**

### 2月9日

**安排：**

- [x] 实现map2dfusion能够读取pi-slam的frame信息
  - [x] 先修改map2dfusion的接口，obtainframe
  - [x] 从pi-slam中传输计算好的frame，可能要考虑一下数据类型的问题！
- [x] 在原pi-slam中合适的地方加入一个RANSAC计算plane参数的部分

**问题：**

- 目前map2dfusion还不是完全的依赖pi-slam的，后面得研究一下是不是可以只让pi-slam接触数据，然后给map2dfusion传处理过的数据。

  （比如图片，目前就是pi-slam把时间戳传过来，然后map2dfusion再读取图片）

  > 可以的，把图像打包到frame对象里，传给map2fusion

- 数据集里trajectory上面四元数的格式确实是wxyz，读取也是按照xyzw读取的，所以map2dfusion里面这里应该是有问题；

  而pi-slam传递给map2dfusion的是xyzw的形式，但是不需要调整成wxyz就能正确建图了，很奇怪。
  
  > 应该是某一个地方的转化没有找到，可以把代码整个处理流程再好好看一下。如果实在找不到，先把程序先跑起来，跑通了之后，在仔细分析一下，是不是库、处理程序等那个地方有问题

### 2月7日~2月8日

**安排：**

- [ ] 考虑如何让线程之间进行数据交换
  - [x] 学习生产者消费者模型
  - [x] 写一个简单的例子尝试一下
  - [x] 尝试构建一个类来传输数据
- [ ] 实现map2dfusion能够读取pi-slam的frame信息
  - [ ] 先修改map2dfusion的接口，obtainframe
  - [ ] 从pi-slam中传输计算好的frame，可能要考虑一下数据类型的问题！

**问题：**

- map2dfusion需要的类型：std::pair\<cv::Mat, pi::SE3d\>
- pi-slam中的类型：std::shared_ptr\<MapFrame\>，MapFrame是一个类，里面包含pi::SE3d但不包含cv::Mat

如何保证pi-slam和map2dfusion的图片是相同的?

目前的解决方案：目前pi-slam还是从固定数据集读取图片，所以可以先让pi-slam向map2dfusion传送时间戳（图片名称）和pi::SE3d，这样让map2dfusion根据获取的时间戳从本地读取图片，然后根据获取的位姿来生成frame。

在pi-slam的trackeropt的track函数里面进行修改？



### 1月31日~2月3日

**安排：**

- [x] 弄清楚为什么在map2dfusion线程中会新创建一个svar实例

  问题出在：pi-slam用的是GSLAM中的svar，map2dfusion用的是PIL中的svar

  > 想办法如何把这个问题解决了

- [x] 想办法统一这个svar类。

- [ ] 考虑如何让线程之间进行数据交换

    > 这个可以考虑使用互斥+类成员变量，或者信号量
    >
    > https://zhuanlan.zhihu.com/p/157171731
    >
    > https://gitee.com/pi-lab/learn_programming/tree/master/5_advanced/5_MultiThread

**疑问：**

- ~~svar用的是智能指针+局部静态变量，按道理说是线程安全的。~~

- 目前发现把map2dfusion中的svar替换成pi-slam中的svar的话，要修改的地方特别多，尝试了一下，感觉不太能成功。。。

  可能尝试一下让这两个svar的内容交换要更靠谱一些？
  
  > 可以让一个更新的版本做为主要版本，把另外一个屏蔽掉，整个项目都使用主要的那个试试

### 1月29日

**今天的安排：**

- [x] 解决参数传递的问题（pi-slam和map2dfusion里面应该有一些同名的参数，要改一下！）

  修改解析命令行参数的函数不是很好，并且两个程序的参数主要在cfg里面，也没有重名的，直接合并到一起就好了。

- [ ] 又发现了svar前几天的那个问题，今天要解决掉！

- [ ] 考虑如何让不同线程进行数据交换

**发现的问题：**

- 把参数的问题解决掉之后又发现svar解析的数据线程中看不到。之前是把这个问题绕开了，但目前看来还是要解决掉！

  这里主要是因为map2dfusion线程自己创建了个svar实例。（为什么合并前单独运行没有问题？？）

  而pi-slam中`slam->call("SetSvar", &svar);`这部分是在主线程中运行的，所以这里的实例应该是之前创建好的。

- 应该是svar用的是懒汉式，这样容易导致线程创建新实例，要从这里修改。（写个简单的例子看看）

### 1月27日

**今天的安排：**

- 继续尝试让pi-slam和map2dfusion合并到一起
  - 先让map2dfusion在线程中跑起来，解决昨天的一些问题
  - 让pi-slam也能够跑起来（这时候应该会有一个参数传递的问题，比如conf=xxxxx这部分）
  - 尝试解决参数传递的问题

**发现的问题：**

- 因为`svar.ParseMain(argc, argv);`在线程中使用的时候会发生段错误（不清楚具体原因），我把它移动到了main函数里面（本来就应该放在main里面呀），结果main函数中确实能够成功解析，但是解析出来的数据线程中居然看不到（奇怪的问题，这个问题之前没有遇到过，你可以写一个简单的多线程测试程序试试，找到到底哪里错了；另外我在代码标注FIXME有可能问题产生的地方，你需要好好读一下里面代码，看看里面到底做了什么，确认是否是这里的问题）。

  原因：svar用的单例模式是懒汉式，在多线程并发下，可能有多个线程同时调用getinstance，然后产生了多个实例，无法保证实例唯一。
  
- 突然有了灵感，终于让pi-slam和map2dfusion一起跑了，也避开了上面这个问题，不过目前还需要做的事情比较多：

  - 解决参数的问题（pi-slam和map2dfusion里面应该有一些同名的参数，要改一下！）

    感觉改改名称可能就好了

  - 想办法让不同线程进行数据交换，这样就可以先让pi-slam计算出plane再交给map2dfusion使用了

    这里要做的事情可能麻烦一些

### 1月26日

**今天的安排：**

- 尝试pi-slam和map2dfusion合并到一起，仅仅是让两个同时跑起来

**一些问题：**

- 继承之后再重写出了点问题，目前感觉最好的还是在pi-slam中进行修改

- 又遇到了重定义的问题，因为在pi-slam中引入了map2dfusion需要的一些头文件，这些东西又冲突了！

  > 我感觉还是给pi-slam和map2dfusion分开，之后在pi-slam-fusion里面可以把这些线程组织起来。
  >
  > 另外我感觉不应该冲突，不然main.cpp里面的代码是跑不起来的。。。。
  
- 把map2dfusion的主函数改成能够用pthread的了，但是单独运行没有问题，放到线程里面就会报错。

  目前发现是svar这部分出的问题，打算把它放到线程外看看。

