# Log

**目前的想法：**

- 目前先尝试让pi-slam和map2dfusion合并到一起运行，不考虑一些逻辑关系之类的问题，先把线程问题解决掉！
- 尝试让map2dfusion读取pi-slam的frame信息，通过这种方式运行
- 在原pi-slam中合适的地方加入一个RANSAC计算plane参数的部分，计算好之后输出结果和计算需要的拍数
- 将上面这一步单独提取出来，用来启动map2dfusion
- 如果使用plane启动map2dfusion有困难，可以直接让plane为空，然后map2dfusion启动后阻塞。（目前想到最简单的方法）

另外：map2dfusion中的错误还是要找出来，不过打算先等前面几步做好了再来找。

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

