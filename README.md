# ceres_R_LocalParam
由于旋转通常采用李代数so(3)或四元数，这些结构对加法不封闭，ceres 提供了一个 局部参数化方法 [LocalParameterization](http://ceres-solver.org/nnls_modeling.html?highlight=local#_CPPv4N5ceres21LocalParameterizationE) 处理自变量的更新问题。本仓库提供一种直接对**旋转矩阵[GlobalSize=9]**进行**局部参数化[LocalSize=3]**的例子, 使用李代数左扰动对旋转进行求导[可参考 [视觉SLAM十四讲](https://github.com/gaoxiang12/slambook) 中左扰动模型推导]。



#### Introduction:

​	通过旋转矩阵作用于点**Po**，构造旋转后的点**P**与目标点**Pt**之间的距离作为误差，验证对旋转矩阵的参数化与求导结果。



#### Note:

1. 使用Eigen的Map对指针和矩阵进行关联时，注意RowMajor与ColMajor的差异。
2. 在LocalParameterization的雅克比计算一般简单放置为单位阵与零，真正的雅克比计算通常实现在CostFunction的Evaluate中。



#### Result:

```
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  1.132070e+00    0.00e+00    5.22e-01   0.00e+00   0.00e+00  1.00e+04        0    2.31e-05    8.82e-05
   1  2.450901e-03    1.13e+00    2.25e-01   5.64e-01   1.04e+00  3.00e+04        1    4.10e-05    1.77e-04
   2  2.922247e-08    2.45e-03    7.45e-04   2.82e-02   1.00e+00  9.00e+04        1    5.96e-06    1.92e-04
   3  4.707057e-18    2.92e-08    9.28e-09   9.29e-05   1.00e+00  2.70e+05        1    4.05e-06    2.04e-04
Ceres Solver Report: Iterations: 4, Initial cost: 1.132070e+00, Final cost: 4.707057e-18, Termination: CONVERGENCE

Init R = 
 -0.364969   0.929756 -0.0485023
 -0.224431    -0.0373   0.973776
  0.903564   0.366283   0.222279
Optim R = 
-0.0786646   0.996492  0.0285602
 0.0539632 -0.0243506   0.998246
   0.99544  0.0800678 -0.0518584
```

