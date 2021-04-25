# ceres_R_LocalParam
由于旋转通常采用李代数se(3)或四元数，这些结构对加法不封闭，ceres 提供了一个 局部参数化方法 LocalParameterization 处理自变量的更新问题。
