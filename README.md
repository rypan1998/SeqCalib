# README

## 1. 功能介绍

什么是“序列标定”？在标定过程中，有时候一组图往往特征点太少，但是 COLMAP 又只能依赖于这一组图，所以提出：能否通过操作数据库的方式，把多帧的特征点写到数据库中，相当于这一组图像集合了所有帧图像的特征点，这样特征点的数量便翻了好多倍。

这种方式的难点是什么？即便是多帧图像，无非就是多了很多三维点而已。对于一个三维点，你需要知道它能被哪些相机看到，也就是共视关系，所以我们选择用 CharUco 板作为特征点检测的标志物，它的好处是有 ID 编号，通过编号就能找到其角点，即三维特征点；通过检测这一组中的其他图像，就知道了相同编号的点对应的二维点坐标了，这就是我们想要的共视关系。

我们使用的二维码板尺寸都较大，所以每组图像都能提供 80 个点，那么总共拍 100 组就能得到 8k 个点。

## 2. 运行介绍

代码的入口在 `Extract.cpp` 中，分为三步，对应 `scripts/calib_aruco.sh` 中的指令。

其中第二步添加了新功能：它有两个入口函数，一个是第一部分介绍的序列标定功能 `Match()` 函数，另一个 `generateRandomPoints` 是随机产生指定范围内的三维点，通过参考的 `xml_gt` 进行反投影，可以对得到的二维点进行加噪声等操作，进行 colmap 标定性能的实验。

所以如果需要运行第一部分的序列标定，可能需要改一下代码，修改脚本中的路径、相机数量等参数，并把数据按照如下目录结构进行组织。

```bash
scripts 
|---0
    |--0000.jpg
    |--0001.jpg
    ...
|---1
    |--0000.jpg
    |--0001.jpg
    ...    
|---calib_aruco.sh
```

最终的结果在 `template/input/0` 中，可以打开 COLMAP 查看标定结果。