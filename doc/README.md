# Introduction

## 开发环境
- os: ubuntu16.04
- 构建工具：cmake
- 编译工具:gnu
- 依赖的库：
    + OpenCV2.0
    + ceres(平差优化)
    + Eigen3(矩阵计算)
    + Gitbook(可选，用于生成api文档)
- 代码管理：Git

## 构建过程
###编译可执行文件
解压文件
```bash
mkdir build
cd build
cmake ..
make
```
###执行可执行文件
```bash
cd build
#相机标定
work/calibration 
#稀疏点云构建
work/sparse
```
###生成API
```bash
cd doc
gitbook serve
```
即可在浏览器中浏览文档


## 附注
1. test文件夹下为测试程序，它会在sfm库编译时一同编译
2. doc/_book 文件夹下有已经生成的API文档。
3. 代码目前仍旧不完善，后续开发可参考[http://www.github.com/ncqwer/computer_vision](http://www.github.com/ncqwer/computer_vision)(dev分支，master分支上没有内容)