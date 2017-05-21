# sparse_solution

## 模块说明
按照本次实验数据，对feature_detect模块的封装，使稀疏点云构建的过程更为简化

## API参考

### SparseSolution
``` c++
class SparseSolution
{
public:
    SparseSolution(){}
    ~SparseSolution()
    {
        if(_vecs)
        {
            delete []_vecs;
        }
        if(_camera)
        {
            delete []_camera;
        }
        if(_grand)
        {
            delete []_grand;
        }
    }
private:
    std::vector<std::string> _img_filenames;
    double* _vecs;
    double* _camera;
    double* _grand; 
};
```
作用：用于稀疏点云构建的解决方案
参数说明：
> - _img_filenames: 图像路径
> - _camera：内方位元素
> - _vecs：外方位元素
> - _grand: 物方点坐标
 
### SparseSolution::readFile
``` c++
void SparseSolution::readFile(const std::string& intrinsic_file,
                  const std::string& external_file,
                  const std::string& img_locations)
```
作用：读取配置文件
参数说明：
> - intrinsic_file: 内方位元素文件
> - external_file: 外方位元素文件
> - img_locations: 用于标定的图像路径的描述文件
 
### SparseSolution::run
``` c++
void SparseSolution::run(const std::string& grand_file)
```
作用：进行稀疏点云构建
参数说明：
> - grand_file:结果文件
 
