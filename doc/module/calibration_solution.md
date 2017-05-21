# calibration_solution

##模块说明
按照本次实验数据，对control_point_detect模块的封装，使相机标定的过程更为简化

##API参考

### ReprojectionError
``` c++
struct ReprojectionError {
  ReprojectionError(double observed_x, 
                    double observed_y,
                    double X,
                    double Y,
                    double Z)
      :_observed_x(observed_x), 
       _observed_y(observed_y), 
       _X(X),
       _Y(Y),
       _Z(Z)
       {}
  double _observed_x;
  double _observed_y;
  double _X;
  double _Y;
  double _Z;
};
```
作用：用于光束法平差的残存计算
参数说明：
> - _observed_x：观测的像点坐标x
> - _observed_y: 观测的像点坐标y
> - _X: 物方点坐标X
> - _Y: 物方点坐标Y
> - _Z: 物方点坐标Z

### ReprojectionError::operator
``` c++
bool ReprojectionError::operator()(const T* const camera,
              const T* const vecs,
              T* residuals) const 
```
作用：用于光束法平差的残存计算,用于ceres库调用
参数说明：
> - camera: 相机参数
> - vecs: 外方位元素
> - residuals: 像点坐标残差

### ReprojectionError::Create
``` c++
static ceres::CostFunction* ReprojectionError::Create(const double observed_x,
                                  const double observed_y,
                                  const double X,
                                  const double Y,
                                  const double Z)
```
作用：产生ceres库要求格式的代价函数
参数说明：
> - observed_x：观测的像点坐标x
> - observed_y: 观测的像点坐标y
> - X: 物方点坐标X
> - Y: 物方点坐标Y
> - Z: 物方点坐标Z


### CalibrationSolution
``` c++
class CalibrationSolution
{
public:
    CalibrationSolution(){}
    ~CalibrationSolution()
    {
        if(_vecs)
        {
            delete []_vecs;
        }
    }

private:
    std::vector<std::string> _img_filenames;
    std::vector<cv::Point3f> _gcps;

    std::vector<std::vector<ControlPoint>> _cptss;

    double* _vecs;
    double _camera[5];
};
```
作用：用于相机标定的解决方案
参数说明：
> - _img_filenames: 用于标定的图像路径的描述文件
> - _gcps：控制点的物方坐标
> - _cptss：控制点信息
> - _vecs：外方位元素
> - _camera：内方位元素

### CalibrationSolution::extractControlPoints
``` c++
void CalibrationSolution::extractControlPoints();
```
作用：提取控制点

### CalibrationSolution::getInitialValue
``` c++
bool CalibrationSolution::getInitialValue();
```
作用：采用张正友的方法获取初始值

### CalibrationSolution::bundle
``` c++
void CalibrationSolution::bundle();
```
作用: 采用光束法的方法获得标定结果

### CalibrationSolution::readFile
``` c++
void CalibrationSolution::readFile(
            const std::string& img_locations,
            const std::string& cpt_locations);

```
作用：读取配置文件
参数说明：
> - img_locations: 用于标定的图像路径的描述文件
> - cpt_locations: 记录控制点的物方坐标的文件
 
### CalibrationSolution::run
``` c++
void CalibrationSolution::run();
```
作用：进行标定
