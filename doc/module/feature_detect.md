# feature_detect

## 模块说明：
采用SURF特征匹配同名点并构建稀疏点云

## API参考

### ReprojectionErrorForward
``` c++
struct ReprojectionErrorForward{
  ReprojectionErrorForward(double observed_x, 
                    double observed_y,
                    const Eigen::Matrix<double,3,4>& essential)
      :_observed_x(observed_x), 
       _observed_y(observed_y), 
       _essential(essential)
       {}
  double _observed_x;
  double _observed_y;
  Eigen::Matrix<double,3,4> _essential;
};
```
作用：用于多相前交的残存计算
参数说明：
> - _observed_x: 观测的像点坐标x
> - _observed_y: 观测的像点坐标y
> - _essential: 本质矩阵

### ReprojectionErrorForward::operator
``` c++
bool ReprojectionErrorForward::operator()(const T* const grand_pts,
              T* residuals) const
```
作用：用于多相前交的残存计算
参数说明：
> - grand: 物方坐标
> - residuals: 像点残差

### ReprojectionErrorForward::Create
``` c++
static ceres::CostFunction* ReprojectionErrorForward::Create(const double observed_x,
                                  const double observed_y,
                                  const Eigen::Matrix<double,3,4>& essential);

```
作用：产生ceres库要求格式的代价函数
参数说明：
> - observed_x: 观测的像点坐标x
> - observed_y: 观测的像点坐标y
> - essential: 本质矩阵

### bundle
``` c++
void bundle(const std::vector<std::vector<cv::KeyPoint>>& feature_pts,
            const std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index,
            const double* camera,
            const double* vecs,
            double* grand_pts);
```
作用：多相前交
参数说明：
> - feature_pts: 像点坐标
> - feature_pts_index: 像点坐标的对应关系
> - camera：内方位元素
> - vecs：外方位元素
> - grand_pts: 物方点坐标

### getR
``` c++
void getR(double phi,double omega,double kappa,
          Eigen::Matrix<double,3,3>& R)
```
作用：计算旋转矩阵
参数说明：
> - phi: phi角
> - omeag: omeag角
> - kappa: kappa角
> - R: 旋转矩阵

### getEssentialMatrix
``` c++
void getEssentialMatrix(size_t photo_index,
                        const double* camera,
                        const double* vecs,
                        Eigen::Matrix<double,3,4>& essential);
```
作用：计算本质矩阵
参数说明：
> - photo_index: 相片序号
> - camera: 内方位元素
> - vecs: 外方位元素
> - essential: 本质矩阵

### flatFeaturePoint
``` c++
void flatFeaturePoint(std::map<FeaturePtIndex,std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_relation,
                      std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index);

```
作用：由立体像对生成多度对应的像点组
参数说明：
> - feature_pts_relation: 立体像对
> - feature_pts_index: 多度对应的像点组

### featurePointMatch_SingleImg
``` c++
void featurePointMatch_SingleImg(const cv::Mat& left_descriptor,
                                 size_t single_index,
                                 const std::vector<cv::Mat>& descriptors,
                                 std::map<FeaturePtIndex,std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_relation);

```
作用：特定相片与其余相片匹配
参数说明：
> - left_descriptor: 特定相片的描述子
> - single_index: 特定相片的序号
> - descriptors: 所有描述子
> - feature_pts_relation：立体像对

### featurePointMatch
``` c++
void featurePointMatch(const std::vector<cv::Mat>& descriptors,
                       std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index);

```
作用：生成同名像对
参数说明：
> - descriptors: 所有描述子
> - feature_pts_index: feature_pts_index

### getInitialValue
``` c++
void getInitialValue(const std::vector<std::vector<cv::KeyPoint>>& feature_pts,
            const std::vector<std::shared_ptr<std::set<FeaturePtIndex>>>& feature_pts_index,
            const double* camera,
            const double* vecs,
            double* grand_pts);
```
作用：双像前交获得初始值
参数说明：
> - feature_pts: 像点坐标
> - feature_pts_index: 像点坐标的对应关系
> - camera：内方位元素
> - vecs：外方位元素
> - grand_pts: 物方点坐标


### forward
``` c++
void forward(size_t left_index,double left_x,double left_y,
             size_t right_index,double right_x,double right_y,
             const double* camera, const double* vecs,
             size_t pt_index,
             double* grand);
```
作用：
参数说明：
> - left_index: 左相片序号
> - left_x: 左像点的x
> - left_y: 左像点的y
> - right_index: 右相片序号
> - right_x: 右像点的x
> - right_y: 右像点的y
> - camera：内方位元素
> - vecs：外方位元素
> - pt_index: 物方点的序号
> - grand_pts: 物方点坐标

