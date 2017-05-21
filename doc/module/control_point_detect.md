# control_point_detect

## 模块说明
本模块提供探测控制点、控制点分组、控制点编号3个功能。

## API参考

###  ControlPoint
``` c++
struct ControlPoint
{
    cv::Point3f circle;
    double angle;
    double distance;
    size_t radius_index;
    size_t cpt_index;
    size_t img_index;
    std::string pattern;
    ControlPoint(){}
    ControlPoint(const cv::Point3f& c,
                 const double a,
                 const double d,
                 const size_t r_i,
                 const size_t pt_i,
                 const size_t img_i,
                 const std::string& p=""):
        circle(c),
        angle(a),
        distance(d),
        radius_index(r_i),
        cpt_index(pt_i),
        img_index(img_i),
        pattern(p)
    {}
};
```
作用:记录控制点对应的像点
参数说明：
> - circle: x y z 分别表示椭圆中心的x,y,radius
> - angle：记录该点和中心点确定直线的角度
> - distance: 记录该点和中心点的距离
> - radius_index： 记录在按半径排序时该点的序号
> - cpt_index：记录对应的控制点编号
> - img_index：记录对应的像片编号
> - pattern：记录对应的二进制串标签

### byDistance
``` c++
bool byDistance(const ControlPoint& lhs,
           const ControlPoint& rhs);
```
作用：按与中心点的距离排序，供排序算法调用
参数说明：
> - lhs:左变量
> - rhs:右变量
 
### byAngle
``` c++
bool byAngle(const ControlPoint& lhs,
           const ControlPoint& rhs);
```
作用：按与中心点确定直线的角度排序，供排序算法调用
参数说明：
> - lhs:左变量
> - rhs:右变量

### byRadius
``` c++
bool byRadius(const ControlPoint& lhs,
           const ControlPoint& rhs);
```
作用：按半径大小排序，供排序算法调用
参数说明：
> - lhs:左变量
> - rhs:右变量

### controlPointDetect
``` c++
void controlPointDetect(const cv::Mat& img,
                        std::vector<ControlPoint>& cpts);
```
作用：提取控制点并为控制点编号
参数说明：
> - img: 含有控制点的图像
> - cpts: 已经编号好的控制点组

### getControlPoint
``` c++
void getControlPoint(const cv::Mat& image,
                     std::vector<cv::Point3f>& circles);
```
作用：提取控制点
参数说明：
> - image:含有控制点的图像
> - circles:控制点，x,y,z分别表示像点的列、行、半径

### getControlPointGroups
``` c++
void getControlPointGroups(const cv::Point2f& image_center,
                           const std::vector<cv::Point3f>& circles,
                           std::vector<std::vector<ControlPoint>>& control_point_groups);

```
作用：为提取的控制点分组
参数说明：
> - imgae_center:中心点坐标（注意：不是图像中心点，而是控制点确定的中心点）
> - circles: 提取的控制点
> - control_point_groups: 分组好的控制点

### fillControlPointIndex
``` c++
void fillControlPointIndex(std::vector<std::vector<ControlPoint>>& control_point_groups,
                          std::vector<ControlPoint>& control_points);

```
作用：为分组好的控制点编号
参数说明：
> - control_point_groups: 分组好的控制点
> - control_points: 去掉分组结构并编号好的控制点

### calculateIndex
``` c++
size_t calculateIndex(size_t radius_index,
                      const std::string& pattern);

```
作用：计算控制点编号
参数说明：
> - radius_index: 控制点按半径排序时的序号
> - pattern: 控制点的二进制串标签



