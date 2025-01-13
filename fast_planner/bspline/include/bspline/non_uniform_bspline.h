/**
* 这个文件是 Fast-Planner 项目的一部分。
* 
* Fast-Planner 是一种自由规划软件，能够进行路径规划、轨迹优化等功能。
* 本文件提供了非均匀 B-spline 曲线的实现。
*/

#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace fast_planner {

/**
 * 非均匀 B-spline 的实现类
 * 支持多维控制点的 B-spline（如 3D 空间下的曲线）。
 * 也可以表示均匀 B-spline，作为非均匀 B-spline 的特例。
 */
class NonUniformBspline {
private:
  // B-spline 的控制点矩阵
  // 每行表示一个控制点，每列表示不同维度（例如 3D 空间下，列数为 3）
  // 例如：具有 N 个点的 3D B-spline -> 控制点矩阵大小为 Nx3
  Eigen::MatrixXd control_points_;

  int             p_, n_, m_;  // p：B-spline 的阶数，n+1：控制点个数，m = n+p+1：节点向量长度
  Eigen::VectorXd u_;          // 节点向量（Knots Vector）
  double          interval_;   // 节点间距（\delta t）

  // 获取 B-spline 的导数控制点
  Eigen::MatrixXd getDerivativeControlPoints();

  // 物理限制：速度、加速度和时间调整比例
  double limit_vel_, limit_acc_, limit_ratio_;

public:
  // 构造函数
  NonUniformBspline() {}
  NonUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);
  // 初始化为非均匀 B-spline
  ~NonUniformBspline();

  // 初始化为均匀 B-spline
  void setUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);

  // 设置和获取 B-spline 的基本信息

  // 设置节点向量
  void                                   setKnot(const Eigen::VectorXd& knot);
  // 获取节点向量
  Eigen::VectorXd                        getKnot();
  // 获取控制点矩阵
  Eigen::MatrixXd                        getControlPoint();
  // 获取节点间隔
  double                                 getInterval();
  // 获取时间跨度（um 表示起始节点，um_p 表示终止节点）
  void                                   getTimeSpan(double& um, double& um_p);
  // 获取首尾控制点的时间范围
  pair<Eigen::VectorXd, Eigen::VectorXd> getHeadTailPts();

  // 计算位置 / 导数

  // 使用 De Boor 算法计算给定参数 u 下的曲线位置
  Eigen::VectorXd   evaluateDeBoor(const double& u);   // 参数范围 u ∈ [up, u_mp]
  Eigen::VectorXd   evaluateDeBoorT(const double& t);  // 参数范围 t ∈ [0, duration]
  NonUniformBspline getDerivative(); // 获取 B-spline 的导数

  // 用于 3D B-spline 插值（根据采样点集与边界条件生成控制点）
  // 输入：采样点集（K+2 点）、边界速度/加速度、时间间隔 ts
  // 输出：控制点矩阵 (K+6)
  static void parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                    const vector<Eigen::Vector3d>& start_end_derivative,
                                    Eigen::MatrixXd&               ctrl_pts);

  /* 检查可行性，调整时间 */

  // 设置物理限制（速度和加速度）
  void   setPhysicalLimits(const double& vel, const double& acc);
  // 检查曲线是否满足物理限制
  bool   checkFeasibility(bool show = false);
  // 检查时间调整比例
  double checkRatio();
  // 按比例拉长时间
  void   lengthenTime(const double& ratio);
  // 重新分配时间（优化时间分配）
  bool   reallocateTime(bool show = false);

  /* 性能评估相关 */

  // 获取总时间
  double getTimeSum();
  // 获取路径长度（通过分辨率 res）
  double getLength(const double& res = 0.01);
  // 获取路径的总加加速度（Jerk）
  double getJerk();
  // 获取平均和最大速度
  void   getMeanAndMaxVel(double& mean_v, double& max_v);
  // 获取平均和最大加速度
  void   getMeanAndMaxAcc(double& mean_a, double& max_a);

  // 重新计算初始状态
  void recomputeInit();

  // 内存对齐优化（用于 Eigen 库）
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
