/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include "bspline/non_uniform_bspline.h"
#include <ros/ros.h>

namespace fast_planner {

NonUniformBspline::NonUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                     const double& interval) {
  setUniformBspline(points, order, interval);
}

NonUniformBspline::~NonUniformBspline() {}

void NonUniformBspline::setUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                          const double& interval) {
  // 设置控制点矩阵
  control_points_ = points; // 将传入的控制点矩阵赋值给成员变量
  // 设置 B-spline 的阶数
  p_ = order; // B-spline 的阶数（如 3 表示三次 B-spline）
  // 设置节点间隔
  interval_ = interval; // 节点间隔（\delta t）

  // 计算控制点的数量
  n_ = points.rows() - 1; // 控制点数量 n+1，其中 n = 控制点行数 - 1
  // 计算节点向量的数量
  m_ = n_ + p_ + 1; // 节点向量的总数 m+1，其中 m = n + p + 1 = 控制点数量 + 阶数 + 1

  // 初始化节点向量 u_，大小为 m+1，初始值全为 0
  u_ = Eigen::VectorXd::Zero(m_ + 1);//节点向量有每个节点的位置，大小为 m+1，初始值全为 0

  // 填充节点向量（均匀分布的节点）
  for (int i = 0; i <= m_; ++i) {

    if (i <= p_) { 
      // 当 i <= p_ 时，节点在起始区域，采用负索引表示（对称分布）
      u_(i) = double(-p_ + i) * interval_; // 例如：-3, -2, -1, 0 （对于三次 B-spline）
    } else if (i > p_ && i <= m_ - p_) {
      // 当 p_ < i <= m_ - p_ 时，节点均匀分布
      u_(i) = u_(i - 1) + interval_; // 每个节点间隔为 interval_
    } else if (i > m_ - p_) {
      // 当 i > m_ - p_ 时，节点在结束区域，同样保持均匀分布
      u_(i) = u_(i - 1) + interval_; // 例如：... , m-2, m-1, m
    }
  }
}


void NonUniformBspline::setKnot(const Eigen::VectorXd& knot) { this->u_ = knot; }

Eigen::VectorXd NonUniformBspline::getKnot() { return this->u_; }

void NonUniformBspline::getTimeSpan(double& um, double& um_p) {
  um   = u_(p_);
  um_p = u_(m_ - p_);
}

Eigen::MatrixXd NonUniformBspline::getControlPoint() { return control_points_; }

pair<Eigen::VectorXd, Eigen::VectorXd> NonUniformBspline::getHeadTailPts() {
  Eigen::VectorXd head = evaluateDeBoor(u_(p_));
  Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_));
  return make_pair(head, tail);
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double& u) {

  // 将输入参数 u 限制在节点向量的合法范围内
  // ub 是限制后的 u，确保 u 在 [u(p_), u(m_ - p_)] 范围内
  double ub = min(max(u_(p_), u), u_(m_ - p_));

  // 确定参数 u 所在的节点区间 [u_k, u_k+1]
  int k = p_; // 初始化 k 为阶数 p_
  while (true) {
    if (u_(k + 1) >= ub) break; // 如果 ub 位于 [u_k, u_k+1]，则停止
    ++k; // 向右移动节点索引
  }

  /* 使用 de Boor 算法计算曲线位置 */
  vector<Eigen::VectorXd> d; // 定义用于存储 de Boor 算法的控制点序列
  for (int i = 0; i <= p_; ++i) {    //传入相关的控制点
    // 将与 u 相关的控制点加入 de Boor 算法的初始点集 d
    d.push_back(control_points_.row(k - p_ + i)); 
    // control_points_.row() 获取控制点矩阵中的一行
  }

  // 进行 de Boor 递推
  for (int r = 1; r <= p_; ++r) { // 递推阶数，从 1 到 p_
    for (int i = p_; i >= r; --i) { // 从 p_ 向前递推，更新控制点
      // 计算权重 alpha
      double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      // 使用 alpha 对控制点进行线性插值
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i]; // 更新 d[i]
    }
  }

  // 返回 de Boor 算法最后得到的点（曲线在 u 位置的点）
  return d[p_];
}


Eigen::VectorXd NonUniformBspline::evaluateDeBoorT(const double& t) {
  return evaluateDeBoor(t + u_(p_));
}

Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints() {
  // B样条的导数仍然是一个B样条，但其阶数会降低1
  // 导数样条的控制点计算公式：Qi = p_ * (Pi+1 - Pi) / (ui+p_+1 - ui+1)

  // 初始化导数的控制点矩阵，行数减少1（原控制点数-1），列数保持不变
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());

  // 遍历每一个导数控制点的位置
  for (int i = 0; i < ctp.rows(); ++i) {
    // 计算第i个导数控制点
    // p_ 是样条的阶数，(control_points_.row(i + 1) - control_points_.row(i)) 是相邻控制点的差
    // (u_(i + p_ + 1) - u_(i + 1)) 是相邻节点之间的间隔，用于归一化
    ctp.row(i) =
        p_ * (control_points_.row(i + 1) - control_points_.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
  }

  // 返回计算得到的导数控制点矩阵
  return ctp;
}


NonUniformBspline NonUniformBspline::getDerivative() {
  Eigen::MatrixXd   ctp = getDerivativeControlPoints();
  NonUniformBspline derivative(ctp, p_ - 1, interval_);

  /* cut the first and last knot */
  Eigen::VectorXd knot(u_.rows() - 2);
  knot = u_.segment(1, u_.rows() - 2);
  derivative.setKnot(knot);

  return derivative;
}

double NonUniformBspline::getInterval() { return interval_; }

void NonUniformBspline::setPhysicalLimits(const double& vel, const double& acc) {
  limit_vel_   = vel;
  limit_acc_   = acc;
  limit_ratio_ = 1.1;
}

bool NonUniformBspline::checkFeasibility(bool show) {
  // 检查非均匀B样条的可行性，包括速度和加速度是否超出限制
  bool fea = true; // 初始化可行性标志位，默认设为可行
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;

  Eigen::MatrixXd P = control_points_; // 获取B样条的控制点矩阵
  int dimension = control_points_.cols(); // 获取控制点的维度（例如3D轨迹则为3）

  /* 检查速度的可行性并计算最大速度 */
  double max_vel = -1.0; // 初始化最大速度
  for (int i = 0; i < P.rows() - 1; ++i) { // 遍历所有控制点之间的线段
    // 计算当前控制点之间的速度
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    // 检查速度是否超出限制（limit_vel_），如果超出则标记为不可行
    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {

      if (show) // 如果需要显示详细信息，则输出超限的速度信息
        cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
      fea = false; // 标记为不可行

      // 记录当前的最大速度
      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }
    }
  }

  /* 检查加速度的可行性 */
  double max_acc = -1.0; // 初始化最大加速度
  for (int i = 0; i < P.rows() - 2; ++i) { // 遍历所有控制点之间的加速度段
    // 计算当前控制点之间的加速度
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    // 检查加速度是否超出限制（limit_acc_），如果超出则标记为不可行
    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {

      if (show) // 如果需要显示详细信息，则输出超限的加速度信息
        cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
      fea = false; // 标记为不可行

      // 记录当前的最大加速度
      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }
    }
  }

  // 计算速度和加速度的比例，用于评估最大超限程度
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

  return fea; // 返回是否可行的标志位
}


double NonUniformBspline::checkRatio() {
  // 检查样条中速度和加速度的最大值，并计算它们相对于限制的比例
  Eigen::MatrixXd P = control_points_;  // 获取样条的控制点矩阵
  int dimension = control_points_.cols();  // 获取控制点的维度（例如3D轨迹则为3）

  // 找到最大速度
  double max_vel = -1.0;  // 初始化最大速度为一个负值
  for (int i = 0; i < P.rows() - 1; ++i) {  // 遍历所有相邻控制点对
    // 使用B样条的一阶导数公式计算速度
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    // 遍历每个维度，找到当前速度中的最大值
    for (int j = 0; j < dimension; ++j) {
      max_vel = max(max_vel, fabs(vel(j)));  // 更新最大速度
    }
  }

  // 找到最大加速度
  double max_acc = -1.0;  // 初始化最大加速度为一个负值
  for (int i = 0; i < P.rows() - 2; ++i) {  // 遍历所有相邻控制点对的加速度段
    // 使用B样条的二阶导数公式计算加速度
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    // 遍历每个维度，找到当前加速度中的最大值
    for (int j = 0; j < dimension; ++j) {
      max_acc = max(max_acc, fabs(acc(j)));  // 更新最大加速度
    }
  }

  // 计算速度和加速度的比例，取其中较大的值
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));
  // 如果比例超过2.0，则输出错误日志，提示速度或加速度超限
  ROS_ERROR_COND(ratio > 2.0, "max vel: %lf, max acc: %lf.", max_vel, max_acc);

  // 返回速度和加速度的最大比例
  return ratio;
}


bool NonUniformBspline::reallocateTime(bool show) {
  // 重新分配B样条的时间节点，确保轨迹满足速度和加速度的限制
  bool fea = true; // 初始化可行性标志，默认认为轨迹是可行的

  Eigen::MatrixXd P = control_points_; // 提取样条的控制点矩阵
  int dimension = control_points_.cols(); // 控制点的维度（例如3D空间则为3）

  double max_vel, max_acc; // 用于记录最大速度和最大加速度

  /* 检查速度的可行性，并调整时间节点 */
  for (int i = 0; i < P.rows() - 1; ++i) { // 遍历控制点之间的速度段
    // 使用B样条的一阶导数公式计算速度
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    // 如果速度超出限制
    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {
      fea = false; // 标记为不可行
      if (show) // 如果需要显示详细信息，则输出超限的速度信息
        cout << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose() << endl;

      max_vel = -1.0; // 初始化当前的最大速度
      for (int j = 0; j < dimension; ++j) { // 遍历每个维度，找到当前速度中的最大值
        max_vel = max(max_vel, fabs(vel(j)));
      }

      // 计算时间比例
      double ratio = max_vel / limit_vel_ + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_; // 限制最大调整比例

      // 重新分配时间
      double time_ori = u_(i + p_ + 1) - u_(i + 1); // 原始时间间隔
      double time_new = ratio * time_ori; // 按比例扩展时间间隔
      double delta_t = time_new - time_ori; // 时间调整量
      double t_inc = delta_t / double(p_); // 每个控制点的时间增量

      for (int j = i + 2; j <= i + p_ + 1; ++j) { // 更新当前段的时间节点
        u_(j) += double(j - i - 1) * t_inc;
      }

      for (int j = i + p_ + 2; j < u_.rows(); ++j) { // 更新后续时间节点
        u_(j) += delta_t;
      }
    }
  }

  /* 检查加速度的可行性，并调整时间节点 */
  for (int i = 0; i < P.rows() - 2; ++i) { // 遍历控制点之间的加速度段
    // 使用B样条的二阶导数公式计算加速度
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    // 如果加速度超出限制
    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {
      fea = false; // 标记为不可行
      if (show) // 如果需要显示详细信息，则输出超限的加速度信息
        cout << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose() << endl;

      max_acc = -1.0; // 初始化当前的最大加速度
      for (int j = 0; j < dimension; ++j) { // 遍历每个维度，找到当前加速度中的最大值
        max_acc = max(max_acc, fabs(acc(j)));
      }

      // 计算时间比例
      double ratio = sqrt(max_acc / limit_acc_) + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_; // 限制最大调整比例

      // 重新分配时间
      double time_ori = u_(i + p_ + 1) - u_(i + 2); // 原始时间间隔
      double time_new = ratio * time_ori; // 按比例扩展时间间隔
      double delta_t = time_new - time_ori; // 时间调整量
      std::cout << "-----------delta_t:----------- " << delta_t << std::endl;
      double t_inc = delta_t / double(p_ - 1); // 每个控制点的时间增量

      if (i == 1 || i == 2) { // 特殊情况的处理
        for (int j = 2; j <= 5; ++j) { // 更新当前段的时间节点
          u_(j) += double(j - 1) * t_inc;
        }

        for (int j = 6; j < u_.rows(); ++j) { // 更新后续时间节点
          u_(j) += 4.0 * t_inc;
        }
      } else {
        for (int j = i + 3; j <= i + p_ + 1; ++j) { // 更新当前段的时间节点
          u_(j) += double(j - i - 2) * t_inc;
        }

        for (int j = i + p_ + 2; j < u_.rows(); ++j) { // 更新后续时间节点
          u_(j) += delta_t;
        }
      }
    }
  }

  return fea; // 返回轨迹是否满足速度和加速度限制
}


void NonUniformBspline::lengthenTime(const double& ratio) {
  int num1 = 5;
  int num2 = getKnot().rows() - 1 - 5;

  double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
  double t_inc   = delta_t / double(num2 - num1);
  for (int i = num1 + 1; i <= num2; ++i) u_(i) += double(i - num1) * t_inc;
  for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;
}

void NonUniformBspline::recomputeInit() {}
          /*----------------------------------转化成为B样条的控制点---------------------------------*/
void NonUniformBspline::parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                              const vector<Eigen::Vector3d>& start_end_derivative,
                                              Eigen::MatrixXd&               ctrl_pts) {
  if (ts <= 0) {
    cout << "[B-spline]:time step error." << endl;
    return;
  }

  if (point_set.size() < 2) {
    cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    cout << "[B-spline]:derivatives error." << endl;
  }

  int K = point_set.size();

  // write A
  Eigen::Vector3d prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;
  vrow << -1, 0, 1;
  arow << 1, -2, 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

  for (int i = 0; i < K; ++i) 
  A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

  A.block(K, 0, 1, 3)         = (1 / 2.0 / ts) * vrow.transpose();
  A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

  A.block(K + 2, 0, 1, 3)     = (1 / ts / ts) * arow.transpose();
  A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();
  cout << "A:\n" << A << endl;

  // A.block(0, 0, K, K + 2) = (1 / 6.0) * A.block(0, 0, K, K + 2);
  // A.block(K, 0, 2, K + 2) = (1 / 2.0 / ts) * A.block(K, 0, 2, K + 2);
  // A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
  // A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

  // write b
  Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
  for (int i = 0; i < K; ++i) {
    bx(i) = point_set[i](0);
    by(i) = point_set[i](1);
    bz(i) = point_set[i](2);
  }

  for (int i = 0; i < 4; ++i) {
    bx(K + i) = start_end_derivative[i](0);
    by(K + i) = start_end_derivative[i](1);
    bz(K + i) = start_end_derivative[i](2);
  }

  // solve Ax = b
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // convert to control pts
  ctrl_pts.resize(K + 2, 3);
  ctrl_pts.col(0) = px;
  ctrl_pts.col(1) = py;
  ctrl_pts.col(2) = pz;

  cout << "[B-spline]: parameterization ok." << endl;//B样条控制点
}

double NonUniformBspline::getTimeSum() {
  double tm, tmp;
  getTimeSpan(tm, tmp);
  return tmp - tm;
}

double NonUniformBspline::getLength(const double& res) {
  double          length = 0.0;
  double          dur    = getTimeSum();
  Eigen::VectorXd p_l    = evaluateDeBoorT(0.0), p_n;
  for (double t = res; t <= dur + 1e-4; t += res) {
    p_n = evaluateDeBoorT(t);
    length += (p_n - p_l).norm();
    p_l = p_n;
  }
  return length;
}

double NonUniformBspline::getJerk() {
  NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

  Eigen::VectorXd times     = jerk_traj.getKnot();
  Eigen::MatrixXd ctrl_pts  = jerk_traj.getControlPoint();
  int             dimension = ctrl_pts.cols();

  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    for (int j = 0; j < dimension; ++j) {
      jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
    }
  }

  return jerk;
}

void NonUniformBspline::getMeanAndMaxVel(double& mean_v, double& max_v) {
  NonUniformBspline vel = getDerivative();
  double            tm, tmp;
  vel.getTimeSpan(tm, tmp);

  double max_vel = -1.0, mean_vel = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
    double          vn  = vxd.norm();

    mean_vel += vn;
    ++num;
    if (vn > max_vel) {
      max_vel = vn;
    }
  }

  mean_vel = mean_vel / double(num);
  mean_v   = mean_vel;
  max_v    = max_vel;
}

void NonUniformBspline::getMeanAndMaxAcc(double& mean_a, double& max_a) {
  NonUniformBspline acc = getDerivative().getDerivative();
  double            tm, tmp;
  acc.getTimeSpan(tm, tmp);

  double max_acc = -1.0, mean_acc = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd axd = acc.evaluateDeBoor(t);
    double          an  = axd.norm();

    mean_acc += an;
    ++num;
    if (an > max_acc) {
      max_acc = an;
    }
  }

  mean_acc = mean_acc / double(num);
  mean_a   = mean_acc;
  max_a    = max_acc;
}
}  // namespace fast_planner
