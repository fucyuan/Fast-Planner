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



#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
// using namespace std;

namespace fast_planner {

const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);
const int BsplineOptimizer::DISTANCE    = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT    = (1 << 3);
const int BsplineOptimizer::GUIDE       = (1 << 4);
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/lambda1", lambda1_, -1.0);
  nh.param("optimization/lambda2", lambda2_, -1.0);
  nh.param("optimization/lambda3", lambda3_, -1.0);
  nh.param("optimization/lambda4", lambda4_, -1.0);
  nh.param("optimization/lambda5", lambda5_, -1.0);
  nh.param("optimization/lambda6", lambda6_, -1.0);
  nh.param("optimization/lambda7", lambda7_, -1.0);
  nh.param("optimization/lambda8", lambda8_, -1.0);

  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/visib_min", visib_min_, -1.0);
  nh.param("optimization/dlmin", dlmin_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);

  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("optimization/order", order_, -1);
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";

  ROS_INFO_STREAM("cost func: " << cost_str);
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  setControlPoints(points);  // 设置控制点
  setBsplineInterval(ts);  // 设置B样条的时间间隔
  setCostFunction(cost_function);  // 设置代价函数类型
  setTerminateCond(max_num_id, max_time_id);  // 设置终止条件

  optimize();  // 执行优化
  return this->control_points_;  // 返回优化后的控制点
}

void BsplineOptimizer::optimize() {
  /* 初始化优化器 */
  iter_num_        = 0;  // 迭代次数初始化为0
  min_cost_        = std::numeric_limits<double>::max();  // 最小代价值初始化为正无穷
  const int pt_num = control_points_.rows();  // 控制点的数量
  g_q_.resize(pt_num);  // 初始化梯度存储
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {  // 如果包含终点代价
    variable_num_ = dim_ * (pt_num - order_);  // 变量数量计算
    // 终点位置用于硬约束
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    variable_num_ = max(0, dim_ * (pt_num - 2 * order_));  // 变量数量计算
  }

  /* 使用NLopt优化器进行优化 */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);  // 选择优化算法
  opt.set_min_objective(BsplineOptimizer::costFunction, this);  // 设置目标函数
  opt.set_maxeval(max_iteration_num_[max_num_id_]);  // 设置最大迭代次数
  opt.set_maxtime(max_iteration_time_[max_time_id_]);  // 设置最大优化时间
  opt.set_xtol_rel(1e-5);  // 设置相对误差容限

  vector<double> q(variable_num_);  // 初始化优化变量
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;  // 跳过未使用的变量
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);  // 初始化优化变量
    }
  }

  if (dim_ != 1) {  // 如果不是1维情况
    vector<double> lb(variable_num_), ub(variable_num_);  // 定义上下界
    const double   bound = 10.0;  // 设置边界值
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;  // 设置下界
      ub[i] = q[i] + bound;  // 设置上界
    }
    opt.set_lower_bounds(lb);  // 应用下界
    opt.set_upper_bounds(ub);  // 应用上界
  }

  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();

    double        final_cost;  // 最终代价
    nlopt::result result = opt.optimize(q, final_cost);  // 执行优化

    /* 检索优化结果 */
    // cout << "Min cost:" << min_cost_ << endl;
  } catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");  // 捕获异常
    cout << e.what() << endl;  // 打印异常信息
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;  // 跳过未使用的变量
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];  // 更新控制点
    }
  }

  if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("iter num: " << iter_num_);  // 输出迭代次数（如果不包含GUIDE约束）
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  // 初始化平滑代价为0
  cost = 0.0;

  // 定义一个零向量用于初始化梯度
  Eigen::Vector3d zero(0, 0, 0);

  // 将梯度初始化为零向量
  std::fill(gradient.begin(), gradient.end(), zero);

  // 定义jerk（抖动）和临时变量temp_j，用于计算梯度
  Eigen::Vector3d jerk, temp_j;

  // 遍历所有的点，计算jerk和对应的平滑代价与梯度
  for (int i = 0; i < q.size() - order_; i++) { // 遍历所有适合的段
    /* 计算jerk值 */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i]; // jerk公式
    cost += jerk.squaredNorm(); // jerk平方和作为代价的一部分

    // 计算梯度临时变量
    temp_j = 2.0 * jerk;

    /* 计算jerk的梯度，逐点累加到相应控制点的梯度上 */
    gradient[i + 0] += -temp_j;          // 第i个控制点的梯度
    gradient[i + 1] += 3.0 * temp_j;     // 第i+1个控制点的梯度
    gradient[i + 2] += -3.0 * temp_j;    // 第i+2个控制点的梯度
    gradient[i + 3] += temp_j;           // 第i+3个控制点的梯度
  }
}


void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0; // 初始化代价为0
  Eigen::Vector3d zero(0, 0, 0); // 初始化梯度为零向量
  std::fill(gradient.begin(), gradient.end(), zero); // 将所有梯度初始化为零向量

  double          dist; // 保存当前点到障碍物的距离
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0); // 保存当前点到障碍物距离的梯度向量

  // 计算的终止索引。如果启用 ENDPOINT，则处理到末尾，否则只处理到倒数第 order_ 个点
  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  // 遍历轨迹控制点，跳过前 order_ 个点
  for (int i = order_; i < end_idx; i++) {
    // 通过距离场（EDT）计算当前点到障碍物的距离和梯度
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);

    // 如果梯度的模长足够大，则对其进行归一化
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    // 如果距离小于阈值 dist0_，即点太靠近障碍物，则计算距离代价和梯度
    if (dist < dist0_) {
      // 增加代价：距离小于阈值的平方差
      cost += pow(dist - dist0_, 2);

      // 增加梯度：2 * (dist - dist0_) * 距离梯度
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}


void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  cost = 0.0; // 初始化总的可行性代价
  Eigen::Vector3d zero(0, 0, 0); // 零向量，用于初始化梯度
  std::fill(gradient.begin(), gradient.end(), zero); // 将梯度向量初始化为零

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_; // 最大速度的平方
  am2 = max_acc_ * max_acc_; // 最大加速度的平方

  ts      = bspline_interval_; // B样条的时间间隔
  ts_inv2 = 1 / ts / ts; // 时间间隔平方的倒数
  ts_inv4 = ts_inv2 * ts_inv2; // 时间间隔四次方的倒数

  /* velocity feasibility */
  for (int i = 0; i < q.size() - 1; i++) { // 遍历控制点，计算速度约束代价
    Eigen::Vector3d vi = q[i + 1] - q[i]; // 控制点之间的速度向量

    for (int j = 0; j < 3; j++) { // 遍历速度向量的每个分量
      double vd = vi(j) * vi(j) * ts_inv2 - vm2; // 当前速度分量的平方与最大速度平方的差值
      if (vd > 0.0) { // 如果当前速度超过了限制
        cost += pow(vd, 2); // 累加速度约束的代价

        double temp_v = 4.0 * vd * ts_inv2; // 速度代价对控制点梯度的缩放因子
        gradient[i + 0](j) += -temp_v * vi(j); // 更新前一个控制点的梯度
        gradient[i + 1](j) += temp_v * vi(j); // 更新后一个控制点的梯度
      }
    }
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.size() - 2; i++) { // 遍历控制点，计算加速度约束代价
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i]; // 控制点之间的加速度向量

    for (int j = 0; j < 3; j++) { // 遍历加速度向量的每个分量
      double ad = ai(j) * ai(j) * ts_inv4 - am2; // 当前加速度分量的平方与最大加速度平方的差值
      if (ad > 0.0) { // 如果当前加速度超过了限制
        cost += pow(ad, 2); // 累加加速度约束的代价

        double temp_a = 4.0 * ad * ts_inv4; // 加速度代价对控制点梯度的缩放因子
        gradient[i + 0](j) += temp_a * ai(j); // 更新第一个控制点的梯度
        gradient[i + 1](j) += -2 * temp_a * ai(j); // 更新第二个控制点的梯度
        gradient[i + 2](j) += temp_a * ai(j); // 更新第三个控制点的梯度
      }
    }
  }
}


void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* 将NLopt格式的向量转换为控制点 */

  // 此优化器支持1D到3D的B样条优化，但我们使用Vector3d来存储每个控制点。
  // 对于1D的情况，第二和第三个元素为零，对于2D的情况类似。
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);  // 初始化前order_个控制点
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;  // 将未使用的维度置为0
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];  // 将变量转换为控制点
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;  // 多余维度置为0
    }
  }

  if (!(cost_function_ & ENDPOINT)) {  // 如果未启用终点约束
    for (int i = 0; i < order_; i++) {
      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);  // 初始化后order_个控制点
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;  // 多余维度置为0
      }
    }
  }

  f_combine = 0.0;  // 初始化组合代价
  grad.resize(variable_num_);  // 调整梯度向量大小
  fill(grad.begin(), grad.end(), 0.0);  // 将梯度向量初始化为0

  /* 计算代价及其梯度 */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

  if (cost_function_ & SMOOTHNESS) {  // 如果启用了平滑度代价
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);  // 计算平滑度代价
    f_combine += lambda1_ * f_smoothness;  // 加入总代价
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);  // 累加梯度
  }
  if (cost_function_ & DISTANCE) {  // 如果启用了距离代价
    calcDistanceCost(g_q_, f_distance, g_distance_);  // 计算距离代价
    f_combine += lambda2_ * f_distance;  // 加入总代价
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);  // 累加梯度
  }
  if (cost_function_ & FEASIBILITY) {  // 如果启用了可行性代价
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);  // 计算可行性代价
    f_combine += lambda3_ * f_feasibility;  // 加入总代价
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);  // 累加梯度
  }
  if (cost_function_ & ENDPOINT) {  // 如果启用了终点代价
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);  // 计算终点代价
    f_combine += lambda4_ * f_endpoint;  // 加入总代价
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);  // 累加梯度
  }
  if (cost_function_ & GUIDE) {  // 如果启用了引导代价
    calcGuideCost(g_q_, f_guide, g_guide_);  // 计算引导代价
    f_combine += lambda5_ * f_guide;  // 加入总代价
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);  // 累加梯度
  }
  if (cost_function_ & WAYPOINTS) {  // 如果启用了航点代价
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);  // 计算航点代价
    f_combine += lambda7_ * f_waypoints;  // 加入总代价
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);  // 累加梯度
  }

  /* 打印代价（可选部分） */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

}  // namespace fast_planner