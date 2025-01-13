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

#include <path_searching/kinodynamic_astar.h>
#include <sstream>
#include <plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{
  // 初始化起始速度和加速度
  start_vel_ = start_v;
  start_acc_ = start_a;

  // 从节点池中获取当前节点并初始化
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;//head(3)返回前3个元素，tail(3)返回后3个元素
  cur_node->state.tail(3) = start_v;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  // 初始化目标状态和索引
  Eigen::VectorXd end_state(6);// 状态向量
  Eigen::Vector3i end_index;
  double time_to_goal;// 到达目标点的时间

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  if (dynamic)
  {
    // 动态搜索时初始化时间信息
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);

  // 初始化变量
  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);// 容差

  while (!open_set_.empty())
  {
    cur_node = open_set_.top();

    // 检查是否达到终止条件
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;

    if (reach_horizon || near_end) // 如果到达搜索的边界或者接近终止点
    {
      terminate_node = cur_node;                     // 设置终止节点为当前节点
      retrievePath(terminate_node);                  // 回溯路径，获取从起点到当前节点的路径

      if (near_end)                                  // 如果接近目标点
      {
        estimateHeuristic(cur_node->state, end_state, time_to_goal);  // 估算从当前节点到目标点的时间
        computeShotTraj(cur_node->state, end_state, time_to_goal);    // 计算“射击轨迹”（直接到达目标点的轨迹）
        if (init_search)                              // 如果是初始搜索阶段
          ROS_ERROR("Shot in first search loop!");   // 打印调试信息，表明第一轮搜索就找到直接通向目标的轨迹
      }
    }

    if (reach_horizon)
    {
      if (is_shot_succ_)  // 如果射击轨迹成功
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;  // 返回表示搜索到达终点的状态
      }
      else
      {
        std::cout << "reach horizon" << std::endl;
        return REACH_HORIZON;  // 返回表示搜索到达边界的状态
      }
    }

    if (near_end)
    {
      if (is_shot_succ_)  // 如果射击轨迹成功
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;  // 返回表示搜索到达终点的状态
      }
      else if (cur_node->parent != NULL)  // 当前节点有父节点（路径有效）
      {
        std::cout << "near end" << std::endl;
        return NEAR_END;  // 返回表示接近终点但未完全到达
      }
      else  // 当前节点无父节点，无法找到路径
      {
        std::cout << "no path" << std::endl;
        return NO_PATH;  // 返回表示搜索失败
      }
    }

    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    // 初始化分辨率和时间分辨率
    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;
    if (init_search) {
      inputs.push_back(start_acc_); // 将起始状态的加速度存入控制输入集合
      for (double tau = time_res_init * 
        init_max_tau_; tau <= init_max_tau_ + 1e-3; tau += time_res_init * init_max_tau_)
          durations.push_back(tau); // 生成时间步长的离散化值
      init_search = false; // 标志位设为 false，表示初始化完成
    }

    else {
    for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
            for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res) {
                um << ax, ay, az; // 生成三维加速度向量
                inputs.push_back(um); // 将加速度存入控制输入集合
            }
    for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau); // 生成时间步长的离散化值
    }


    // 遍历所有输入和时间组合
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        Eigen::Vector3d pro_pos = pro_state.head(3);

        // 检查是否在关闭集合中
        Eigen::Vector3i pro_id = posToIndex(pro_pos);
        int pro_t_id = timeToIndex(pro_t);
        // PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
       // 判断是否是动态搜索
        PathNodePtr pro_node = nullptr;
        if (dynamic) {
            // 动态场景，需要使用位置和时间索引
            pro_node = expanded_nodes_.find(pro_id, pro_t_id);
        } else {
            // 静态场景，只需要使用位置索引
            pro_node = expanded_nodes_.find(pro_id);
        }

        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          continue;
        }

        // 检查速度是否超限
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
          if (init_search)
            std::cout << "vel" << std::endl;
          continue;
        }

        // 检查是否在同一体素中
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)
            std::cout << "same" << std::endl;
          continue;
        }

        // 检查安全性
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);
          if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1 )
          {
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }

        // 计算启发式和代价函数
        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);
        
        
        /* -------------------剪枝操作-------------------- */


        // 比较同一父节点展开的节点
      bool prune = false; // 标记是否需要对当前节点进行剪枝
      for (int j = 0; j < tmp_expand_nodes.size(); ++j) // 遍历临时扩展节点列表
      {
        PathNodePtr expand_node = tmp_expand_nodes[j]; // 获取临时扩展节点
        if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx)) 
        {
          // 检查当前扩展节点是否与临时扩展节点在同一体素内（位置相同，动态模式下时间索引也需相同）
          prune = true; // 如果在同一体素内，标记需要剪枝
          if (tmp_f_score < expand_node->f_score) 
          {
            // 如果当前节点的总代价小于已存在节点的总代价，更新已存在节点的信息
            expand_node->f_score = tmp_f_score; // 更新总代价
            expand_node->g_score = tmp_g_score; // 更新路径代价
            expand_node->state = pro_state;     // 更新状态
            expand_node->input = um;           // 更新控制输入
            expand_node->duration = tau;       // 更新时间步长
            if (dynamic)
              expand_node->time = cur_node->time + tau; // 如果是动态规划，更新时间戳
          }
          break; // 找到相同节点后直接退出循环
        }
      }

      // 如果当前节点不需要剪枝（即不与已有节点在同一体素内）
      if (!prune) 
      {
        if (pro_node == NULL) // 如果当前节点是一个全新的节点
        {
          pro_node = path_node_pool_[use_node_num_]; // 从节点池中分配一个新节点
          pro_node->index = pro_id;                 // 设置节点位置索引
          pro_node->state = pro_state;              // 设置节点状态
          pro_node->f_score = tmp_f_score;          // 设置总代价
          pro_node->g_score = tmp_g_score;          // 设置路径代价
          pro_node->input = um;                     // 设置控制输入
          pro_node->duration = tau;                 // 设置时间步长
          pro_node->parent = cur_node;              // 设置父节点
          pro_node->node_state = IN_OPEN_SET;       // 将节点标记为开放状态
          if (dynamic) 
          {
            pro_node->time = cur_node->time + tau;      // 如果是动态规划，设置时间戳
            pro_node->time_idx = timeToIndex(pro_node->time); // 设置时间索引
          }
          open_set_.push(pro_node); // 将新节点加入开放集合

          if (dynamic)
            expanded_nodes_.insert(pro_id, pro_node->time, pro_node); // 动态模式下记录 (位置, 时间) 节点
          else
            expanded_nodes_.insert(pro_id, pro_node); // 静态模式下仅记录位置

          tmp_expand_nodes.push_back(pro_node); // 将新节点加入临时扩展节点列表

          use_node_num_ += 1; // 已使用节点数量加 1
          if (use_node_num_ == allocate_num_) 
          {
            // 如果超出节点池的容量，输出错误信息并退出
            cout << "run out of memory." << endl;
            return NO_PATH;
          }
        } 
        else if (pro_node->node_state == IN_OPEN_SET) // 如果当前节点已经在开放集合中
        {
          if (tmp_g_score < pro_node->g_score) 
          {
            // 如果当前路径代价更小，更新开放集合中的节点信息
            pro_node->state = pro_state;        // 更新状态
            pro_node->f_score = tmp_f_score;    // 更新总代价
            pro_node->g_score = tmp_g_score;    // 更新路径代价
            pro_node->input = um;               // 更新控制输入
            pro_node->duration = tau;           // 更新时间步长
            pro_node->parent = cur_node;        // 更新父节点
            if (dynamic)
              pro_node->time = cur_node->time + tau; // 如果是动态规划，更新时间戳
          }
        } 
        else 
        {
          // 如果当前节点状态既不在开放集合中也不为全新节点，报告错误
          cout << "error type in searching: " << pro_node->node_state << endl;
        }
      }

      }
    // init_search = false;
  }

  cout << "open set empty, no path!" << endl;// open set为空，没有路径
  cout << "use node num: " << use_node_num_ << endl;// 使用节点数量
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}
void KinodynamicAstar::setParam(ros::NodeHandle& nh)
{
  // 从参数服务器加载最大规划时间步长 tau
  nh.param("search/max_tau", max_tau_, -1.0);
  // 从参数服务器加载初始化的最大规划时间步长 tau
  nh.param("search/init_max_tau", init_max_tau_, -1.0);
  // 从参数服务器加载最大速度限制
  nh.param("search/max_vel", max_vel_, -1.0);
  // 从参数服务器加载最大加速度限制
  nh.param("search/max_acc", max_acc_, -1.0);
  // 从参数服务器加载时间代价权重
  nh.param("search/w_time", w_time_, -1.0);
  // 从参数服务器加载规划的前视范围（horizon）
  nh.param("search/horizon", horizon_, -1.0);
  // 从参数服务器加载 A* 搜索的空间分辨率
  nh.param("search/resolution_astar", resolution_, -1.0);
  // 从参数服务器加载时间分辨率
  nh.param("search/time_resolution", time_resolution_, -1.0);
  // 从参数服务器加载启发式函数的权重系数
  nh.param("search/lambda_heu", lambda_heu_, -1.0);
  // 从参数服务器加载分配的节点池大小
  nh.param("search/allocate_num", allocate_num_, -1);
  // 从参数服务器加载路径安全性检查的离散点数量
  nh.param("search/check_num", check_num_, -1);
  // 从参数服务器加载是否使用乐观规划策略
  nh.param("search/optimistic", optimistic_, true);
  // 打破平局的权重因子，默认为略大于 1
  tie_breaker_ = 1.0 + 1.0 / 10000;
  // 从参数服务器加载速度裕量（为保证速度安全性引入的偏移量）
  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);
  // 更新最大速度限制，将速度裕量加到最大速度中
  max_vel_ += vel_margin;
}


void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}
double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  // 计算状态差值，dp 为目标点与当前点之间的位移向量
  const Vector3d dp = x2.head(3) - x1.head(3);
  // 提取起点和终点的速度向量
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  // 多项式的系数计算
  double c1 = -36 * dp.dot(dp);                      // c1 表示与位置平方相关的系数
  double c2 = 24 * (v0 + v1).dot(dp);               // c2 表示速度与位置相关的系数
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));  // c3 表示速度平方相关的系数
  double c4 = 0;                                    // c4 为零（不影响结果）
  double c5 = w_time_;                              // c5 表示时间权重系数

  // 求解四次方程，得到时间解集
  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  // 计算一个初步的时间下界 t_bar，以确保解的合理性
  double v_max = max_vel_ * 0.5;                    // 采用最大速度的一半作为参考
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;  // 使用无穷范数计算时间下界
  ts.push_back(t_bar);                              // 将 t_bar 加入时间集合

  double cost = 100000000;                          // 初始化代价为一个很大的值
  double t_d = t_bar;                               // 初始化最佳时间为 t_bar

  // 遍历所有时间解，计算每个时间下的代价并选择最优解
  for (auto t : ts)
  {
    if (t < t_bar)                                  // 如果时间小于下界，则跳过
      continue;
    // 计算当前时间 t 对应的代价函数值
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)                                   // 如果代价更小，则更新最优代价和时间
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;                               // 将最优时间赋值给输出参数

  return 1.0 * (1 + tie_breaker_) * cost;           // 返回最终代价，考虑到打破平局的权重
}


bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
        coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
    {
      return false;
    }

    // if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
    //   return false;
    // }
    if (edt_environment_->sdf_map_->getInflateOccupancy(coord) == 1)
    {
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

void KinodynamicAstar::init()
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;
}

void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_environment_ = env;
}

void KinodynamicAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
  // 定义存储轨迹点的列表
  vector<Vector3d> state_list;

  /* ---------- 获取搜索路径的轨迹 ---------- */
  PathNodePtr node = path_nodes_.back(); // 从路径节点列表的末尾开始回溯（即目标点）
  Matrix<double, 6, 1> x0, xt;           // 定义状态向量 x0（初始状态）和 xt（当前状态）

  while (node->parent != NULL) // 回溯路径，直到起点
  {
    Vector3d ut = node->input;      // 获取当前节点的控制输入（加速度或其他输入）
    double duration = node->duration; // 获取当前节点到父节点的时间步长
    x0 = node->parent->state;       // 获取父节点的状态作为初始状态

    for (double t = duration; t >= -1e-5; t -= delta_t) // 按时间间隔 delta_t 从后向前进行积分
    {
      stateTransit(x0, xt, ut, t); // 通过前向积分计算时间 t 时的状态 xt
      state_list.push_back(xt.head(3)); // 保存状态中的位置（3D 坐标）
    }
    node = node->parent; // 移动到父节点，继续回溯路径
  }

  reverse(state_list.begin(), state_list.end()); // 反转轨迹列表，使轨迹从起点到终点

  /* ---------- 获取 one-shot 部分的轨迹 ---------- */
  if (is_shot_succ_) // 如果 one-shot 部分（直达目标的轨迹）成功
  {
    Vector3d coord;   // 存储当前时刻的 3D 坐标
    VectorXd poly1d;  // 存储一维多项式系数
    VectorXd time(4); // 定义时间的幂次项

    for (double t = delta_t; t <= t_shot_; t += delta_t) // 按时间间隔 delta_t 遍历 one-shot 时间段
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j); // 计算时间的幂次项（1, t, t^2, t^3）

      for (int dim = 0; dim < 3; dim++) // 遍历 x、y、z 三个维度
      {
        poly1d = coef_shot_.row(dim);     // 获取对应维度的多项式系数
        coord(dim) = poly1d.dot(time);    // 通过多项式计算当前时刻的坐标
      }
      state_list.push_back(coord); // 保存计算的 3D 坐标
    }
  }

  return state_list; // 返回轨迹点列表
}


void KinodynamicAstar::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives)
{
  /* ---------- 路径总持续时间 ---------- */
  double T_sum = 0.0; // 初始化总路径时间
  if (is_shot_succ_)   // 如果存在 one-shot 轨迹
    T_sum += t_shot_;  // 将 one-shot 轨迹的时间加入总时间

  // 计算搜索路径的总时间
  PathNodePtr node = path_nodes_.back(); // 从路径的最后一个节点开始回溯
  while (node->parent != NULL) // 回溯路径，累加每一段的持续时间
  {
    T_sum += node->duration;  // 累加当前节点到父节点的时间
    node = node->parent;      // 移动到父节点
  }
  // cout << "duration:" << T_sum << endl;

  /* ---------- 计算路径边界条件 ---------- */
  Eigen::Vector3d end_vel, end_acc; // 定义终点速度和加速度
  double t;                         // 当前时间
  if (is_shot_succ_) // 如果存在 one-shot 轨迹
  {
    t = t_shot_;         // 初始化时间为 one-shot 的时间
    end_vel = end_vel_;  // 终点速度即为目标速度
    for (int dim = 0; dim < 3; ++dim) // 计算终点加速度
    {
      Vector4d coe = coef_shot_.row(dim);              // 获取对应维度的多项式系数
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_; // 加速度公式：2 * a2 + 6 * a3 * t
    }
  }
  else // 如果没有 one-shot 轨迹
  {
    t = path_nodes_.back()->duration; // 终点时间为路径最后一段的持续时间
    end_vel = node->state.tail(3);    // 终点速度为最后节点的速度
    end_acc = path_nodes_.back()->input; // 终点加速度为最后节点的输入
  }

  /* ---------- 获取轨迹采样点 ---------- */
  int seg_num = floor(T_sum / ts);      // 根据时间间隔计算采样段数
  seg_num = max(8, seg_num);            // 最小采样段数为 8
  ts = T_sum / double(seg_num);         // 更新时间间隔
  bool sample_shot_traj = is_shot_succ_; // 标志是否采样 one-shot 轨迹
  node = path_nodes_.back();            // 从路径的最后一个节点开始

  for (double ti = T_sum; ti > -1e-5; ti -= ts) // 从总时间开始，按照时间间隔采样
  {
    if (sample_shot_traj) // 如果采样的是 one-shot 轨迹
    {
      // 采样 one-shot 轨迹
      Vector3d coord;     // 当前采样点坐标
      Vector4d poly1d, time; // 一维多项式系数和时间幂次项

      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j); // 计算时间的幂次项（1, t, t^2, t^3）

      for (int dim = 0; dim < 3; dim++) // 遍历 x, y, z 三个维度
      {
        poly1d = coef_shot_.row(dim);  // 获取多项式系数
        coord(dim) = poly1d.dot(time); // 计算当前时刻的坐标
      }

      point_set.push_back(coord); // 将采样点加入轨迹点集合
      t -= ts;                    // 时间减少一个间隔

      /* 如果 one-shot 部分采样完成，切换到搜索路径采样 */
      if (t < -1e-5)
      {
        sample_shot_traj = false;    // 标志切换到搜索路径采样
        if (node->parent != NULL)    // 如果当前节点有父节点
          t += node->duration;       // 时间调整为搜索路径的持续时间
      }
    }
    else // 如果采样的是搜索路径
    {
      // 采样搜索路径
      Eigen::Matrix<double, 6, 1> x0 = node->parent->state; // 获取父节点的状态
      Eigen::Matrix<double, 6, 1> xt;                      // 当前状态
      Vector3d ut = node->input;                           // 当前节点的输入

      stateTransit(x0, xt, ut, t); // 前向积分，计算当前时刻的状态

      point_set.push_back(xt.head(3)); // 保存位置信息
      t -= ts;                         // 时间减少一个间隔

      /* 如果搜索路径段采样完成，切换到上一段路径 */
      if (t < -1e-5 && node->parent->parent != NULL)
      {
        node = node->parent;         // 切换到父节点
        t += node->duration;         // 时间调整为上一段路径的持续时间
      }
    }
  }
  reverse(point_set.begin(), point_set.end()); // 反转采样点集合，使轨迹从起点到终点

  /* ---------- 计算起点和终点的导数信息（速度和加速度） ---------- */
  Eigen::Vector3d start_acc; // 定义起点加速度
  if (path_nodes_.back()->parent == NULL) // 如果没有搜索路径，只有 one-shot 轨迹
  {
    start_acc = 2 * coef_shot_.col(2); // 通过 one-shot 轨迹的二次系数计算加速度
  }
  else // 如果有搜索路径
  {
    start_acc = node->input; // 起点加速度为第一个节点的输入
  }

  // 将起点和终点的导数信息加入结果
  start_end_derivatives.push_back(start_vel_); // 起点速度
  start_end_derivatives.push_back(end_vel);    // 终点速度
  start_end_derivatives.push_back(start_acc);  // 起点加速度
  start_end_derivatives.push_back(end_acc);    // 终点加速度
}


std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
  // origin_(1)) * inv_resolution_),
  //     floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

}  // namespace fast_planner
