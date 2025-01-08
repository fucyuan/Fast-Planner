#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

// 头文件保护，防止重复包含

// #include <path_searching/matrix_hash.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include "plan_env/edt_environment.h"

namespace fast_planner {
// 命名空间 `fast_planner`，避免命名冲突

// 状态定义
#define IN_CLOSE_SET 'a' // 在关闭集合中
#define IN_OPEN_SET 'b'  // 在开放集合中
#define NOT_EXPAND 'c'   // 未展开
#define inf 1 >> 30      // 定义一个近似无穷大的值

// 路径节点类
class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector3i index;                          // 节点索引
  Eigen::Matrix<double, 6, 1> state;             // 节点状态向量
  double g_score, f_score;                       // 节点的g值和f值
  Eigen::Vector3d input;                         // 输入值（控制量）
  double duration;                               // 时间跨度
  double time;                                   // 动态时间
  int time_idx;                                  // 时间索引
  PathNode* parent;                              // 父节点指针
  char node_state;                               // 节点状态

  /* -------------------- */
  PathNode() {                                   // 构造函数
    parent = NULL;                               // 初始化父节点为空
    node_state = NOT_EXPAND;                     // 初始化状态为未展开
  }
  ~PathNode(){};                                 // 析构函数
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW                // 确保对齐以兼容Eigen
};

typedef PathNode* PathNodePtr;                  // 定义节点指针类型

// 节点比较器类，用于优先队列排序
class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score > node2->f_score;      // 按f值从小到大排序
  }
};

// 哈希函数模板，适配Eigen矩阵
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);                      // 哈希计算公式
    }
    return seed;
  }
};

// 节点哈希表类
class NodeHashTable {
 private:
  /* 数据成员 */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;                                  // 三维哈希表
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;                                  // 四维哈希表

 public:
  NodeHashTable(/* args */) {}                   // 构造函数
  ~NodeHashTable() {}                            // 析构函数
  
  void insert(Eigen::Vector3i idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(idx, node));  // 插入三维索引节点
  }

  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));  // 插入四维索引节点
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;  // 查找三维索引节点
  }

  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;  // 查找四维索引节点
  }

  void clear() {
    data_3d_.clear();                            // 清空三维哈希表
    data_4d_.clear();                            // 清空四维哈希表
  }
};

// 动力学A*算法类
class KinodynamicAstar {
 private:
  /* ---------- 主数据结构 ---------- */
  vector<PathNodePtr> path_node_pool_;           // 节点池
  int use_node_num_, iter_num_;                  // 已用节点数和迭代次数
  NodeHashTable expanded_nodes_;                 // 展开节点的哈希表
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;                                 // 优先队列（开放集合）
  std::vector<PathNodePtr> path_nodes_;          // 路径节点列表

  /* ---------- 记录数据 ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;  // 起始速度、结束速度和起始加速度
  Eigen::Matrix<double, 6, 6> phi_;                  // 状态转移矩阵
  EDTEnvironment::Ptr edt_environment_;             // EDT环境指针
  bool is_shot_succ_ = false;                        // 是否射击成功
  Eigen::MatrixXd coef_shot_;                        // 射击轨迹系数
  double t_shot_;                                    // 射击时间
  bool has_path_ = false;                            // 是否找到路径

  /* ---------- 参数 ---------- */
  /* 搜索参数 */
  double max_tau_, init_max_tau_;                    // 最大时间跨度和初始时间跨度
  double max_vel_, max_acc_;                         // 最大速度和加速度
  double w_time_, horizon_, lambda_heu_;             // 权重、搜索范围、启发式参数
  int allocate_num_, check_num_;                     // 分配数量和检查数量
  double tie_breaker_;                               // 打破平局的因子
  bool optimistic_;                                  // 是否乐观搜索

  /* 地图参数 */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;  // 分辨率和倒数
  Eigen::Vector3d origin_, map_size_3d_;             // 原点和地图大小
  double time_origin_;                               // 时间原点

  /* 辅助函数 */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);   // 坐标转换为索引
  int timeToIndex(double time);                     // 时间转换为索引
  void retrievePath(PathNodePtr end_node);          // 获取路径

  /* 射击轨迹 */
  vector<double> cubic(double a, double b, double c, double d);  // 三次方程解法
  vector<double> quartic(double a, double b, double c, double d, double e); // 四次方程解法
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);  // 计算射击轨迹
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double& optimal_time);  // 估算启发式

  /* 状态传播 */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                    double tau);  // 状态传播函数

 public:
  KinodynamicAstar(){};                              // 构造函数
  ~KinodynamicAstar();                               // 析构函数

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 }; // 搜索结果枚举

  /* 主API接口 */
  void setParam(ros::NodeHandle& nh);               // 设置参数
  void init();                                      // 初始化
  void reset();                                     // 重置
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
             Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);            // 搜索函数

  void setEnvironment(const EDTEnvironment::Ptr& env);  // 设置环境

  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t); // 获取动力学轨迹

  void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                  vector<Eigen::Vector3d>& start_end_derivatives); // 获取采样点

  std::vector<PathNodePtr> getVisitedNodes();      // 获取访问过的节点

  typedef shared_ptr<KinodynamicAstar> Ptr;        // 智能指针类型

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW                  // 确保Eigen兼容对齐
};

}  // namespace fast_planner

#endif
