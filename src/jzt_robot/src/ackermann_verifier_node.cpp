#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>
#include <algorithm>

class AckermannVerifier : public rclcpp::Node
{
public:
  AckermannVerifier() : Node("ackermann_verifier")
  {
    // ========== 底盘几何参数 ==========
    this->declare_parameter<double>("wheelbase", 0.8);             // 轴距 L
    this->declare_parameter<double>("track_width", 0.56);          // 轮距 T (m)
    this->declare_parameter<double>("max_steering_angle", 0.5236); // 最大转向角 rad (默认30°)
    this->declare_parameter<std::string>("left_steer_joint", "front_left_steer_joint");
    this->declare_parameter<std::string>("right_steer_joint", "front_right_steer_joint");

    // 诊断参数
    this->declare_parameter<double>("v_tolerance", 0.05);
    this->declare_parameter<double>("w_tolerance", 0.1);
    this->declare_parameter<double>("steer_tolerance_deg", 3.0);

    this->get_parameter("wheelbase", L_);
    this->get_parameter("track_width", T_);
    this->get_parameter("max_steering_angle", max_steer_);
    this->get_parameter("left_steer_joint", left_joint_name_);
    this->get_parameter("right_steer_joint", right_joint_name_);
    this->get_parameter("v_tolerance", v_tol_);
    this->get_parameter("w_tolerance", w_tol_);
    this->get_parameter("steer_tolerance_deg", steer_tol_deg_);

    RCLCPP_INFO(this->get_logger(),
                "Ackermann Verifier (Precise): L=%.2fm, T=%.2fm, max_steer=%.1f°",
                L_, T_, max_steer_ * 180.0 / M_PI);

    // 订阅
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/ackermann_steering_controller/reference_unstamped", 10,
        std::bind(&AckermannVerifier::cmdCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&AckermannVerifier::odomCallback, this, std::placeholders::_1));

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&AckermannVerifier::jointCallback, this, std::placeholders::_1));

    // 验证定时器（5Hz）
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&AckermannVerifier::verifyLoop, this));
  }

private:
  // 底盘参数
  double L_;         // 轴距
  double T_;         // 轮距
  double max_steer_; // 最大转角

  // 关节名
  std::string left_joint_name_;
  std::string right_joint_name_;

  // 最新指令
  double v_cmd_ = 0.0, w_cmd_ = 0.0;

  // 真实速度
  double v_real_ = 0.0, w_real_ = 0.0;

  // 左右前轮转角（单位：rad）
  double steer_left_ = 0.0;
  double steer_right_ = 0.0;
  bool has_left_ = false;
  bool has_right_ = false;

  // 诊断阈值
  double v_tol_, w_tol_, steer_tol_deg_;

  // 订阅与定时器
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ========== 回调 ==========
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    v_cmd_ = msg->linear.x;
    w_cmd_ = msg->angular.z;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    v_real_ = msg->twist.twist.linear.x;
    w_real_ = msg->twist.twist.angular.z;
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 重置标志
    has_left_ = false;
    has_right_ = false;

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == left_joint_name_)
      {
        steer_left_ = msg->position[i];
        has_left_ = true;
      }
      if (msg->name[i] == right_joint_name_)
      {
        steer_right_ = msg->position[i];
        has_right_ = true;
      }
    }
  }

  // ========== 精确几何模型核心 ==========
  /**
   * 从左右前轮转角计算精确等效转向角 δ_eff
   * 公式：
   *   α_L = atan( L / (R - T/2) )  →  R = L/tan(α_L) + T/2
   *   α_R = atan( L / (R + T/2) )  →  R = L/tan(α_R) - T/2
   *   δ_eff = atan( L / R_avg )
   * 若只有一个关节可用，则降级为单关节反算。
   * 若都没有，返回 0.0 并置无效。
   */
  double computePreciseSteering() const
  {
    if (has_left_ && has_right_)
    {
      double aL = steer_left_;
      double aR = steer_right_;

      // 处理零转角情况（直线行驶）
      if (std::abs(aL) < 1e-6 && std::abs(aR) < 1e-6)
        return 0.0;

      // 防止除零
      double inv_tanL = std::tan(aL);
      double inv_tanR = std::tan(aR);
      if (std::abs(inv_tanL) < 1e-6 || std::abs(inv_tanR) < 1e-6)
      {
        // 某一侧转角接近0，使用另一侧估算
        if (std::abs(inv_tanR) > 1e-6)
        {
          double R = L_ / inv_tanR - T_ / 2.0;
          return safeEffectiveSteering(R);
        }
        else if (std::abs(inv_tanL) > 1e-6)
        {
          double R = L_ / inv_tanL + T_ / 2.0;
          return safeEffectiveSteering(R);
        }
        return 0.0;
      }

      // 双侧有效：分别计算转弯半径并取平均（提高抗噪性）
      double R_left = L_ / inv_tanL + T_ / 2.0;
      double R_right = L_ / inv_tanR - T_ / 2.0;
      double R_avg = (R_left + R_right) / 2.0;

      return safeEffectiveSteering(R_avg);
    }
    else if (has_left_)
    {
      double aL = steer_left_;
      if (std::abs(aL) < 1e-6)
        return 0.0;
      double inv_tanL = std::tan(aL);
      if (std::abs(inv_tanL) < 1e-6)
        return (aL > 0 ? max_steer_ : -max_steer_);
      double R = L_ / inv_tanL + T_ / 2.0;
      return safeEffectiveSteering(R);
    }
    else if (has_right_)
    {
      double aR = steer_right_;
      if (std::abs(aR) < 1e-6)
        return 0.0;
      double inv_tanR = std::tan(aR);
      if (std::abs(inv_tanR) < 1e-6)
        return (aR > 0 ? max_steer_ : -max_steer_);
      double R = L_ / inv_tanR - T_ / 2.0;
      return safeEffectiveSteering(R);
    }
    // 没有关节数据，返回标记（调用者可识别）
    return std::nan("");
  }

  // 工具：给定转弯半径 R，返回等效转向角 δ，并限制在物理范围内
  double safeEffectiveSteering(double R) const
  {
    if (std::abs(R) < 1e-6)
      return (R >= 0 ? max_steer_ : -max_steer_);
    double delta = std::atan(L_ / R);
    // 钳位到机械限位
    if (delta > max_steer_)
      delta = max_steer_;
    if (delta < -max_steer_)
      delta = -max_steer_;
    return delta;
  }

  // 自行车模型理论转角（用于和指令对比）
  double computeTheoreticalSteering(double v, double w) const
  {
    if (std::abs(v) < 1e-6)
    {
      return (w >= 0) ? max_steer_ : -max_steer_;
    }
    double delta = std::atan(L_ * w / v);
    // 钳位
    if (delta > max_steer_)
      delta = max_steer_;
    if (delta < -max_steer_)
      delta = -max_steer_;
    return delta;
  }

  // ========== 主验证循环 ==========
  void verifyLoop()
  {
    // 无有效指令则跳过
    if (std::abs(v_cmd_) < 1e-6 && std::abs(w_cmd_) < 1e-6)
      return;

    // 1. 理论转向角（从指令 v,w 计算）
    double delta_theory = computeTheoreticalSteering(v_cmd_, w_cmd_);

    // 2. 实际等效转向角（精确几何模型）
    double delta_actual = computePreciseSteering();

    // 如果精确模型失败（无关节数据），回退到从里程计反算
    bool used_fallback = false;
    if (std::isnan(delta_actual))
    {
      delta_actual = computeTheoreticalSteering(v_real_, w_real_);
      used_fallback = true;
    }

    // 3. 速度误差
    double v_err = std::abs(v_cmd_ - v_real_);
    double w_err = std::abs(w_cmd_ - w_real_);
    double steer_err_deg = std::abs(delta_theory - delta_actual) * 180.0 / M_PI;

    // 4. 判定状态
    std::string status = "OK";
    if (v_err > v_tol_)
      status = "V_WARN";
    if (w_err > w_tol_)
      status = "W_WARN";
    if (steer_err_deg > steer_tol_deg_)
      status = "STEER_WARN";
    if (used_fallback)
      status = "FALLBACK"; // 使用里程计反算不代表真实转角

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "[%s] CMD(v=%.2f,w=%.2f) REAL(v=%.2f,w=%.2f) "
                         "Steer(theory=%.1f°,actual=%.1f°,err=%.1f°)%s",
                         status.c_str(), v_cmd_, w_cmd_, v_real_, w_real_,
                         delta_theory * 180.0 / M_PI,
                         delta_actual * 180.0 / M_PI,
                         steer_err_deg,
                         used_fallback ? " (from odom)" : "");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannVerifier>());
  rclcpp::shutdown();
}