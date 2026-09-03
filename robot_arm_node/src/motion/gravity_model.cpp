/**
 * @file   gravity_model.cpp
 * @brief  力矩控制有用：GravityModel 实现 —— pinocchio 建模 + computeGeneralizedGravity
 *
 * @details
 * 该模块负责机械臂重力补偿模型的加载与计算：
 *
 * 1. **URDF 模型加载**（onDescription）：
 *    - 从 ROS 话题接收 URDF 描述，仅建模一次
 *    - 构建 pinocchio 多体动力学模型和数据结构
 *    - 映射关节名称到配置/速度向量索引
 *
 * 2. **重力矢量计算**（gravity）：
 *    - 输入：关节位置完整映射、目标关节名称列表
 *    - 调用 pinocchio::computeGeneralizedGravity 计算广义重力力矩
 *    - 输出：指定关节的重力补偿力矩（向量形式）
 *    - 若位形不完整则返回 false，避免用默认值计算出错误的补偿力
 *
 * 3. **线程安全**：
 *    - gravity() 与 onDescription() 通过互斥锁同步
 *    - pinocchio::Data 结构在 gravity() 中被修改，故需持锁保护
 *
 * 4. **应用场景**：
 *    - 用于实时机械臂控制中的前馈重力补偿
 *    - 减轻伺服控制器负担，提高位置精度
 */
#include "robot_arm_node/motion/gravity_model.hpp"

#include <algorithm>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace robot_arm_node::motion {

/**
 * @brief GravityModel 的实现细节（pImpl），把 pinocchio 类型挡在头文件之外。
 *
 * 头文件只前置声明本结构体，使用方无需引入 pinocchio 的重型模板头。
 */
struct GravityModel::Impl {
    pinocchio::Model model;
    // data 随 model 走；gravity() 里会写它，故 gravity()
    // 需要持锁（见头文件说明）
    mutable pinocchio::Data data;
    bool ready{false};

    // 关节名 → (idx_q, idx_v)。q 用于填位形，v 用于取 g 向量分量。
    // 二者对旋转关节都是 1 维且一一对应，但分开存以免将来加多自由度关节时踩坑。
    std::map<std::string, std::pair<int, int>> joint_index;
    int nq{0};
};

/**
 * @brief 构造重力模型，订阅 URDF 话题以异步完成建模。
 *
 * 构造时模型尚未就绪（ready() 为 false），需等 URDF 到达并建模成功。
 *
 * @param node 宿主节点，用于取日志器与创建订阅。
 * @param topic URDF 描述话题名（通常为 /robot_description）。
 */
GravityModel::GravityModel(rclcpp::Node& node, const std::string& topic)
    : logger_(node.get_logger().get_child("gravity_model")),
      impl_(std::make_unique<Impl>()) {
    // URDF 是 latched（TRANSIENT_LOCAL）发布的，晚订阅也能收到最后一帧
    sub_ = node.create_subscription<std_msgs::msg::String>(
        topic, rclcpp::QoS(1).transient_local().reliable(),
        [this](const std_msgs::msg::String& m) { onDescription(m); });
}

/**
 * @brief 析构函数，pImpl 由 unique_ptr 自动释放（需在此处定义以见到完整类型）。
 */
GravityModel::~GravityModel() = default;

/**
 * @brief URDF 话题回调：解析描述并构建 pinocchio 模型与关节索引表。
 *
 * 只建模一次 —— URDF 在一次运行内不会变，重复建模纯属浪费且会打断
 * 正在进行的 gravity() 调用。建模失败时把 ready 置 false 而非抛出，
 * 调用方通过 ready() 得知重力补偿不可用，由上层决定退化策略。
 * 日志里的质量合计是自检项：若为 0 kg 说明 URDF 缺 <inertial>，
 * 重力补偿会算出全零力矩（静默失效，故必须打出来）。
 *
 * @param msg URDF XML 字符串消息。
 */
void GravityModel::onDescription(const std_msgs::msg::String& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (impl_->ready) return;  // 只建一次：URDF 在一次运行里不会变

    try {
        pinocchio::Model model;
        // 固定基座（不给 root joint）：URDF 的根是 world，经 fixed joint 连到
        // arm_base_link
        pinocchio::urdf::buildModelFromXML(msg.data, model);

        std::map<std::string, std::pair<int, int>> index;
        for (pinocchio::JointIndex j = 1; j < model.joints.size(); ++j) {
            const auto& jm = model.joints[j];
            if (jm.nq() == 0) continue;  // 固定关节等无自由度的跳过
            index[model.names[j]] = {jm.idx_q(), jm.idx_v()};
        }

        impl_->model = std::move(model);
        impl_->data = pinocchio::Data(impl_->model);
        impl_->joint_index = std::move(index);
        impl_->nq = impl_->model.nq;
        impl_->ready = true;

        std::string names;
        for (const auto& [n, _] : impl_->joint_index)
            names += (names.empty() ? "" : ", ") + n;
        // 质量合计只作日志自检（0 kg 说明 URDF 没有 <inertial>，重力补偿会全是
        // 0）—— 直接累加 model.inertias，省掉 center-of-mass.hpp 这个额外的重头
        double total_mass = 0.0;
        for (const auto& I : impl_->model.inertias) total_mass += I.mass();
        RCLCPP_INFO(logger_,
                    "重力模型就绪：nq=%d nv=%d 质量合计 %.3f kg  可动关节 [%s]",
                    impl_->model.nq, impl_->model.nv, total_mass,
                    names.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "pinocchio 建模失败，重力补偿不可用：%s",
                     e.what());
        impl_->ready = false;
    }
}

/**
 * @brief 查询模型是否已就绪。
 *
 * @return 已收到 URDF 且建模成功则返回 true。
 */
bool GravityModel::ready() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return impl_->ready;
}

/**
 * @brief 列出模型中所有可动关节名（有自由度的关节，固定关节不计）。
 *
 * @return 关节名列表，按名称字典序（源自 std::map）。
 */
std::vector<std::string> GravityModel::movable_joints() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<std::string> out;
    for (const auto& [n, _] : impl_->joint_index) out.push_back(n);
    return out;
}

/**
 * @brief 计算指定关节的广义重力力矩。
 *
 * **fail-closed 设计**：模型里任一可动关节缺少位置回读时整体返回 false，
 * 而不是拿默认值 0 顶替。用错误位形算出的重力力矩是一个「主动施加的错误力」，
 * 比完全不补偿危险得多（详见工作空间 CLAUDE.md 力矩模式相关条目）。
 *
 * @param positions 关节名 → 当前位置（rad）的完整映射。
 * @param joints 需要输出力矩的关节名列表（决定 out 的顺序）。
 * @param out 出参，与 joints 同序的重力力矩（N·m）。
 * @return 位形完整且关节名均在模型中则返回 true，否则 false 且 out 未定义。
 */
bool GravityModel::gravity(const std::map<std::string, double>& positions,
                           const std::vector<std::string>& joints,
                           std::vector<double>& out) const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!impl_->ready) return false;

    // 位形必须完整：模型里每个可动关节都得有回读。
    // 缺就整体放弃 —— 用默认值（0）顶替缺失关节会算出一个错误的重力力矩，
    // 那是主动施加的错误力，比"不补偿"危险得多。
    Eigen::VectorXd q = Eigen::VectorXd::Zero(impl_->nq);
    for (const auto& [name, idx] : impl_->joint_index) {
        auto it = positions.find(name);
        if (it == positions.end()) return false;
        q[idx.first] = it->second;
    }

    // pinocchio 的 RNEA 重力项：g(q)，结果写进 impl_->data.g
    pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, q);

    // 按调用方给的 joints 顺序取分量（不依赖模型内部关节序）
    out.clear();
    out.reserve(joints.size());
    for (const auto& name : joints) {
        auto it = impl_->joint_index.find(name);
        if (it == impl_->joint_index.end()) return false;
        out.push_back(impl_->data.g[it->second.second]);
    }
    return true;
}

}  // namespace robot_arm_node::motion
