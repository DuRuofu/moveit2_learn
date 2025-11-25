#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>
#include <tf2/LinearMath/Quaternion.h>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class Commander
{
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<my_robot_interfaces::msg::PoseCommand>::SharedPtr pose_cmd_sub_;

    /**
     * @brief 通用规划与执行函数
     * @param interface MoveGroupInterface 指针（arm 或 gripper）
     */
    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            interface->execute(plan);
        }
    }

    /**
     * @brief 接收 /open_gripper 话题回调
     */
    void openGripperCallback(const example_interfaces::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
            openGripper();
        else
            closeGripper();
    }
    /**
     * @brief 接收 /joint_cmd 话题回调
     */
    void jointCmdCallback(const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
    {
        std::vector<double> joint_values;

        if (msg->data.size()== 6)
        {
            goToJointTarget(msg->data);
        }
    }

    /**
     * @brief 接收 /pose_cmd 话题回调
     */
    void poseCmdCallback(const my_robot_interfaces::msg::PoseCommand::SharedPtr msg)
    {
        goToPoseTarget(
            msg->x,
            msg->y,
            msg->z,
            msg->roll,
            msg->pitch,
            msg->yaw,
            msg->cartesian_path
        );
    }

public:
    /**
     * @brief 构造函数，初始化两个 MoveGroup（arm + gripper）
     */
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;

        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        // 创建订阅者（打开/关闭手爪）
        open_gripper_sub_ = node_->create_subscription<example_interfaces::msg::Bool>(
            "open_gripper",
            10,
            std::bind(&Commander::openGripperCallback, this, std::placeholders::_1)
        );

        // 创建订阅者（关节角度控制）
        joint_cmd_sub_ = node_->create_subscription<example_interfaces::msg::Float64MultiArray>(
            "joint_command",
            10,
            std::bind(&Commander::jointCmdCallback, this, std::placeholders::_1)
        );

        // 创建订阅者（位姿控制）
        pose_cmd_sub_ = node_->create_subscription<my_robot_interfaces::msg::PoseCommand>(
            "pose_command",
            10,
            std::bind(&Commander::poseCmdCallback, this, std::placeholders::_1)
        );


    }

    /**
     * @brief 通过 MoveIt NamedTarget 运动
     */
    void goToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    /**
     * @brief 通过关节角度运动
     */
    void goToJointTarget(const std::vector<double> &joint_values)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joint_values);
        planAndExecute(arm_);
    }

    /**
     * @brief 通过末端位姿运动，可切换关节空间规划或笛卡尔路径
     */
    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path=false)
    {
        // 转姿态为四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        if (!cartesian_path)
        {
            // 普通关节空间规划
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }
        else
        {
            // 笛卡尔路径规划
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm_->computeCartesianPath(
                waypoints,   // 路点列表
                0.01,        // eef_step（越小越平滑）
                0.0,         // jump_threshold
                trajectory,  // 输出轨迹
                false        // 不避障，可保证路径成功
            );

            if (fraction >= 0.9)
            {
                arm_->execute(trajectory);
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning failed (%.2f)", fraction);
            }
        }
    }

    /**
     * @brief 打开手爪
     */
    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        planAndExecute(gripper_);
    }

    /**
     * @brief 关闭手爪
     */
    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");
        planAndExecute(gripper_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("commander");

    Commander commander(node);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
