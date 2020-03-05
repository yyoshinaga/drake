#include "drake/examples/yaskawa_arm/primitive_executor.h"

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(lcm_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");
DEFINE_string(lcm_plan_channel, "COMMITTED_ROBOT_PLAN",
              "Channel on which to send robot_plan_t messages.");
DEFINE_string(ee_name, "yaskawa_link_ee", "Name of the end effector link");

// Stuff for Schunk wsg gripper
DEFINE_string(lcm_gripper_status_channel, "EE_STATUS", "Channel on which to listen for lcmt_ee_status messages");
DEFINE_string(lcm_gripper_command_channel, "EE_COMMAND", "Channel on which to send lcmt_ee_commands messages");

// Stuff for detecting the object's position
DEFINE_string(lcm_package_state_channel, "PACKAGE_STATE", "Channel on which to listen for robot_state_t messages for the object");

namespace drake {
namespace examples {
namespace yaskawa_arm_runner{

const char kYaskawaUrdf[] =
      "drake/manipulation/models/yaskawa_description/urdf/"
        "yaskawa_no_collision.urdf";
    // const char kEEUrdf[] =
    //       "drake/manipulation/models/yaskawa_end_effector_description/"
    //         "urdf/end_effector_3.urdf";


    PrimitiveExecutor::PrimitiveExecutor(): plant_(0.0) {
        yaskawa_urdf_ =
            (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kYaskawaUrdf));
        // ee_urdf_ = 
        //     (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kEEUrdf));

        const auto X_WI = RigidTransform<double>::Identity();
        auto iiwa_instance = internal::AddAndWeldModelFrom(
            sdf_path, "yaskawa", plant_->world_frame(), "arm_base", X_WI, plant_);

        // const drake::multibody::Frame<T>& link6 =
        //     plant_->GetFrameByName("ee_mount", iiwa_model_.model_instance);
        // const RigidTransform<double> X_6G(RollPitchYaw<double>(0, 0, 0),
        //                                     Vector3d(0, 0, 0.114));
        // auto wsg_instance = internal::AddAndWeldModelFrom(sdf_path, "gripper", link6,
        //                                                     "ee_base", X_6G, plant_);

        //           const drake::multibody::ModelInstanceIndex new_model =
        // multibody::Parser(&plant_).AddModelFromFile(yaskawa_urdf_);

        //         plant_.WeldFrames(plant_.world_frame(),
        //                         plant_.GetBodyByName("arm_base").body_frame());

        plant_.Finalize();
        context_ = plant_.CreateDefaultContext();

        lcm_.subscribe(FLAGS_lcm_status_channel, &PrimitiveExecutor::HandleStatusYaskawa, this);
        lcm_.subscribe(FLAGS_lcm_gripper_status_channel, &PrimitiveExecutor::HandleStatusGripper, this);
        lcm_.subscribe(FLAGS_lcm_belt_status_channel, &PrimitiveExecutor::HandleStatusBelt, this);
        lcm_.subscribe(FLAGS_lcm_package_channel, &PrimitiveExecutor::HandleStatusPackage, this);

        std::map<int, std::string> position_names;
        const int num_positions = plant_.num_positions();
        for (int i = 0; i < plant_.num_joints(); ++i) {
            const multibody::Joint<double>& joint =
                plant_.get_joint(multibody::JointIndex(i));
            if (joint.num_positions() == 0) {
                continue;
            }
            DRAKE_DEMAND(joint.num_positions() == 1);
            DRAKE_DEMAND(joint.position_start() < num_positions);

            position_names[joint.position_start()] = joint.name();
        }

        DRAKE_DEMAND(static_cast<int>(position_names.size()) == num_positions);
        for (int i = 0; i < num_positions; ++i) {
            joint_names_.push_back(position_names[i]);
        }

        for(int i=0; i< 1; i++)
        {
            object_positions_.push_back(Eigen::Vector3d());
            object_quaternions_.push_back(Eigen::Quaterniond());
        }

        ee_update_ = false;
        iiwa_update_ = false;
        object_update_ = false;
    }

    int PrimitiveExecutor::action_Grip(double width, double force, double time)
    {
        while(lcm_.handle() >= 0 && !ee_update_);
        uint64_t init_time = ee_status_.utime / 1e6;
        double final_time = init_time + time;
        // drake::log()->info("Init time: {}", init_time);
        // drake::log()->info("final time: {}", final_time);
        while(lcm_.handle() >= 0 && (ee_status_.utime / 1e6) < final_time + 1)
        {
            if(ee_update_)
            {
                ee_update_ = false;
                // drake::log()->info("looping: {}", ee_status_.utime / 1e6);
                double cur_dt_time = (ee_status_.utime/1e6 - init_time);

                lcmt_yaskawa_ee_command command{};
                
                command.target_position_mm = ee_status_.actual_position_mm + (width - ee_status_.actual_position_mm) * (cur_dt_time / time);
                
                command.utime = ee_status_.utime;
                command.force = force;
                lcm_.publish(FLAGS_lcm_gripper_command_channel, &command);  
            }
        }
        return 1;
    }

    int PrimitiveExecutor::action_GoToPoint(Eigen::Vector3d position, Eigen::Vector3d orientation, double time)
    {
        while(lcm_.handle() >= 0 && !iiwa_update_);

        drake::manipulation::planner::ConstraintRelaxingIk::IkCartesianWaypoint wp;    
        wp.pose.translation() = position;
        drake::math::RollPitchYaw<double> rpy(orientation(0), orientation(1), orientation(2));
        wp.pose.linear() = rpy.ToMatrix3ViaRotationMatrix();
        wp.constrain_orientation = true;

        std::vector<manipulation::planner::ConstraintRelaxingIk::IkCartesianWaypoint> waypoints;
        waypoints.push_back(wp);

        // Create the plan
        manipulation::planner::ConstraintRelaxingIk ik(urdf_, FLAGS_ee_name, Isometry3<double>::Identity());
        
        Eigen::VectorXd iiwa_q(iiwa_status_.num_joints);
        for (int i = 0; i < iiwa_status_.num_joints; i++) {
            iiwa_q[i] = iiwa_status_.joint_position_measured[i];
        }

        IKResults ik_res;
        ik.PlanSequentialTrajectory(waypoints, iiwa_q, &ik_res);

        std::vector<double> times;
        times.push_back(0);
        times.push_back(time);

        drake::log()->info("executing ik result");
        if (ik_res.info[0] == 1) 
        {
            drake::log()->info("IK sol size {}", ik_res.q_sol.size());

            // Convert the resulting waypoints into the format expected by
            // ApplyJointVelocityLimits and EncodeKeyFrames.
            MatrixX<double> q_mat(ik_res.q_sol.front().size(), ik_res.q_sol.size());
            for (size_t i = 0; i < ik_res.q_sol.size(); ++i) 
            {
            q_mat.col(i) = ik_res.q_sol[i];
            }

            // Ensure that the planned motion would not exceed the joint
            // velocity limits.  This function will scale the supplied
            // times if needed.
            examples::yaskawa_arm::ApplyJointVelocityLimits(q_mat, &times);
            robotlocomotion::robot_plan_t plan =
                examples::yaskawa_arm::EncodeKeyFrames(tree_, times, ik_res.info, q_mat);
            // drake::log()->info("publishing");
            lcm_.publish(FLAGS_lcm_plan_channel, &plan);
        }

        double init_time = iiwa_status_.utime / 1e6;
        double final_time = init_time + time;
        drake::log()->info("Init time: {}", init_time);
        drake::log()->info("final time: {}", final_time);
        while(lcm_.handle() >= 0 && (iiwa_status_.utime / 1e6) < final_time + 3);

        return 1;

    }

    void PrimitiveExecutor::HandleStatusObject(const lcm::ReceiveBuffer*, const std::string&, const bot_core::robot_state_t* status){
        object_positions_ = Eigen::Vector3d (status->pose.translation.x, status->pose.translation.y, status->pose.translation.z);
        object_quaternions_ = Eigen::Quaterniond (status->pose.rotation.w, status->pose.rotation.x, status->pose.rotation.y, status->pose.rotation.z);
        object_update_ = true;
    }


    void PrimitiveExecutor::HandleStatusGripper(const lcm::ReceiveBuffer*, const std::string&, const lcmt_schunk_wsg_status* status) {
        ee_status_ = *status;
        ee_update_ = true;
    }

    void PrimitiveExecutor::HandleStatusYaskawa(const lcm::ReceiveBuffer*, const std::string&, const lcmt_iiwa_status* status){
        iiwa_status_ = *status;
        iiwa_update_ = true;

        status_count_++;

        Eigen::VectorXd iiwa_q(status->num_joints);
        for (int i = 0; i < status->num_joints; i++) {
            iiwa_q[i] = status->joint_position_measured[i];
        }

        if (status_count_ % 100 == 1) {
            plant_.SetPositions(context_.get(), iiwa_q);

            const math::RigidTransform<double> ee_pose =
                plant_.EvalBodyPoseInWorld(
                    *context_, plant_.GetBodyByName(FLAGS_ee_name));
            const math::RollPitchYaw<double> rpy(ee_pose.rotation());
            drake::log()->info("End effector at: {} {}",
                                ee_pose.translation().transpose(),
                                rpy.vector().transpose());
        }
    }



    drake::multibody::ModelInstanceIndex PrimitiveExecutor::AddAndWeldModelFrom(
        const std::string& model_path, const std::string& model_name,
        const drake::multibody::Frame<T>& parent, const std::string& child_frame_name,
        const RigidTransform<double>& X_PC, MultibodyPlant<T>* plant) {

        DRAKE_THROW_UNLESS(!plant->HasModelInstanceNamed(model_name));

        drake::multibody::Parser parser(plant);
        //This line gives isPhysicallyValid()
        const drake::multibody::ModelInstanceIndex new_model =
            parser.AddModelFromFile(model_path, model_name);

        const auto& child_frame = plant->GetFrameByName(child_frame_name, new_model);
        plant->WeldFrames(parent, child_frame, X_PC);
        drake::log()->info("AddAndWeldModelFrom 1");
        return new_model;
    }

    std::vector<Eigen::Vector3d> PrimitiveExecutor::getObjectLocations()
    {
        while(lcm_.handle() >= 0 && !object_update_);
        return object_positions_;
    }

    std::vector<Eigen::Quaterniond> PrimitiveExecutor::getObjectQuats()
    {
        while(lcm_.handle() >= 0 && !object_update_);
        return object_quaternions_;
    }
 

}
}
}