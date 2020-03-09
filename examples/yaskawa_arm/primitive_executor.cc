#include "drake/examples/yaskawa_arm/primitive_executor.h"

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(lcm_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");
DEFINE_string(lcm_plan_channel, "COMMITTED_ROBOT_PLAN",
              "Channel on which to send robot_plan_t messages.");
DEFINE_string(ee_name, "ee_mount", "Name of the end effector link");

// Gripper channels
DEFINE_string(lcm_gripper_status_channel, "EE_STATUS", "Channel on which to listen for lcmt_ee_status messages");
DEFINE_string(lcm_gripper_command_channel, "EE_COMMAND", "Channel on which to send lcmt_ee_commands messages");

// Channel for detecting the object's position
DEFINE_string(lcm_package_state_channel, "PACKAGE_STATE", "Channel on which to listen for robot_state_t messages for the object");

DEFINE_double(whiskers_up_angle, -0.1, "The angle at which the whiskers are in the open position");
DEFINE_double(whiskers_down_angle, -1.4, "The angle at which the whiskers are in the closed position");

namespace drake {
namespace examples {
namespace yaskawa_arm_runner{

using multibody::MultibodyPlant;
using drake::math::RigidTransform;

const char kYaskawaUrdf[] =
      "drake/manipulation/models/yaskawa_description/urdf/"
        "yaskawa_no_collision.urdf";
const char kEEUrdf[] =
        "drake/manipulation/models/yaskawa_end_effector_description/"
        "urdf/end_effector_3.urdf";


    PrimitiveExecutor::PrimitiveExecutor(): plant_(0.0){

        drake::log()->info("started");

        yaskawa_urdf_ =
            (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kYaskawaUrdf));
        ee_urdf_ =
            (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kEEUrdf));

        const math::RigidTransform<double> X_WI = math::RigidTransform<double>::Identity();
        const math::RigidTransform<double> X_6G(math::RollPitchYaw<double>(0, 0, 0),
                                            Eigen::Vector3d(0, 0, 0.114));

        DRAKE_THROW_UNLESS(!plant_.HasModelInstanceNamed("yaskawa"));
        DRAKE_THROW_UNLESS(!plant_.HasModelInstanceNamed("gripper"));

        drake::multibody::Parser parser(&plant_);

        yaskawa_model_idx =
            parser.AddModelFromFile(yaskawa_urdf_, "yaskawa");
        ee_model_idx =
            parser.AddModelFromFile(ee_urdf_, "gripper");   

        //Add EE to model
        const drake::multibody::Frame<double>& ee_frame =
            plant_.GetFrameByName("ee_mount", yaskawa_model_idx);

        //Weld models
        plant_.WeldFrames(plant_.world_frame(), plant_.GetFrameByName("arm_base", yaskawa_model_idx), X_WI);
        plant_.WeldFrames(ee_frame, plant_.GetFrameByName("ee_base", ee_model_idx), X_6G);

        plant_.Finalize();
        drake::log()->info("Plant finalized");

        //Define context. Use context to get state of models
        context_ = plant_.CreateDefaultContext();

        //Link a subscription function to the channel
        lcm_.subscribe(FLAGS_lcm_status_channel, &PrimitiveExecutor::HandleStatusYaskawa, this);
        lcm_.subscribe(FLAGS_lcm_gripper_status_channel, &PrimitiveExecutor::HandleStatusGripper, this);
        // lcm_.subscribe(FLAGS_lcm_belt_status_channel, &PrimitiveExecutor::HandleStatusBelt, this);
        lcm_.subscribe(FLAGS_lcm_package_state_channel, &PrimitiveExecutor::HandleStatusPackage, this);

        //Retrieving position names for the yaskawa arm
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

        ee_update_ = false;
        yaskawa_update_ = false;
        object_update_ = false;
    }

//-------------------------------- actions ---------------------------------------------------
    
    // TODO: Finish this function
    int PrimitiveExecutor::action_GoHome(){
        // while(lcm_.handle() >= 0 && !ee_update_);
        // while(lcm_.handle() >= 0 is_at_desired_position()){

        // }

        return 1;
    }

    int PrimitiveExecutor::action_Collect(const double time) {
        while(lcm_.handle() >= 0 && !ee_update_);

        //Checks: 
        if(!are_whiskers_up() ||  
            is_carrying() || 
            !is_at_belt() || 
            !is_pickable() ||  
            !is_at_package_location()){
            printer(0,1,0,1,0,1,0,1,1);
            return 0; //If any of the checks fail, function returns 0.
        }

        double final_time = static_cast<double>(ee_status_.utime / 1e6) + time;

        while(lcm_.handle() >= 0 && (ee_status_.utime / 1e6) < final_time + 1){
            if(ee_update_){
                ee_update_ = false;

                lcmt_yaskawa_ee_command command{};
                command.utime = ee_status_.utime;
                command.puller_move = true;
                
                lcm_.publish(FLAGS_lcm_gripper_command_channel, &command);  
            }
        }
        return 1;
    }

    int PrimitiveExecutor::action_Release(const double time){
        while(lcm_.handle() >= 0 && !ee_update_);

        //Checks: 
        if(!are_whiskers_up() || 
           !is_carrying() ||
           !is_at_shelf()){
            printer(0,1,0,1,1,0,0,0,0);
            return 0; //If any of the checks fail, function returns 0.
        }

        double final_time = static_cast<double>(ee_status_.utime / 1e6) + time;

        while(lcm_.handle() >= 0 && (ee_status_.utime / 1e6) < final_time + 1){
            if(ee_update_){
                ee_update_ = false;

                lcmt_yaskawa_ee_command command{};
                command.utime = ee_status_.utime;
                command.pusher_move = true;
                
                lcm_.publish(FLAGS_lcm_gripper_command_channel, &command);  
            }
        }
        return 1;
    }

    int PrimitiveExecutor::action_WhiskersUp(){
        while(lcm_.handle() >= 0 && !ee_update_);

        //Checks: 
        if(are_whiskers_up()){ return 1; }  //Action already completed
        if(is_moving()){ return 0; }        //If any of the checks fail, function returns 0.

        while(lcm_.handle() >= 0 && !are_whiskers_up()){    //While whiskers not up, keep running
            if(ee_update_){
                ee_update_ = false;

                lcmt_yaskawa_ee_command command{};
                command.utime = ee_status_.utime;
                command.whiskers_down = false;
                
                lcm_.publish(FLAGS_lcm_gripper_command_channel, &command);  
            }
        }
        return 1;
    }

    int PrimitiveExecutor::action_WhiskersDown(){
        while(lcm_.handle() >= 0 && !ee_update_);

        //Checks: 
        if(are_whiskers_down()){ return 1; }    //Action already completed
        if(is_carrying()){ return 0; }          //If any of the checks fail, function returns 0.

        while(lcm_.handle() >= 0 && !are_whiskers_down()){  //While whiskers not up, keep running
            if(ee_update_){
                ee_update_ = false;

                lcmt_yaskawa_ee_command command{};
                command.utime = ee_status_.utime;
                command.whiskers_down = true;
                
                lcm_.publish(FLAGS_lcm_gripper_command_channel, &command);  
            }
        }
        return 1;
    }

    int PrimitiveExecutor::action_GoToPoint(const Eigen::Vector3d position, const Eigen::Vector3d orientation, const double time) {
        while(lcm_.handle() >= 0 && !yaskawa_update_);

        //Checks:
        if(is_at_desired_position()){ return 1; } //Action already completed

        drake::manipulation::planner::ConstraintRelaxingIk::IkCartesianWaypoint wp;    
        wp.pose.translation() = position;
        drake::math::RollPitchYaw<double> rpy(orientation(0), orientation(1), orientation(2));
        wp.pose.linear() = rpy.ToMatrix3ViaRotationMatrix();
        wp.constrain_orientation = true;

        std::vector<manipulation::planner::ConstraintRelaxingIk::IkCartesianWaypoint> waypoints;
        waypoints.push_back(wp);

        // Create the plan
        manipulation::planner::ConstraintRelaxingIk ik(yaskawa_urdf_, FLAGS_ee_name, Isometry3<double>::Identity());
        
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
        if (ik_res.info[0] == 1) {
            drake::log()->info("IK sol size {}", ik_res.q_sol.size());

            // Convert the resulting waypoints into the format expected by
            // ApplyJointVelocityLimits and EncodeKeyFrames.
            MatrixX<double> q_mat(ik_res.q_sol.front().size(), ik_res.q_sol.size());
            for (size_t i = 0; i < ik_res.q_sol.size(); ++i) {
                q_mat.col(i) = ik_res.q_sol[i];
            }

            // Ensure that the planned motion would not exceed the joint
            // velocity limits.  This function will scale the supplied
            // times if needed.
            examples::yaskawa_arm::ApplyJointVelocityLimits(q_mat, &times);
            robotlocomotion::robot_plan_t plan =
                examples::yaskawa_arm::EncodeKeyFrames(joint_names_, times, ik_res.info, q_mat);
            // drake::log()->info("publishing");
            lcm_.publish(FLAGS_lcm_plan_channel, &plan);
        }

        //What is this? 
        // double init_time = iiwa_status_.utime / 1e6;
        // double final_time = init_time + time;
        // drake::log()->info("Init time: {}", init_time);
        // drake::log()->info("final time: {}", final_time);
        // while(lcm_.handle() >= 0 && (iiwa_status_.utime / 1e6) < final_time + 3);

        return 1;
    }


//---------------------------------------- STATUSES ----------------------------------------------------

    void PrimitiveExecutor::HandleStatusPackage(const real_lcm::ReceiveBuffer*, const std::string&, const bot_core::robot_state_t* status){
        drake::log()->info("status->pose.rotation.w: {}",status->pose.rotation.w);
        object_position_ = Eigen::Vector3d (status->pose.translation.x, status->pose.translation.y, status->pose.translation.z);
        object_quaternion_ = Eigen::Quaterniond (status->pose.rotation.w, status->pose.rotation.x, status->pose.rotation.y, status->pose.rotation.z);
        object_update_ = true;
    }

    void PrimitiveExecutor::HandleStatusGripper(const real_lcm::ReceiveBuffer*, const std::string&, const lcmt_yaskawa_ee_status* status) {
        ee_status_ = *status;
        ee_update_ = true;
    }

    void PrimitiveExecutor::HandleStatusYaskawa(const real_lcm::ReceiveBuffer*, const std::string&, const lcmt_iiwa_status* status){
        iiwa_status_ = *status;
        yaskawa_update_ = true;


        Eigen::VectorXd iiwa_q(status->num_joints);
        for (int i = 0; i < status->num_joints; i++) {
            iiwa_q[i] = status->joint_position_measured[i];
        }

        status_count_++;    //Increase this count for the if statement below
        if (status_count_ % 100 == 1) {
            plant_.SetPositions(context_.get(),yaskawa_model_idx, iiwa_q);

            ee_pose = plant_.EvalBodyPoseInWorld(
                    *context_, plant_.GetBodyByName(FLAGS_ee_name));
            const math::RollPitchYaw<double> rpy(ee_pose.rotation());
            drake::log()->info("End effector at: {} {}",
                                ee_pose.translation().transpose(),
                                rpy.vector().transpose());
        }
    }

    Eigen::Vector3d PrimitiveExecutor::getObjectLocation() {
        while(lcm_.handle() >= 0 && !object_update_);
        return object_position_;
    }

    Eigen::Quaterniond PrimitiveExecutor::getObjectQuat() {
        while(lcm_.handle() >= 0 && !object_update_);
        return object_quaternion_;
    }
 
// ----------------------------------- Checker ------------------------------------------
    // TODO: Finish this function
    bool PrimitiveExecutor::is_at_desired_position(VectorX<double> des_pos){

        DRAKE_DEMAND(des_pos.size() == 6);

        // Is this another way to obtain end effector position?
        // VectorX<double> positions = plant_.GetPositions(*context_, yaskawa_model_idx); 
        ee_pose = plant_.EvalBodyPoseInWorld(
                *context_, plant_.GetBodyByName(FLAGS_ee_name));

        const math::RollPitchYaw<double> rpy(ee_pose.rotation());

        drake::log()->info("translation: \n{} \n{} \n{}\nrotation: \n{}\n{}\n{}",
            ee_pose.translation().transpose()[0],
            ee_pose.translation().transpose()[1],
            ee_pose.translation().transpose()[2],
            rpy.vector().transpose()[0],
            rpy.vector().transpose()[1],
            rpy.vector().transpose()[2],
        );

        const double buffer = 0.01; //Temporary value

        for(int i = 0; i < 6; i++){
            if(i < 3){
                if(des_pos[i] - ee_pose.translation().transpose()[i] < buffer){
                    return 0;
                }
            }
            else {
                if(des_pos[i] - rpy.vector().transpose()[i] < buffer){
                    return 0;
                }  
            }
        }
        // drake::log()->info("reading position size: {}", positions.size());

        return 1;
    }
    bool PrimitiveExecutor::are_whiskers_up(){
        drake::log()->info("whisker angle: {} {}",ee_status_.actual_whisker_angle, FLAGS_whiskers_up_angle);
        return ee_status_.actual_whisker_angle >= FLAGS_whiskers_up_angle;
    }
    bool PrimitiveExecutor::are_whiskers_down(){
        drake::log()->info("whisker angle: {} {}",ee_status_.actual_whisker_angle, FLAGS_whiskers_down_angle);
        return ee_status_.actual_whisker_angle <= FLAGS_whiskers_down_angle;
    }
    bool PrimitiveExecutor::is_carrying(){
        return 0;
    }
    bool PrimitiveExecutor::is_at_shelf(){
        VectorX<double> atShelf(6);
        atShelf << 0.4,0.3,0.9,0,0,0;
        is_at_desired_position(atShelf) ? return 1 : return 0;
    }
    bool PrimitiveExecutor::is_at_belt(){   //TODO:Forward Kinematics
        VectorX<double> atBelt(6);
        atBelt << 0.4,0.3,0.4,0,0,0;
        is_at_desired_position(atBelt) ? return 1 :return 0;
    }
    bool PrimitiveExecutor::is_moving(){
 
        VectorX<double> velocities = plant_.GetVelocities(*context_, yaskawa_model_idx);  
        for(int i = 0; i < velocities.size(); i++){
            if(velocities[i] != 0){
                return 0;
            }
        }
        return 1;
    }
    bool PrimitiveExecutor::is_pickable(){
        return 1;
    }
    bool PrimitiveExecutor::is_at_package_location(double x){
        VectorX<double> atPkg(6);
        atPkg << 0.4+x,0.3,0.4,0,0,0;
        is_at_desired_position(atPkg) ? return 1 : return 0;
    }

    void PrimitiveExecutor::printer(bool desPos, bool whiskUp, bool whiskDwn, bool carry, bool atShlf, 
                                    bool atBlt, bool isMvng, bool isPckble, bool atPkgLoc){
        drake::log()->info("Check printer starts below:");

        if(desPos)   
            drake::log()->info("    at desired position: {}",is_at_desired_position());
        if(whiskUp)
            drake::log()->info("    whiskers up: {}",are_whiskers_up());
        if(whiskDwn)
            drake::log()->info("    whiskers down: {}",are_whiskers_down());
        if(carry)
            drake::log()->info("    is carrying: {}",is_carrying());
        if(atShlf)
            drake::log()->info("    at shelf: {}",is_at_shelf());
        if(atBlt)
            drake::log()->info("    is at belt: {}",is_at_belt());
        if(isMvng)
            drake::log()->info("    is moving: {}",is_moving());
        if(isPckble)
            drake::log()->info("    is pickable: {}",is_pickable());
        if(atPkgLoc)
            drake::log()->info("    is at package location: {}",is_at_package_location());

        drake::log()->info("Printer end");

    }

}
}
}