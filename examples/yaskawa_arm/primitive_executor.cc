#include "drake/examples/yaskawa_arm/primitive_executor.h"

DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_string(lcm_status_channel, "IIWA_STATUS","Channel on which to listen for lcmt_iiwa_status messages.");
DEFINE_string(lcm_plan_channel, "COMMITTED_ROBOT_PLAN","Channel on which to send robot_plan_t messages.");
DEFINE_string(ee_name, "ee_mount", "Name of the end effector link");

// Gripper channels
DEFINE_string(lcm_gripper_status_channel, "EE_STATUS", "Channel on which to listen for lcmt_ee_status messages");
DEFINE_string(lcm_gripper_command_channel, "EE_COMMAND", "Channel on which to send lcmt_ee_commands messages");

// Channel for detecting the object's position
DEFINE_string(lcm_package_state_channel, "PACKAGE_STATE", "Channel on which to listen for robot_state_t messages for the object");

DEFINE_double(whiskers_up_angle, -0.1, "The angle at which the whiskers are in the open position");
DEFINE_double(whiskers_down_angle, -1.4, "The angle at which the whiskers are in the closed position");

//Determines if trajectory is created by DDP/iLQR or just by IK
#define use_Solver 1

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

        // Add EE to model
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
        const int num_positions = plant_.num_positions(yaskawa_model_idx);
        for (int i = 0; i < 9; ++i) {  //There exists 9? joints on only the arm (including fixed)
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
        has_package = false;
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
            !is_pusher_back()||
            is_puller_back()||
            !is_pickable() ||  
            !is_at_package_location(0.0)){

            printer(0,1,0,1,0,1,1,1,0,1,1);
            drake::log()->info("is pusher back? {}",is_pusher_back());
            drake::log()->info("is puller back: {}",is_puller_back());

            return 0; //If any of the checks fail, function returns 0.
        }

        double final_time = static_cast<double>(ee_status_.utime / 1e6) + time;

        while(lcm_.handle() >= 0 && (ee_status_.utime / 1e6) < final_time + 1){
            if(ee_update_){
                ee_update_ = false;

                lcmt_yaskawa_ee_command command{};
                command.utime = ee_status_.utime;
                command.whiskers_down = false;
                command.pusher_move = false;
                command.puller_move = true;
                
                lcm_.publish(FLAGS_lcm_gripper_command_channel, &command);  
            }
        }

        has_package = true; //Assumes package has been collected
        drake::log()->info("Package received");
        return 1;
    }

    int PrimitiveExecutor::action_Release(const double time){
        while(lcm_.handle() >= 0 && !ee_update_);

        //Checks: 
        if(!are_whiskers_up() || 
           !is_carrying() ||
           !is_pusher_back() ||
           is_puller_back() ||
           !is_at_shelf()){
            printer(0,1,0,1,1,1,1,0,0,0,0);
            return 0; //If any of the checks fail, function returns 0.
        }

        double final_time = static_cast<double>(ee_status_.utime / 1e6) + time;

        while(lcm_.handle() >= 0 && (ee_status_.utime / 1e6) < final_time + 1){
            if(ee_update_){
                ee_update_ = false;

                lcmt_yaskawa_ee_command command{};
                command.utime = ee_status_.utime;
                command.whiskers_down = false;
                command.pusher_move = true;
                command.puller_move = true;

                lcm_.publish(FLAGS_lcm_gripper_command_channel, &command);  
            }
        }

        has_package = false; //Assumes package has been dropped
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
        if(!is_carrying()){ return 0; }         //If any of the checks fail, function returns 0.

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

        VectorX<double> des_pos(6);
        des_pos << position, orientation;

        //Checks:
        if(is_at_desired_position(des_pos)){ return 1; } //Action already completed

        //Start creating IK trajectory
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


        //Form the trajectory, either by using DDP or not
        #if use_Solver
            drake::examples::yaskawa_arm::RunDDPLibrary ddp;   

            for(size_t i = 0; i < ik_res.q_sol.size(); i++){
                drake::log()->info(ik_res.q_sol[1][i]);
            }
            //6 positions, 6 velocities
            drake::examples::yaskawa_arm::stateVec_t xinit,xgoal;
            xinit << iiwa_q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            xgoal << ik_res.q_sol[1], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            DRAKE_DEMAND(xinit.size() == 12);
            DRAKE_DEMAND(xgoal.size() == 12);

            ddp.RunUDP(xinit, xgoal);

            drake::log()->info("outputting initial angels:");
            for(int i = 0; i < 6; i++){
                drake::log()->info(iiwa_q[i]);
            }

            drake::log()->info("outputting final angels:");
            for(int i = 0; i < 6; i++){
                drake::log()->info(ik_res.q_sol[1][i]);
            }

        #else

            std::vector<double> times;
            times.push_back(0);
            times.push_back(time);

            drake::log()->info("executing ik result");
            if (ik_res.info[0] == 1) {
                drake::log()->info("IK sol size {}", ik_res.q_sol.size());
                yaskawa_update_ = false;   

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

                drake::log()->info("publishing");
                lcm_.publish(FLAGS_lcm_plan_channel, &plan);
            }
        #endif

        drake::log()->info("time start: {}",ee_status_.utime / 1e6);
        // const double final_time = static_cast<double>(ee_status_.utime / 1e6) + time;

        //Don't finish function until the arm finishes its movement
        while(lcm_.handle() >= 0 && !is_at_desired_position(des_pos)){
            // drake::log()->info("final time : {}\n{}",final_time,ee_status_.utime/ 1e6);
        };

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
        // drake::log()->info("acutal pusher: {}",ee_status_.actual_pusher_position);
        // drake::log()->info("actual puller: {}",ee_status_.actual_puller_position);

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
            // const math::RollPitchYaw<double> rpy(ee_pose.rotation());
            // drake::log()->info("End effector at: {} {}",
            //                     ee_pose.translation().transpose(),
            //                     rpy.vector().transpose());
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
    bool PrimitiveExecutor::is_at_desired_position(VectorX<double> des_pos){    //TODO:Forward Kinematics

        DRAKE_DEMAND(des_pos.size() == 6);

        const math::RollPitchYaw<double> rpy(ee_pose.rotation());

        drake::log()->info("\ntranslation-des_pos: \n{} \n{} \n{}\nrotation: \n{}\n{}\n{}",
            ee_pose.translation()[0]-des_pos[0],
            ee_pose.translation()[1]-des_pos[1],
            ee_pose.translation()[2]-des_pos[2],
            rpy.vector()[0]-des_pos[3],
            rpy.vector()[1]-des_pos[4],
            rpy.vector()[2]-des_pos[5]
        );

        const double buffer = 0.05; //Temporary value

        for(int i = 0; i < 6; i++){
            if(i < 3 && abs(des_pos[i] - ee_pose.translation()[i]) > buffer){
                drake::log()->info("failing at: {} {}",i,des_pos[i] - ee_pose.translation()[i]);
                return 0;
            }
            else if(i >= 3 && abs(des_pos[i] - rpy.vector()[i-3]) > buffer){
                drake::log()->info("failing at: {} {}",i,des_pos[i] - rpy.vector()[i-3]);
                return 0;
            }
        }
        drake::log()->info("Finished moving");

        return 1;
    }
    bool PrimitiveExecutor::are_whiskers_up(){
        drake::log()->info("whisker angle: {} {}",ee_status_.actual_whisker_angle, FLAGS_whiskers_up_angle);
        return ee_status_.actual_whisker_angle >= FLAGS_whiskers_up_angle;
    }
    bool PrimitiveExecutor::are_whiskers_down(){
        drake::log()->info("whisker angle: {} {}",ee_status_.actual_whisker_angle, FLAGS_whiskers_down_angle);
        return -ee_status_.actual_whisker_angle <= FLAGS_whiskers_down_angle;
    }
    bool PrimitiveExecutor::is_carrying(){
        return has_package;
    }
    bool PrimitiveExecutor::is_at_shelf(){
        VectorX<double> atShelf(6);
        atShelf << 0.6,0.75,1.7,0,0.4,1.65;     //Approximation for now
        return is_at_desired_position(atShelf) ? 1 : 0;
    }
    bool PrimitiveExecutor::is_at_belt(){   
        VectorX<double> atBelt(6);
        atBelt << 1.65,0.1,0.9,0,0,1.57;        //Approximation for now
        return is_at_desired_position(atBelt) ? 1 : 0;
    }

    bool PrimitiveExecutor::is_pusher_back(){
        drake::log()->info("is pusher back? : {}",ee_status_.actual_pusher_position);
        return ee_status_.actual_pusher_position <= 0.05;
    }
    bool PrimitiveExecutor::is_puller_back(){
        drake::log()->info("is puller back? : {}",ee_status_.actual_puller_position);

        return ee_status_.actual_puller_position <= -0.01;
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
    bool PrimitiveExecutor::is_pickable(){  //TODO: Finish this function
        return 1;
    }
    bool PrimitiveExecutor::is_at_package_location(double x){
        VectorX<double> atPkg(6);
        atPkg << 1.65+x,0.1,0.9,0,0,1.57;
        return is_at_desired_position(atPkg) ? 1 : 0;
    }

    void PrimitiveExecutor::printer(bool desPos, bool whiskUp, bool whiskDwn, bool carry, bool atShlf,  
                                    bool pshBck, bool pllBck, bool atBlt, bool isMvng, bool isPckble, bool atPkgLoc){
        drake::log()->info("Check printer starts below:");
        VectorX<double> atBelt(6);
        atBelt << 1.65,0.1,0.9,0,0,1.57;
        if(desPos)   
            drake::log()->info("    at desired position: {}",is_at_desired_position(atBelt));
        if(whiskUp)
            drake::log()->info("    whiskers up: {}",are_whiskers_up());
        if(whiskDwn)
            drake::log()->info("    whiskers down: {}",are_whiskers_down());
        if(carry)
            drake::log()->info("    is carrying: {}",is_carrying());
        if(atShlf)
            drake::log()->info("    at shelf: {}",is_at_shelf());
        if(pshBck)
            drake::log()->info("    push is back: {}",is_pusher_back());
        if(pllBck)
            drake::log()->info("    pull is back: {}",is_puller_back());
        if(atBlt)
            drake::log()->info("    is at belt: {}",is_at_belt());
        if(isMvng)
            drake::log()->info("    is moving: {}",is_moving());
        if(isPckble)
            drake::log()->info("    is pickable: {}",is_pickable());
        if(atPkgLoc)
            drake::log()->info("    is at package location: {}",is_at_package_location(0.0));

        drake::log()->info("Printer end");

    }

}
}
}