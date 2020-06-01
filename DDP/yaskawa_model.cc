// #include "drake/DDP/yaskawa_model.h"
DEFINE_string(lcm_iiwa_status_channel, "IIWA_STATUS",
              "Channel on which to listen for lcmt_iiwa_status messages.");
// DEFINE_string(ee_name, "ee_mount", "Name of the end effector link");


namespace drake {
namespace examples {
namespace yaskawa_arm {

const char* const yaskawa_urdf_ =
    "drake/manipulation/models/yaskawa_description/urdf/yaskawa_with_model_collision.urdf";
const char* const ee_urdf_ = 
    "drake/manipulation/models/yaskawa_end_effector_description/urdf/end_effector_3.urdf";

// yaskawa_dt = time step
// yaskawa_N = number of knots
// yaskawa_xgoal = final goal in state space (7pos, 7vel)
YaskawaModel::YaskawaModel(double& yaskawa_dt, unsigned int& yaskawa_N, stateVec_t& yaskawa_xgoal)
{

    //Link a subscription function to the channel
    lcm_.subscribe(FLAGS_lcm_iiwa_status_channel, &YaskawaModel::HandleStatusYaskawa, this);

    arm_updated = false;
    //#####
    globalcnt = 0;
    //#####
    stateNb = 12;
    commandNb = 6;
    dt = yaskawa_dt;
    N = yaskawa_N;
    xgoal = yaskawa_xgoal;
    fxList.resize(N);
    fuList.resize(N);

    fxxList.resize(stateSize);
    for(unsigned int i=0;i<stateSize;i++)
        fxxList[i].resize(N);
    fxuList.resize(commandSize);
    fuuList.resize(commandSize);
    for(unsigned int i=0;i<commandSize;i++){
        fxuList[i].resize(N);
        fuuList[i].resize(N);
    }

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    lowerCommandBounds << -50.0;
    upperCommandBounds << 50.0;

    H.setZero();
    C.setZero();
    G.setZero();
    Bu.setZero();
    velocity.setZero();
    accel.setZero();
    Xdot_new.setZero();
    // Xdot_new_thread.resize(NUMBER_OF_THREAD);
    // vd_thread.resize(NUMBER_OF_THREAD);

    A1.setZero();
    A2.setZero();
    A3.setZero();
    A4.setZero();
    B1.setZero();
    B2.setZero();
    B3.setZero();
    B4.setZero();
    IdentityMat.setIdentity();

    Xp1.setZero();
    Xp2.setZero();
    Xp3.setZero();
    Xp4.setZero();

    Xm1.setZero();
    Xm2.setZero();
    Xm3.setZero();
    Xm4.setZero();

    AA.setZero();
    BB.setZero();
    A_temp.resize(N);
    B_temp.resize(N);
    
    debugging_print = 0;
    finalTimeProfile.counter0_ = 0;
    finalTimeProfile.counter1_ = 0;
    finalTimeProfile.counter2_ = 0;

    initial_phase_flag_ = 1;
    q.resize(stateSize/2);
    qd.resize(stateSize/2);
    // q_thread.resize(NUMBER_OF_THREAD);
    // qd_thread.resize(NUMBER_OF_THREAD);
    // for(unsigned int i=0;i<NUMBER_OF_THREAD;i++){
    //     q_thread[i].resize(stateSize/2);
    //     qd_thread[i].resize(stateSize/2);
    // }

    finalTimeProfile.time_period1 = 0;
    finalTimeProfile.time_period2 = 0;
    finalTimeProfile.time_period3 = 0;
    finalTimeProfile.time_period4 = 0;

    if(initial_phase_flag_ == 1){
        auto plant_ = std::make_unique<multibody::MultibodyPlant<double>>(0.0);

        const math::RigidTransform<double> X_WI = math::RigidTransform<double>::Identity();
        const math::RigidTransform<double> X_6G(math::RollPitchYaw<double>(0, 0, 0),
                                            Eigen::Vector3d(0, 0, 0.114));

        DRAKE_THROW_UNLESS(!plant_->HasModelInstanceNamed("yaskawa"));
        DRAKE_THROW_UNLESS(!plant_->HasModelInstanceNamed("gripper"));

        drake::multibody::Parser parser(plant_.get());

        yaskawa_model_idx =
            parser.AddModelFromFile(yaskawa_urdf_, "yaskawa");
        ee_model_idx =
            parser.AddModelFromFile(ee_urdf_, "gripper");   

        // Add EE to model
        const drake::multibody::Frame<double>& ee_frame =
            plant_->GetFrameByName("ee_mount", yaskawa_model_idx);

        //Weld models
        plant_->WeldFrames(plant_->world_frame(), plant_->GetFrameByName("arm_base", yaskawa_model_idx), X_WI);
        plant_->WeldFrames(ee_frame, plant_->GetFrameByName("ee_base", ee_model_idx), X_6G);

        plant_->Finalize();
        drake::log()->info("Plant finalized");

        //Define context. Use context to get state of models
        context_ = plant_->CreateDefaultContext();

        initial_phase_flag_ = 0;
    }
}

YaskawaModel::YaskawaModel(double& yaskawa_dt, unsigned int& yaskawa_N, stateVec_t& yaskawa_xgoal, std::unique_ptr<MultibodyPlant<double>>& plant_, drake::multibody::ModelInstanceIndex idx)
{
    //Link a subscription function to the channel
    lcm_.subscribe(FLAGS_lcm_iiwa_status_channel, &YaskawaModel::HandleStatusYaskawa, this);
    arm_updated = false;

    //#####
    globalcnt = 0;
    //#####
    stateNb = 12;
    commandNb = 6;
    dt = yaskawa_dt;
    N = yaskawa_N;
    xgoal = yaskawa_xgoal;
    fxList.resize(N);
    fuList.resize(N);

    fxxList.resize(stateSize);
    for(unsigned int i=0;i<stateSize;i++)
        fxxList[i].resize(N);
    fxuList.resize(commandSize);
    fuuList.resize(commandSize);
    for(unsigned int i=0;i<commandSize;i++){
        fxuList[i].resize(N);
        fuuList[i].resize(N);
    }

    fxx[0].setZero();
    fxx[1].setZero();
    fxx[2].setZero();
    fxx[3].setZero();
    fuu[0].setZero();
    fux[0].setZero();
    fxu[0].setZero();

    lowerCommandBounds << -50.0;
    upperCommandBounds << 50.0;

    H.setZero();
    C.setZero();
    G.setZero();
    Bu.setZero();
    velocity.setZero();
    accel.setZero();
    Xdot_new.setZero();
    // Xdot_new_thread.resize(NUMBER_OF_THREAD);
    // vd_thread.resize(NUMBER_OF_THREAD);

    A1.setZero();
    A2.setZero();
    A3.setZero();
    A4.setZero();
    B1.setZero();
    B2.setZero();
    B3.setZero();
    B4.setZero();
    IdentityMat.setIdentity();

    Xp1.setZero();
    Xp2.setZero();
    Xp3.setZero();
    Xp4.setZero();

    Xm1.setZero();
    Xm2.setZero();
    Xm3.setZero();
    Xm4.setZero();

    AA.setZero();
    BB.setZero();
    A_temp.resize(N);
    B_temp.resize(N);
    
    debugging_print = 0;
    finalTimeProfile.counter0_ = 0;
    finalTimeProfile.counter1_ = 0;
    finalTimeProfile.counter2_ = 0;

    initial_phase_flag_ = 1;
    q.resize(stateSize/2);
    qd.resize(stateSize/2);
    // q_thread.resize(NUMBER_OF_THREAD);
    // qd_thread.resize(NUMBER_OF_THREAD);
    // for(unsigned int i=0;i<NUMBER_OF_THREAD;i++){
    //     q_thread[i].resize(stateSize/2);
    //     qd_thread[i].resize(stateSize/2);
    // }

    finalTimeProfile.time_period1 = 0;
    finalTimeProfile.time_period2 = 0;
    finalTimeProfile.time_period3 = 0;
    finalTimeProfile.time_period4 = 0;

    if(initial_phase_flag_ == 1){
        context_ = plant_->CreateDefaultContext();

        robot_thread_ = std::move(plant_);        
        yaskawa_model_idx = idx;
        initial_phase_flag_ = 0;
    }
}

stateVec_t YaskawaModel::yaskawa_arm_dynamics(const stateVec_t& X, const commandVec_t& tau)
{

    finalTimeProfile.counter0_ += 1;

    if(finalTimeProfile.counter0_ == 10)
        gettimeofday(&tbegin_period,NULL);

    q << X.head(stateSize/2);
    qd << X.tail(stateSize/2);
    
    if(WHOLE_BODY){
        Eigen::Matrix<double,stateSize/2+2,1> q_full;
        Eigen::Matrix<double,stateSize/2+2,1> qd_full;
        q_full.setZero();
        qd_full.setZero();
        q_full.topRows(stateSize/2)=q;
        qd_full.topRows(stateSize/2)=qd;

/*
        // VectorX<double> q_0 = VectorX<double>::Zero(9);
        // VectorX<double> qd_0 = VectorX<double>::Zero(9);
        // VectorX<double> qd_0 = VectorX<double>::Zero(9);

    // KinematicsCache<double> cache_ = robot_thread_->CreateKinematicsCache();
    // cache_.initialize(q_full,qd_full);
    // robot_thread_->doKinematics(cache_, true);

        //const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
        //gettimeofday(&tbegin_period,NULL);

    // MatrixX<double> M_ = robot_thread_->massMatrix(cache_); // Inertial matrix


        //gettimeofday(&tend_period,NULL);
        //finalTimeProfile.time_period2 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;

        //gettimeofday(&tbegin_period,NULL);
    // drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext;
        //gettimeofday(&tend_period,NULL);
        //finalTimeProfile.time_period3 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        
        //gettimeofday(&tbegin_period,NULL);
        
    // VectorX<double> bias_term_ = robot_thread_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda
        //    VectorX<double> bias_term_2 = robot_thread_->dynamicsBiasTerm(cache_, f_ext, false);
        //gettimeofday(&tend_period,NULL);
        //finalTimeProfile.time_period4 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
*/

        //Wait for Yaskawa arm status to be received
        while(lcm_.handle() >= 0 && !arm_updated);
        arm_updated = false;

        //Find out if tau should be 6 or a different num
        const Eigen::Ref<const VectorX<double>>& u_instance = tau;
        //drake::log()->info("Tau size: {}",tau.size());

        const int wholeBodyActuationSize = robot_thread_->num_velocities();

        Eigen::Matrix<double, Eigen::Dynamic, 1> u(wholeBodyActuationSize);
        robot_thread_->SetActuationInArray(yaskawa_model_idx,u_instance,&u);

        MatrixX<double> M(wholeBodyActuationSize,wholeBodyActuationSize);
        Eigen::Matrix<double, Eigen::Dynamic, 1> bias_term(
            wholeBodyActuationSize, 1);

        robot_thread_->CalcMassMatrixViaInverseDynamics(*context_,&M);
        robot_thread_->CalcBiasTerm(*context_,&bias_term);
        MatrixX<double> M_ = M;
        VectorX<double> bias_term_ = bias_term;
        vd = (M_.inverse()*(tau - bias_term_));

        Xdot_new << qd, vd;
        
        if(finalTimeProfile.counter0_ == 10){
            gettimeofday(&tend_period,NULL);
            finalTimeProfile.time_period1 += (static_cast<double>(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        }
        
        if ((globalcnt%4 == 0) && (globalcnt<40)) {
            // cout << "=== q ===" << endl << q << endl;
            // cout << "=== qd ===" << endl << qd << endl;
            // cout << "=== kinematic cache Q ===" << endl<< cache_.getQ() <<endl;
            // cout << "===kinematic cache V ===" << endl<< cache_.getV() <<endl;
            // cout << "=== M ===: " << endl << M_ << endl;
            // cout << "===Bias ==:" << endl << bias_term_ << endl;
            // cout << "=== gtau=== : " << endl << gtau << endl;
            // cout << "=== tau=== : " << endl << tau << endl;
            // cout << "size::" <<  f_ext.size() << endl;
            // for (auto& x: f_ext) {
            //     cout << x.first << ": " << x.second << endl;
            // }
        }
        if (globalcnt < 40)
            globalcnt += 1;

        // vdot is newly calculated using q, qdot, u
        // (qdot, vdot) = f((q, qdot), u) ??? Makes sense??
    }
    else{
    // KinematicsCache<double> cache_ = robot_thread_->CreateKinematicsCache();
    // cache_.initialize(q,qd);
    // robot_thread_->doKinematics(cache_, true);

        //const RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
        //gettimeofday(&tbegin_period,NULL);
    //  MatrixX<double> M_ = robot_thread_->massMatrix(cache_); // Inertial matrix



        //gettimeofday(&tend_period,NULL);
        //finalTimeProfile.time_period2 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;

        //gettimeofday(&tbegin_period,NULL);
    //  drake::eigen_aligned_std_unordered_map<RigidBody<double> const*, drake::TwistVector<double>> f_ext;
        //gettimeofday(&tend_period,NULL);
        //finalTimeProfile.time_period3 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        
        //gettimeofday(&tbegin_period,NULL);
        
        // VectorX<double> bias_term_ = robot_thread_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda
    //  VectorX<double> bias_term_ = robot_thread_->dynamicsBiasTerm(cache_, f_ext);  // Bias term: M * vd + h = tau + J^T * lambda
        

        
        //    VectorX<double> bias_term_2 = robot_thread_->dynamicsBiasTerm(cache_, f_ext, false);
        //gettimeofday(&tend_period,NULL);
        //finalTimeProfile.time_period4 += ((double)(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        drake::log()->info("Not Whole body");
        while(lcm_.handle() >= 0 && !arm_updated);
        arm_updated = false;

        EigenPtr<MatrixX<double>> M;
        EigenPtr<VectorX<double>> bias_term;
        robot_thread_->CalcMassMatrixViaInverseDynamics(*context_,M);
        robot_thread_->CalcBiasTerm(*context_,bias_term);
        MatrixX<double> M_ = *M;
        VectorX<double> bias_term_ = *bias_term;
        vd = (M_.inverse()*(tau - bias_term_));

        Xdot_new << qd, vd;
        
        if(finalTimeProfile.counter0_ == 10){
            gettimeofday(&tend_period,NULL);
            finalTimeProfile.time_period1 += (static_cast<double>(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
        }
        
        if ((globalcnt%4 == 0) && (globalcnt<40)) {
            // cout << "=== q ===" << endl << q << endl;
            // cout << "=== qd ===" << endl << qd << endl;
            // cout << "=== kinematic cache Q ===" << endl<< cache_.getQ() <<endl;
            // cout << "===kinematic cache V ===" << endl<< cache_.getV() <<endl;
            // cout << "=== M ===: " << endl << M_ << endl;
            // cout << "===Bias ==:" << endl << bias_term_ << endl;
            // cout << "=== gtau=== : " << endl << gtau << endl;
            // cout << "=== tau=== : " << endl << tau << endl;
            // cout << "size::" <<  f_ext.size() << endl;
            // for (auto& x: f_ext) {
            //     cout << x.first << ": " << x.second << endl;
            // }
        }
        if (globalcnt < 40)
            globalcnt += 1;

        // vdot is newly calculated using q, qdot, u
        // (qdot, vdot) = f((q, qdot), u) ??? Makes sense??
    }
    return Xdot_new;
}

YaskawaModel::timeprofile YaskawaModel::getFinalTimeProfile()
{    
    return finalTimeProfile;
}

void YaskawaModel::yaskawa_arm_dyn_cst_ilqr(const int& nargout, const stateVecTab_t& xList, const commandVecTab_t& uList, stateVecTab_t& FList, 
                                CostFunctionYaskawa*& costFunction){
    // // for a positive-definite quadratic, no control cost (indicated by the iLQG function using nans), is equivalent to u=0
    if(debugging_print) TRACE_YASKAWA_ARM("initialize dimensions\n");
    unsigned int Nl = xList.size();
    
    costFunction->getc() = 0;
    AA.setZero();
    BB.setZero();

    if(debugging_print) TRACE_YASKAWA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;

    if(nargout == 2){
        const int nargout_update1 = 3;        
        for(unsigned int k=0;k<Nl;k++){
            if(k == Nl-1){//isNanVec(uList[k])
                if(debugging_print) TRACE_YASKAWA_ARM("before the update1\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose()) * costFunction->getQf() * (xList[k] - xgoal);
                costFunction->getc() += c_mat_to_scalar(0,0);
                if(debugging_print) TRACE_YASKAWA_ARM("after the update1\n");
            }else{
                // if u is not NaN (not final), add state and control cost
                if(debugging_print) TRACE_YASKAWA_ARM("before the update2\n");
                FList[k] = update(nargout_update1, xList[k], uList[k], AA, BB);
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal);
                if(debugging_print) TRACE_YASKAWA_ARM("after the update2\n");
                c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k];
                costFunction->getc() += c_mat_to_scalar(0,0);
            }
        }
    }
    else {
        const int nargout_update2 = 3;
        for(unsigned int k=0;k<Nl;k++) {
            if(k == Nl-1) {//isNanVec(uList[k])
                // if(debugging_print) TRACE_YASKAWA_ARM("before the update3\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQf()*(xList[k] - xgoal);
                costFunction->getc() += c_mat_to_scalar(0,0);
                // if(debugging_print) TRACE_YASKAWA_ARM("after the update3\n");
            }
            else {
                // if(debugging_print) TRACE_YASKAWA_ARM("before the update4\n");
                FList[k] = update(nargout_update2, xList[k], uList[k], AA, BB);//assume three outputs, code needs to be optimized
                // if(debugging_print) TRACE_YASKAWA_ARM("before the update4-1\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal);
                // if(debugging_print) TRACE_YASKAWA_ARM("after the update4\n");

                c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k];
                costFunction->getc() += c_mat_to_scalar(0,0); // TODO: to be checked
                // if(debugging_print) TRACE_YASKAWA_ARM("after the update5\n");
                
                A_temp[k] = AA;
                B_temp[k] = BB;
            }
        }

        stateVec_t cx_temp;
        
        if(debugging_print) TRACE_YASKAWA_ARM("compute dynamics and cost derivative\n");

        for(unsigned int k=0;k<Nl-1;k++){
            fxList[k] = A_temp[k];
            fuList[k] = B_temp[k];
            
            // cx_temp << xList[k](0,0)-xgoal(0), xList[k](1,0)-xgoal(1), xList[k](2,0)-xgoal(2), xList[k](3,0)-xgoal(3);
            cx_temp << xList[k] - xgoal;
            
            costFunction->getcx()[k] = costFunction->getQ()*cx_temp;
            costFunction->getcu()[k] = costFunction->getR()*uList[k];
            costFunction->getcxx()[k] = costFunction->getQ();
            costFunction->getcux()[k].setZero();
            costFunction->getcuu()[k] = costFunction->getR();            
        }
        if(debugging_print) TRACE_YASKAWA_ARM("update the final value of cost derivative \n");

        costFunction->getcx()[Nl-1] = costFunction->getQf()*(xList[Nl-1]-xgoal);
        costFunction->getcu()[Nl-1] = costFunction->getR()*uList[Nl-1];
        costFunction->getcxx()[Nl-1] = costFunction->getQf();
        costFunction->getcux()[Nl-1].setZero();
        costFunction->getcuu()[Nl-1] = costFunction->getR();

        // for (unsigned int a=0; a<Nl; a++) {
        //     cout << "cost derivative: " << a << endl;
        //     cout << costFunction->getcx()[a] << endl;
        //     cout << "xList: " << a << endl;
        //     cout << xList[a] << endl;
        // }

        if(debugging_print) TRACE_YASKAWA_ARM("set unused matrices to zero \n");

        // the following useless matrices are set to Zero.
        //fxx, fxu, fuu are not defined since never used
        for(unsigned int k=0;k<Nl;k++){
            FList[k].setZero();
        }
        costFunction->getc() = 0;
    }
    if(debugging_print) TRACE_YASKAWA_ARM("finish kuka_arm_dyn_cst\n");
}

//****** ALERT ***** nargout and dt_p was not used in the function so the parameters has been changed to accompany that*****
void YaskawaModel::yaskawa_arm_dyn_cst_min_output(const int& , const double& , const stateVec_t& xList_curr, const commandVec_t& uList_curr, const bool& isUNan, stateVec_t& xList_next, CostFunctionYaskawa*& costFunction){
    if(debugging_print) TRACE_YASKAWA_ARM("initialize dimensions\n");
    unsigned int Nc = xList_curr.cols(); //xList_curr is 14x1 vector -> col=1

    costFunction->getc() = 0; // temporary cost container? initializes every timestep
    AA.setZero(); 
    BB.setZero();

    if(debugging_print) TRACE_YASKAWA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;
    xList_next.setZero(); // zeroing previous trajectory timestep by timestep

    const int nargout_update1 = 1;   

    //How come Nc = 1 (Nc is the num of col of current state. Defined right above)
    for(unsigned int k=0;k<Nc;k++) {

        //Comes here only on last pass(probably)
        if (isUNan) { 
            // cout << "R: " <<  costFunction->getR() << endl;
            // cout << "Q: " <<  costFunction->getQ() << endl;
            // cout << "QF: " <<  costFunction->getQf() << endl;
            // if(debugging_print) TRACE_YASKAWA_ARM("before the update1\n");

            //Cost function for Qfinal
            c_mat_to_scalar = 0.5*(xList_curr.transpose() - xgoal.transpose())*costFunction->getQf()*(xList_curr - xgoal);
            costFunction->getc() += c_mat_to_scalar(0,0);
            // if(debugging_print) TRACE_YASKAWA_ARM("after the update1\n");
        }
        else {  
            //Call parareal
            //xList_next = parareal(nargout,dt,xList_curr,uList_curr,isUNan,xList_next,costFunction);


            //Is this correct?
            //L(x,u) = 1/2 * (x - xd)ᵀ * Q * (x - xd) + 1/2 * (u - ud)ᵀ * R * (u - ud) + (x - xd)ᵀ * S * (u - ud)
            c_mat_to_scalar = 0.5*(xList_curr.transpose() - xgoal.transpose()) * costFunction->getQ()*(xList_curr - xgoal);
            c_mat_to_scalar += 0.5*uList_curr.transpose()*costFunction->getR() * uList_curr;

            costFunction->getc() += c_mat_to_scalar(0,0);
        }

    }
    if(debugging_print) TRACE_YASKAWA_ARM("finish kuka_arm_dyn_cst\n");
}

//The physics simulation in forward pass
stateVec_t YaskawaModel::update(const int& nargout, const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B){
    // 4th-order Runge-Kutta step
    if(debugging_print) TRACE_YASKAWA_ARM("update: 4th-order Runge-Kutta step\n");
    gettimeofday(&tbegin_period4,NULL);

    // output of kuka arm dynamics is xdot = f(x,u)
    Xdot1 = yaskawa_arm_dynamics(X, U);
    Xdot2 = yaskawa_arm_dynamics(X + 0.5*dt*Xdot1, U);
    Xdot3 = yaskawa_arm_dynamics(X + 0.5*dt*Xdot2, U);
    Xdot4 = yaskawa_arm_dynamics(X + dt*Xdot3, U);
    stateVec_t X_new;
    X_new = X + (dt/6)*(Xdot1 + 2*Xdot2 + 2*Xdot3 + Xdot4);
    // Simple Euler Integration (for debug)
    //    X_new = X + (dt)*Xdot1;
    
    //Since the if statement didn't have anything inside other than comments, it has been removed
    // if ((globalcnt%4 == 0) && (globalcnt<40)) {
    //     // cout << "X " << endl << X << endl;
    //     // cout << "Xdot1 " << endl << Xdot1 << endl;
    //     // cout << "Xdot2 " << endl << Xdot2 << endl;
    //     // cout << "Xdot3 " << endl << Xdot3 << endl;
    //     // cout << "Xdot4 " << endl << Xdot4 << endl;
    //     // cout << "X_NEW: " << endl << X_new << endl;
    // }
    if(debugging_print) TRACE_YASKAWA_ARM("update: X_new\n");

    //######3
    // int as = 0;
    //#########3

    if(nargout > 1){// && (as!=0)){
        //cout << "NEVER HERE" << endl;
        unsigned int n = X.size();
        unsigned int m = U.size();

        double delta = 1e-7;
        stateMat_t Dx;
        commandMat_t Du;
        Dx.setIdentity();
        Dx = delta*Dx;
        Du.setIdentity();
        Du = delta*Du;

        // State perturbation?
        for(unsigned int i=0;i<n;i++){
            Xp1 = yaskawa_arm_dynamics(X+Dx.col(i),U);
            Xm1 = yaskawa_arm_dynamics(X-Dx.col(i),U);
            A1.col(i) = (Xp1 - Xm1)/(2*delta);

            Xp2 = yaskawa_arm_dynamics(X+0.5*dt*Xdot1+Dx.col(i),U);
            Xm2 = yaskawa_arm_dynamics(X+0.5*dt*Xdot1-Dx.col(i),U);
            A2.col(i) = (Xp2 - Xm2)/(2*delta);

            Xp3 = yaskawa_arm_dynamics(X+0.5*dt*Xdot2+Dx.col(i),U);
            Xm3 = yaskawa_arm_dynamics(X+0.5*dt*Xdot2-Dx.col(i),U);
            A3.col(i) = (Xp3 - Xm3)/(2*delta);

            Xp4 = yaskawa_arm_dynamics(X+0.5*dt*Xdot3+Dx.col(i),U);
            Xm4 = yaskawa_arm_dynamics(X+0.5*dt*Xdot3-Dx.col(i),U);
            A4.col(i) = (Xp4 - Xm4)/(2*delta);
        }

        // Control perturbation?
        for(unsigned int i=0;i<m;i++){
            Xp1 = yaskawa_arm_dynamics(X,U+Du.col(i));
            Xm1 = yaskawa_arm_dynamics(X,U-Du.col(i));
            B1.col(i) = (Xp1 - Xm1)/(2*delta);

            Xp2 = yaskawa_arm_dynamics(X+0.5*dt*Xdot1,U+Du.col(i));
            Xm2 = yaskawa_arm_dynamics(X+0.5*dt*Xdot1,U-Du.col(i));
            B2.col(i) = (Xp2 - Xm2)/(2*delta);

            Xp3 = yaskawa_arm_dynamics(X+0.5*dt*Xdot2,U+Du.col(i));
            Xm3 = yaskawa_arm_dynamics(X+0.5*dt*Xdot2,U-Du.col(i));
            B3.col(i) = (Xp3 - Xm3)/(2*delta);

            Xp4 = yaskawa_arm_dynamics(X+0.5*dt*Xdot3,U+Du.col(i));
            Xm4 = yaskawa_arm_dynamics(X+0.5*dt*Xdot3,U-Du.col(i));
            B4.col(i) = (Xp4 - Xm4)/(2*delta);
        }

        A = (IdentityMat + A4 * dt/6)*(IdentityMat + A3 * dt/3)*(IdentityMat + A2 * dt/3)*(IdentityMat + A1 * dt/6);
        B = B4 * dt/6 + (IdentityMat + A4 * dt/6) * B3 * dt/3 + (IdentityMat + A4 * dt/6)*(IdentityMat + A3 * dt/3)* B2 * dt/3 + (IdentityMat + (dt/6)*A4)*(IdentityMat + (dt/3)*A3)*(IdentityMat + (dt/3)*A2)*(dt/6)*B1;
    }
    if(debugging_print) TRACE_YASKAWA_ARM("update: X_new\n");
    gettimeofday(&tend_period4,NULL);
    finalTimeProfile.time_period4 += (static_cast<double>(1000.0*(tend_period4.tv_sec-tbegin_period4.tv_sec)+((tend_period4.tv_usec-tbegin_period4.tv_usec)/1000.0)))/1000.0;

    return X_new;
}

void YaskawaModel::grad(const stateVec_t& X, const commandVec_t& U, stateMat_t& A, stateR_commandC_t& B){
    unsigned int n = X.size();
    unsigned int m = U.size();

    double delta = 1e-7;
    stateMat_t Dx;
    commandMat_t Du;
    Dx.setIdentity();
    Dx = delta*Dx;
    Du.setIdentity();
    Du = delta*Du;

    AA.setZero();
    BB.setZero();

    int nargout = 1;
    for(unsigned int i=0;i<n;i++){
        Xp = update(nargout, X+Dx.col(i), U, AA, BB);
        Xm = update(nargout, X-Dx.col(i), U, AA, BB);
        A.col(i) = (Xp - Xm)/(2*delta);
    }

    for(unsigned int i=0;i<m;i++){
        Xp = update(nargout, X, U+Du.col(i), AA, BB);
        Xm = update(nargout, X, U-Du.col(i), AA, BB);
        B.col(i) = (Xp - Xm)/(2*delta);
    }
}

// parameters are called by reference. Name doesn't matter
void YaskawaModel::hessian(const stateVec_t& X, const commandVec_t& U, stateTens_t& fxx_p, stateR_stateC_commandD_t& fxu_p, stateR_commandC_commandD_t& fuu_p){
    unsigned int n = X.size();
    unsigned int m = U.size();

    double delta = 1e-5;
    stateMat_t Dx;
    commandMat_t Du;
    Dx.setIdentity();
    Dx = delta*Dx;
    Du.setIdentity();
    Du = delta*Du;

    stateMat_t Ap;
    Ap.setZero();
    stateMat_t Am;
    Am.setZero();
    stateR_commandC_t B;
    B.setZero();

    for(unsigned int i=0;i<n;i++){
        fxx_p[i].setZero();
        fxu_p[i].setZero();
        fuu_p[i].setZero();
    }

    for(unsigned int i=0;i<n;i++){
        grad(X+Dx.col(i), U, Ap, B);
        grad(X-Dx.col(i), U, Am, B);
        fxx_p[i] = (Ap - Am)/(2*delta);
    }

    stateR_commandC_t Bp;
    Bp.setZero();
    stateR_commandC_t Bm;
    Bm.setZero();

    for(unsigned int j=0;j<m;j++){
        grad(X, U+Du.col(j), Ap, Bp);
        grad(X, U-Du.col(j), Am, Bm);
        fxu_p[j] = (Ap - Am)/(2*delta);
        fuu_p[j] = (Bp - Bm)/(2*delta);
    }
}

unsigned int YaskawaModel::getStateNb()
{
    return stateNb;
}

unsigned int YaskawaModel::getCommandNb()
{
    return commandNb;
}

commandVec_t& YaskawaModel::getLowerCommandBounds()
{
    return lowerCommandBounds;
}

commandVec_t& YaskawaModel::getUpperCommandBounds()
{
    return upperCommandBounds;
}

stateMatTab_t& YaskawaModel::getfxList()
{
    return fxList;
}

stateR_commandC_tab_t& YaskawaModel::getfuList()
{
    return fuList;
}

void YaskawaModel::yaskawa_arm_dyn_cst_udp(const int& nargout, const stateVecTab_t& xList, const commandVecTab_t& uList, stateVecTab_t& FList,
                                CostFunctionYaskawa*& costFunction){
    if(debugging_print) TRACE_YASKAWA_ARM("initialize dimensions\n");
    unsigned int Nl = xList.size();
    
    costFunction->getc() = 0;
    AA.setZero();
    BB.setZero();
    if(debugging_print) TRACE_YASKAWA_ARM("compute cost function\n");

    scalar_t c_mat_to_scalar;
    c_mat_to_scalar.setZero();

    if(nargout == 2){
        const int nargout_update1 = 3;        
        for(unsigned int k=0;k<Nl;k++){
            if (k == Nl-1){
                if(debugging_print) TRACE_YASKAWA_ARM("before the update1\n");
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose()) * costFunction->getQf() * (xList[k] - xgoal);
                costFunction->getc() += c_mat_to_scalar(0,0);
                if(debugging_print) TRACE_YASKAWA_ARM("after the update1\n");
            }else{
                if(debugging_print) TRACE_YASKAWA_ARM("before the update2\n");
                FList[k] = update(nargout_update1, xList[k], uList[k], AA, BB);
                c_mat_to_scalar = 0.5*(xList[k].transpose() - xgoal.transpose())*costFunction->getQ()*(xList[k] - xgoal);
                if(debugging_print) TRACE_YASKAWA_ARM("after the update2\n");
                c_mat_to_scalar += 0.5*uList[k].transpose()*costFunction->getR()*uList[k];
                costFunction->getc() += c_mat_to_scalar(0,0);
            }
        }
    }else{
        stateVec_t cx_temp;
        if(debugging_print) TRACE_YASKAWA_ARM("compute cost derivative\n");
        for(unsigned int k=0;k<Nl-1;k++){
            // cx_temp << xList[k](0,0)-xgoal(0), xList[k](1,0)-xgoal(1), xList[k](2,0)-xgoal(2), xList[k](3,0)-xgoal(3);
            cx_temp << xList[k] - xgoal;

            costFunction->getcx()[k] = costFunction->getQ()*cx_temp;
            costFunction->getcu()[k] = costFunction->getR()*uList[k];
            costFunction->getcxx()[k] = costFunction->getQ();
            costFunction->getcux()[k].setZero();
            costFunction->getcuu()[k] = costFunction->getR();            
        }
        if(debugging_print) TRACE_YASKAWA_ARM("update the final value of cost derivative \n");
        costFunction->getcx()[Nl-1] = costFunction->getQf()*(xList[Nl-1]-xgoal);
        costFunction->getcu()[Nl-1] = costFunction->getR()*uList[Nl-1];
        costFunction->getcxx()[Nl-1] = costFunction->getQf();
        costFunction->getcux()[Nl-1].setZero();
        costFunction->getcuu()[Nl-1] = costFunction->getR();
        if(debugging_print) TRACE_YASKAWA_ARM("set unused matrices to zero \n");

        // the following useless matrices and scalars are set to Zero.
        for(unsigned int k=0;k<Nl;k++){
            FList[k].setZero();
        }
        costFunction->getc() = 0;
    }
    if(debugging_print) TRACE_YASKAWA_ARM("finish kuka_arm_dyn_cst\n");
}

void YaskawaModel::HandleStatusYaskawa(const real_lcm::ReceiveBuffer*, const std::string&, const lcmt_iiwa_status* status){
    iiwa_status_ = *status;
    Eigen::VectorXd iiwa_q(status->num_joints);

    for (int i = 0; i < status->num_joints; i++) {
        iiwa_q[i] = status->joint_position_measured[i];
    }
    arm_updated = true;
    status_count_++;    //Increase this count for the if statement below

    robot_thread_->SetPositions(context_.get(),yaskawa_model_idx, iiwa_q);

    // if (status_count_ % 100 == 1) {
        
    //     // ee_pose = robot_thread_->EvalBodyPoseInWorld(
    //     //         *context_, robot_thread_->GetBodyByName(FLAGS_ee_name));
    //     // const math::RollPitchYaw<double> rpy(ee_pose.rotation());
    //     // drake::log()->info("End effector at: {} {}",
    //     //                     ee_pose.translation().transpose(),
    //     //                     rpy.vector().transpose());
    // }

}


}  // namespace yaskawa_arm
}  // namespace examples
}  // namespace drake
