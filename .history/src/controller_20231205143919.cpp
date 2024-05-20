#include "controller.h"

CController::CController()
{
	_k = 7;
	Initialize();
}

CController::~CController()
{
}



void CController::read(double t, double* q, double* qdot)
{	
	_t = t;
	if (_bool_init == true)
	{
		_init_t = _t;
		_bool_init = false;
	}

	_dt = t - _pre_t;
	// cout<<"_dt : "<<_dt<<endl;
	_pre_t = t;

	for (int i = 0; i < _k; i++)
	{
		_q(i) = q[i];
		_qdot(i) = qdot[i];
		// _qdot(i) = CustomMath::VelLowpassFilter(0.001, 2.0*PI* 10.0, _pre_q(i), _q(i), _pre_qdot(i)); //low-pass filter
		_pre_q(i) = _q(i);
		_pre_qdot(i) = _qdot(i);		
		if(_t < 2.0)///use filtered data after convergece
        {
			_qdot(i) = qdot[i];
		}
	}
}

void CController::write(double* torque)
{
	for (int i = 0; i < _k; i++)
	{
		torque[i] = _torque(i);
	}
}

void CController::control_mujoco()
{
    ModelUpdate();
    motionPlan();
	if(_control_mode == 1) //joint space control
	{
		if (_t - _init_t < 0.1 && _bool_joint_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			JointTrajectory.reset_initial(_start_time, _q, _qdot);
			JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);
			_bool_joint_motion = true;
		}
		
		JointTrajectory.update_time(_t);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();

		JointControl();

		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
	}
	else if(_control_mode == 2)
	{		
		if (_t - _init_t < 0.1 && _bool_ee_motion == false)
		{
			_start_time = _init_t;
			_end_time = _start_time + _motion_time;
			HandTrajectory.reset_initial(_start_time, _x_hand, _xdot_hand);
			HandTrajectory.update_goal(_x_goal_hand, _xdot_goal_hand, _end_time);
			_bool_ee_motion = true;
			cout<<"_t : "<<_t<<endl;
			// cout<<"_x_hand 	: "<<_x_hand.transpose()<<endl;
		}

		
		HandTrajectory.update_time(_t);
		_x_des_hand.head(3) = HandTrajectory.position_cubicSpline();
		_R_des_hand = HandTrajectory.rotationCubic();
		_x_des_hand.segment<3>(3) = CustomMath::GetBodyRotationAngle(_R_des_hand);
		_xdot_des_hand.head(3) = HandTrajectory.velocity_cubicSpline();
		_xdot_des_hand.segment<3>(3) = HandTrajectory.rotationCubicDot();		

		if (HandTrajectory.check_trajectory_complete() == 1)
		{
			_bool_plan(_cnt_plan) = 1;
			_bool_init = true;
		}
		CLIK();
	}
	
}

void CController::ModelUpdate()
{
    Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
    Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

	_J_hands = Model._J_hand;

	_x_hand.head(3) = Model._x_hand;
	_x_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_hand);
	_xdot_hand = Model._xdot_hand;
}	

void CController::motionPlan()
{	
	if (_bool_plan(_cnt_plan) == 1)
	{
		if(_cnt_plan == 0)
		{
			// _q_order(0) = 0.742;
			// _q_order(1) = -1.83;
			// _q_order(2) = -2.97;
			// _q_order(3) = -3.14;
			// _q_order(4) = -2.79;
			// _q_order(5) = 0.478;
			// _q_order(6) = 0.565;
			Vector3d target_pos;
			Vector3d target_ori;
			target_pos(0) = 0.5;
			target_pos(1) = 0.5;
			target_pos(2) = 0.5;
			target_ori(0) = 0 * DEG2RAD;
			target_ori(1) = 0 * DEG2RAD;
			target_ori(2) = 0 * DEG2RAD;

 			reset_target(10.0, target_pos, target_ori);
			_cnt_plan++;
		}

		else if(_cnt_plan == 1)
		{
			_q_order(0) = -1.87;
			_q_order(1) = -1.83;
			_q_order(2) = -2.55;
			_q_order(3) = -2.8;
			_q_order(4) = 0.623;
			_q_order(5) = 0.48;
			_q_order(6) = 0.564;		                    
			reset_target(10.0, _q_order, _qdot);
			_cnt_plan++;
		}
		else if(_cnt_plan == 2)
		{
			_q_order(0) = -1.87;
			_q_order(1) = -1.1;
			_q_order(2) = -1.16;
			_q_order(3) = -2.1;
			_q_order(4) = 0.89;
			_q_order(5) = 0.44;
			_q_order(6) = -0.386;		                    
			reset_target(10.0, _q_order, _qdot);
			_cnt_plan++;			
		}
		else if(_cnt_plan == 3)
		{
			_q_order(0) = -1.47;
			_q_order(1) = -0.5;
			_q_order(2) = -0.4;
			_q_order(3) = -1.3;
			_q_order(4) = 0.3;
			_q_order(5) = 0.1;
			_q_order(6) = -0.1;		                    
			reset_target(10.0, _q_order, _qdot);
			_cnt_plan++;			
		}
	}
}

void CController::reset_target(double motion_time, VectorXd target_joint_position)
{
	_control_mode = 3;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	// _q_goal = target_joint_position.head(7);
	// _qdot_goal.setZero();
}

void CController::reset_target(double motion_time, VectorXd target_joint_position, VectorXd target_joint_velocity)
{
	_control_mode = 1;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_goal = target_joint_position.head(7);
	// _qdot_goal = target_joint_velocity.head(7);
	_qdot_goal.setZero();
}

void CController::reset_target(double motion_time, Vector3d target_pos, Vector3d target_ori)
{
	_control_mode = 2;
	_motion_time = motion_time;
	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_x_goal_hand.head(3) = target_pos;
	_x_goal_hand.tail(3) = target_ori;
	_xdot_goal_hand.setZero();
}

void CController::JointControl()
{	
	_torque.setZero();
	_A_diagonal = Model._A;
	for(int i = 0; i < 7; i++){
		_A_diagonal(i,i) += 1.0;
	}
	_torque = _A_diagonal*(400*(_q_des - _q) + 40*(_qdot_des - _qdot)) + Model._bg;
	// cout<<"_q_des 	 : "<<_q_des.transpose()<<endl;
	// cout<<"_q 		 : "<<_q.transpose()<<endl;
	// cout<<"_qdot_des : "<<_qdot_des.transpose()<<endl;
	// cout<<"_qdot	 : "<<_qdot.transpose()<<endl<<endl;
}

void CController::CLIK()
{
	_torque.setZero();	

	_x_err_hand.segment(0,3) = _x_des_hand.head(3) - _x_hand.head(3);
	_x_err_hand.segment(3,3) = -CustomMath::getPhi(Model._R_hand, _R_des_hand);

	_J_bar_hands = CustomMath::pseudoInverseQR(_J_hands);

	_dt = 0.003;
	_qdot_des = _J_bar_hands*(_xdot_des_hand + _x_kp*(_x_err_hand));
	_q_des = _q_des + _dt*_qdot_des;
	_A_diagonal = Model._A;
	for(int i = 0; i < 7; i++){
		_A_diagonal(i,i) += 1.0;
	}
	// _torque(0) = 400*(_q_des(0)-_q(0)) + 20*(_qdot_des(0)-_qdot(0));
	// _torque(1) = 2500*(_q_des(1)-_q(1)) + 250*(_qdot_des(1)-_qdot(1));
	// _torque(2) = 1500*(_q_des(2)-_q(2)) + 170*(_qdot_des(2)-_qdot(2));
	// _torque(3) = 1700*(_q_des(3)-_q(3)) + 320*(_qdot_des(3)-_qdot(3));
	// _torque(4) = 700*(_q_des(4)-_q(4)) + 70*(_qdot_des(4)-_qdot(4));
	// _torque(5) = 500*(_q_des(5)-_q(5)) + 50*(_qdot_des(5)-_qdot(5));
	// _torque(6) = 520*(_q_des(6)-_q(6)) + 15*(_qdot_des(6)-_qdot(6));

	_torque = _A_diagonal * (400 * (_q_des - _q) + 40 * (_qdot_des - _qdot)) + Model._bg;

}

void CController::Initialize()
{
    _control_mode = 1; //1: joint space, 2: task space(CLIK)

	_bool_init = true;
	_t = 0.0;
	_init_t = 0.0;
	_pre_t = 0.0;
	_dt = 0.0;

	_kpj = 400.0;
	_kdj = 20.0;

	// _kpj_diagonal.setZero(_k, _k);
	// //							0 		1	2		3	   4	5 	6
	// _kpj_diagonal.diagonal() << 400., 2500., 1500., 1700., 700., 500., 520.;
	// _kdj_diagonal.setZero(_k, _k);
	// _kdj_diagonal.diagonal() << 20., 250., 170., 320., 70., 50., 15.;
	_x_kp =1;//작게 0.1
	// _x_kp = 20.0;

    _q.setZero(_k);
	_qdot.setZero(_k);
	_torque.setZero(_k);

	_J_hands.setZero(6,_k);
	_J_bar_hands.setZero(_k,6);

	_x_hand.setZero(6);
	_xdot_hand.setZero(6);

	//////////////////원본///////////////////
	// _cnt_plan = 0;
	_bool_plan.setZero(30);
	// _time_plan.resize(30);
	// _time_plan.setConstant(5.0);
	//////////////////원본///////////////////

	_q_home.setZero(_k);
	// _q_home(0) = 0.0;
	// _q_home(1) = -30.0 * DEG2RAD;
	// _q_home(2) = 30.0 * DEG2RAD;
	// _q_home(3) = -30.0 * DEG2RAD;
	// _q_home(4) = 30.0 * DEG2RAD;
	// _q_home(5) = -60.0 * DEG2RAD;
	// _q_home(6) = 30.0 * DEG2RAD;
	_q_home(0) = 0.374;
	_q_home(1) = -1.02;
	_q_home(2) = 0.245;
	_q_home(3) = -1.51;
	_q_home(4) = 0.0102;
	_q_home(5) = 0.655;
	_q_home(6) = 0.3;

	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;

	_bool_joint_motion = false;
	_bool_ee_motion = false;

	_q_des.setZero(_k);
	_qdot_des.setZero(_k);
	_q_goal.setZero(_k);
	_qdot_goal.setZero(_k);

	_x_des_hand.setZero(6);
	_xdot_des_hand.setZero(6);
	_x_goal_hand.setZero(6);
	_xdot_goal_hand.setZero(6);

	_pos_goal_hand.setZero(); // 3x1 
	_rpy_goal_hand.setZero(); // 3x1
	JointTrajectory.set_size(_k);
	_A_diagonal.setZero(_k,_k);

	_x_err_hand.setZero(6);
	_R_des_hand.setZero();

	_I.setIdentity(7,7);

	_pre_q.setZero(7);
	_pre_qdot.setZero(7);

	///////////////////save_stack/////////////////////
	_q_order.setZero(7);
	_qdot_order.setZero(7);
	// _max_joint_position.setZero(7);
	// _min_joint_position.setZero(7);

	// _min_joint_position(0) = -2.9671;
	// _min_joint_position(1) = -1.8326;
	// _min_joint_position(2) = -2.9671;
	// _min_joint_position(3) = -3.1416;
	// _min_joint_position(4) = -2.9671;
	// _min_joint_position(5) = -0.0873;
	// _min_joint_position(6) = -2.9671;

	// _max_joint_position(0) = 2.9671;
	// _max_joint_position(1) = 1.8326;
	// _max_joint_position(2) = 2.9671;
	// _max_joint_position(3) = 0.0;
	// _max_joint_position(4) = 2.9671;
	// _max_joint_position(5) = 3.8223;
	// _max_joint_position(6) = 2.9671;

	///////////////////estimate_lr/////////////////////

	// cout << fixed;
	// cout.precision(3);
	_cnt_plan = 0;
	_bool_plan(_cnt_plan) = 1;
}