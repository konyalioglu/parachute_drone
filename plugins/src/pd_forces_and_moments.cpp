
/*
*
*
*
*
*
*
*
*/

#include "../include/pd_forces_and_moments.hpp"



namespace gazebo
{


ParachuteDroneForcesAndMoments::ParachuteDroneForcesAndMoments() :
  ModelPlugin(), node_handle_(nullptr), prev_time(0)  {}


ParachuteDroneForcesAndMoments::~ParachuteDroneForcesAndMoments()
{
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


void ParachuteDroneForcesAndMoments::PublishForcesAndMoments()
{
    link_->AddRelativeForce(ignition::math::Vector3d(TOTAL_FM_.X, -TOTAL_FM_.Y, -TOTAL_FM_.Z));
    link_->AddRelativeTorque(ignition::math::Vector3d(TOTAL_FM_.L, -TOTAL_FM_.M, -TOTAL_FM_.N));
}


void ParachuteDroneForcesAndMoments::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    std::cerr << "\nThe Physics Plugin is attached as [" <<
        _model->GetName() << "]\n";


    model_ = _model;
    world_ = model_->GetWorld();      
    joint_ = _model->GetJoints()[0];


    if (_sdf->HasElement("mass")){mass_ = _sdf->Get<double>("mass");}

    if (_sdf->HasElement("ct")){ct_ = _sdf->Get<double>("ct");}

    if (_sdf->HasElement("ch")){ch_ = _sdf->Get<double>("ch");}

    if (_sdf->HasElement("Cd")){Cd_ = _sdf->Get<double>("Cd");}

    if (_sdf->HasElement("S")){S_ = _sdf->Get<double>("S");}

    if (_sdf->HasElement("rho")){rho_ = _sdf->Get<double>("rho");}

    if (_sdf->HasElement("cog_x")){cog_x_ = _sdf->Get<double>("cog_x");}

    if (_sdf->HasElement("cog_y")){cog_y_ = _sdf->Get<double>("cog_y");}

    if (_sdf->HasElement("cog_z")){cog_z_ = _sdf->Get<double>("cog_z");}

    if (_sdf->HasElement("cop_x")){cop_x_ = _sdf->Get<double>("cop_x");}

    if (_sdf->HasElement("cop_y")){cop_y_ = _sdf->Get<double>("cop_y");}

    if (_sdf->HasElement("cop_z")){cop_z_ = _sdf->Get<double>("cop_z");}

    if (_sdf->HasElement("r_motor1_y")){r_motor1_y_ = _sdf->Get<double>("r_motor1_y");}

    if (_sdf->HasElement("r_motor2_y")){r_motor2_y_ = _sdf->Get<double>("r_motor2_y");}

    if (_sdf->HasElement("gravity")){g_ = _sdf->Get<double>("gravity");}

    if (_sdf->HasElement("dref")){dref_ = _sdf->Get<double>("dref");}

    if (_sdf->HasElement("Cdd")){Cdd_ = _sdf->Get<double>("Cdd");}



    if (_sdf->HasElement("link_name"))
      link_name_ = _sdf->Get<std::string>("link_name");
    else
      gzerr << "[gazebo_aircraft_forces_and_moments] Please specify a linkName of the forces and moments plugin.\n";
      link_ = model_->GetLink(link_name_);
      std::cerr << "Link name is " << link_name_;
    if (link_ == NULL)
      gzthrow("[gazebo_aircraft_forces_and_moments] Couldn't find specified link \"" << link_name_ << "\".");


    R_COP_.Set(cop_x_, cop_y_, cop_z_);
    R_COG_.Set(cog_x_, cog_y_, cog_z_);
    PROP_R_1.Set(0, r_motor1_y_, 0);
    PROP_R_2.Set(0, r_motor2_y_, 0);


    std::cerr << "Mass is  " << mass_ << ". \n";
    std::cerr << "Thrust coefficient is  " << ct_ << ". \n";
    std::cerr << "Aerodynamic reference area is  " << S_ << " m. \n";
    std::cerr << "Center of Gravity Vector is  (" << R_COG_[0] << "," << R_COG_[1] << "," <<  R_COG_[2] <<  "). \n";
    std::cerr << "Center of Pressure Vector is  (" << R_COP_[0] << "," << R_COP_[1]<< "," <<  R_COP_[2] <<  "). \n";



    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "pd_forces_and_moments",
          ros::init_options::NoSigintHandler);
    }



    node_handle_ = new ros::NodeHandle("pd_forces_and_moments");

    prop_sub_ = node_handle_->subscribe("/" + this->model_->GetName() + "/joint_vel", 1, &ParachuteDroneForcesAndMoments::callBackFunction, this);

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ParachuteDroneForcesAndMoments::OnUpdate, this, _1));

}

void ParachuteDroneForcesAndMoments::callBackFunction(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  w1_ = msg->data.at(0);
  w2_ = msg->data.at(1);
}

void ParachuteDroneForcesAndMoments::OnUpdate(const common::UpdateInfo& _info) {
  ParachuteDroneForcesAndMoments::calculateForcesAndMoments();
}


void ParachuteDroneForcesAndMoments::calculateForcesAndMoments(){
  
  WORLD_COG_POSE_ = link_ -> WorldCoGPose();
  EUL_ANG_GAZEBO_ = WORLD_COG_POSE_.Rot().Euler();
  phi     =  EUL_ANG_GAZEBO_[0];
  theta   = -EUL_ANG_GAZEBO_[1];
  psi     = -EUL_ANG_GAZEBO_[2];


  LIN_VEL_GAZEBO_ = link_-> RelativeLinearVel();
  LIN_VEL_NED_[0] =  LIN_VEL_GAZEBO_[0];
  LIN_VEL_NED_[1] = -LIN_VEL_GAZEBO_[1];
  LIN_VEL_NED_[2] = -LIN_VEL_GAZEBO_[2];


  ANG_VEL_GAZEBO_ = link_-> RelativeAngularVel();
  ANG_VEL_NED_[0] =  ANG_VEL_GAZEBO_[0];
  ANG_VEL_NED_[1] = -ANG_VEL_GAZEBO_[1];
  ANG_VEL_NED_[2] = -ANG_VEL_GAZEBO_[2];


  ParachuteDroneForcesAndMoments::calculateGravitationalForces();  

  ParachuteDroneForcesAndMoments::calculateAerodynamicForcesAndMoments();

  ParachuteDroneForcesAndMoments::calculatePropulsiveForcesAndMoments();


  TOTAL_FM_.X = FG_[0] + FA_[0] + FP_[0];
  TOTAL_FM_.Y = FG_[1] + FA_[1] + FP_[1];
  TOTAL_FM_.Z = FG_[2] + FA_[2] + FP_[2];

  TOTAL_FM_.L = MA_[0] + MP_[0];
  TOTAL_FM_.M = MA_[1] + MP_[1];
  TOTAL_FM_.N = MA_[2] + MP_[2];


  link_->AddRelativeForce(ignition::math::Vector3d(TOTAL_FM_.X, -TOTAL_FM_.Y, -TOTAL_FM_.Z));
  
  link_->AddRelativeTorque(ignition::math::Vector3d(TOTAL_FM_.L, -TOTAL_FM_.M, -TOTAL_FM_.N));
}


void ParachuteDroneForcesAndMoments::calculateGravitationalForces(){
  FG_[0] = -mass_ * g_ * std::sin(theta);
  FG_[1] =  mass_ * g_ * std::cos(theta) * std::sin(phi);
  FG_[2] =  mass_ * g_ * std::cos(theta) * std::cos(phi);
}


void ParachuteDroneForcesAndMoments::calculateAerodynamicForcesAndMoments(){
  double u = LIN_VEL_NED_[0];
  double v = LIN_VEL_NED_[1];
  double w = LIN_VEL_NED_[2];


  ignition::math::Vector3d VEL_VEC_(u, v, w);


  Vt_ = pow(pow(u, 2.0) + pow(v, 2.0) + pow(w, 2.0), 0.5);
  FD_ = 0.5 * rho_ * S_ * Cd_ * pow(Vt_, 2.0);


  if (Vt_ > 0.1)
  {
    FA_ = -FD_ * VEL_VEC_.Normalize();
    MA_ = R_COP_.Cross(FA_);
    MA_[0] = MA_[0] + dref_ * ANG_VEL_NED_[0] / (2 * Vt_) * Cdd_ * 0.5 * rho_ * pow(Vt_, 2.0) * S_ * dref_;
    MA_[1] = MA_[1] + dref_ * ANG_VEL_NED_[1] / (2 * Vt_) * Cdd_ * 0.5 * rho_ * pow(Vt_, 2.0) * S_ * dref_;
  }
}


void ParachuteDroneForcesAndMoments::calculatePropulsiveForcesAndMoments(){
  double F1_ = ct_ * w1_ * w1_;
  double F2_ = ct_ * w2_ * w2_;
  double H1_ = ch_ * w1_ * w1_;
  double H2_ = ch_ * w2_ * w2_;

  FP_[0] = F1_ + F2_;
  MP_ = PROP_R_1.Cross(ignition::math::Vector3d(F1_, 0, 0)) + PROP_R_2.Cross(ignition::math::Vector3d(F2_, 0, 0)) + ignition::math::Vector3d(H1_ + H2_,0,0);
}


GZ_REGISTER_MODEL_PLUGIN(ParachuteDroneForcesAndMoments);
}
