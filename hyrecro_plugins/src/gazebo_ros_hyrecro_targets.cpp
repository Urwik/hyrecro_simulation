//Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_msgs/SetModelConfigurationRequest.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/sensors.hh>
 
// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <ros/service.h>
#include "hyrecro_plugins/GetJointsValue.h"

// Misc
#include <map>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <functional>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <typeinfo>
#include <chrono>
#include <thread>
#include <typeinfo>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>


namespace gazebo
{
  class HyrecroTargetPlugin : public WorldPlugin
  {

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      std::cout << "----- Loading HyReCRo Targets Plugin -----" << std::endl;

      // Obtiene un puntero al elemento World
      this->world = _world;
      this->world->SetPhysicsEnabled(false);

      // Indica los eventos que suceden en el mundo y la función a la que llaman
      // cuando sucede uno de los eventos indicados
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&HyrecroTargetPlugin::OnUpdate, this));

      this->updateEntity = event::Events::ConnectAddEntity(
      std::bind(&HyrecroTargetPlugin::OnModelLoad, this, std::placeholders::_1));

      // Inicia ROS y se subscribe al tópico     
      this->rosNode = init_ROS("hyrecro");

      // PUBLISHERS AND SUBSCRIBERS
      this->positionPub = this->rosNode->advertise<sensor_msgs::JointState>("joints_position", 1);
      this->joints_target_sub = this->target_subscriber("/joints_tmp_target");

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&HyrecroTargetPlugin::QueueThread, this));

      this->rosNode->setParam("fixed_leg", "A");
      this->rosNode->getParam("fixed_leg", this->actual_leg);   

      std::cout << "----- HyReCRo Targets Plugin Loaded Correctly -----" << std::endl;

    }


    /*** FUNCTIONS ************************************************************************/
    
    // Devuelve la posición ordenada de cada una de las articulaciones
    public: std::vector<double> getJointsPosition(physics::Joint_V joints){
      std::vector<double> positions;
      physics::JointState state;

      try{
        for (auto joint : joints){
          state = physics::JointState(joint);
          positions.push_back(state.Position());
        }
      }

      catch(const std::exception& e){
        std::cerr << e.what() << '\n';
      }

      return positions;
    }

    // Devuelve un vector de punteros ordenado a cada joint
    public: physics::Joint_V getJoints(physics::ModelPtr model, std::vector<std::string> names){
      physics::Joint_V joint_v;

      for (auto name : names)
        joint_v.push_back(model->GetJoint(name));

      return joint_v;
    }

    // Publica la posición de cada joint
    public: void publishJointsPosition(std::vector<double> positions){
      sensor_msgs::JointState msg;

      msg.header.stamp = ros::Time::now();
      msg.name = this->JOINT_NAMES;
      msg.position = positions;

      this->positionPub.publish(msg);
    }

    // Función asociada a evento cuando se introduce un modelo en el world
    public: void OnModelLoad(const std::string &name) {
      std::cout << "----- New Model Loaded -----" << std::endl;
      this->model_loaded = 1;
      this->model_name = name;
    }
    

    // Función que se ejecuta cada periodo de actualización de la simulación
    public: void OnUpdate() {
      // std::cout << " --- OnUpdate ---" << std::endl;

      if(this->model_loaded){
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        this->actual_model = this->world->ModelByName(this->model_name);
        this->joint_ptrs = this->getJoints(this->actual_model, this->JOINT_NAMES);

        // swapConfiguration();
        std::cout << "Actual Model: " << this->actual_model->GetName() << std::endl;
        this->model_loaded = 0;
        this->do_stuff = 1;
        // Si no es la primera vez que se carga un modelo
        // (evita problemas al iniciar la simulación)
        if(this->first_time != 1)
          set_link_poses(this->poses_vector);

        else{
          this->first_time = 0;
        }
      }

      if (this->do_stuff){
        std::vector<double> joint_poses = this->getJointsPosition(this->joint_ptrs);
        this->publishJointsPosition(joint_poses);
      }

      if(param_changed()){
        std::cout <<"----- Fixed leg change request -----" << std::endl;
        std::cout <<"      New fixed leg: " << this->actual_leg << std::endl;
        this->poses_vector = get_link_poses();
        change_model();
        this->do_stuff = 0;
      }


    }


    // Función que comprueba si se ha modificado el parámetro que indica que pata
    // ha de fijarse
    public: bool param_changed() {
      std::string actual_param;
      this->rosNode->getParam("fixed_leg", actual_param);

      if(actual_param == this->actual_leg)
        return 0;
      else{
        this->actual_leg = actual_param;
        return 1;   
      }
    }


    // Función para iniciar ROS
    public: std::unique_ptr<ros::NodeHandle> init_ROS(std::string nameSpace){
      std::unique_ptr<ros::NodeHandle>  nh;

      // Make sure ROS is running
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      nh.reset(new ros::NodeHandle(nameSpace));

      if (nh == nullptr)
        ROS_ERROR("NODE COULDN'T BE INITIALIZED");
      else
        ROS_INFO("NODE CORRECTLY INITIALIZED");
      
      return nh;
    }

    
    // Función para suscribirse a un tópico
    public: ros::Subscriber target_subscriber(std::string topicName){
      ros::Subscriber subs;
      
      ros::SubscribeOptions so =  ros::SubscribeOptions::create<sensor_msgs::JointState>
        (topicName, 1, boost::bind(&HyrecroTargetPlugin::OnTargetMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
      
      subs = this->rosNode->subscribe(so);

      return subs;
    }
    
    // Callback para nuevos targets   
    public: void OnTargetMsg(const sensor_msgs::JointState::ConstPtr& msg){
      gazebo_msgs::SetModelConfigurationRequest req;

      req.joint_names = msg->name;

      // En caso de ser el modelo A, mantener el valor de las joints
      // (esto implica que el modelo inicial ha de ser el modelo A)
      if(this->actual_leg == "A"){
        req.joint_positions = msg->position;
      }

      // En caso de ser el modelo B, cambiar el sentido del valor de las joints
      else {
        sensor_msgs::JointState new_msg;
        new_msg = *msg;
        
        for(int i=0; i<8; i++)
          new_msg.position[i] = -new_msg.position[i];
        
        req.joint_positions = new_msg.position;
      }

      bool reached_pose = setModelConfiguration(req);
    }
    
    // Función que gestiona la cola de mensajes de ROS
    public: void QueueThread() {
      static const double timeout = 0.1;
      while (this->rosNode->ok())
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }

    // Mueve las articulaciones a las posiciones establecidas
    public: bool setModelConfiguration(gazebo_msgs::SetModelConfigurationRequest &req) {

      if (req.joint_names.size() == req.joint_positions.size())
      {
        std::map<std::string, double> joint_position_map;
        for (unsigned int i = 0; i < req.joint_names.size(); i++)
        {
          joint_position_map[req.joint_names[i]] = req.joint_positions[i];
        }

        // make the service call to pause gazebo
        bool is_paused = this->world->IsPaused();
        if (!is_paused) this->world->SetPaused(true);

        this->actual_model->SetJointPositions(joint_position_map);

        // resume paused state before this call
        this->world->SetPaused(is_paused);
        return true;
      }
      else
      {
        return false;
      }
    }


    // --- GESTIÓN DEL CAMBIO DE PATA ---
    // Obtiene las posiciones de todos los links respecto del world
    public: std::vector<ignition::math::Pose3d> get_link_poses(){
      std::vector<ignition::math::Pose3d> linkPoses;
      physics::Link_V links;

      links = this->actual_model->GetLinks();

      for(int i=0; i<links.size(); i++)
        linkPoses.push_back(links[i]->WorldPose());


      return linkPoses;
    }

    // Establece las posiciones de todos los links respecto del world
    public: void set_link_poses(std::vector<ignition::math::Pose3d> linkPoses){
      physics::Link_V links;

      links = this->actual_model->GetLinks();

      for(int i=0; i<links.size(); i++)
        links[i]->SetWorldPose(linkPoses[i]);
    }

    // Elimina un modelo e inserta otro
    public: void change_model() {

      this->world->SetPaused(true);

      if(this->actual_leg=="A"){
        this->world->RemoveModel(this->actual_model);

        this->world->InsertModelFile("model://hyrecro_A_serial_realsense");
      }

      else if(this->actual_leg=="B"){
        this->world->RemoveModel(this->actual_model);

        this->world->InsertModelFile("model://hyrecro_B_serial_realsense");
      }
      else
        ROS_WARN("MODEL INDICATED DON'T EXIST, MODEL CAN BE A OR B");
    
      this->world->SetPaused(false);
    }


    /*** VARIABLES ************************************************************************/

    // Gazebo Variables
    public: physics::ModelPtr actual_model;
    public: physics::WorldPtr world;
    public: physics::Joint_V joint_ptrs;

    public: event::ConnectionPtr updateConnection;
    public: event::ConnectionPtr updateEntity;
    public: event::ConnectionPtr deleteEntity;
    public: gazebo::sensors::SensorPtr ousterPtr;
    // public: physics::LinkPtr foot_A;
    // public: physics::LinkPtr foot_B;
    // public: ignition::math::Pose3d new_pose;
    public: std::vector<ignition::math::Pose3d> poses_vector;


    // ROS Variables
    public: std::unique_ptr<ros::NodeHandle> rosNode;
    public: ros::Subscriber joints_gui_sub;
    public: ros::Subscriber joints_target_sub;
    public: ros::Publisher positionPub;
    public: ros::CallbackQueue rosQueue;
    public: std::thread rosQueueThread;
    public: std::string actual_leg;
    public: bool model_loaded = 0;
    public: bool model_deleted = 0;
    public: std::string model_name;
    public: bool first_time = 1;
    public: sensor_msgs::JointState::ConstPtr prev_msg;
    public: bool first_msg = 1;
    public: bool first_model_loaded = 0;
    public: bool do_stuff = 0;

    // CONSTANTS
    public: std::vector<std::string> JOINT_NAMES = {"q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8"}; 
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(HyrecroTargetPlugin)
}