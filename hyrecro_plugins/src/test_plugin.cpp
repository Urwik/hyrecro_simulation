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

//#include <gazebo_ros_api_plugin.h>
// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
// Misc
#include <map>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <functional>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <typeinfo>
#include <chrono>
#include <thread>
#include <typeinfo>

namespace gazebo
{
  class TestWorldPlugin : public WorldPlugin
  {

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      ROS_INFO("!!! LOADING WORLD PLUGIN !!!");

      // Obtiene un puntero al elemento World
      this->world = _world;
      this->world->SetPhysicsEnabled(false);

      // Indica los eventos que suceden en el mundo y la función a la que llaman
      // cuando sucede uno de los eventos indicados
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TestWorldPlugin::OnUpdate, this));

      this->updateEntity = event::Events::ConnectAddEntity(
      std::bind(&TestWorldPlugin::OnModelLoad, this, std::placeholders::_1));

      // Inicia ROS y se subscribe al tópico     
      this->rosNode = init_ROS("testPlugin");

      this->rosSub = ros_subscriber("/joint_states");

      ROS_INFO("SUBSCRIBED TO \"%s\" TOPIC", this->rosSub.getTopic().c_str());

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&TestWorldPlugin::QueueThread, this));

      this->rosNode->setParam("fixed_Leg", "A");
      this->rosNode->getParam("fixed_Leg", this->actual_leg);   

      ROS_INFO("!!! PLUGIN CORRECTLY LOADED !!!");

    }


    /*** FUNCTIONS ************************************************************************/

    // Función asociada a evento cuando se introduce un modelo en el world
    public: void OnModelLoad(const std::string &name) {
      ROS_INFO("NEW MODEL LOADED");
      this->model_loaded = 1;
      this->model_name = name;
      // this->actual_model = this->world->ModelByName(this->model_name);
      ROS_INFO("Handler to model get");
    }
    
    
    public: void OnModelDeleted(const std::string &name){
      std::cout << "OnModelDeleted" << std::endl;
      this->model_deleted = 1;
      ROS_INFO("REMOVED MODEL %s",  name.c_str());
    }

    // Función que se ejecuta cada periodo de actualización de la simulación
    public: void OnUpdate() {
      // std::cout << "OnUpdate" << std::endl;

      if(this->model_deleted){
        if(this->actual_leg == "A")
          this->world->InsertModelFile("model://hyrecro_A_serial_ouster");
        else
          this->world->InsertModelFile("model://hyrecro_B_serial_ouster");
      
        this->model_deleted = 0;
      }


      if(this->model_loaded){
        std::cout << "OnUpdate-ModelLoaded" << std::endl;

        this->actual_model = this->world->ModelByName(this->model_name);
        // swapConfiguration();
        std::cout << "Actual Model: " << this->actual_model->GetName() << std::endl;
        this->model_loaded = 0;

        // Si no es la primera vez que se carga un modelo
        // (evita problemas al iniciar la simulación)
        if(this->first_time != 1)
          set_link_poses(this->poses_vector);

        else
          this->first_time = 0;

      }

      if(param_changed()){
        ROS_INFO("Fixed leg change, actual fixed leg: %s", this->actual_leg.c_str());

        this->poses_vector = get_link_poses();
        change_model();

        // std::cout << "Change model finished corectly" << std::endl;
      }
    }


    // Función que comprueba si se ha modificado el parámetro que indica que pata
    // ha de fijarse
    private: bool param_changed() {
      std::string actual_param;
      this->rosNode->getParam("fixed_Leg", actual_param);

      if(actual_param == this->actual_leg)
        return 0;
      else{
        this->actual_leg = actual_param;
        return 1;   
      }
    }


    // Función para iniciar ROS
    private: std::unique_ptr<ros::NodeHandle> init_ROS(std::string nameSpace){
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
    public: ros::Subscriber ros_subscriber(std::string topicName){
      ros::Subscriber subs;
      
      ros::SubscribeOptions so =  ros::SubscribeOptions::create<sensor_msgs::JointState>
        (topicName, 1, boost::bind(&TestWorldPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
      
      subs = this->rosNode->subscribe(so);

      return subs;
    }
    
    
    // Callback de mensajes recibidos en /joint_states
    public: void OnRosMsg(const sensor_msgs::JointState::ConstPtr& msg) {
      gazebo_msgs::SetModelConfigurationRequest req;

      if (this->first_msg == 1){
        this->prev_msg = msg;
        this->first_msg = 0;
      }

      else if (this->prev_msg->position != msg->position){
        std::cout << "New Pose Received" << std::endl;
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
          // std::cout << new_msg.position[0] << std::endl;
        }

        setModelConfiguration(req);
        this->prev_msg = msg;
      }
    }


    // Función que gestiona la cola de mensajes de ROS
    private: void QueueThread() {
      static const double timeout = 0.01;
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

    // Obtiene las posiciones de todos los links respecto del world
    private: std::vector<ignition::math::Pose3d> get_link_poses(){
      std::vector<ignition::math::Pose3d> linkPoses;
      physics::Link_V links;

      links = this->actual_model->GetLinks();

      for(int i=0; i<links.size(); i++)
        linkPoses.push_back(links[i]->WorldPose());


      return linkPoses;
    }

    // Establece las posiciones de todos los links respecto del world
    private: void set_link_poses(std::vector<ignition::math::Pose3d> linkPoses){
      physics::Link_V links;

      links = this->actual_model->GetLinks();

      for(int i=0; i<links.size(); i++)
        links[i]->SetWorldPose(linkPoses[i]);
    }


    // Elimina un modelo e inserta otro
    private: void change_model() {

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


    private: void swapConfiguration(){
      physics::Joint_V joints = this->actual_model->GetJoints();
      physics::LinkPtr parent, child;

      for (int i = 0; i<joints.size(); i++){
        std::cout << "q" << i+1 << ": " << joints[i]->AnchorErrorPose() << std::endl;

      }

    }

    /*** VARIABLES ************************************************************************/

    // Gazebo Variables
    private: physics::ModelPtr actual_model;
    private: physics::WorldPtr world;
    private: event::ConnectionPtr updateConnection;
    private: event::ConnectionPtr updateEntity;
    private: event::ConnectionPtr deleteEntity;
    private: gazebo::sensors::SensorPtr ousterPtr;
    // private: physics::LinkPtr foot_A;
    // private: physics::LinkPtr foot_B;
    // private: ignition::math::Pose3d new_pose;
    private: std::vector<ignition::math::Pose3d> poses_vector;


        // ROS Variables
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;
    private: std::string actual_leg;
    private: bool model_loaded = 0;
    private: bool model_deleted = 0;
    private: std::string model_name;
    private: bool first_time = 1;
    public: sensor_msgs::JointState::ConstPtr prev_msg;
    private: bool first_msg = 1;
    public: bool first_model_loaded = 0;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(TestWorldPlugin)
}