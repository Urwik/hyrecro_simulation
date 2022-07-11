//Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo-11/gazebo/gazebo.hh>
#include <gazebo-11/gazebo/util/util.hh>
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
#include <sdf/sdf.hh>

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
#include <cstdlib>


namespace gazebo
{
  class HyrecroGUIPlugin : public WorldPlugin
  {

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      std::cout << "----- Loading HyReCRo GUI Plugin -----" << std::endl;

      // Obtiene un puntero al elemento World
      this->world = _world;
      this->world->SetPhysicsEnabled(false);

      // Indica los eventos que suceden en el mundo y la función a la que llaman
      // cuando sucede uno de los eventos indicados
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&HyrecroGUIPlugin::OnUpdate, this));

      this->updateEntity = event::Events::ConnectAddEntity(
      std::bind(&HyrecroGUIPlugin::OnModelLoad, this, std::placeholders::_1));

      // Inicia ROS y se subscribe al tópico     
      this->rosNode = init_ROS("hyrecro");

      // PUBLISHERS AND SUBSCRIBERS
      this->joints_gui_sub    = this->ros_subscriber("/joint_states");

      ROS_INFO("SUBSCRIBED TO \"%s\" TOPIC", this->joints_gui_sub.getTopic().c_str());

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&HyrecroGUIPlugin::QueueThread, this));

      this->rosNode->setParam("fixed_leg", "A");
      this->rosNode->getParam("fixed_leg", this->actual_leg);   

      std::cout << "----- HyReCRo GUI Plugin Loaded Correctly -----" << std::endl;

    }


    /*** FUNCTIONS ************************************************************************/
    
    // Función asociada a evento cuando se introduce un modelo en el world
    public: void OnModelLoad(const std::string &name) {
      std::cout << "----- New Model Loaded -----" << std::endl;
      this->model_loaded = 1;
      this->model_name = name;
      this->changing_leg = 0;
    }
    

    // Función que se ejecuta cada periodo de actualización de la simulación
    public: void OnUpdate() {
    //  std::cout << "OnUpdate" << std::endl;
    //  std::cout << "Changing_leg: " << this->changing_leg << std::endl;

      if (!this->changing_leg){
        if(this->model_loaded){
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          std::cout << "OnUpdate-ModelLoaded" << std::endl;
          this->actual_model = this->world->ModelByName(this->model_name);
          this->actual_model->Init();
          std::cout << "New model name: " << this->actual_model->GetName() << std::endl;
          this->model_loaded = 0;
          // Si no es la primera vez que se carga un modelo
          // (evita problemas al iniciar la simulación)
          if(this->first_time != 1)
            set_link_poses(this->poses_vector);
          else
            this->first_time = 0;
        }

        if(param_changed()){
          std::cout << "----- Fixed leg change request -----" << std::endl;
          std::cout << "      New fixed leg: " << this->actual_leg << std::endl;
          std::cout << "      Change numbr: " << this->cuenta << std::endl;
          this->poses_vector = get_link_poses();
          this->changing_leg = 1;
          change_model();
          std::cout << "----- Pasa el change_model -----" << std::endl;

        }
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
    public: ros::Subscriber ros_subscriber(std::string topicName){
      ros::Subscriber subs;
      
      ros::SubscribeOptions so =  ros::SubscribeOptions::create<sensor_msgs::JointState>
        (topicName, 1, boost::bind(&HyrecroGUIPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);
      
      subs = this->rosNode->subscribe(so);

      return subs;
    }
    
    
    // Callback de mensajes recibidos en /joint_states
    public: void OnRosMsg(const sensor_msgs::JointState::ConstPtr& msg) {
      gazebo_msgs::SetModelConfigurationRequest req;

      if(!this->changing_leg){
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
    }
    
    // Función que gestiona la cola de mensajes de ROS
    public: void QueueThread() {
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


    // --- GESTIÓN DEL CAMBIO DE PATA ---
    // Obtiene las posiciones de todos los links respecto del world
    public: std::vector<ignition::math::Pose3d> get_link_poses(){
      std::vector<ignition::math::Pose3d> linkPoses;
      physics::Link_V links;
      if (!this->changing_leg){
        for(int i=0; i<this->LINK_NAMES.size(); i++){
          links.push_back(this->actual_model->GetLink(this->LINK_NAMES[i]));
          // std::cout << links[i]->GetName() << std::endl;
        }

        for(int i=0; i<links.size(); i++){
          linkPoses.push_back(links[i]->WorldPose());
          // std::cout << linkPoses[i] << std::endl;
        }
      }
      return linkPoses;
    }

    // Establece las posiciones de todos los links respecto del world
    public: void set_link_poses(std::vector<ignition::math::Pose3d> linkPoses){
      physics::Link_V links;

      if(!this->changing_leg){
        for(int i=0; i<this->LINK_NAMES.size(); i++){
          links.push_back(this->actual_model->GetLink(this->LINK_NAMES[i]));
        }

        for(int i=0; i<links.size(); i++)
          links[i]->SetWorldPose(linkPoses[i]);
      }
    }

    // Elimina un modelo e inserta otro
    public: void change_model() {

      this->world->SetPaused(true);
      this->cuenta += 1;

      if(this->actual_leg=="A"){
        this->world->RemoveModel(this->actual_model);
        this->actual_model->Fini();
        // std::this_thread::sleep_for(std::chrono::microseconds(1000));
        this->world->InsertModelFile("model://hyrecro_A_serial_realsense");
        // this->world->InsertModelSDF(this->model_A);
        // this->world->InsertModelFile("/home/arvc/hyrecro_ws/src/hyrecro/sdf/hyrecro_A_serial_realsense/model.sdf");

        // std::this_thread::sleep_for(std::chrono::microseconds(1000));

      }

      else if(this->actual_leg=="B"){
        this->world->RemoveModel(this->actual_model);
        this->actual_model->Fini();
        // std::this_thread::sleep_for(std::chrono::microseconds(1000));
        this->world->InsertModelFile("model://hyrecro_B_serial_realsense");
        // this->world->InsertModelSDF(this->model_B);
        // this->world->InsertModelFile("/home/arvc/hyrecro_ws/src/hyrecro/sdf/hyrecro_B_serial_realsense/model.sdf");

        // std::this_thread::sleep_for(std::chrono::microseconds(1000));

      }
      else
        ROS_WARN("MODEL INDICATED DON'T EXIST, MODEL CAN BE A OR B");
    
      this->world->SetPaused(false);
    }


    /*** VARIABLES ************************************************************************/

    // Gazebo Variables
    public: physics::ModelPtr actual_model;
    public: physics::WorldPtr world;
    public: event::ConnectionPtr updateConnection;
    public: event::ConnectionPtr updateEntity;

    public: std::vector<ignition::math::Pose3d> poses_vector;

    public: sdf::SDF model_A;
    public: sdf::SDF model_B;


    // ROS Variables
    public: std::unique_ptr<ros::NodeHandle> rosNode;
    public: ros::Subscriber joints_gui_sub;
    public: ros::CallbackQueue rosQueue;
    public: std::thread rosQueueThread;
    public: std::string actual_leg;
    public: bool model_loaded = 0;
    public: bool model_deleted = 0;
    public: std::string model_name;
    public: bool first_time = 1;
    public: sensor_msgs::JointState::ConstPtr prev_msg;
    public: bool first_msg = 1;
    public: bool changing_leg = 0;
    public: int cuenta = 0;

    public: std::vector<std::string> LINK_NAMES = {"foot_A", "foot_B", "hip", "hip_A", "hip_B", "pist_1A", "pist_1B", "pist_2A", "pist_2B", "d435"};
    public: std::vector<std::string> JOINT_NAMES = {"q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8"};
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(HyrecroGUIPlugin)
}