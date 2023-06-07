#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/conversions/sensor_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Events.hh>
#include <boost/make_shared.hpp>
#include <boost/variant.hpp>
#include <string>

#include <gazebo_rotating_lidar_plugin/gazebo_rotating_lidar_plugin.hpp>

namespace gazebo_plugins {
  class GazeboRangeSensorPlugin : public gazebo::SensorPlugin {
  public:

    gazebo_ros::Node::SharedPtr ros_node_;
    gazebo::sensors::RaySensorPtr raySensor;
    gazebo::event::ConnectionPtr updateConnection;
    gazebo::common::Timer timer;

    using LaserScan = sensor_msgs::msg::LaserScan;
    using LaserScanPub = rclcpp::Publisher<LaserScan>::SharedPtr;

    boost::variant<LaserScanPub> pub_;

    std::string frame_name_;
    unsigned int rayIndex;
    float* measurements;

    // Contr, Deconstructor
    GazeboRangeSensorPlugin() {};
    ~GazeboRangeSensorPlugin() {delete[] measurements;}; 

    void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
      // Create a GazeboRos node 
      this->ros_node_ = gazebo_ros::Node::Get(_sdf); 

      // Get QoS profiles
      const gazebo_ros::QoS & qos = this->ros_node_->get_qos();

      // Get QoS profile for the publisher
      rclcpp::QoS pub_qos = qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable());

      // Create a publisher
      this->pub_ = this->ros_node_->create_publisher<LaserScan>("~/out", pub_qos);

      // Get tf frame for output
      this->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

      // Check if the sensor is a ray sensor
      if (_sensor->Type() != "ray") {
        std::cerr << "GazeboRangeSensorPlugin requires a ray sensor. Exiting." << std::endl;
        return;
      }

      // Get the ray sensor pointer
      this->raySensor = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(_sensor);

      // Check if the ray sensor is valid
      if (!this->raySensor) {
        std::cerr << "Failed to cast sensor pointer to RaySensor. Exiting." << std::endl;
        return;
      }

      // Create number of rays size empty array, it will be a circular buffer
      measurements = new float[this->raySensor->RayCount()]();

      // Callback. Every tick in simu time.
      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRangeSensorPlugin::OnUpdate, this));
      
      // Set up timer for measure time between ray measurements
      this->timer.Reset();
      this->timer.Start();
      this->rayIndex = 0;

    }

    void OnUpdate() {
      
      // Check if enough time elapsed to make a new measurement
      if (this->timer.GetElapsed() < (1.0 / this->raySensor->UpdateRate())/this->raySensor->RayCount())
        return;

      // Create the base of the LaserScan message (ToDO: contructor..)
      LaserScan laserMsg;
      laserMsg.header.frame_id = this->frame_name_;
      laserMsg.header.stamp = this->ros_node_->now();
      laserMsg.angle_min = this->raySensor->AngleMin().Radian();
      laserMsg.angle_max = this->raySensor->AngleMax().Radian();
      laserMsg.angle_increment = (this->raySensor->AngleMax().Radian()-this->raySensor->AngleMin().Radian())/this->raySensor->RayCount();
      laserMsg.time_increment = (1.0 / this->raySensor->UpdateRate())/this->raySensor->RayCount();
      laserMsg.scan_time = 1.0 / this->raySensor->UpdateRate();
      laserMsg.range_min = this->raySensor->RangeMin();
      laserMsg.range_max = this->raySensor->RangeMax();
      laserMsg.ranges.resize(this->raySensor->RayCount(), 0.0);
      laserMsg.intensities.resize(1, 0.0);

      // Fill the ranges array with previous measurement
      for (int i = 0; i < this->raySensor->RayCount(); ++i) {
        laserMsg.ranges[i] = measurements[i];
      }

      // Measure distance for the current ray, and update the buffer and the message
      double range = MeasureRayDistance();
      laserMsg.ranges[this->rayIndex] = range;
      measurements[this->rayIndex] = range;

      // Store the ray index in intesities, shows which element of ranges is currently updated (for debug)
      laserMsg.intensities[0] = this->rayIndex % this->raySensor->RayCount();

      // Publish the laser scan message
      boost::get<LaserScanPub>(pub_)->publish(laserMsg);

      // Move to the next ray
      this->rayIndex = (this->rayIndex + 1) % this->raySensor->RayCount();

      // Reset the timer
      this->timer.Reset();
      this->timer.Start();
    }

    double MeasureRayDistance() {
      // Update the sensor to perform measurements
      this->raySensor->Update(true);
      gazebo::common::Time::Sleep(0.0001);

      // Retrieve the range for the current ray
      double range = this->raySensor->Range(this->rayIndex);
      return range;
    }
  };
  // Register the plugin
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRangeSensorPlugin)
} 