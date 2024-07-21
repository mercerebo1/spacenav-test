/***** Test example to teleoperate TEO humanoid's right arm with 3D SpaceMouse , DH data for Teo's right arm is used. ******/

/***** DH data for TEO's right arm is applied using "teo-fixedTrunk-rightArm-fetch.ini" file. /***** 


// dh-root-rightArm.csv
H0 (0 1 0 0    0 0 1 -0.3469    1 0 0 0.4982    0 0 0 1)

// dh-rightArm.csv
numLinks 6

link_0 (offset   0.0) (D  0.0    ) (A  0.0      ) (alpha -90.0) (mass 0       ) (cog 0 0 0) (inertia 0 0 0)
link_1 (offset -90.0) (D  0.0    ) (A  0.0      ) (alpha -90.0) (mass 0       ) (cog 0 0 0) (inertia 0 0 0)
link_2 (offset -90.0) (D -0.32901) (A  0.0      ) (alpha -90.0) (mass 1.750625) (cog 0 0 0) (inertia 0 0 0)
link_3 (offset   0.0) (D  0.0    ) (A  0.0      ) (alpha  90.0) (mass 0       ) (cog 0 0 0) (inertia 0 0 0)
link_4 (offset   0.0) (D -0.215  ) (A  0.0      ) (alpha -90.0) (mass 2.396   ) (cog 0 0 0) (inertia 0 0 0)
link_5 (offset -90.0) (D  0.0    ) (A -0.09     ) (alpha   0.0) (mass 0.300   ) (cog 0 0 0) (inertia 0 0 0)

// dh-fetch.csv
HN (0 0 -1 -0.0975  -1 0 0 0    0 1 0 0    0 0 0 1) */

#include <memory> 
#include <mutex> 

#include <kdl/chain.hpp> 
#include <kdl/chainiksolvervel_pinv.hpp> 

#include <rclcpp/rclcpp.hpp> 
#include <geometry_msgs/msg/twist.hpp> 
#include <sensor_msgs/msg/joint_state.hpp> 
#include <yarp_control_msgs/msg/position_direct.hpp> 

constexpr auto EPS = 1e-3; 
constexpr auto MAX_ITER = 1000; 
constexpr auto SCALE = 1e-2; 
constexpr auto PREFIX = "/teoSim/rightArm"; 

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      using std::placeholders::_1;
      subscription_spnav_ = this->create_subscription<geometry_msgs::msg::Twist>("/spacenav/twist", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      
      KDL::Joint axis(KDL::Joint::RotZ); 
      KDL::Joint fixed(KDL::Joint(KDL::Joint::None)); //Fixed joint - cannot move or rotate
      KDL::Frame H0(KDL::Rotation(0, 1, 0, 0, 0, 1, 1, 0, 0), KDL::Vector(0.0, -0.3469, 0.4982));
      KDL::Frame HN(KDL::Rotation(0, 0, -1, -1, 0, 0, 0, 1, 0), KDL::Vector(-0.0975, 0.0, 0.0));
  
      chain.addSegment(KDL::Segment(fixed, H0));
      chain.addSegment(KDL::Segment(axis, KDL::Frame::DH(0.0, KDL::deg2rad * -90.0, 0.0, KDL::deg2rad * 0.0)));
      chain.addSegment(KDL::Segment(axis, KDL::Frame::DH(0.0, KDL::deg2rad * -90.0, 0.0, KDL::deg2rad * -90.0)));
      chain.addSegment(KDL::Segment(axis, KDL::Frame::DH(0.0, KDL::deg2rad * -90.0, -0.32901, KDL::deg2rad * -90.0)));
      chain.addSegment(KDL::Segment(axis, KDL::Frame::DH(0.0, KDL::deg2rad * 90.0, 0.0, KDL::deg2rad * 0.0)));
      chain.addSegment(KDL::Segment(axis, KDL::Frame::DH(0.0, KDL::deg2rad * -90.0, -0.215, KDL::deg2rad * 0.0)));
      chain.addSegment(KDL::Segment(axis, KDL::Frame::DH(-0.09, KDL::deg2rad * 0.0, 0.0, KDL::deg2rad * -90.0)));
      chain.addSegment(KDL::Segment(fixed, HN));

      ikSolverVel = new KDL::ChainIkSolverVel_pinv(chain, EPS, MAX_ITER);

      RCLCPP_INFO(this->get_logger(), "Got %d joints", chain.getNrOfJoints());

      q.resize(chain.getNrOfJoints());

      subscription_state_ = this->create_subscription<sensor_msgs::msg::JointState>(PREFIX + std::string("/state"), 10, std::bind(&MinimalSubscriber::state_callback, this, _1));

      publisher_position_ = this->create_publisher<yarp_control_msgs::msg::PositionDirect>(PREFIX + std::string("/position_direct"), 10);
    }
        
  private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const 
    {
      RCLCPP_INFO(this->get_logger(), "spnav: [%f %f %f] [%f %f %f]",
                  msg->linear.x, msg->linear.y, msg->linear.z,
                  msg->angular.x, msg->angular.y, msg->angular.z);

      mtx.lock();
      KDL::JntArray qd = q;
      mtx.unlock();

      KDL::Vector vel(msg->linear.x, msg->linear.y, msg->linear.z);
      KDL::Vector rot(msg->angular.x, msg->angular.y, msg->angular.z);
      KDL::Twist tw(vel, rot);
      KDL::JntArray qdot(chain.getNrOfJoints());  

      if (ikSolverVel->CartToJnt(qd, tw, qdot) == KDL::SolverI::E_NOERROR)
      {
        RCLCPP_WARN(this->get_logger(), "Something bad happened");
      }

      RCLCPP_INFO(this->get_logger(), "qdot: %f %f %f %f %f %f", qdot(0), qdot(1), qdot(2), qdot(3), qdot(4), qdot(5));

      for (size_t i = 0; i < chain.getNrOfJoints(); i++)
      {
        qd(i) += qdot(i) * SCALE;
      }

      RCLCPP_INFO(this->get_logger(), "qd: %f %f %f %f %f %f",
                  qd(0) * KDL::rad2deg, qd(1) * KDL::rad2deg, qd(2) * KDL::rad2deg, qd(3) * KDL::rad2deg, qd(4) * KDL::rad2deg, qd(5) * KDL::rad2deg);

      yarp_control_msgs::msg::PositionDirect msg_position;
      msg_position.positions.resize(chain.getNrOfJoints());

      for (size_t i = 0; i < chain.getNrOfJoints(); i++)
      {
        msg_position.positions[i] = qd(i);
      }

      publisher_position_->publish(msg_position);
    }

    void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
      mtx.lock();

      for (size_t i = 0; i < chain.getNrOfJoints(); i++)
      {
        q(i) = msg->position[i]; // radians!
      }

      mtx.unlock();

      RCLCPP_INFO_ONCE(this->get_logger(), "Initial joint configuration: %f %f %f %f %f %f",
                       q(0) * KDL::rad2deg, q(1) * KDL::rad2deg, q(2) * KDL::rad2deg, q(3) * KDL::rad2deg, q(4) * KDL::rad2deg, q(5) * KDL::rad2deg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_spnav_; 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_state_; 
    rclcpp::Publisher<yarp_control_msgs::msg::PositionDirect>::SharedPtr publisher_position_; 

    KDL::Chain chain;
    KDL::ChainIkSolverVel_pinv * ikSolverVel {nullptr};
    KDL::JntArray q;
    
    mutable std::mutex mtx;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}


