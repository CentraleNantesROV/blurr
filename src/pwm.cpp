#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <navio2_ros/pwm.hpp>

namespace blurr
{

using sensor_msgs::msg::JointState;
using std_msgs::msg::Bool;
using std_msgs::msg::Float32;
using navio2_ros::PWM;
using namespace std::chrono_literals;

// special pins
constexpr auto thruster_pins{std::array{0,2,4,6,8,10}};
constexpr auto light_pin{13};
constexpr auto tilt_pin{12};

// velocity -> pwm map from BlueRobotics @ 16 V
constexpr auto velocities{std::array{-362.82,-336.69,-299.99,-265.21,-224.11,-180.25,-130.49,-63.83,0.0,0.0,63.0,129.54,180.06,222.91,263.67,302.43,340.46,370.02}};
constexpr auto pwm{std::array{1100.f, 1150.f, 1200.f, 1250.f, 1300.f, 1350.f, 1400.f, 1450.f, 1480.f, 1520.f, 1550.f, 1600.f, 1650.f, 1700.f, 1750.f, 1800.f, 1850.f, 1900.f}};
static_assert (velocities.size() == pwm.size(), "vel/pwm map should have same sizes");

constexpr auto watchdog_period{1};

float interpPWM(double v)
{
  // no extrapolation
  if(v <= velocities.front())
    return pwm.front();
  else if(v >=velocities.back())
    return pwm.back();

  uint i = 0;
  while ( v > velocities[i+1] ) i++;

  const auto xL = velocities[i], xR = velocities[i+1];
  const auto yL = pwm[i], yR = pwm[i+1];
  return yL + ( yR - yL ) / ( xR - xL ) * ( v - xL );
}



class BlurrPWM : public rclcpp::Node
{
public:
  explicit BlurrPWM(rclcpp::NodeOptions options) :
    rclcpp::Node("pwm", options),
    pwm(this, {0, 2, 4, 6, 8, 10, tilt_pin, light_pin}),
    pwm_timer{create_wall_timer(20ms, [&](){toPWM();})},
    thruster_watchdog{create_wall_timer(std::chrono::seconds(watchdog_period), [&](){watchDog();})}
  { 
    run_sub = create_subscription<Bool>("run", 10, [&](Bool::UniquePtr msg){running = msg->data;});

    tilt_sub = create_subscription<JointState>("joint_setpoint", 10, [&](JointState::UniquePtr msg)
    {readTilt(*msg);});

    thruster_sub = create_subscription<JointState>("thruster_command", 10, [&](JointState::UniquePtr msg)
    {readThrusters(*msg);});
    light_sub = create_subscription<Float32>("light", 10, [&](Float32::UniquePtr msg) {light_intensity = msg->data;});

    tilt_pub = create_publisher<JointState>("joint_states", rclcpp::SensorDataQoS());
    tilt_msg.name = {"tilt"};
    tilt_msg.position = {0};
  }

private:    

  PWM pwm;
  rclcpp::TimerBase::SharedPtr pwm_timer, thruster_watchdog;

  rclcpp::Subscription<Bool>::SharedPtr run_sub;
  bool running{true};

  rclcpp::Subscription<Float32>::SharedPtr light_sub;
  rclcpp::Subscription<JointState>::SharedPtr tilt_sub, thruster_sub;

  rclcpp::Publisher<JointState>::SharedPtr tilt_pub;
  JointState tilt_msg;

  std::array<double, 6> thruster_force{0};
  double thruster_time{};

  double tilt_angle{0};
  float light_intensity{0};

  // callbacks

  void readTilt(const JointState &msg)
  {
    if(msg.name.size() && msg.position.size())
      tilt_angle = std::min(0.785, std::max(-0.785, msg.position[0]));
  }

  void readThrusters(const JointState &msg)
  {
    thruster_time = now().seconds();
    const static std::vector<std::string> names{"thruster0","thruster1","thruster2","thruster3","thruster4","thruster5"};
    for(uint i = 0; i<msg.name.size();++i)
    {
      for(uint j = 0; j<6; ++j)
      {
        if(msg.name[i] == names[j])
        {
          thruster_force[j] = msg.velocity[i];
          break;
        }
      }
    }
  }

  void watchDog()
  {
    if(now().seconds() - thruster_time > watchdog_period)
      pwm.stop();
  }

  void toPWM()
  {
    // at least remind the tilt angle
    tilt_msg.position[0] = tilt_angle;
    tilt_msg.header.stamp = now();
    tilt_pub->publish(tilt_msg);

    if(!running)  return;

    // thrusters    
    for(uint i = 0; i < thruster_pins.size(); ++i)
      pwm.set_duty_cycle(thruster_pins[i], interpPWM(thruster_force[i]));

    // tilt: pwm + joint_states
    pwm.set_duty_cycle(tilt_pin, 1500 + 509.3*tilt_angle);

    // lumen light
    pwm.set_duty_cycle(light_pin, 1100 + 800*light_intensity);
  }
};

}

// boilerplate main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<blurr::BlurrPWM>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
