#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>

namespace blurr
{

using sensor_msgs::msg::JointState;
using std_msgs::msg::Bool;
using std_msgs::msg::Float32;
using mavros_msgs::msg::OverrideRCIn;
using namespace std::chrono_literals;

// special pins
constexpr auto thruster_pins{std::array{0,1,2,3,4,5}};
constexpr auto light_pin{9};
constexpr auto tilt_pin{7};

// thrust -> pwm map from BlueRobotics @ 16 V
constexpr auto pwm{std::array{ 1100.00f,1150.00f,1200.00f,1250.00f,1300.00f,1350.00f,1400.00f,1450.00f,1480.00f,1520.00f,1550.00f,1600.00f,1650.00f,1700.00f,1750.00f,1800.00f,1850.00f,1900.00f }};
//constexpr auto vels{std::array{ 362.81f,336.68f,299.98f,265.20f,224.11f,180.25f,130.49f,63.83f,0.00f,0.00f,63.00f,129.54f,180.06f,222.91f,263.67f,302.42f,340.46f,370.01f }};
constexpr auto thrusts{std::array{ -39.92f,-34.20f,-26.56f,-20.14f,-14.11f,-9.06f,-4.72f,-1.13f,0.00f,0.00f,1.38f,5.92f,11.41f,17.89f,25.65f,33.55f,43.61f,51.45f }};

static_assert (thrusts.size() == pwm.size(), "vel/pwm map should have same sizes");
static_assert (thruster_pins.size() == 6, "thruster pins should be size 6");

constexpr auto watchdog_period{1};

float interpPWM(double v)
{
  // no extrapolation
  if(v <= thrusts.front())
    return pwm.front();
  else if(v >= thrusts.back())
    return pwm.back();

  uint i{0};
  while ( v > thrusts[i+1] ) i++;

  const auto xL{thrusts[i]}, xR{thrusts[i+1]};
  const auto yL{pwm[i]}, yR{pwm[i+1]};
  return yL + (( yR - yL )  * ( v - xL ))/ ( xR - xL );
}

class BlurrPWM : public rclcpp::Node
{
public:
  explicit BlurrPWM(rclcpp::NodeOptions options) :
      rclcpp::Node("pwm", options)
  {
    pwm_pub = create_publisher<OverrideRCIn>("rc/override", 2);

    tilt_pub = create_publisher<JointState>("joint_states", rclcpp::SensorDataQoS());
    tilt_msg.name = {"tilt"};
    tilt_msg.position = {0};

    light_sub = create_subscription<Float32>("light", 10, [&](Float32::UniquePtr msg) {light_intensity = msg->data;});

    tilt_sub = create_subscription<Float32>("cmd_tilt", 10, [&](Float32::UniquePtr msg)
                                               {tilt_angle() = std::clamp<double>(msg->data, -M_PI_4, M_PI_4);});

    thruster_sub = create_subscription<JointState>("cmd_thrust", 10, [&](JointState::UniquePtr msg)
                                                   {cmd_thrust_cb(*msg);});

  }

private:

  inline double& tilt_angle()
  {
    return tilt_msg.position[0];
  }

  rclcpp::TimerBase::SharedPtr
      pwm_timer{create_wall_timer(20ms, [&](){publish();})},
      thruster_watchdog{create_wall_timer(std::chrono::seconds(watchdog_period), [&](){watchDog();})};

  rclcpp::Subscription<Float32>::SharedPtr tilt_sub, light_sub;
  rclcpp::Subscription<JointState>::SharedPtr thruster_sub;

  // received reference
  std::array<double, 6> thruster_force{0};
  rclcpp::Time thruster_time{};
  JointState tilt_msg;
  float light_intensity{0};
  rclcpp::Publisher<JointState>::SharedPtr tilt_pub;

  // bridge to RC override
  OverrideRCIn pwm;
  rclcpp::Publisher<OverrideRCIn>::SharedPtr pwm_pub;

  // callbacks

  void cmd_thrust_cb(const JointState &msg)
  {
    thruster_time = now();
    const static std::vector<std::string> names{"thruster1","thruster2","thruster3","thruster4","thruster5","thruster6"};
    for(uint i = 0; i<msg.name.size();++i)
    {
      for(uint j = 0; j<6; ++j)
      {
        if(msg.name[i] == names[j])
        {
          thruster_force[j] = msg.effort[i];
          break;
        }
      }
    }
  }

  void watchDog()
  {
    if((now() - thruster_time).seconds() > watchdog_period)
    {
      for(auto pin: thruster_pins)
        pwm.channels[pin] = 1500;
      tilt_angle() = 0;
    }
  }

  void publish()
  {
    // at least remind the tilt angle
    tilt_msg.header.stamp = now();
    tilt_pub->publish(tilt_msg);

    // thrusters
    for(uint i = 0; i < thruster_pins.size(); ++i)
      pwm.channels[thruster_pins[i]] = interpPWM(thruster_force[i]);

    // tilt
    pwm.channels[tilt_pin] = 1500 + 509.3*tilt_angle();

    // lumen light
    pwm.channels[light_pin] = 1100 + 800*light_intensity;
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

