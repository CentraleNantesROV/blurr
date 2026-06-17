#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace blurr
{

using sensor_msgs::msg::JointState;
using geometry_msgs::msg::Wrench;
using std_msgs::msg::Bool;
using std_msgs::msg::Float32;
using mavros_msgs::msg::OverrideRCIn;
using namespace std::chrono_literals;

// special pins
//constexpr auto thruster_pins{std::array{0,1,2,3,4,5}};
constexpr auto light_pin{9};
constexpr auto tilt_pin{7};

// thrust -> pwm map from BlueRobotics @ 16 V
constexpr auto pwm{std::array{ 1100.00f,1150.00f,1200.00f,1250.00f,1300.00f,1350.00f,1400.00f,1450.00f,1480.00f,1520.00f,1550.00f,1600.00f,1650.00f,1700.00f,1750.00f,1800.00f,1850.00f,1900.00f }};
//constexpr auto vels{std::array{ 362.81f,336.68f,299.98f,265.20f,224.11f,180.25f,130.49f,63.83f,0.00f,0.00f,63.00f,129.54f,180.06f,222.91f,263.67f,302.42f,340.46f,370.01f }};
constexpr auto thrusts{std::array{ -39.92f,-34.20f,-26.56f,-20.14f,-14.11f,-9.06f,-4.72f,-1.13f,-0.50f,0.00f,1.38f,5.92f,11.41f,17.89f,25.65f,33.55f,43.61f,51.45f }};

static_assert (thrusts.size() == pwm.size(), "vel/pwm map should have same sizes");
//static_assert (thruster_pins.size() == 6, "thruster pins should be size 6");

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

uint interpWrench(double force, double min, double max)
{
  if(force < min)
    return 1100;
  else if(force > max)
    return 1900;
  return 1100 + (force - min)/(max-min)*800; // Scale to [1100, 1900]
}

class BlurrPWM : public rclcpp::Node
{
public:
  explicit BlurrPWM(rclcpp::NodeOptions options) :
      rclcpp::Node("pwm", options)
  {
    for(auto &c: pwm.channels)
      c = pwm.CHAN_NOCHANGE;

    pwm_pub = create_publisher<OverrideRCIn>("rc/override", 2);

    tilt_pub = create_publisher<JointState>("joint_states", rclcpp::SensorDataQoS());
    joint_states.name = {"thruster1","thruster2","thruster3","thruster4","thruster5","thruster6", "tilt"};
    joint_states.position.resize(7, 0);

    light_sub = create_subscription<Float32>("cmd_light", 10, [&](Float32::UniquePtr msg)
                                             {light_intensity = std::clamp(msg->data, 0.f, 1.f);});

    tilt_sub = create_subscription<Float32>("cmd_tilt", 10, [&](Float32::UniquePtr msg)
                                            {tilt_angle() = std::clamp<double>(msg->data, -M_PI_4, M_PI_4);});


    if(use_wrench)
    {
      wrench_sub = create_subscription<Wrench>("cmd_wrench", 10, [&](Wrench::UniquePtr msg)
                                               {cmd_wrench_cb(*msg);});
      wrench_min.resize(6, -40.);
      wrench_max.resize(6, 40.);

      const std::array<std::string,6> axis{"pitch", "roll", "heave", "yaw", "surge", "sway"};
      for(int i = 0; i < 6; ++i)
      {
        wrench_min[i] = declare_parameter(axis[i]+".min", wrench_min[i]);
        wrench_max[i] = declare_parameter(axis[i]+".max", wrench_max[i]);
      }
      wrench_bound_callback = add_on_set_parameters_callback([&](const std::vector<rclcpp::Parameter> &params)
                                                             {return updateParams(params);});


    }
    else
    {
      thruster_sub = create_subscription<JointState>("cmd_thrust", 10, [&](JointState::UniquePtr msg)
                                                     {cmd_thrust_cb(*msg);});
    }
  }

private:

  inline double& tilt_angle()
  {
    return joint_states.position.back();
  }

  rclcpp::TimerBase::SharedPtr
      pwm_timer{create_wall_timer(20ms, [&](){publish();})},
      thruster_watchdog{create_wall_timer(std::chrono::seconds(watchdog_period), [&](){watchDog();})};

  rclcpp::Subscription<Float32>::SharedPtr tilt_sub, light_sub;

  // whether we use raw thrusts or body wrench
  bool use_wrench{declare_parameter("use_wrench_input", true)};

  rclcpp::Subscription<JointState>::SharedPtr thruster_sub;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub;
  std::vector<double> wrench_min;
  std::vector<double> wrench_max;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr wrench_bound_callback;

  // received reference
  std::array<double, 6> thruster_force{0};
  bool thruster_change{true};
  rclcpp::Time thruster_time{now()};
  JointState joint_states;
  float light_intensity{0};
  rclcpp::Publisher<JointState>::SharedPtr tilt_pub;

  // bridge to RC override
  OverrideRCIn pwm;
  rclcpp::Publisher<OverrideRCIn>::SharedPtr pwm_pub;

  // callbacks

  void cmd_wrench_cb(const Wrench &msg)
  {
    thruster_time = now();

    pwm.channels[0] = interpWrench(msg.torque.y, wrench_min[0], wrench_max[0]);
    pwm.channels[1] = interpWrench(msg.torque.x, wrench_min[1], wrench_max[1]);
    pwm.channels[2] = interpWrench(msg.force.z, wrench_min[2], wrench_max[2]);
    pwm.channels[3] = interpWrench(msg.torque.z, wrench_min[3], wrench_max[3]);
    pwm.channels[4] = interpWrench(msg.force.x, wrench_min[4], wrench_max[4]);
    pwm.channels[5] = interpWrench(msg.force.y, wrench_min[5], wrench_max[5]);
  }

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
          thruster_change = true;
          thruster_force[j] = msg.effort[i];
          joint_states.position[j] += msg.effort[i]*0.02;
          if(std::abs(joint_states.position[j]) > 4000)
            joint_states.position[j] = 0;
          break;
        }
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult updateParams(const std::vector<rclcpp::Parameter> &params)
  {
    const std::array<std::string,6> axis{"pitch", "roll", "heave", "yaw", "surge", "sway"};
    for(int i = 0; i < 6; ++i)
    {
      if(auto p = std::find_if(params.begin(), params.end(), [&](const rclcpp::Parameter &p){return p.get_name() == axis[i]+".min";}); p != params.end())
        wrench_min[i] = p->as_double();
      if(auto p = std::find_if(params.begin(), params.end(), [&](const rclcpp::Parameter &p){return p.get_name() == axis[i]+".max";}); p != params.end())
        wrench_max[i] = p->as_double();
    }
    return rcl_interfaces::msg::SetParametersResult().set__successful(true);
  }


  void watchDog()
  {
    if((now() - thruster_time).seconds() > watchdog_period)
    {
      for(auto pin: {0,1,2,3,4,5})
        pwm.channels[pin] = 1500;
      tilt_angle() = 0;
      light_intensity = 0;
      thruster_change = true;
    }
  }

  void publish()
  {
    // at least remind the tilt angle
    joint_states.header.stamp = now();
    tilt_pub->publish(joint_states);

    // thrusters
    if(thruster_change && !use_wrench)
    {
      thruster_change = false;
      for(auto i: {0,1,2,3,4,5})
        pwm.channels[i] = interpPWM(thruster_force[i]);
    }

    // tilt
    pwm.channels[tilt_pin] = 1500 + 509.3*tilt_angle();

    // lumen light
    pwm.channels[light_pin] = 1100 + 800*light_intensity;

    pwm_pub->publish(pwm);
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

