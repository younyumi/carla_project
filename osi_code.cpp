#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <fstream>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/client/Vehicle.h>
#include <carla/geom/Vector3D.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <carla/rpc/VehiclePhysicsControl.h>
#include <carla/rpc/VehicleControl.h>

#include <osi_hostvehicledata.pb.h>

#include "KinematicBicycle.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace cr = carla::rpc;

using namespace std::chrono_literals;
using namespace std::string_literals;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }

class InterruptException : public std::exception {
public:
  InterruptException(int s) : S(s) {}
  int S;
};

void sig_to_exception(int s) {
  throw InterruptException(s);
}

template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}

static auto ParseArguments(int argc, const char *argv[]) {
  EXPECT_TRUE((argc == 1u) || (argc == 3u));
  using ResultType = std::tuple<std::string, uint16_t>;
  return argc == 3u ? ResultType{argv[1u], std::stoi(argv[2u])} : ResultType{"localhost", 2000u};
}

double normalize_yaw(double angle) {
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

int main(int argc, const char *argv[]) {
  try {
    std::ofstream csv_file("carla_simulation_data.csv");
    if (!csv_file.is_open()) {
      std::cerr << "Error opening file" << std::endl;
      return 1;
    }
    csv_file << "Time,Kinematic_x,Kinematic_y,Kinematic_yaw,Kinematic_v,CARLA_x,CARLA_y,CARLA_yaw,CARLA_v,Kinematic_acceleration,CARLA_throttle,Kinematic_steering_angle,CARLA_steer\n";

    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);

    std::mt19937_64 rng((std::random_device())());

    auto client = cc::Client(host, port);
    client.SetTimeout(40s);

    std::cout << "Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';

    auto world = client.LoadWorld("Town05");

    auto blueprint_library = world.GetBlueprintLibrary();
    auto blueprint = blueprint_library->Find("vehicle.lincoln.mkz_2020");

    auto map = world.GetMap();
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    transform.location.x = 143.923f;
    transform.location.y = -1.96234f;
    transform.location.z = 0.3124363f;
    transform.rotation.roll = 0.00397258f;
    transform.rotation.pitch = -0.0157777f;
    transform.rotation.yaw = -179.751f;

    auto transform2 = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    transform2 = transform;
    transform2.location.y = -1.96234f + 4;

    auto actor = world.SpawnActor(*blueprint, transform);
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

    auto actor2 = world.SpawnActor(*blueprint, transform2);
    auto vehicle2 = boost::static_pointer_cast<cc::Vehicle>(actor2);

    auto spectator = world.GetSpectator();
    transform.location += 32.0f * transform.GetForwardVector();
    transform.location.z += 2.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -15.0f;
    spectator->SetTransform(transform);

    cr::VehiclePhysicsControl vehicle2_physics_control; // 이미 설정된 값에서 바뀌지 않음.
    // vehicle2_physics_control.torque_curve = { {0, 500}, {7500, 500} };
    // vehicle2_physics_control.max_rpm = 7500;
    // vehicle2->ApplyPhysicsControl(vehicle2_physics_control);

    auto vehicle2_get_physics = vehicle2->GetPhysicsControl();
    auto wheels2 = vehicle2_get_physics.GetWheels();
    float wheel_radius = wheels2[0].radius * 0.01;
    float mass = vehicle2_get_physics.mass;

    float wheel_base = 2.86436f;

    KinematicBicycle my_vehicle(143.923, -1.96234, -179.751*M_PI/180, 0);
    usleep(1000000);
    cc::Vehicle::Control control2;

    int time_step = 0;

    while (time_step < 1000) { // 10초 후 루프 종료 -> csv 파일 생성
      auto vehicle_physics_control = vehicle->GetPhysicsControl();
      auto wheels = vehicle_physics_control.GetWheels();

      auto vehicle_transform = vehicle->GetTransform();

      std::cout << "\nKinematicBicycle Model:" << std::endl;
      std::cout << "  Position - x: " << my_vehicle.x << ", y: " << my_vehicle.y << std::endl;
      std::cout << "  Orientation - yaw: " << my_vehicle.yaw * 180 / M_PI << " degrees" << std::endl;
      std::cout << "  Velocity - v: " << my_vehicle.v << " m/s" << std::endl;

      double acceleration = 0.4 * 500 / (mass * wheel_radius);  
      double steering = 0.0;
      my_vehicle.Update(acceleration, steering, 0.01, wheel_base);

      //OSI
      osi3::HostVehicleData host_vehicle_data;
      auto *motion = host_vehicle_data.mutable_vehicle_motion();
      motion->mutable_position()->set_x(my_vehicle.x);
      motion->mutable_position()->set_y(my_vehicle.y);
      motion->mutable_position()->set_z(vehicle_transform.location.z);
      motion->mutable_orientation()->set_roll(vehicle_transform.rotation.roll);
      motion->mutable_orientation()->set_pitch(vehicle_transform.rotation.pitch);
      motion->mutable_orientation()->set_yaw(my_vehicle.yaw);
      

      cg::Location new_location(
        host_vehicle_data.vehicle_motion().position().x(), 
        host_vehicle_data.vehicle_motion().position().y(), 
        host_vehicle_data.vehicle_motion().position().z());
      cg::Rotation new_rotation(
        host_vehicle_data.vehicle_motion().orientation().pitch(), 
        host_vehicle_data.vehicle_motion().orientation().yaw() * 180 / M_PI, 
        host_vehicle_data.vehicle_motion().orientation().roll());

      cg::Transform new_transform(new_location, new_rotation);
      vehicle->SetTransform(new_transform);

      auto vehicle2_transform = vehicle2->GetTransform();
      control2.throttle = 0.5f;
      control2.steer = 0.0f;
      vehicle2->ApplyControl(control2);

      auto vehicle2_velocity = vehicle2->GetVelocity();
      double vehicle2_speed = std::sqrt(vehicle2_velocity.x*vehicle2_velocity.x + vehicle2_velocity.y*vehicle2_velocity.y + vehicle2_velocity.z*vehicle2_velocity.z);

      std::cout << "\nCARLA Model:" << std::endl;
      std::cout << "  Position - x: " << vehicle2_transform.location.x << ", y: " << vehicle2_transform.location.y << std::endl;
      std::cout << "  Orientation - yaw: " << vehicle2_transform.rotation.yaw << " degrees" << std::endl;
      std::cout << "  Velocity - v: " << vehicle2_speed << " m/s" << std::endl;

      double kinematic_acceleration = acceleration;
      double carla_throttle = control2.throttle;

      double kinematic_steering_angle = steering;
      double carla_steer = control2.steer;

      std::cout << "\nComparison:" << std::endl;
      std::cout << "  Kinematic Acceleration: " << kinematic_acceleration << ", CARLA Throttle: " << carla_throttle << std::endl;
      std::cout << "  Kinematic Steering Angle: " << kinematic_steering_angle << ", CARLA Steer: " << carla_steer << std::endl;

      //csv 파일 내용 
      csv_file << time_step << ","
               << my_vehicle.x << ","
               << my_vehicle.y << ","
               << my_vehicle.yaw * 180 / M_PI << ","
               << my_vehicle.v << ","
               << vehicle2_transform.location.x << ","
               << vehicle2_transform.location.y << ","
               << vehicle2_transform.rotation.yaw << ","
               << vehicle2_speed << ","
               << kinematic_acceleration << ","
               << carla_throttle << ","
               << kinematic_steering_angle << ","
               << carla_steer << "\n";

      time_step++;
      usleep(10000);
    }

    csv_file.close(); 
    vehicle->Destroy();
    vehicle2->Destroy();

  } catch (const cc::TimeoutException &e) {
    std::cout << '\n' << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cout << "\nException: " << e.what() << std::endl;
    return 2;
  }
}
