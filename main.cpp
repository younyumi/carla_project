#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unistd.h>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

#include "KinematicBicycle.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }

class InterruptException : public std::exception
{
public:
  InterruptException(int s) : S(s) {}
  int S;
};


void sig_to_exception(int s)
{
  throw InterruptException(s);
}

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}

/// Save a semantic segmentation image to disk converting to CityScapes palette.
/*
static void SaveSemSegImageToDisk(const csd::Image &image) {
  using namespace carla::image;

  char buffer[9u];
  std::snprintf(buffer, sizeof(buffer), "%08zu", image.GetFrame());
  auto filename = "_images/"s + buffer + ".png";

  auto view = ImageView::MakeColorConvertedView(
      ImageView::MakeView(image),
      ColorConverter::CityScapesPalette());
  ImageIO::WriteView(filename, view);
}
*/

static auto ParseArguments(int argc, const char *argv[]) {
  EXPECT_TRUE((argc == 1u) || (argc == 3u));
  using ResultType = std::tuple<std::string, uint16_t>;
  return argc == 3u ?
      ResultType{argv[1u], std::stoi(argv[2u])} :
      ResultType{"localhost", 2000u};
}

int main(int argc, const char *argv[]) {
  try {

    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);

    std::mt19937_64 rng((std::random_device())());

    auto client = cc::Client(host, port);
    client.SetTimeout(40s);

    std::cout << "Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';

    // Load a random town.
    auto town_name = RandomChoice(client.GetAvailableMaps(), rng);
    // std::cout << "Loading world: " << town_name << std::endl;
    auto world = client.LoadWorld("Town05");


    // Get a random vehicle blueprint.
    auto blueprint_library = world.GetBlueprintLibrary();
    // auto vehicles = blueprint_library->Filter("vehicle");

    auto blueprint = blueprint_library->Find("vehicle.lincoln.mkz_2020");

    // auto blueprint = RandomChoice(*vehicles, rng);
    // auto blueprint = RandomChoice(*vehicles, rng);

    // Randomize the blueprint.
    // if (blueprint->ContainsAttribute("color")) {
    //   auto &attribute = blueprint->GetAttribute("color");
    //   blueprint->SetAttribute(
    //       "color",
    //       RandomChoice(attribute.GetRecommendedValues(), rng));
    // }

    // Find a valid spawn point.
    auto map = world.GetMap();
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    transform.location.x = 143.923f;
    transform.location.y = -1.96234f;
    transform.location.z = 0.3124363f;
    transform.rotation.roll = 0.00397258f;
    transform.rotation.pitch = -0.0157777f;
    transform.rotation.yaw = -179.751f;

    // Spawn the vehicle.
    auto actor = world.SpawnActor(*blueprint, transform);
    // std::cout << "Spawned " << actor->GetDisplayId() << '\n';
    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(actor);

    // Move spectator so we can see the vehicle from the simulator window.
    auto spectator = world.GetSpectator();
    transform.location += 32.0f * transform.GetForwardVector();
    transform.location.z += 2.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -15.0f;
    spectator->SetTransform(transform);

    float max_steer_angle = 70.0f;
    float wheel_base = 2.86436f;

    KinematicBicycle my_vehicle(transform.location.x, transform.location.y, transform.rotation.yaw, 10.0);

    double steering_angle = 0.0;
    double steer_increment = 0.1 *(M_PI / 180);
    double dT = 0.01;

    while (1)
    {
      usleep(1000000);
      // Apply control to vehicle.
      // cc::Vehicle::Control control;
      // control.throttle = 0.0f;
      // vehicle->ApplyControl(control);
      // vehicle->SetTransform(transform);
      auto vehicle_physics_control = vehicle->GetPhysicsControl();
      auto vehicle_transform = vehicle->GetTransform();
/*
      std::cout << "x : " << vehicle_transform.location.x << std::endl;
      std::cout << "y : " << vehicle_transform.location.y << std::endl;
      std::cout << "z : " << vehicle_transform.location.z << std::endl;
      std::cout << "roll : " << vehicle_transform.rotation.roll << std::endl;
      std::cout << "pitch : " << vehicle_transform.rotation.pitch << std::endl;
      std::cout << "yaw : " << vehicle_transform.rotation.yaw << std::endl;
      std::cout << "------" << std::endl;
*/
      //Assume vehicle only moves straight
      double acceleration = 0.0; 
      my_vehicle.Update(acceleration, steering_angle, dT, wheel_base);

      steering_angle += steer_increment;

      vehicle_transform.location.x = my_vehicle.x;
      vehicle_transform.location.y = my_vehicle.y;
      vehicle_transform.rotation.yaw = my_vehicle.psi;


      vehicle->ApplyTransform(vehicle_transform);
      sleep(0.01);
    }
/*
    // Find a camera blueprint.
    auto camera_bp = blueprint_library->Find("sensor.camera.semantic_segmentation");
    EXPECT_TRUE(camera_bp != nullptr);

    // Spawn a camera attached to the vehicle.
    auto camera_transform = cg::Transform{
        cg::Location{-5.5f, 0.0f, 2.8f},   // x, y, z.
        cg::Rotation{-15.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    auto cam_actor = world.SpawnActor(*camera_bp, camera_transform, actor.get());
    auto camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);

    // Register a callback to save images to disk.
    camera->Listen([](auto data) {
        auto image = boost::static_pointer_cast<csd::Image>(data);
        EXPECT_TRUE(image != nullptr);
        SaveSemSegImageToDisk(*image);
    });

    std::this_thread::sleep_for(10s);

    // Remove actors from the simulation.
    camera->Destroy();
*/  try{
        std::cout << "Loop continue" << std::endl;
    } catch (InterruptException &e){
      std::cout << "Interrupt occurred" << std::endl;
      vehicle->Destroy();
      std::cout << "Actors destroyed." << std::endl;
    }
    // vehicle->Destroy();
    // std::cout << "Actors destroyed." << std::endl;

  } catch (const cc::TimeoutException &e) {
    std::cout << '\n' << e.what() << std::endl;
    return 1;
  } catch (const std::exception &e) {
    std::cout << "\nException: " << e.what() << std::endl;
    return 2;
  }
}