// Independent installed-library probe: actual ROS2 parameter service -> Pinhole.
#include <sophus/se3.hpp>
#include <vikit/camera_loader.h>
#include <vikit/pinhole_camera.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

int main(int argc, char **argv)
{
  const std::string mode = argc > 1 ? argv[1] : "delayed";
  rclcpp::init(argc, argv);
  auto client = std::make_shared<rclcpp::Node>("impact_camera_loader_probe");
  rclcpp::executors::SingleThreadedExecutor server_executor;
  std::atomic<bool> stop_server{false};
  std::exception_ptr server_error;
  std::thread server;
  if (mode != "missing") {
    server = std::thread([&]() {
      try {
        // Reproduce the old 100ms discovery race deliberately.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto node = std::make_shared<rclcpp::Node>("parameter_blackboard");
        node->declare_parameter("cam_model", "Pinhole");
        node->declare_parameter("cam_width", 320);
        node->declare_parameter("cam_height", 240);
        node->declare_parameter("scale", 1.0);
        if (mode == "wrong-type") node->declare_parameter("cam_fx", "invalid");
        else node->declare_parameter("cam_fx", 277.128129211);
        node->declare_parameter("cam_fy", 277.128129211);
        node->declare_parameter("cam_cx", 159.5);
        node->declare_parameter("cam_cy", 119.5);
        for (const auto *key : {"cam_d0", "cam_d1", "cam_d2", "cam_d3"}) {
          node->declare_parameter(key, 0.0);
        }
        server_executor.add_node(node);
        while (!stop_server && rclcpp::ok()) {
          server_executor.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      } catch (...) { server_error = std::current_exception(); }
    });
  }

  vk::AbstractCamera *raw_camera = nullptr;
  bool success = false;
  const auto start = std::chrono::steady_clock::now();
  try {
    const bool loaded = vk::camera_loader::loadFromRosNs(client, "parameter_blackboard", raw_camera);
    std::unique_ptr<vk::AbstractCamera> camera(raw_camera);
    const auto *pinhole = dynamic_cast<const vk::PinholeCamera *>(camera.get());
    if (mode == "delayed" && loaded && pinhole) {
      const Eigen::Vector2d center = pinhole->world2cam(Eigen::Vector3d(0, 0, 1));
      const Eigen::Vector3d ray = pinhole->cam2world(159.5, 119.5);
      success = pinhole->width() == 320 && pinhole->height() == 240 &&
        std::abs(pinhole->fx() - 277.128129211) < 1e-9 &&
        std::abs(pinhole->fy() - 277.128129211) < 1e-9 &&
        (center - Eigen::Vector2d(159.5, 119.5)).norm() < 1e-9 &&
        (ray - Eigen::Vector3d(0, 0, 1)).norm() < 1e-9 &&
        Sophus::SE3d().matrix().isIdentity();
    }
  } catch (const std::exception &error) {
    std::cerr << "loader exception (expected for negative modes): " << error.what() << '\n';
    success = mode == "missing" || mode == "wrong-type";
  }
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  if (mode == "missing") success = success && elapsed >= 4.5 && elapsed < 11.0;
  stop_server = true;
  if (server.joinable()) server.join();
  if (server_error) success = false;
  rclcpp::shutdown();
  std::cout << "camera_loader_probe mode=" << mode << " status=" << (success ? "PASS" : "FAIL")
            << " elapsed_s=" << elapsed << '\n';
  return success ? 0 : 1;
}
