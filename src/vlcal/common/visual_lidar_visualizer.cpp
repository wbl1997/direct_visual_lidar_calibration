#include <vlcal/common/visual_lidar_visualizer.hpp>

#include <algorithm>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <glk/texture_opencv.hpp>
#include <guik/camera/arcball_camera_control.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <vlcal/common/estimate_fov.hpp>

namespace vlcal {

VisualLiDARVisualizer::VisualLiDARVisualizer(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<VisualLiDARData::ConstPtr>& dataset,
  const bool draw_sphere,
  const bool show_image_cv)
: draw_sphere(draw_sphere),
  show_image_cv(show_image_cv),
  proj(proj),
  dataset(dataset),
  T_camera_lidar(Eigen::Isometry3d::Identity()) {
  //
  auto viewer = guik::LightViewer::instance();
  viewer->set_draw_xy_grid(false);
  viewer->use_arcball_camera_control();

  image_display_scale = std::min(1920.0 / dataset[0]->image.cols, 1080.0 / dataset[0]->image.rows);

  selected_bag_id = -1;
  blend_weight = 0.7f;
  viewer->register_ui_callback("ui", [this] { ui_callback(); });

  kill_switch = false;
  color_update_thread = std::thread([this] { color_update_task(); });
}

VisualLiDARVisualizer::~VisualLiDARVisualizer() {
  kill_switch = true;
  color_update_thread.join();
}

void VisualLiDARVisualizer::ui_callback() {
  auto viewer = guik::LightViewer::instance();

  ImGui::Begin("visualizer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  const int prev_selected_bag_id = selected_bag_id;
  if (ImGui::ArrowButton("##Left", ImGuiDir_Left)) {
    selected_bag_id--;
  }
  ImGui::SameLine();
  if (ImGui::ArrowButton("##Right", ImGuiDir_Right)) {
    selected_bag_id++;
  }
  ImGui::SameLine();
  ImGui::DragInt("bag_id", &selected_bag_id, 1, 0, dataset.size() - 1);
  selected_bag_id = std::max<int>(0, std::min<int>(dataset.size() - 1, selected_bag_id));

  if (prev_selected_bag_id != selected_bag_id) {
    std::lock_guard<std::mutex> lock(updater_mutex);
    color_updater.reset(new PointsColorUpdater(proj, dataset[selected_bag_id]->image, dataset[selected_bag_id]->points));
    viewer->update_drawable("points", color_updater->cloud_buffer, guik::VertexColor());

    if (draw_sphere) {
      sphere_updater.reset(new PointsColorUpdater(proj, dataset[selected_bag_id]->image));
      viewer->update_drawable("sphere", sphere_updater->cloud_buffer, guik::VertexColor());
    }

    if (show_image_cv) {
      cv::Mat canvas;
      cv::resize(dataset[selected_bag_id]->image, canvas, cv::Size(), image_display_scale, image_display_scale);
      cv::imshow("image", canvas);
    } else {
      cv::Mat bgr;
      cv::cvtColor(dataset[selected_bag_id]->image, bgr, cv::COLOR_GRAY2BGR);
      viewer->update_image("image", glk::create_texture(bgr));
    }
  }
  ImGui::DragFloat("blend_weight", &blend_weight, 0.01f, 0.0f, 1.0f);

  ImGui::End();
}

void VisualLiDARVisualizer::set_T_camera_lidar(const Eigen::Isometry3d& T_camera_lidar) {
  this->T_camera_lidar = T_camera_lidar;
}

void VisualLiDARVisualizer::set_view_camera(const Eigen::Isometry3d& T_lidar_camera) {
  auto viewer = guik::LightViewer::instance();
  const Eigen::Isometry3f T_world_camera =
    T_lidar_camera.cast<float>() * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());

  double max_distance = 1.0;
  const Eigen::Vector3f eye = T_world_camera.translation();
  for (size_t i = 0; i < dataset[0]->points->size(); i++) {
    const Eigen::Vector3f point = dataset[0]->points->points[i].head<3>().cast<float>();
    max_distance = std::max(max_distance, static_cast<double>((point - eye).norm()));
  }
  const float distance = static_cast<float>(std::max(1.0, max_distance * 0.1));

  Eigen::Matrix3f camera_orientation;
  camera_orientation << 0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 1.0f,
                        1.0f, 0.0f, 0.0f;
  const Eigen::Quaternionf orientation(T_world_camera.rotation() * camera_orientation);
  const Eigen::Vector3f center = eye + T_world_camera.rotation() * Eigen::Vector3f(0.0f, 0.0f, -distance);

  auto camera_control = std::make_shared<guik::ArcBallCameraControl>(distance, orientation);
  camera_control->lookat(center);
  viewer->set_camera_control(camera_control);

  const auto image_size = dataset[0]->image.size();
  const Eigen::Vector3d top_center = estimate_direction(proj, {image_size.width * 0.5, 0.0});
  const double vertical_fov = 2.0 * std::acos(std::clamp(top_center.normalized().z(), -1.0, 1.0)) * 180.0 / M_PI;
  viewer->use_perspective_projection_control(static_cast<float>(vertical_fov), 0.01f, 1000.0f);
}

bool VisualLiDARVisualizer::spin_once() {
  auto viewer = guik::LightViewer::instance();
  return viewer->spin_once();
}

void VisualLiDARVisualizer::color_update_task() {
  while (!kill_switch) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::lock_guard<std::mutex> lock(updater_mutex);
    if (color_updater) {
      color_updater->update(T_camera_lidar, blend_weight);
    }
    if (sphere_updater) {
      sphere_updater->update(T_camera_lidar, 1.0);
    }
  }
}

}  // namespace vlcal
