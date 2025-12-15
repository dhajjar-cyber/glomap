#include "glomap/io/colmap_io.h"

#include <colmap/util/file.h>
#include <colmap/util/misc.h>
#include <fstream>
#include <unordered_set>

namespace glomap {

void WriteGlomapReconstruction(
    const std::string& reconstruction_path,
    const std::unordered_map<rig_t, Rig>& rigs,
    const std::unordered_map<camera_t, Camera>& cameras,
    const std::unordered_map<frame_t, Frame>& frames,
    const std::unordered_map<image_t, Image>& images,
    const std::unordered_map<track_t, Track>& tracks,
    const std::string output_format,
    const std::string image_path,
    const std::string image_list_path) {
  // Load filtered image names if provided
  std::unordered_set<std::string> image_name_filter;
  if (!image_list_path.empty() && colmap::ExistsFile(image_list_path)) {
    std::ifstream file(image_list_path);
    std::string line;
    while (std::getline(file, line)) {
      line.erase(0, line.find_first_not_of(" \t\r\n"));
      line.erase(line.find_last_not_of(" \t\r\n") + 1);
      if (!line.empty()) {
        image_name_filter.insert(line);
      }
    }
    LOG(INFO) << "Loaded " << image_name_filter.size() << " image names for color extraction filtering";
  }
  
  // Check whether reconstruction pruning is applied.
  // If so, export seperate reconstruction
  int largest_component_num = -1;
  for (const auto& [frame_id, frame] : frames) {
    if (frame.cluster_id > largest_component_num)
      largest_component_num = frame.cluster_id;
  }
  // If it is not seperated into several clusters, then output them as whole
  if (largest_component_num == -1) {
    colmap::Reconstruction reconstruction;
    ConvertGlomapToColmap(
        rigs, cameras, frames, images, tracks, reconstruction);
    // Read in colors - only for images in reconstruction (already filtered)
    if (image_path != "") {
      LOG(INFO) << "Extracting colors for " << reconstruction.NumRegImages() << " registered images...";
      LOG(INFO) << "Image path: " << image_path;
      LOG(INFO) << "Reconstruction has " << reconstruction.Points3D().size() << " 3D points before color extraction";
      reconstruction.ExtractColorsForAllImages(image_path);
      LOG(INFO) << "Color extraction complete. Checking first few points...";
      // Log first few point colors to verify
      int count = 0;
      for (const auto& point3D_pair : reconstruction.Points3D()) {
        if (count++ >= 5) break;
        const auto& point3D = point3D_pair.second;
        LOG(INFO) << "Point3D " << point3D_pair.first << " color: (" 
                  << (int)point3D.color(0) << ", " << (int)point3D.color(1) << ", " << (int)point3D.color(2) << ")";
      }
    }
    colmap::CreateDirIfNotExists(reconstruction_path + "/0", true);
    if (output_format == "txt") {
      reconstruction.WriteText(reconstruction_path + "/0");
    } else if (output_format == "bin") {
      reconstruction.WriteBinary(reconstruction_path + "/0");
    } else {
      LOG(ERROR) << "Unsupported output type";
    }
  } else {
    for (int comp = 0; comp <= largest_component_num; comp++) {
      std::cout << "\r Exporting reconstruction " << comp + 1 << " / "
                << largest_component_num + 1 << std::flush;
      colmap::Reconstruction reconstruction;
      ConvertGlomapToColmap(
          rigs, cameras, frames, images, tracks, reconstruction, comp);
      // Read in colors - only for images in reconstruction (already filtered)
      if (image_path != "") {
        reconstruction.ExtractColorsForAllImages(image_path);
      }
      colmap::CreateDirIfNotExists(
          reconstruction_path + "/" + std::to_string(comp), true);
      if (output_format == "txt") {
        reconstruction.WriteText(reconstruction_path + "/" +
                                 std::to_string(comp));
      } else if (output_format == "bin") {
        reconstruction.WriteBinary(reconstruction_path + "/" +
                                   std::to_string(comp));
      } else {
        LOG(ERROR) << "Unsupported output type";
      }
    }
    std::cout << std::endl;
  }
}

void WriteColmapReconstruction(const std::string& reconstruction_path,
                               const colmap::Reconstruction& reconstruction,
                               const std::string output_format) {
  colmap::CreateDirIfNotExists(reconstruction_path, true);
  if (output_format == "txt") {
    reconstruction.WriteText(reconstruction_path);
  } else if (output_format == "bin") {
    reconstruction.WriteBinary(reconstruction_path);
  } else {
    LOG(ERROR) << "Unsupported output type";
  }
}

void WriteExtraData(const std::string& path,
                    const ViewGraph& view_graph,
                    const std::unordered_map<frame_t, Frame>& frames) {
  std::ofstream file(path, std::ios::binary);
  if (!file.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << path;
    return;
  }

  // 1. Write ViewGraph
  const uint64_t num_pairs = view_graph.image_pairs.size();
  file.write(reinterpret_cast<const char*>(&num_pairs), sizeof(uint64_t));

  for (const auto& [pair_id, pair] : view_graph.image_pairs) {
    file.write(reinterpret_cast<const char*>(&pair.image_id1), sizeof(image_t));
    file.write(reinterpret_cast<const char*>(&pair.image_id2), sizeof(image_t));
    file.write(reinterpret_cast<const char*>(&pair.is_valid), sizeof(bool));
    file.write(reinterpret_cast<const char*>(&pair.weight), sizeof(double));
    file.write(reinterpret_cast<const char*>(&pair.config), sizeof(int));

    file.write(reinterpret_cast<const char*>(pair.E.data()), 9 * sizeof(double));
    file.write(reinterpret_cast<const char*>(pair.F.data()), 9 * sizeof(double));
    file.write(reinterpret_cast<const char*>(pair.H.data()), 9 * sizeof(double));

    const Eigen::Quaterniond q = pair.cam2_from_cam1.rotation;
    const Eigen::Vector3d t = pair.cam2_from_cam1.translation;
    file.write(reinterpret_cast<const char*>(q.coeffs().data()), 4 * sizeof(double));
    file.write(reinterpret_cast<const char*>(t.data()), 3 * sizeof(double));

    const uint64_t num_matches = pair.matches.rows();
    file.write(reinterpret_cast<const char*>(&num_matches), sizeof(uint64_t));
    if (num_matches > 0) {
      file.write(reinterpret_cast<const char*>(pair.matches.data()), pair.matches.size() * sizeof(int));
    }

    const uint64_t num_inliers = pair.inliers.size();
    file.write(reinterpret_cast<const char*>(&num_inliers), sizeof(uint64_t));
    if (num_inliers > 0) {
      file.write(reinterpret_cast<const char*>(pair.inliers.data()), num_inliers * sizeof(int));
    }
  }

  // 2. Write Frames Extra Data
  const uint64_t num_frames = frames.size();
  file.write(reinterpret_cast<const char*>(&num_frames), sizeof(uint64_t));

  for (const auto& [frame_id, frame] : frames) {
    file.write(reinterpret_cast<const char*>(&frame_id), sizeof(frame_t));
    file.write(reinterpret_cast<const char*>(&frame.cluster_id), sizeof(int));

    bool has_gravity = frame.HasGravity();
    file.write(reinterpret_cast<const char*>(&has_gravity), sizeof(bool));
    if (has_gravity) {
      Eigen::Vector3d gravity = frame.gravity_info.GetGravity();
      file.write(reinterpret_cast<const char*>(gravity.data()), 3 * sizeof(double));
    }
  }
}

bool ReadExtraData(const std::string& path,
                   ViewGraph& view_graph,
                   std::unordered_map<frame_t, Frame>& frames) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    LOG(ERROR) << "Could not open file for reading: " << path;
    return false;
  }

  // 1. Read ViewGraph
  uint64_t num_pairs;
  file.read(reinterpret_cast<char*>(&num_pairs), sizeof(uint64_t));

  view_graph.image_pairs.clear();
  view_graph.image_pairs.reserve(num_pairs);

  for (uint64_t i = 0; i < num_pairs; ++i) {
    image_t image_id1, image_id2;
    file.read(reinterpret_cast<char*>(&image_id1), sizeof(image_t));
    file.read(reinterpret_cast<char*>(&image_id2), sizeof(image_t));

    ImagePair pair(image_id1, image_id2);

    file.read(reinterpret_cast<char*>(&pair.is_valid), sizeof(bool));
    file.read(reinterpret_cast<char*>(&pair.weight), sizeof(double));
    file.read(reinterpret_cast<char*>(&pair.config), sizeof(int));

    file.read(reinterpret_cast<char*>(pair.E.data()), 9 * sizeof(double));
    file.read(reinterpret_cast<char*>(pair.F.data()), 9 * sizeof(double));
    file.read(reinterpret_cast<char*>(pair.H.data()), 9 * sizeof(double));

    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    file.read(reinterpret_cast<char*>(q.coeffs().data()), 4 * sizeof(double));
    file.read(reinterpret_cast<char*>(t.data()), 3 * sizeof(double));
    pair.cam2_from_cam1 = Rigid3d(q, t);

    uint64_t num_matches;
    file.read(reinterpret_cast<char*>(&num_matches), sizeof(uint64_t));
    if (num_matches > 0) {
      pair.matches.resize(num_matches, 2);
      file.read(reinterpret_cast<char*>(pair.matches.data()), pair.matches.size() * sizeof(int));
    }

    uint64_t num_inliers;
    file.read(reinterpret_cast<char*>(&num_inliers), sizeof(uint64_t));
    if (num_inliers > 0) {
      pair.inliers.resize(num_inliers);
      file.read(reinterpret_cast<char*>(pair.inliers.data()), num_inliers * sizeof(int));
    }

    view_graph.image_pairs.emplace(pair.pair_id, std::move(pair));
  }

  // 2. Read Frames Extra Data (if available)
  if (file.peek() == EOF) {
    return true; // Backward compatibility
  }

  uint64_t num_frames;
  file.read(reinterpret_cast<char*>(&num_frames), sizeof(uint64_t));

  for (uint64_t i = 0; i < num_frames; ++i) {
    frame_t frame_id;
    int cluster_id;
    bool has_gravity;

    file.read(reinterpret_cast<char*>(&frame_id), sizeof(frame_t));
    file.read(reinterpret_cast<char*>(&cluster_id), sizeof(int));
    file.read(reinterpret_cast<char*>(&has_gravity), sizeof(bool));

    if (frames.find(frame_id) != frames.end()) {
      frames[frame_id].cluster_id = cluster_id;
      if (has_gravity) {
        Eigen::Vector3d gravity;
        file.read(reinterpret_cast<char*>(gravity.data()), 3 * sizeof(double));
        frames[frame_id].gravity_info.SetGravity(gravity);
      }
    } else {
      // Skip gravity data if frame not found (should not happen if DB is consistent)
      if (has_gravity) {
        file.seekg(3 * sizeof(double), std::ios::cur);
      }
    }
  }

  return true;
}

}  // namespace glomap
