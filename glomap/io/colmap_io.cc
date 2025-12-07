#include "glomap/io/colmap_io.h"

#include <colmap/util/file.h>
#include <colmap/util/misc.h>
#include <fstream>

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
    const std::string suffix) {
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
    // Read in colors
    if (image_path != "") {
      LOG(INFO) << "Extracting colors ...";
      reconstruction.ExtractColorsForAllImages(image_path);
    }
    colmap::CreateDirIfNotExists(reconstruction_path, true);
    if (output_format == "txt") {
      reconstruction.WriteText(reconstruction_path);
    } else if (output_format == "bin") {
      reconstruction.WriteBinary(reconstruction_path);
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
      // Read in colors
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

void WriteViewGraph(const std::string& path, const ViewGraph& view_graph) {
  std::ofstream file(path, std::ios::binary);
  if (!file.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << path;
    return;
  }

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
}

bool ReadViewGraph(const std::string& path, ViewGraph& view_graph) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    LOG(ERROR) << "Could not open file for reading: " << path;
    return false;
  }

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
  return true;
}

}  // namespace glomap
