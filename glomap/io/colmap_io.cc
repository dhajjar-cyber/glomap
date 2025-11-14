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

}  // namespace glomap
