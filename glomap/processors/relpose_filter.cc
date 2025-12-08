#include "glomap/processors/relpose_filter.h"

#include "glomap/math/rigid3d.h"
#include <regex>

namespace glomap {

static bool IsOverlappingRigPair(const std::string& name1, const std::string& name2) {
    // First check if they are in the same frame (IsRigPair logic)
    static const std::regex frame_regex("f(\\d+)");
    std::smatch match1, match2;
    if (!std::regex_search(name1, match1, frame_regex) || 
        !std::regex_search(name2, match2, frame_regex) ||
        match1[1] != match2[1]) {
        return false;
    }

    // Now check for camera overlap based on R{r}C{c} pattern
    static const std::regex cam_regex("R(\\d+)C(\\d+)");
    std::smatch cam_match1, cam_match2;
    
    if (std::regex_search(name1, cam_match1, cam_regex) && 
        std::regex_search(name2, cam_match2, cam_regex)) {
        
        int r1 = std::stoi(cam_match1[1]);
        int c1 = std::stoi(cam_match1[2]);
        int r2 = std::stoi(cam_match2[1]);
        int c2 = std::stoi(cam_match2[2]);

        // Same ring neighbors (assuming 8 cameras per ring)
        if (r1 == r2) {
            int diff = std::abs(c1 - c2);
            if (diff == 1 || diff == 7) return true; 
        }
        
        // Vertical overlap (same column, different ring)
        if (c1 == c2 && r1 != r2) return true;
    }
    
    return false;
}

void RelPoseFilter::FilterRotations(
    ViewGraph& view_graph,
    const std::unordered_map<image_t, Image>& images,
    double max_angle) {
  int num_invalid = 0;
  for (auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false) continue;

    const Image& image1 = images.at(image_pair.image_id1);
    const Image& image2 = images.at(image_pair.image_id2);

    if (image1.IsRegistered() == false || image2.IsRegistered() == false) {
      continue;
    }

    Rigid3d pose_calc = image2.CamFromWorld() * Inverse(image1.CamFromWorld());

    double angle = CalcAngle(pose_calc, image_pair.cam2_from_cam1);
    if (angle > max_angle) {
      /*
      if (IsOverlappingRigPair(image1.file_name, image2.file_name)) {
          LOG(WARNING) << "Rig Pair Rejected at FilterRotations (EXPECTED OVERLAP): " 
                       << image1.file_name << " - " << image2.file_name
                       << " Angle Error: " << angle << " > " << max_angle;
      }
      */

      image_pair.is_valid = false;
      num_invalid++;
    } else {
      /*
      if (IsOverlappingRigPair(image1.file_name, image2.file_name)) {
          LOG(INFO) << "Rig Pair Accepted at FilterRotations (EXPECTED OVERLAP): " 
                       << image1.file_name << " - " << image2.file_name
                       << " Angle Error: " << angle << " <= " << max_angle;
      }
      */
    }
  }
  /*
  LOG(INFO) << "Filtered " << num_invalid << " relative rotation with angle > "
            << max_angle << " degrees";
    */
}

void RelPoseFilter::FilterInlierNum(ViewGraph& view_graph, 
                                    const std::unordered_map<image_t, Image>& images,
                                    int min_inlier_num) {
  int num_invalid = 0;
  for (auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false) continue;

    if (image_pair.inliers.size() < min_inlier_num) {
      if (IsOverlappingRigPair(images.at(image_pair.image_id1).file_name, images.at(image_pair.image_id2).file_name)) {

        // LOG(WARNING) << "Rig Pair Rejected at FilterInlierNum (EXPECTED OVERLAP): " 
        //                << images.at(image_pair.image_id1).file_name << " - " << images.at(image_pair.image_id2).file_name
        //                << " Inliers: " << image_pair.inliers.size() << " < " << min_inlier_num;

                      }
      image_pair.is_valid = false;
      num_invalid++;
    } else {
      if (IsOverlappingRigPair(images.at(image_pair.image_id1).file_name, images.at(image_pair.image_id2).file_name)) {
          // LOG(INFO) << "Rig Pair Accepted at FilterInlierNum (EXPECTED OVERLAP): " 
          //              << images.at(image_pair.image_id1).file_name << " - " << images.at(image_pair.image_id2).file_name
          //              << " Inliers: " << image_pair.inliers.size() << " >= " << min_inlier_num;
      }
    }
  }

  // LOG(INFO) << "Filtered " << num_invalid
  //           << " relative poses with inlier number < " << min_inlier_num;
}

void RelPoseFilter::FilterInlierRatio(ViewGraph& view_graph,
                                      const std::unordered_map<image_t, Image>& images,
                                      double min_inlier_ratio) {
  int num_invalid = 0;
  for (auto& [pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false) continue;

    double ratio =
        image_pair.inliers.size() * 1.0 / image_pair.matches.rows();
    if (ratio < min_inlier_ratio) {
      if (IsOverlappingRigPair(images.at(image_pair.image_id1).file_name, images.at(image_pair.image_id2).file_name)) {
          // LOG(WARNING) << "Rig Pair Rejected at FilterInlierRatio (EXPECTED OVERLAP): " 
          //              << images.at(image_pair.image_id1).file_name << " - " << images.at(image_pair.image_id2).file_name
          //              << " Ratio: " << ratio << " < " << min_inlier_ratio;
      }
      image_pair.is_valid = false;
      num_invalid++;
    } else {
      // if (IsOverlappingRigPair(images.at(image_pair.image_id1).file_name, images.at(image_pair.image_id2).file_name)) {
      //     LOG(INFO) << "Rig Pair Accepted at FilterInlierRatio (EXPECTED OVERLAP): " 
      //                  << images.at(image_pair.image_id1).file_name << " - " << images.at(image_pair.image_id2).file_name
      //                  << " Ratio: " << ratio << " >= " << min_inlier_ratio;
      // }
    }
  }

//   LOG(INFO) << "Filtered " << num_invalid
//             << " relative poses with inlier ratio < " << min_inlier_ratio;
}

}  // namespace glomap
