#include "glomap/controllers/track_retriangulation.h"

#include "glomap/io/colmap_converter.h"

#include <colmap/controllers/incremental_pipeline.h>
#include <colmap/estimators/bundle_adjustment.h>
#include <colmap/scene/database_cache.h>

#include <glog/logging.h>
#include <iomanip>
#include <set>

namespace glomap {

bool RetriangulateTracks(const TriangulatorOptions& options,
                         const colmap::Database& database,
                         std::unordered_map<rig_t, Rig>& rigs,
                         std::unordered_map<camera_t, Camera>& cameras,
                         std::unordered_map<frame_t, Frame>& frames,
                         std::unordered_map<image_t, Image>& images,
                         std::unordered_map<track_t, Track>& tracks) {
  // Following code adapted from COLMAP
  auto database_cache =
      colmap::DatabaseCache::Create(database,
                                    options.min_num_matches,
                                    false,  // ignore_watermarks
                                    {}      // reconstruct all possible images
      );

  // Check whether the image is in the database cache. If not, set the image
  // as not registered to avoid memory error.
  std::vector<image_t> image_ids_notconnected;
  for (auto& image : images) {
    if (!database_cache->ExistsImage(image.first) &&
        image.second.IsRegistered()) {
      image_ids_notconnected.push_back(image.first);
      image.second.frame_ptr->is_registered = false;
    }
  }

  // Convert the glomap data structures to colmap data structures
  std::shared_ptr<colmap::Reconstruction> reconstruction_ptr =
      std::make_shared<colmap::Reconstruction>();
  ConvertGlomapToColmap(rigs,
                        cameras,
                        frames,
                        images,
                        std::unordered_map<track_t, Track>(),
                        *reconstruction_ptr);

  colmap::IncrementalPipelineOptions options_colmap;
  options_colmap.triangulation.complete_max_reproj_error =
      options.tri_complete_max_reproj_error;
  options_colmap.triangulation.merge_max_reproj_error =
      options.tri_merge_max_reproj_error;
  options_colmap.triangulation.min_angle = options.tri_min_angle;

  reconstruction_ptr->DeleteAllPoints2DAndPoints3D();
  reconstruction_ptr->TranscribeImageIdsToDatabase(database);

  colmap::IncrementalMapper mapper(database_cache);
  mapper.BeginReconstruction(reconstruction_ptr);

  // Triangulate all images.
  const auto tri_options = options_colmap.Triangulation();
  const auto mapper_options = options_colmap.Mapper();

  const std::vector<image_t> reg_image_ids = reconstruction_ptr->RegImageIds();

  std::cout << "Starting retriangulation with " << reg_image_ids.size() << " registered images." << std::endl;
  size_t initial_num_points = reconstruction_ptr->NumPoints3D();
  size_t initial_num_observations = reconstruction_ptr->ComputeNumObservations();

  // Suppress verbose COLMAP logs during the loop
  const int previous_min_loglevel = FLAGS_minloglevel;
  FLAGS_minloglevel = 2; // ERROR level only

  size_t image_idx = 0;
  for (const image_t image_id : reg_image_ids) {
    image_idx++;
    if (image_idx % 10 == 0 || image_idx == reg_image_ids.size()) {
        float progress = static_cast<float>(image_idx) / reg_image_ids.size() * 100.0f;
        std::cout << "\r Triangulating images... " << std::fixed << std::setprecision(1) 
                  << progress << "% (" << image_idx << "/" << reg_image_ids.size() << ")" << std::flush;
    }

    const auto& image = reconstruction_ptr->Image(image_id);

    int num_tris = mapper.TriangulateImage(tri_options, image_id);
  }
  
  // Restore log level
  FLAGS_minloglevel = previous_min_loglevel;
  std::cout << std::endl;
  std::cout << std::endl;

  // Merge and complete tracks.
  size_t num_merged = mapper.CompleteAndMergeTracks(tri_options);
  std::cout << "Initial triangulation complete. Merged " << num_merged << " tracks." << std::endl;

  // -------------------------------------------------------------------------
  // HYBRID STRATEGY: Downsample points to avoid OOM
  // -------------------------------------------------------------------------
  if (options.max_num_tracks > 0 && reconstruction_ptr->NumPoints3D() > options.max_num_tracks) {
    std::cout << "Downsampling points from " << reconstruction_ptr->NumPoints3D() 
              << " to limit " << options.max_num_tracks << "..." << std::endl;

    // Step 1: Hard Cut - Remove weak tracks (< 3 observations)
    const size_t min_track_len = 3;
    std::vector<colmap::point3D_t> points_to_delete;
    for (const auto& point : reconstruction_ptr->Points3D()) {
      if (point.second.track.Length() < min_track_len) {
        points_to_delete.push_back(point.first);
      }
    }
    for (const auto point3D_id : points_to_delete) {
      reconstruction_ptr->DeletePoint3D(point3D_id);
    }
    std::cout << "  After removing weak tracks (<3 obs): " << reconstruction_ptr->NumPoints3D() << " points." << std::endl;

    // Step 2: Top-N by Track Length
    if (reconstruction_ptr->NumPoints3D() > options.max_num_tracks) {
      std::cout << "  Still above limit. Sorting by track length..." << std::endl;
      
      // Store (length, point3D_id) pairs
      std::vector<std::pair<size_t, colmap::point3D_t>> track_lengths;
      track_lengths.reserve(reconstruction_ptr->NumPoints3D());
      for (const auto& point : reconstruction_ptr->Points3D()) {
        track_lengths.emplace_back(point.second.track.Length(), point.first);
      }

      // Sort descending (longest tracks first)
      // Use partial_sort to find the top N elements we want to KEEP
      // Actually, we want to find the elements to DELETE (the smallest ones)
      // So let's sort ascending and keep the tail? Or sort descending and keep head?
      // Let's sort descending.
      std::sort(track_lengths.begin(), track_lengths.end(), 
                [](const std::pair<size_t, colmap::point3D_t>& a, const std::pair<size_t, colmap::point3D_t>& b) {
                  return a.first > b.first;
                });

      // Delete everything after the limit
      for (size_t i = options.max_num_tracks; i < track_lengths.size(); ++i) {
        reconstruction_ptr->DeletePoint3D(track_lengths[i].second);
      }
      std::cout << "  After Top-N filter: " << reconstruction_ptr->NumPoints3D() << " points." << std::endl;
    }
  }
  // -------------------------------------------------------------------------

  auto ba_options = options_colmap.GlobalBundleAdjustment();
  ba_options.refine_focal_length = false;
  ba_options.refine_principal_point = false;
  ba_options.refine_extra_params = false;
  ba_options.refine_sensor_from_rig = false;
  ba_options.refine_rig_from_world = false;

  // Configure bundle adjustment.
  colmap::BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);
  }

  colmap::ObservationManager observation_manager(*reconstruction_ptr);

  for (int i = 0; i < options_colmap.ba_global_max_refinements; ++i) {
    std::cout << "\r Global bundle adjustment iteration " << i + 1 << " / "
              << options_colmap.ba_global_max_refinements << std::flush;
    // Avoid degeneracies in bundle adjustment.
    observation_manager.FilterObservationsWithNegativeDepth();

    const size_t num_observations =
        reconstruction_ptr->ComputeNumObservations();

    std::unique_ptr<colmap::BundleAdjuster> bundle_adjuster;
    bundle_adjuster =
        CreateDefaultBundleAdjuster(ba_options, ba_config, *reconstruction_ptr);
    if (bundle_adjuster->Solve().termination_type == ceres::FAILURE) {
      return false;
    }

    size_t num_changed_observations = 0;
    num_changed_observations += mapper.CompleteAndMergeTracks(tri_options);
    num_changed_observations += mapper.FilterPoints(mapper_options);
    const double changed =
        static_cast<double>(num_changed_observations) / num_observations;
    
    std::cout << "BA Iteration " << i + 1 << ": Changed " << num_changed_observations 
              << " observations (" << (changed * 100.0) << "%). Current points: " 
              << reconstruction_ptr->NumPoints3D() << std::endl;

    if (changed < options_colmap.ba_global_max_refinement_change) {
      break;
    }
  }
  std::cout << std::endl;

  size_t final_num_points = reconstruction_ptr->NumPoints3D();
  size_t final_num_observations = reconstruction_ptr->ComputeNumObservations();
  
  std::cout << "Retriangulation Summary:" << std::endl;
  std::cout << "  Points: " << initial_num_points << " -> " << final_num_points 
            << " (+" << (final_num_points - initial_num_points) << ")" << std::endl;
  std::cout << "  Observations: " << initial_num_observations << " -> " << final_num_observations 
            << " (+" << (final_num_observations - initial_num_observations) << ")" << std::endl;

  // Add the removed images to the reconstruction
  for (const auto& image_id : image_ids_notconnected) {
    images[image_id].frame_ptr->is_registered = true;
    colmap::Image image_colmap;
    ConvertGlomapToColmapImage(images[image_id], image_colmap, true);
    reconstruction_ptr->AddImage(std::move(image_colmap));
  }

  // Convert the colmap data structures back to glomap data structures
  ConvertColmapToGlomap(
      *reconstruction_ptr, rigs, cameras, frames, images, tracks);

  return true;
}

}  // namespace glomap
