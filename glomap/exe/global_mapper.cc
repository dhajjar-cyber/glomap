#include "glomap/controllers/global_mapper.h"

#include "glomap/controllers/option_manager.h"
#include "glomap/io/colmap_converter.h"
#include "glomap/io/colmap_io.h"
#include "glomap/io/pose_io.h"
#include "glomap/types.h"

#include <colmap/scene/reconstruction.h>
#include <colmap/util/file.h>
#include <colmap/util/misc.h>
#include <colmap/util/timer.h>

namespace glomap {
// -------------------------------------
// Mappers starting from COLMAP database
// -------------------------------------
int RunMapper(int argc, char** argv) {
  std::string database_path;
  std::string output_path;

  std::string image_path = "";
  std::string image_list_path = "";
  std::string constraint_type = "ONLY_POINTS";
  std::string output_format = "bin";

  OptionManager options;
  options.AddRequiredOption("database_path", &database_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("image_path", &image_path);
  options.AddDefaultOption("image_list_path", &image_list_path);
  options.AddDefaultOption("constraint_type",
                           &constraint_type,
                           "{ONLY_POINTS, ONLY_CAMERAS, "
                           "POINTS_AND_CAMERAS_BALANCED, POINTS_AND_CAMERAS}");
  options.AddDefaultOption("output_format", &output_format, "{bin, txt}");
  options.AddGlobalMapperFullOptions();

  options.Parse(argc, argv);

  // Pass output path to mapper options for checkpointing
  options.mapper->output_path = output_path;

  if (!colmap::ExistsFile(database_path)) {
    LOG(ERROR) << "`database_path` is not a file";
    return EXIT_FAILURE;
  }

  if (constraint_type == "ONLY_POINTS") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::ONLY_POINTS;
  } else if (constraint_type == "ONLY_CAMERAS") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::ONLY_CAMERAS;
  } else if (constraint_type == "POINTS_AND_CAMERAS_BALANCED") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED;
  } else if (constraint_type == "POINTS_AND_CAMERAS") {
    options.mapper->opt_gp.constraint_type =
        GlobalPositionerOptions::POINTS_AND_CAMERAS;
  } else {
    LOG(ERROR) << "Invalid constriant type";
    return EXIT_FAILURE;
  }

  // Check whether output_format is valid
  if (output_format != "bin" && output_format != "txt") {
    LOG(ERROR) << "Invalid output format";
    return EXIT_FAILURE;
  }

  // Load the database
  ViewGraph view_graph;
  std::unordered_map<rig_t, Rig> rigs;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<frame_t, Frame> frames;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;

  auto database = colmap::Database::Open(database_path);
  ConvertDatabaseToGlomap(*database, view_graph, rigs, cameras, frames, images, image_list_path);

  if (view_graph.image_pairs.empty()) {
    LOG(ERROR) << "Can't continue without image pairs";
    return EXIT_FAILURE;
  }

  // Resumption Logic
  std::string checkpoint_ba_path = output_path + "/checkpoint_ba";
  std::string checkpoint_gp_path = output_path + "/checkpoint_gp";
  std::string checkpoint_tracks_path = output_path + "/checkpoint_tracks";
  std::string checkpoint_rotation_path = output_path + "/checkpoint_rotation";

  auto IsValidCheckpoint = [](const std::string& path) {
    return colmap::ExistsDir(path) &&
           (colmap::ExistsFile(path + "/cameras.bin") || colmap::ExistsFile(path + "/cameras.txt")) &&
           (colmap::ExistsFile(path + "/images.bin") || colmap::ExistsFile(path + "/images.txt")) &&
           (colmap::ExistsFile(path + "/points3D.bin") || colmap::ExistsFile(path + "/points3D.txt")) &&
           colmap::ExistsFile(path + "/view_graph.bin");
  };

  if (IsValidCheckpoint(checkpoint_ba_path)) {
    LOG(INFO) << "Found checkpoint: " << checkpoint_ba_path;
    LOG(INFO) << "Resuming from Bundle Adjustment...";

    colmap::Reconstruction recon;
    recon.Read(checkpoint_ba_path);
    ConvertColmapToGlomap(recon, rigs, cameras, frames, images, tracks);
    ReadExtraData(checkpoint_ba_path + "/view_graph.bin", view_graph, frames);

    options.mapper->skip_preprocessing = true;
    options.mapper->skip_view_graph_calibration = true;
    options.mapper->skip_relative_pose_estimation = true;
    options.mapper->skip_rotation_averaging = true;
    options.mapper->skip_track_establishment = true;
    options.mapper->skip_global_positioning = true;
    options.mapper->skip_bundle_adjustment = true;

  } else if (IsValidCheckpoint(checkpoint_gp_path)) {
    LOG(INFO) << "Found checkpoint: " << checkpoint_gp_path;
    LOG(INFO) << "Resuming from Global Positioning...";

    colmap::Reconstruction recon;
    recon.Read(checkpoint_gp_path);
    ConvertColmapToGlomap(recon, rigs, cameras, frames, images, tracks);
    ReadExtraData(checkpoint_gp_path + "/view_graph.bin", view_graph, frames);

    options.mapper->skip_preprocessing = true;
    options.mapper->skip_view_graph_calibration = true;
    options.mapper->skip_relative_pose_estimation = true;
    options.mapper->skip_rotation_averaging = true;
    options.mapper->skip_track_establishment = true;
    options.mapper->skip_global_positioning = true;

  } else if (IsValidCheckpoint(checkpoint_tracks_path)) {
    LOG(INFO) << "Found checkpoint: " << checkpoint_tracks_path;
    LOG(INFO) << "Resuming from Track Establishment...";

    colmap::Reconstruction recon;
    recon.Read(checkpoint_tracks_path);
    ConvertColmapToGlomap(recon, rigs, cameras, frames, images, tracks);
    ReadExtraData(checkpoint_tracks_path + "/view_graph.bin", view_graph, frames);

    options.mapper->skip_preprocessing = true;
    options.mapper->skip_view_graph_calibration = true;
    options.mapper->skip_relative_pose_estimation = true;
    options.mapper->skip_rotation_averaging = true;
    options.mapper->skip_track_establishment = true;

  } else if (IsValidCheckpoint(checkpoint_rotation_path)) {
    LOG(INFO) << "Found checkpoint: " << checkpoint_rotation_path;
    LOG(INFO) << "Resuming from Rotation Averaging...";

    colmap::Reconstruction recon;
    recon.Read(checkpoint_rotation_path);
    ConvertColmapToGlomap(recon, rigs, cameras, frames, images, tracks);
    ReadExtraData(checkpoint_rotation_path + "/view_graph.bin", view_graph, frames);

    options.mapper->skip_preprocessing = true;
    options.mapper->skip_view_graph_calibration = true;
    options.mapper->skip_relative_pose_estimation = true;
    options.mapper->skip_rotation_averaging = true;
  } else {
    LOG(INFO) << "No checkpoints found. Starting reconstruction from scratch.";
  }

  GlobalMapper global_mapper(*options.mapper);

  // Main solver
  LOG(INFO) << "Loaded database";
  colmap::Timer run_timer;
  run_timer.Start();
  global_mapper.Solve(
      *database, view_graph, rigs, cameras, frames, images, tracks);
  run_timer.Pause();

  LOG(INFO) << "Reconstruction done in " << run_timer.ElapsedSeconds()
            << " seconds";

  WriteGlomapReconstruction(output_path,
                            rigs,
                            cameras,
                            frames,
                            images,
                            tracks,
                            output_format,
                            image_path,
                            image_list_path);
  LOG(INFO) << "Export to COLMAP reconstruction done";

  return EXIT_SUCCESS;
}

// -------------------------------------
// Mappers starting from COLMAP reconstruction
// -------------------------------------
int RunMapperResume(int argc, char** argv) {
  std::string input_path;
  std::string output_path;
  std::string image_path = "";
  std::string output_format = "bin";

  OptionManager options;
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddDefaultOption("image_path", &image_path);
  options.AddDefaultOption("output_format", &output_format, "{bin, txt}");
  options.AddGlobalMapperResumeFullOptions();

  options.Parse(argc, argv);

  if (!colmap::ExistsDir(input_path)) {
    LOG(ERROR) << "`input_path` is not a directory";
    return EXIT_FAILURE;
  }

  // Check whether output_format is valid
  if (output_format != "bin" && output_format != "txt") {
    LOG(ERROR) << "Invalid output format";
    return EXIT_FAILURE;
  }

  // Load the reconstruction
  ViewGraph view_graph;                        // dummy variable
  std::shared_ptr<colmap::Database> database;  // dummy variable

  std::unordered_map<rig_t, Rig> rigs;
  std::unordered_map<camera_t, Camera> cameras;
  std::unordered_map<frame_t, Frame> frames;
  std::unordered_map<image_t, Image> images;
  std::unordered_map<track_t, Track> tracks;
  colmap::Reconstruction reconstruction;
  reconstruction.Read(input_path);
  ConvertColmapToGlomap(reconstruction, rigs, cameras, frames, images, tracks);

  GlobalMapper global_mapper(*options.mapper);

  // Main solver
  colmap::Timer run_timer;
  run_timer.Start();
  global_mapper.Solve(
      *database, view_graph, rigs, cameras, frames, images, tracks);
  run_timer.Pause();

  LOG(INFO) << "Reconstruction done in " << run_timer.ElapsedSeconds()
            << " seconds";

  WriteGlomapReconstruction(output_path,
                            rigs,
                            cameras,
                            frames,
                            images,
                            tracks,
                            output_format,
                            image_path);
  LOG(INFO) << "Export to COLMAP reconstruction done";

  return EXIT_SUCCESS;
}

}  // namespace glomap
