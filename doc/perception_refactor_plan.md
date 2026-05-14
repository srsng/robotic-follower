# Perception Refactor Design And Progress

## Goal

Use one unified `detect_track_node` to replace the old standalone perception nodes:

- `detection_node`
- `tracking_node`
- `rgbd_detect_track_node`

The new node should keep ROS input/output concerns in launch/node parameters, while one YAML file defines the perception algorithm pipeline. The YAML should describe detector and tracker stages, not camera or topic wiring.

## Design Direction

The unified YAML schema follows the same pipeline idea as the old algorithm detector config, but moves it under a top-level `perception` task:

```yaml
perception:
  name: example_task
  global: {}

  detector:
    pre:
      - type: some_pre_stage
        params: {}
    process:
      - type: some_detector_stage
        params: {}
    post:
      - type: some_post_stage
        params: {}

  tracker:
    enabled: true
    pre: []
    process:
      - type: kalman3d
        params: {}
    post: []
```

`perception.global` provides shared values referenced by stage params with `${name}`.

## Architecture

### Node Boundary

`detect_track_node` owns ROS-specific work:

- `input_mode=rgbd` subscribes RGB, depth, and camera info with approximate sync.
- `input_mode=pointcloud` subscribes `PointCloud2`.
- TF lookup and fallback behavior stay in the node.
- ROS message conversion stays in the node.
- The node publishes the existing downstream topics:
  - `/perception/detections`
  - `/perception/tracked_objects`
  - `/perception/tracked_objects_custom`
  - `/perception/segmentation_debug`
  - `/perception/class_names_info`
  - `/perception/pointcloud_transformed`

YAML does not define input topics, camera source, or simulation source. Those remain launch/ROS parameters.

### Pipeline Boundary

The new `robotic_follower.perception` package owns algorithm orchestration:

- `PerceptionFrame`
  - Numeric frame data passed from the ROS node into the algorithm layer.
  - Carries RGBD arrays, point cloud arrays, camera intrinsics, TF matrix, timestamp, stale flag, and debug context.
- `PerceptionPipeline`
  - Runs detector and tracker stages.
  - Maintains tracker quality metadata used for custom tracked object messages.
- `PerceptionResult`
  - Returns detections, tracks, debug overlay, raw count, and metadata.
- `create_perception_pipeline_from_config`
  - Builds the pipeline from the new `perception` YAML schema.
  - Also contains a compatibility path for old detector/tracker-shaped configs.

### Detector Pipeline

The detector pipeline supports both RGBD and point cloud workflows.

RGBD stages use the existing `RgbdStageRegistry`:

- `segment`
- `mask_clean`
- `mask_erode`
- `segment_and_project`
- `distance_gate`
- `detection_merge`
- `table_estimate`

Point cloud stages use the existing `StageRegistry`:

- `radius_filter`
- `height_filter`
- `voxel_filter`
- `statistical_outlier_removal`
- `ground_estimation`
- `dbscan_cluster`
- `euclidean_cluster`
- `compute_bbox`
- `merge_overlapping`
- other registered point cloud stages

`mmdet3d` is represented as a detector `process` stage that wraps the existing mmdet3d detector factory.

### Tracker Pipeline

The tracker pipeline currently supports one active tracker process stage:

- `kalman3d`
- `iou3d`
- `ema3d`

If `tracker.enabled: false`, the pipeline only publishes detections and returns no tracks.

## Current Implementation Progress

### Completed In Current Worktree

- Added unified perception package:
  - `src/robotic_follower/robotic_follower/perception/__init__.py`
  - `src/robotic_follower/robotic_follower/perception/pipeline.py`
- Added unified ROS node:
  - `src/robotic_follower/robotic_follower/ros_nodes/perception/detect_track_node.py`
- Removed old ROS node source files:
  - `detection_node.py`
  - `tracking_node.py`
  - `rgbd_detect_track_node.py`
- Updated package exports:
  - `ros_nodes/perception/__init__.py` exports `DetectTrackNode`.
- Updated console scripts in `setup.py`:
  - Added `detect_track_node`.
  - Removed `detection_node`, `tracking_node`, `rgbd_detect_track_node`.
- Updated launch files:
  - `perception_real.launch.py` now starts `detect_track_node` with `input_mode=rgbd`.
  - `perception_sim.launch.py` now starts `detect_track_node` with `input_mode=pointcloud`.
  - `track_and_follow.launch.py` comments/docs now refer to `detect_track_node`.
- Migrated config files to the new `perception` schema:
  - `yolov8_seg_rgbd_track.yaml`
  - `fastsam_rgbd_track.yaml`
  - `ground_cluster.yaml`
  - `votenet_config.yaml`
  - `density_votenet_config.yaml`
  - `density_votenet_to_scene-70c.yaml`
- Added RGBD `segment` preprocessor stage:
  - Configures YOLOv8/FastSAM through the unified detector `pre` list.
- Preserved earlier regression fixes:
  - RGBD mask cleaning keeps raw masks for occlusion gating.
  - `segment_and_project` computes occlusion from raw mask area.
  - `seg_projection` has a default RGBD pipeline fallback for old configs.
- Preserved earlier point cloud helper consolidation:
  - Core point cloud operations moved to `detection/pipeline/impl/pointcloud_ops.py`.
  - ROS PointCloud2 helpers moved to `util/ros_pointcloud.py`.
  - Old `point_cloud` import paths remain compatibility wrappers.
- Added/updated tests:
  - `test_perception_pipeline.py`
  - `test_architecture_refactor_static.py`
  - `test_rgbd_pipeline_regressions.py`
- Updated `TODO.md` architecture status.

## Verification Already Run

Do not repeat these unless needed.

- `python -m compileall src\robotic_follower\robotic_follower`
  - Passed.
- `uv run --with ruff ruff check ...`
  - Passed for the changed core pipeline, unified node, launch files, setup, and tests before the final user interruption.
- `uv run --with pytest --with numpy --with pyyaml python -m pytest test\test_perception_pipeline.py test\test_architecture_refactor_static.py test\test_rgbd_pipeline_regressions.py -q`
  - Passed: `12 passed`.
  - Warning: pytest cache path could not be created due Windows permission issue.
- YAML parsing check:
  - `uv run --with pyyaml python -c "... yaml.safe_load(...) ..."`
  - Passed before final interruption.

The last combined verification run was interrupted by the user. At that point, compileall had passed again. The user explicitly requested no more testing.

## Known Constraints

- Current development environment is Windows.
- Real deployment environment is Linux.
- Do not start ROS nodes, RealSense, RViz, or other real runtime processes in the current environment.
- `uv run` may require access to the global uv cache and can hit Windows permission issues unless run with the appropriate approval.
- The worktree includes earlier uncommitted refactor/regression-fix changes. Do not revert them.

## Open Risks

- `detect_track_node` has not been run in a real ROS graph in this environment.
- The unified node has compile/static/test coverage, but not hardware/runtime validation.
- The tracker pipeline currently supports only one tracker `process` stage. `tracker.pre` and `tracker.post` are parsed but not behaviorally implemented as independent stage registries.
- `ema3d` support exists through the tracker wrapper, but the current RGBD detection postprocessing can merge detections, so 2D track IDs are not yet safely propagated through all detector post stages.
- `class_names_info` is preserved for wrapped point-cloud detector backends, but purely stage-based point cloud pipelines currently publish an empty class metadata payload.

## Next Plan

1. Review the current diff manually.
   - Confirm old nodes should remain deleted rather than converted to compatibility wrappers.
   - Confirm docs/tests should keep references to old node names only as historical context.

2. Tighten tracker pipeline if needed.
   - Decide whether `tracker.pre` and `tracker.post` need real stage registries now.
   - If not, document them as reserved fields for the next iteration.

3. Improve class metadata for stage-based detectors.
   - Collect `class_names` from point cloud algorithm stages.
   - Publish populated `/perception/class_names_info` for `ground_cluster.yaml`.

4. Decide how to handle 2D track IDs.
   - If `ema3d` is required, add a safe propagation strategy through detector post stages.
   - Otherwise keep `kalman3d` as the recommended default tracker.

5. Runtime validation on Linux/ROS environment.
   - Run `perception_sim.launch.py` first.
   - Then run `perception_real.launch.py` with the RealSense stack.
   - Validate published topic compatibility with RViz, `track_selector_node`, and `following_node`.

6. Cleanup after runtime validation.
   - Remove any obsolete docs mentioning old standalone nodes as active entrypoints.
   - Decide whether compatibility wrappers under `point_cloud` can be deleted or should stay for external imports.
