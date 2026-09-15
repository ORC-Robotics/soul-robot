"""Fachada compatível da visão inferior modularizada."""

import cv2  # type: ignore

import numpy as np

from vision.camera_config import (
    CAMERA_ARRAY_COLOR_ORDER,
    CAMERA_CONTRAST,
    CAMERA_EXPOSURE_VALUE,
    CAMERA_INDEX_ENVIRONMENT,
    CAMERA_PIXEL_FORMATS,
    CAMERA_PROFILES,
    CAMERA_ROTATION_DEGREES,
    CAMERA_SATURATION,
    CAMERA_SHARPNESS,
    DEFAULT_CAMERA_INDICES,
    DISPLAY_MODES,
    DISPLAY_MODE_GREEN,
    DISPLAY_MODE_LINE,
    DISPLAY_MODE_REAL,
    DOWNWARD_REFERENCE_FRAME_HEIGHT,
    FAR_TRUST_MIN_CONFIDENCE,
    FAR_TRUST_MIN_THICKNESS_PX,
    FRAME_PATH,
    FUSION_EXTREME_PIVOT_GUARD_ENTER_ERROR_DEG,
    FUSION_EXTREME_PIVOT_GUARD_RELEASE_ERROR_DEG,
    FUSION_FULL_NORMAL_STEERING_DEG,
    FUSION_FULL_PIVOT_STEERING_DEG,
    FUSION_LIGHT_REVERSE_END_DEG,
    FUSION_MIN_FORWARD_POWER,
    FUSION_MIN_FORWARD_SPEED_SCALE,
    FUSION_NEAR_ZERO_END_DEG,
    FUSION_POWER_CURVE,
    FUSION_STEERING_DEADBAND_DEG,
    FUSION_STRONG_CORRECTION_ERROR_DEG,
    FUSION_STRONG_POSITIVE_END_DEG,
    FUSION_STYLE_NEAR_BAND_HEIGHT_RATIO,
    FUSION_STYLE_TARGET_BAND_HEIGHT_RATIO,
    FUSION_STYLE_TARGET_SEARCH_HEIGHT_RATIO,
    FUSION_STYLE_TRANSVERSE_WIDTH_STABILITY_RATIO,
    FUSION_TARGET_CONSISTENCY_ANGLE_SCALE_DEG,
    FUSION_TARGET_CONSISTENCY_SHIFT_WIDTH_RATIO,
    FUSION_TARGET_ESTABLISHED_LENGTH_RATIO,
    FUSION_TARGET_HISTORY_MAX_MISSED_FRAMES,
    FUSION_TARGET_SHORT_LENGTH_RATIO,
    FUSION_TARGET_STABLE_ANGLE_DELTA_DEG,
    FUSION_TARGET_STABLE_FRAMES_FOR_FULL_SPEED,
    FUSION_TARGET_STABLE_SHIFT_WIDTH_RATIO,
    GAP_NEAR_HISTORY_FRAMES,
    GEOMETRIC_GAP_CONFIRM_OFFSET_PX,
    GEOMETRIC_GAP_FORWARD_MAX_FRAMES,
    GEOMETRIC_GAP_MAX_CENTER_SHIFT_PX,
    GEOMETRIC_GAP_MAX_SEARCH_PX,
    GEOMETRIC_GAP_PROJECTION_POINTS,
    GEOMETRIC_GAP_REACQUIRE_FRAMES,
    GEOMETRIC_GAP_SEARCH_STEP_PX,
    GEOMETRIC_PATH_BAND_HALF_HEIGHT,
    GEOMETRIC_PATH_FAR_Y_RATIO,
    GEOMETRIC_PATH_LOCAL_HEADING_POINTS,
    GEOMETRIC_PATH_NEAR_Y_RATIO,
    GEOMETRIC_TRACE_EXIT_MARGIN_PX,
    GEOMETRIC_TRACE_MAX_BACKTRACK_Y_PX,
    GEOMETRIC_TRACE_MAX_POINTS,
    GEOMETRIC_TRACE_MAX_TURN_DEG,
    GEOMETRIC_TRACE_SEARCH_RANGE_PX,
    GEOMETRIC_TRACE_STEP_PX,
    GEOMETRIC_TRACE_TARGET_PROGRESS_GAIN,
    GPIO,
    GREEN_ASPECT_RATIO_MAX,
    GREEN_ASPECT_RATIO_MIN,
    GREEN_CANDIDATE_HOLD_MAX_FRAMES,
    GREEN_CAPTURE_CANDIDATES_PATH,
    GREEN_CAPTURE_FINAL_MASK_PATH,
    GREEN_CAPTURE_HSV_MASK_PATH,
    GREEN_CAPTURE_RAW_PATH,
    GREEN_CAPTURE_REQUEST_PATH,
    GREEN_CAPTURE_STATS_PATH,
    GREEN_CLEAR_HYSTERESIS_FRAMES,
    GREEN_CLOSE_ITERATIONS,
    GREEN_CLOSE_KERNEL_SIZE,
    GREEN_CONFIRMATION_FRAMES,
    GREEN_DIRECTION_RETENTION_SECONDS,
    GREEN_FRAGMENT_MERGE_DISTANCE_PX,
    GREEN_FUSION_TARGET_HOLD_MAX_FRAMES,
    GREEN_HUE_MAX,
    GREEN_HUE_MIN,
    GREEN_INTERPRETATIONS,
    GREEN_MANEUVER_TIMEOUT_FRAMES,
    GREEN_MIN_ACTIVE_FRAMES_BEFORE_COMPLETION,
    GREEN_MIN_AREA_PX,
    GREEN_MIN_AREA_RATIO,
    GREEN_MIN_DIMENSION_PX,
    GREEN_MIN_EXTENT,
    GREEN_OBSERVATION_STATES,
    GREEN_OPEN_ITERATIONS,
    GREEN_OPEN_KERNEL_SIZE,
    GREEN_PAIR_MAX_VERTICAL_DISTANCE_HEIGHTS,
    GREEN_PARTIAL_AREA_FACTOR,
    GREEN_PARTIAL_ASPECT_RATIO_MIN,
    GREEN_PARTIAL_BORDER_TOLERANCE_PX,
    GREEN_PARTIAL_DIMENSION_FACTOR,
    GREEN_PARTIAL_EXTENT_MIN,
    GREEN_PROCESSING_ENABLED,
    GREEN_ROI_HALF_SIZE_DIVISOR,
    GREEN_ROI_MIN_BLACK_RATIO,
    GREEN_ROI_MIN_VISIBLE_RATIO,
    GREEN_SIDE_ROI_MIN_BLACK_RATIO,
    GREEN_SATURATION_MIN,
    GREEN_SINGLE_OBSERVATION_FRAMES,
    GREEN_TURNAROUND_CONFIRMATION_FRAMES,
    GREEN_TRUSTED_POSITION_CENTER_LIMIT,
    GREEN_UPPER_ROI_HALF_WIDTH_SCALE,
    GREEN_VALUE_MIN,
    JPEG_QUALITY,
    LEGACY_LINE_DEBUG_ENABLED,
    LIGHT_PIN_BOARD,
    LIMIAR_CENTRALIZACAO_VERDE,
    LIMIAR_CURVA_VERDE_INICIADA,
    LINE_MIN_COMPONENT_AREA_PX,
    LINE_MIN_COMPONENT_CORE_RATIO,
    LINE_MIN_COMPONENT_THICKNESS_PX,
    LINE_STATUS_PATH,
    LINE_TRUST_ABSOLUTE_THIN_VETO_PX,
    MEDIUM_TRUST_MIN_CONFIDENCE,
    MEDIUM_TRUST_MIN_THICKNESS_PX,
    MJPEG_STREAM_FPS,
    MJPEG_STREAM_PATH,
    MJPEG_STREAM_PORT,
    NORMAL_BASE_POWER,
    NORMAL_FULL_STEERING_ERROR,
    NORMAL_INNER_MIN_POWER,
    NORMAL_MAX_POWER,
    NORMAL_TRAJECTORY_BAND_HALF_HEIGHT_PX,
    NORMAL_TRAJECTORY_MAX_MISSING_SCANLINES,
    NORMAL_TRAJECTORY_MAX_SHIFT_WIDTH_RATIO,
    NORMAL_TRAJECTORY_MAX_SLOPE_X_PER_Y,
    NORMAL_TRAJECTORY_MAX_WIDTH_FACTOR,
    NORMAL_TRAJECTORY_MIN_POINTS,
    NORMAL_TRAJECTORY_MIN_VERTICAL_COVERAGE,
    NORMAL_TRAJECTORY_MIN_WIDTH_FACTOR,
    NORMAL_TRAJECTORY_REFERENCE_HEIGHT_PX,
    NORMAL_TRAJECTORY_SCAN_STEP_PX,
    PIVOT_ENTER_THRESHOLD,
    PIVOT_EXIT_THRESHOLD,
    PIVOT_INNER_POWER,
    PIVOT_OUTER_POWER,
    PIVOT_STATE_LEFT,
    PIVOT_STATE_NONE,
    PIVOT_STATE_RIGHT,
    Picamera2,
    QUADROS_CENTRALIZADO_PARA_CONCLUIR,
    QUADROS_PARA_REARMAR_VERDE,
    SNAPSHOT_FRAME_FPS,
    SPECULAR_REPAIR_MAX_DIAMETER_PX,
    SPECULAR_REPAIR_MAX_SATURATION,
    SPECULAR_REPAIR_MIN_VALUE,
    SPECULAR_REPAIR_REFERENCE_FRAME_HEIGHT,
    STATUS_FPS,
    STATUS_PATH,
    TEMP_FRAME_PATH,
    TEMP_LINE_STATUS_PATH,
    TEMP_STATUS_PATH,
    Transform,
    VIRTUAL_BLIND_SEARCH_BACKUP_FRAMES,
    VIRTUAL_BLIND_SEARCH_BACKUP_POWER,
    VIRTUAL_BLIND_SEARCH_CONFIRMATION_FRAMES,
    VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES,
    VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES,
    VIRTUAL_CENTER_X0,
    VIRTUAL_CENTER_X1,
    VIRTUAL_FAR_BAND_Y0,
    VIRTUAL_FAR_BAND_Y1,
    VIRTUAL_FAR_CENTER_X0,
    VIRTUAL_FAR_CENTER_X1,
    VIRTUAL_FAR_LEFT_X0,
    VIRTUAL_FAR_LEFT_X1,
    VIRTUAL_FAR_RIGHT_X0,
    VIRTUAL_FAR_RIGHT_X1,
    VIRTUAL_FAR_Y0,
    VIRTUAL_FAR_Y1,
    VIRTUAL_FINE_CENTER_DEADBAND,
    VIRTUAL_FINE_CENTER_GAIN,
    VIRTUAL_FINE_CENTER_MAX_CORRECTION,
    VIRTUAL_HARD_CORNER_FAR_RECOVERY_FRAMES,
    VIRTUAL_HARD_CORNER_MAX_FRAMES,
    VIRTUAL_HARD_CORNER_MEDIUM_RECOVERY_THRESHOLD,
    VIRTUAL_HARD_CORNER_NEAR_RECOVERY_THRESHOLD,
    VIRTUAL_HARD_CORNER_RECOVERY_FRAMES,
    VIRTUAL_HEADING_FULL_SCALE_DEG,
    VIRTUAL_HEADING_GAIN,
    VIRTUAL_LEFT_X0,
    VIRTUAL_LEFT_X1,
    VIRTUAL_LINE_CONFIDENCE_AREA_WEIGHT,
    VIRTUAL_LINE_CONFIDENCE_CONSISTENCY_WEIGHT,
    VIRTUAL_LINE_CONFIDENCE_CONTINUITY_WEIGHT,
    VIRTUAL_LINE_CONFIDENCE_REFERENCE_HEIGHT_PX,
    VIRTUAL_LINE_CONFIDENCE_REFERENCE_WIDTH_PX,
    VIRTUAL_LINE_CONFIDENCE_SAMPLE_STEP_PX,
    VIRTUAL_LINE_CONFIDENCE_THICKNESS_WEIGHT,
    VIRTUAL_LINE_CONSISTENCY_MAX_RELATIVE_DISPERSION,
    VIRTUAL_LINE_EXPECTED_THICKNESS_BOTTOM_PX,
    VIRTUAL_LINE_EXPECTED_THICKNESS_TOP_PX,
    VIRTUAL_LINE_THICKNESS_CORE_PERCENTILE,
    VIRTUAL_MEDIUM_CENTER_X0,
    VIRTUAL_MEDIUM_CENTER_X1,
    VIRTUAL_MEDIUM_CRITICAL_INVALID_MAX_FRAMES,
    VIRTUAL_MEDIUM_DIRECTION_LOST_THRESHOLD,
    VIRTUAL_MEDIUM_LEFT_X0,
    VIRTUAL_MEDIUM_LEFT_X1,
    VIRTUAL_MEDIUM_RIGHT_X0,
    VIRTUAL_MEDIUM_RIGHT_X1,
    VIRTUAL_MEDIUM_SCAN_MAX_FRAMES,
    VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD,
    VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD,
    VIRTUAL_MEDIUM_SPIN_EXIT_THRESHOLD,
    VIRTUAL_MEDIUM_SPIN_POWER,
    VIRTUAL_MEDIUM_STRONG_THRESHOLD,
    VIRTUAL_MEDIUM_WING_Y0,
    VIRTUAL_MEDIUM_WING_Y1,
    VIRTUAL_MEDIUM_Y0,
    VIRTUAL_MEDIUM_Y1,
    VIRTUAL_NEAR_Y0,
    VIRTUAL_NEAR_Y1,
    VIRTUAL_REORIENT_CONFIRMATION_FRAMES,
    VIRTUAL_REORIENT_DIRECTION_THRESHOLD,
    VIRTUAL_REORIENT_RECOVERY_FRAMES,
    VIRTUAL_RIGHT_X0,
    VIRTUAL_RIGHT_X1,
    VIRTUAL_ROW_MIN_ACTIVATION,
    VIRTUAL_STATE_NORMAL,
    VIRTUAL_STATE_REORIENT_LEFT,
    VIRTUAL_STATE_REORIENT_RIGHT,
    VISIBLE_GREEN_INTERPRETATIONS,
    camera_number,
    camera_role_publishes_line_status,
    configured_camera_indices,
    environment_flag,
    log_camera_inventory,
    require_camera_assignment,
    resolve_camera_assignments,
)

from vision.camera import (
    camera_runtime_details,
    camera_transform_settings,
    rectangle_values,
    tune_camera_image,
)

from vision.stream_display import (
    CameraStreamHandler,
    ReusableThreadingHTTPServer,
    active_stream_clients,
    create_display_frame,
    display_mode_lock,
    draw_green_candidate_overlays,
    draw_green_decision_symbol,
    draw_green_rejection_details,
    draw_green_roi_overlays,
    draw_line_illumination_overlay,
    draw_line_mode_green_overlays,
    draw_silver_shadow_overlay,
    encode_frame,
    frame_condition,
    get_display_mode,
    handle_signal,
    latest_jpeg,
    latest_jpeg_sequence,
    normalize_display_mode,
    parse_camera_profile,
    publish_stream_frame,
    register_stream_client,
    running,
    selected_display_mode,
    set_display_mode,
    start_stream_server,
    stream_frame_is_due,
    unregister_stream_client,
)

from vision.line_masks import (
    cached_relative_line_threshold_lut,
    cached_structuring_element,
    component_has_min_thickness,
    create_filtered_line_mask,
    create_line_binary_mask,
    create_line_candidate_mask,
    create_structural_line_mask,
    repair_small_specular_holes,
    resolve_vision_geometry,
    scale_reference_y,
)

from vision.illumination_correction import (
    apply_line_illumination_correction,
    build_smoothed_line_illumination_reference,
    empty_line_illumination_status,
    line_illumination_data,
)

from vision.green_detection import (
    GreenObservationTracker,
    analyze_green_marker_contours,
    build_green_capture_stats,
    build_green_status,
    camera_array_rgb_channels,
    channel_percentiles,
    component_pixel_statistics,
    contour_touches_useful_border,
    create_green_mask,
    create_green_mask_stages,
    describe_green_contour,
    empty_green_status,
    expanded_boxes_overlap,
    find_green_candidates,
    frame_to_hsv,
    green_geometry_is_valid,
    green_geometry_rejection_reasons,
    green_marker_roi_geometry,
    green_minimum_area,
    green_observation_state,
    group_fragment_boxes,
    is_hsv_green,
    json_safe_camera_metadata,
    mask_active_percent,
    measure_black_roi,
    measure_horizontal_black_roi,
    merge_green_fragments,
    rgb_pixel_to_camera_array,
    save_green_capture,
)

from vision.numeric import (
    finite_virtual_position,
    scaled_odd_kernel_size,
    scaled_reference_pixels,
)

from vision.virtual_sensors import (
    apply_virtual_fine_center_deadband,
    cached_virtual_sensor_geometry,
    calculate_green_heading_angle,
    calculate_virtual_heading_angle,
    calculate_virtual_near_fine_position,
    calculate_virtual_row_line_confidence,
    calculate_virtual_row_position,
    calculate_virtual_steering_error,
    component_labels_in_row,
    create_green_row_point,
    create_virtual_row_component_mask,
    draw_virtual_sensor_geometry,
    empty_virtual_row_line_measurement,
    expected_virtual_line_thickness_px,
    green_point_to_normalized_position,
    line_measurement_is_trusted,
    measure_component_thickness,
    measure_virtual_row_line_confidence,
    non_negative_line_measurement,
    normalized_line_confidence,
    normalized_thickness_consistency,
    read_virtual_sensor,
    resolve_virtual_sensor_geometry,
    robust_component_thickness_px,
    select_virtual_trust_row_geometry,
    virtual_fine_position_to_point,
    virtual_row_position_to_point,
    virtual_sensor_is_active,
    virtual_sensor_regions,
    virtual_sensor_trust_is_active,
)

from vision.normal_trajectory import (
    cached_normal_trajectory_envelope,
    draw_normal_trajectory_overlay,
    empty_normal_trajectory,
    extract_normal_line_trajectory,
    find_active_band_segments,
    find_normal_trajectory_segments,
    normal_trajectory_horizontal_bounds,
    normal_trajectory_scanline_ys,
    normal_trajectory_segment_telemetry,
    resolve_normal_trajectory_envelope,
    select_normal_trajectory_continuation,
)

from vision.fusion_guidance import (
    apply_fusion_extreme_pivot_direction_guard,
    apply_fusion_forward_speed_limit,
    apply_green_candidate_fusion_hold,
    calculate_fusion_control_status,
    calculate_fusion_style_angle,
    calculate_fusion_style_edge_segment_center,
    calculate_fusion_style_top_contour,
    capture_valid_fusion_command,
    create_fusion_style_physical_mask,
    create_fusion_style_selected_contour_mask,
    draw_fusion_style_line_overlay,
    empty_fusion_style_line,
    fusion_style_angle_direction,
    fusion_style_band_components,
    fusion_style_blind_search_direction,
    fusion_style_has_forward_target,
    fusion_style_has_lateral_continuation,
    fusion_style_previous_target,
    fusion_target_is_valid,
    green_direction_to_search_direction,
    green_maneuver_entry_is_confirmed,
    green_maneuver_is_geometrically_complete,
    map_fusion_angle_to_motor_powers,
    map_fusion_angle_to_steering_error,
    map_normal_steering_error,
    select_confirmed_green_direction,
    select_fusion_style_contour,
    select_fusion_style_reference_point,
    update_fusion_style_history,
    update_fusion_style_target_metrics,
    update_green_control_telemetry,
    update_green_fusion_target_hold,
    update_green_rearm_state,
)

from vision.geometric_guidance import (
    calculate_geometric_far_heading,
    estimate_geometric_initial_direction,
    extract_geometric_line_path,
    find_geometric_gap_start,
    find_geometric_lateral_exit,
    find_next_geometric_path_point,
    project_geometric_gap_to_near,
)

from vision.line_control import (
    LineFollowerController,
    VirtualLineSearchTracker,
    VirtualMediumSpinTracker,
    VirtualPivotStateTracker,
    VirtualTurnStateTracker,
    draw_line_control_overlay,
    gap_entry_is_required,
    update_gap_forward_recovery,
    update_gap_recent_near_frames,
    update_green_maneuver_state,
    virtual_medium_scan_direction,
    virtual_raw_line_is_visible,
    virtual_recovery_sensor_direction,
    virtual_reorient_direction,
)

from vision.maneuver_state import (
    LineManeuverState,
)

from vision.status_publisher import (
    LineStatusPublisher,
)

from vision.application import (
    DownwardCameraApplication,
    main,
)

from vision import camera as _camera
from vision import fusion_guidance as _fusion_guidance
from vision import geometric_guidance as _geometric_guidance
from vision import line_control as _line_control
from vision import status_publisher as _status_publisher
from vision import stream_display as _stream_display

from vision import virtual_sensors as _virtual_sensors

def calculate_line_follower_command(
    processed_line_mask,
    green_detection_result,
    direcao_verde_ativa="NENHUMA",
    gap_forward_active=False,
    virtual_turn_tracker=None,
    pivot_state_tracker=None,
    medium_spin_tracker=None,
    virtual_sensors=None,
    line_search_tracker=None,
    blind_search_requested=False,
    sensor_recovery_requested=False,
    fusion_style_line=None,
    curva_verde_iniciada=False,
    blind_search_preferred_direction=None,
    gap_fusion_reacquire_active=False,
    green_active_frames=0,
):
    _line_control.read_virtual_line_sensors = read_virtual_line_sensors
    return _line_control.calculate_line_follower_command(
        processed_line_mask, green_detection_result,
        direcao_verde_ativa, gap_forward_active, virtual_turn_tracker,
        pivot_state_tracker, medium_spin_tracker, virtual_sensors,
        line_search_tracker, blind_search_requested,
        sensor_recovery_requested, fusion_style_line,
        curva_verde_iniciada, blind_search_preferred_direction,
        gap_fusion_reacquire_active=gap_fusion_reacquire_active,
        green_active_frames=green_active_frames,
    )

def create_camera(camera_profile, camera_index):
    _camera.Picamera2 = Picamera2
    _camera.Transform = Transform
    return _camera.create_camera(camera_profile, camera_index)

def extract_fusion_style_line(
    processed_line_mask,
    previous_fusion_line=None,
    accepted_contours=None,
    preferred_direction=None,
):
    _fusion_guidance.FUSION_STYLE_TARGET_BAND_HEIGHT_RATIO = (
        FUSION_STYLE_TARGET_BAND_HEIGHT_RATIO
    )
    return _fusion_guidance.extract_fusion_style_line(
        processed_line_mask, previous_fusion_line, accepted_contours,
        preferred_direction,
    )

def extract_line_diagnostics(
    processed_line_mask,
    camera_role,
    legacy_debug_enabled=False,
    previous_fusion_line=None,
    accepted_contours=None,
    preferred_direction=None,
):
    _fusion_guidance.extract_normal_line_trajectory = (
        extract_normal_line_trajectory
    )
    return _fusion_guidance.extract_line_diagnostics(
        processed_line_mask, camera_role, legacy_debug_enabled,
        previous_fusion_line, accepted_contours, preferred_direction,
    )

def read_virtual_line_sensors(
    processed_line_mask,
    direcao_verde_ativa="NENHUMA",
    curva_verde_iniciada=False,
):
    _virtual_sensors.measure_virtual_row_line_confidence = (
        measure_virtual_row_line_confidence
    )
    return _virtual_sensors.read_virtual_line_sensors(
        processed_line_mask, direcao_verde_ativa,
        curva_verde_iniciada,
    )

def extract_gap_geometric_guidance(
    processed_line_mask,
    gap_forward_active,
    green_direction,
    near_center_visible,
    forward_control_trusted=True,
):
    _geometric_guidance.extract_geometric_line_path = (
        extract_geometric_line_path
    )
    return _geometric_guidance.extract_gap_geometric_guidance(
        processed_line_mask, gap_forward_active, green_direction,
        near_center_visible, forward_control_trusted,
    )

def save_frame(jpeg):
    _stream_display.FRAME_PATH = FRAME_PATH
    _stream_display.TEMP_FRAME_PATH = TEMP_FRAME_PATH
    return _stream_display.save_frame(jpeg)

def save_line_status(
    line_follower_command,
    line_timestamp,
    line_sequence,
    green_status,
    specular_repair_status=None,
    silver_status=None,
):
    _status_publisher.LINE_STATUS_PATH = LINE_STATUS_PATH
    _status_publisher.TEMP_LINE_STATUS_PATH = TEMP_LINE_STATUS_PATH
    return _status_publisher.save_line_status(
        line_follower_command,
        line_timestamp,
        line_sequence,
        green_status,
        specular_repair_status,
        silver_status,
    )

def save_status(
    fps,
    camera_profile,
    camera_details,
    camera_format="",
    active=True,
    error_message="",
    line_timestamp=0.0,
    line_sequence=0,
    specular_repair_status=None,
    green_status=None,
    line_timings=None,
    far_line_confidence=0.0,
    medium_line_confidence=0.0,
    far_thickness_consistency=0.0,
    medium_thickness_consistency=0.0,
    normal_trajectory=None,
    fusion_style_line=None,
    line_follower_command=None,
    silver_shadow_status=None,
):
    _status_publisher.STATUS_PATH = STATUS_PATH
    _status_publisher.TEMP_STATUS_PATH = TEMP_STATUS_PATH
    return _status_publisher.save_status(
        fps, camera_profile, camera_details, camera_format, active,
        error_message, line_timestamp, line_sequence,
        specular_repair_status, green_status, line_timings,
        far_line_confidence, medium_line_confidence,
        far_thickness_consistency, medium_thickness_consistency,
        normal_trajectory, fusion_style_line, line_follower_command,
        silver_shadow_status,
    )


if __name__ == "__main__":
    main()
