import importlib.util
import os
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np


SCRIPT_PATH = os.path.join(os.path.dirname(__file__), "camera_line_frame.py")
try:
    import cv2  # type: ignore[import]
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    sys.modules.setdefault("cv2", mock.MagicMock())
SPEC = importlib.util.spec_from_file_location("camera_line_frame", SCRIPT_PATH)
camera_line_frame = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(camera_line_frame)
if not CV2_AVAILABLE:
    # O mock pertence somente a este módulo de teste. Mantê-lo em sys.modules
    # faria outras suítes acreditarem que o OpenCV está instalado.
    sys.modules.pop("cv2", None)


def synthetic_line_axis(near=(320, 260), far=(320, 160)):
    return camera_line_frame.build_line_axis(near, far)


def synthetic_line_mask(
    left_branch=False,
    right_branch=False,
    forward_branch=False,
):
    mask = np.zeros((425, 640), dtype=np.uint8)
    mask[140:401, 310:331] = 255
    if forward_branch:
        mask[40:140, 310:331] = 255
    if left_branch:
        mask[130:151, 100:321] = 255
    if right_branch:
        mask[130:151, 320:541] = 255
    return mask


def synthetic_topology(**branches):
    return camera_line_frame.analyze_line_topology(
        synthetic_line_mask(**branches),
        synthetic_line_axis(),
        21.0,
    )


def rectangle_contour(left, top, right, bottom):
    """Cria um contorno retangular previsível para validar as ROIs verdes."""

    return np.array(
        [[[left, top]], [[right, top]], [[right, bottom]], [[left, bottom]]],
        dtype=np.int32,
    )


def synthetic_gap_mask(
    left_x=310,
    right_x=330,
    endpoint_y=130,
    continuous=False,
    branch=False,
    return_segment=False,
):
    """Cria geometrias binárias na mesma escala validada da câmera inferior."""

    mask = np.zeros((480, 640), dtype=np.uint8)
    start_y = 0 if continuous else endpoint_y
    mask[start_y:425, left_x:right_x + 1] = 255
    if branch:
        mask[endpoint_y:endpoint_y + 21, 180:461] = 255
    if return_segment:
        mask[30:80, left_x:right_x + 1] = 255
    return mask


def connected_components_for_gap_test(binary_mask, connectivity=8):
    """Substitui apenas a rotulagem do OpenCV nos testes sem o pacote instalado."""

    del connectivity
    active = binary_mask != 0
    labels = np.zeros(active.shape, dtype=np.int32)
    components = []
    next_label = 0
    height, width = active.shape
    for start_y, start_x in zip(*np.nonzero(active)):
        if labels[start_y, start_x] != 0:
            continue
        next_label += 1
        stack = [(int(start_y), int(start_x))]
        labels[start_y, start_x] = next_label
        pixels = []
        while stack:
            pixel_y, pixel_x = stack.pop()
            pixels.append((pixel_y, pixel_x))
            for delta_y in (-1, 0, 1):
                for delta_x in (-1, 0, 1):
                    if delta_y == 0 and delta_x == 0:
                        continue
                    neighbor_y = pixel_y + delta_y
                    neighbor_x = pixel_x + delta_x
                    if (
                        0 <= neighbor_y < height
                        and 0 <= neighbor_x < width
                        and active[neighbor_y, neighbor_x]
                        and labels[neighbor_y, neighbor_x] == 0
                    ):
                        labels[neighbor_y, neighbor_x] = next_label
                        stack.append((neighbor_y, neighbor_x))
        components.append(pixels)

    stats = np.zeros((next_label + 1, 5), dtype=np.int32)
    centroids = np.zeros((next_label + 1, 2), dtype=np.float64)
    for label, pixels in enumerate(components, start=1):
        y_values = np.array([pixel[0] for pixel in pixels])
        x_values = np.array([pixel[1] for pixel in pixels])
        stats[label] = (
            int(x_values.min()),
            int(y_values.min()),
            int(x_values.max() - x_values.min() + 1),
            int(y_values.max() - y_values.min() + 1),
            len(pixels),
        )
        centroids[label] = (float(x_values.mean()), float(y_values.mean()))
    return next_label + 1, labels, stats, centroids


def synthetic_candidate(
    centroid,
    line_axis=None,
    partial=False,
    associated=True,
    area=625.0,
    side=None,
    aspect_ratio=1.0,
    short_side=25.0,
    extent=0.80,
):
    line_axis = line_axis or synthetic_line_axis()
    longitudinal, lateral = camera_line_frame.project_point_on_line_axis(
        centroid, line_axis
    )
    return {
        "area": float(area),
        "centroid": tuple(float(value) for value in centroid),
        "partial": partial,
        "aspect_ratio": float(aspect_ratio),
        "short_side": float(short_side),
        "extent": float(extent),
        "associated_with_line": associated,
        "local_line_width_px": 21.0,
        "longitudinal": longitudinal,
        "lateral": lateral,
        "side": side or ("DIREITA" if lateral > 0.0 else "ESQUERDA"),
    }


class CameraProfilesTest(unittest.TestCase):
    def analyze_synthetic_gap(self, mask, near_center=(320, 160)):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        geometry = camera_line_frame.resolve_vision_geometry(480, profile)
        if CV2_AVAILABLE:
            return camera_line_frame.analyze_gap_geometry(
                mask,
                geometry,
                profile,
                near_center,
            )
        with (
            mock.patch.object(
                camera_line_frame.cv2,
                "connectedComponentsWithStats",
                side_effect=connected_components_for_gap_test,
            ),
            mock.patch.object(camera_line_frame.cv2, "CC_STAT_TOP", 1),
            mock.patch.object(camera_line_frame.cv2, "CC_STAT_HEIGHT", 3),
        ):
            return camera_line_frame.analyze_gap_geometry(
                mask,
                geometry,
                profile,
                near_center,
            )

    def test_gap_detector_accepts_centered_endpoint(self):
        result = self.analyze_synthetic_gap(synthetic_gap_mask())

        self.assertTrue(result["candidate"])
        self.assertTrue(result["alignment_valid"])
        self.assertAlmostEqual(result["alignment_error"], 0.0, places=3)
        self.assertFalse(result["return_valid"])

    def test_gap_detector_accepts_endpoint_inside_near_when_far_is_empty(self):
        # Reproduz o enquadramento deslocado: a ponta está em y=155, abaixo do
        # fim da FAR (y=100), mas o segmento ainda cruza a NEAR.
        result = self.analyze_synthetic_gap(
            synthetic_gap_mask(endpoint_y=155),
            near_center=(320, 165),
        )

        self.assertTrue(result["candidate"])
        self.assertTrue(result["alignment_valid"])
        self.assertAlmostEqual(result["alignment_error"], 0.0, places=3)

    def test_gap_detector_rejects_continuous_line_curve_and_crossing(self):
        continuous = synthetic_gap_mask(continuous=True)
        curve = np.zeros((425, 640), dtype=np.uint8)
        for y in range(0, 271):
            center_x = 320 + int(round(40 * (1.0 - y / 270.0)))
            curve[y, center_x - 10:center_x + 11] = 255
        crossing = synthetic_gap_mask(branch=True)

        self.assertFalse(self.analyze_synthetic_gap(continuous)["candidate"])
        self.assertFalse(self.analyze_synthetic_gap(curve)["candidate"])
        self.assertFalse(self.analyze_synthetic_gap(crossing)["candidate"])

    def test_gap_detector_keeps_alignment_error_and_ignores_noise(self):
        mask = synthetic_gap_mask(left_x=390, right_x=410)
        mask[20:23, 40:43] = 255
        result = self.analyze_synthetic_gap(mask, near_center=(400, 160))

        self.assertTrue(result["candidate"])
        self.assertGreater(result["alignment_error"], 0.20)
        self.assertFalse(result["return_valid"])

    def test_gap_detector_finds_disconnected_return_on_projected_axis(self):
        result = self.analyze_synthetic_gap(
            synthetic_gap_mask(return_segment=True)
        )

        self.assertTrue(result["candidate"])
        self.assertTrue(result["return_valid"])
        self.assertAlmostEqual(result["return_error"], 0.0, places=3)

    def test_dual_camera_assignments_keep_cam0_down_and_cam1_forward(self):
        camera_infos = [
            {"Num": 0, "Model": "imx219", "Id": "physical-cam0"},
            {"Num": 1, "Model": "ov5647", "Id": "physical-cam1"},
        ]
        assignments = camera_line_frame.resolve_camera_assignments(
            camera_infos,
            {"down": 0, "forward": 1},
        )

        self.assertEqual(assignments["down"]["index"], 0)
        self.assertEqual(assignments["down"]["info"]["Id"], "physical-cam0")
        self.assertEqual(assignments["forward"]["index"], 1)
        self.assertEqual(assignments["forward"]["info"]["Id"], "physical-cam1")

    def test_only_cam0_keeps_down_available_and_forward_unavailable(self):
        assignments = camera_line_frame.resolve_camera_assignments(
            [{"Num": 0, "Model": "imx219"}],
            {"down": 0, "forward": 1},
        )

        self.assertTrue(assignments["down"]["available"])
        self.assertFalse(assignments["forward"]["available"])
        with self.assertRaisesRegex(RuntimeError, "forward.*índice 1"):
            camera_line_frame.require_camera_assignment(assignments, "forward")

    def test_missing_cam0_fails_line_camera_without_role_swap(self):
        assignments = camera_line_frame.resolve_camera_assignments(
            [{"Num": 1, "Model": "ov5647"}],
            {"down": 0, "forward": 1},
        )

        self.assertFalse(assignments["down"]["available"])
        self.assertTrue(assignments["forward"]["available"])
        with self.assertRaisesRegex(RuntimeError, "down.*índice 0"):
            camera_line_frame.require_camera_assignment(assignments, "down")

    def test_camera_index_out_of_range_fails_clearly(self):
        assignments = camera_line_frame.resolve_camera_assignments(
            [{"Num": 0}, {"Num": 1}],
            {"down": 4, "forward": 1},
        )

        with self.assertRaisesRegex(RuntimeError, "down.*índice 4"):
            camera_line_frame.require_camera_assignment(assignments, "down")

    def test_camera_models_never_change_configured_roles(self):
        assignments = camera_line_frame.resolve_camera_assignments(
            [
                {"Num": 0, "Model": "ov5647"},
                {"Num": 1, "Model": "imx219"},
            ],
            {"down": 0, "forward": 1},
        )

        self.assertEqual(assignments["down"]["info"]["Model"], "ov5647")
        self.assertEqual(assignments["forward"]["info"]["Model"], "imx219")

    def test_only_downward_role_can_publish_line_status(self):
        self.assertTrue(camera_line_frame.camera_role_publishes_line_status("down"))
        self.assertFalse(
            camera_line_frame.camera_role_publishes_line_status("forward")
        )

    def test_camera_index_environment_rejects_duplicate_roles(self):
        with self.assertRaisesRegex(ValueError, "mesmo índice"):
            camera_line_frame.configured_camera_indices({
                "OBR_DOWNWARD_CAMERA_INDEX": "0",
                "OBR_FORWARD_CAMERA_INDEX": "0",
            })

    def test_display_modes_preserve_raw_frame_and_vision_masks(self):
        raw_frame = np.array(
            [
                [[10, 20, 30], [40, 50, 60]],
                [[70, 80, 90], [100, 110, 120]],
                [[130, 140, 150], [160, 170, 180]],
                [[190, 200, 210], [220, 230, 240]],
            ],
            dtype=np.uint8,
        )
        line_mask = np.array([[0, 255], [255, 0]], dtype=np.uint8)
        green_mask = np.array(
            [[0, 255], [255, 0], [0, 0]],
            dtype=np.uint8,
        )
        raw_before = raw_frame.copy()
        line_before = line_mask.copy()
        green_before = green_mask.copy()

        real = camera_line_frame.create_display_frame(
            raw_frame, line_mask, green_mask, 2, "real"
        )
        line = camera_line_frame.create_display_frame(
            raw_frame, line_mask, green_mask, 2, "line"
        )

        np.testing.assert_array_equal(real, raw_before)
        self.assertFalse(np.shares_memory(real, raw_frame))
        np.testing.assert_array_equal(
            line,
            np.array(
                [
                    [[0, 0, 0], [0, 0, 0]],
                    [[0, 0, 0], [0, 0, 0]],
                    [[0, 0, 0], [255, 255, 255]],
                    [[255, 255, 255], [0, 0, 0]],
                ],
                dtype=np.uint8,
            ),
        )
        np.testing.assert_array_equal(raw_frame, raw_before)
        np.testing.assert_array_equal(line_mask, line_before)
        np.testing.assert_array_equal(green_mask, green_before)

    def test_display_mode_normalization_defaults_to_real(self):
        self.assertEqual(camera_line_frame.normalize_display_mode("REAL"), "real")
        self.assertEqual(camera_line_frame.normalize_display_mode("line"), "line")
        self.assertEqual(camera_line_frame.normalize_display_mode("green"), "real")
        self.assertEqual(camera_line_frame.normalize_display_mode("invalid"), "real")

    def test_green_status_overlay_uses_semantic_colors(self):
        status = camera_line_frame.empty_green_status()
        self.assertEqual(
            camera_line_frame.green_status_overlay(False, status),
            ("FALHA DE PROCESSAMENTO", (0, 0, 255)),
        )
        status["greenInterpretation"] = "AMBIGUO"
        self.assertEqual(
            camera_line_frame.green_status_overlay(True, status)[1],
            (0, 255, 255),
        )
        status["greenInterpretation"] = "DIREITA"
        status["greenConfirmed"] = True
        self.assertEqual(
            camera_line_frame.green_status_overlay(True, status)[1],
            (64, 255, 96),
        )

    def resolve_green_case(self, name, candidates, topology):
        """Executa e mostra o diagnóstico determinístico do resolvedor verde."""

        line_axis = synthetic_line_axis()
        result = camera_line_frame.interpret_green_candidates(
            candidates, line_axis, topology
        )
        tracker = camera_line_frame.GreenObservationTracker()
        tracker_result = None
        for sequence in (1, 2, 3):
            tracker_result = tracker.update(sequence, result["interpretation"])
        status = camera_line_frame.build_green_status(
            candidates,
            0,
            result,
            topology,
            tracker_result,
            0.0,
        )

        valid_candidates = []
        print(f"\nCASO VERDE: {name}")
        for candidate_id, candidate in enumerate(candidates, start=1):
            vote_state, reason = camera_line_frame.classify_green_candidate_vote(
                candidate, line_axis
            )
            line_reference = camera_line_frame.point_from_line_axis(
                line_axis, candidate["longitudinal"]
            )
            if vote_state == "VALIDO":
                valid_candidates.append(candidate)
            print(
                f"id={candidate_id} area={candidate['area']:.1f} "
                f"centroide={candidate['centroid']} "
                f"referência_local_linha={line_reference} "
                f"distância_lateral_assinada={candidate['lateral']:.1f} "
                f"lado={candidate['side']} estado={vote_state} motivo={reason}"
            )

        left_candidates = [
            candidate for candidate in valid_candidates
            if candidate["side"] == "ESQUERDA"
        ]
        right_candidates = [
            candidate for candidate in valid_candidates
            if candidate["side"] == "DIREITA"
        ]
        print(
            f"quantidade LEFT={len(left_candidates)} "
            f"quantidade RIGHT={len(right_candidates)} "
            f"área total LEFT={sum(c['area'] for c in left_candidates):.1f} "
            f"área total RIGHT={sum(c['area'] for c in right_candidates):.1f} "
            f"decisão bruta={result['interpretation']} "
            f"decisão confirmada={tracker_result[0]} "
            f"confirmado={tracker_result[1]}"
        )
        return result, tracker_result, status

    def test_experimental_green_flags_accept_only_explicit_boolean_values(self):
        with mock.patch.dict(os.environ, {"TEST_GREEN_FLAG": "0"}):
            self.assertFalse(camera_line_frame.environment_flag(
                "TEST_GREEN_FLAG", True
            ))
        with mock.patch.dict(os.environ, {"TEST_GREEN_FLAG": "on"}):
            self.assertTrue(camera_line_frame.environment_flag(
                "TEST_GREEN_FLAG", False
            ))
        with mock.patch.dict(os.environ, {"TEST_GREEN_FLAG": "maybe"}):
            with self.assertRaises(ValueError):
                camera_line_frame.environment_flag("TEST_GREEN_FLAG", True)

    def test_regression_profiler_rate_uses_new_monotonic_events(self):
        self.assertEqual(
            camera_line_frame.VisionRegressionProfiler.rate(None, 10.0),
            0.0,
        )
        self.assertAlmostEqual(
            camera_line_frame.VisionRegressionProfiler.rate(10.0, 10.04),
            25.0,
        )

    def test_regression_profiler_leaves_300_sample_completion_to_cpp(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            request_path = os.path.join(temporary_directory, "request")
            sample_path = os.path.join(temporary_directory, "sample.csv")
            temporary_sample_path = os.path.join(
                temporary_directory,
                "sample.tmp.csv",
            )
            open(request_path, "w", encoding="utf-8").close()
            with mock.patch.object(
                camera_line_frame,
                "LINE_TRACE_REQUEST_PATH",
                request_path,
            ), mock.patch.object(
                camera_line_frame,
                "VISION_TRACE_SAMPLE_PATH",
                sample_path,
            ), mock.patch.object(
                camera_line_frame,
                "TEMP_VISION_TRACE_SAMPLE_PATH",
                temporary_sample_path,
            ):
                profiler = camera_line_frame.VisionRegressionProfiler()
                self.assertTrue(profiler.begin_frame(0.0))
                profiler.sample_count = 299
                profiler.publish_sample(300, 10.0, {}, "SEM_DECISAO", False, 10.0)
                self.assertTrue(os.path.isfile(request_path))
                profiler.publish_sample(301, 61.0, {}, "SEM_DECISAO", False, 61.0)
                self.assertFalse(os.path.exists(request_path))

    def test_green_experimental_modes_preserve_a_and_isolate_b_and_c(self):
        self.assertEqual(
            camera_line_frame.resolve_green_experiment_mode(True, True, True),
            ("A", True, True),
        )
        self.assertEqual(
            camera_line_frame.resolve_green_experiment_mode(True, True, False),
            ("B", True, False),
        )
        self.assertEqual(
            camera_line_frame.resolve_green_experiment_mode(True, False, True),
            ("C", False, False),
        )

    def assert_control_preview(
        self,
        profile_role,
        near_valid,
        near_error,
        far_valid,
        far_error,
        expected,
    ):
        vision_profile = camera_line_frame.CAMERA_PROFILES[profile_role]["vision"]
        actual = camera_line_frame.calculate_control_preview(
            vision_profile,
            near_valid,
            near_error,
            far_valid,
            far_error,
        )

        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=6)

    def test_forward_profile_preserves_original_capture(self):
        profile = camera_line_frame.CAMERA_PROFILES["forward"]

        self.assertEqual(profile["main_size"], (960, 540))
        self.assertEqual(profile["sensor_size"], (1920, 1080))
        self.assertEqual(profile["sensor_bit_depth"], 10)
        self.assertEqual(profile["target_fps"], 30)

    def test_down_profile_uses_full_fov_sensor_mode(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]

        self.assertEqual(profile["main_size"], (640, 480))
        self.assertEqual(profile["sensor_size"], (1640, 1232))
        self.assertEqual(profile["sensor_bit_depth"], 10)
        self.assertEqual(profile["target_fps"], 30)

    def test_down_rois_move_to_top_without_changing_dimensions(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        geometry = camera_line_frame.resolve_vision_geometry(480, profile)

        self.assertEqual(
            (
                geometry["far_band_start_y"],
                geometry["far_band_end_y"],
                geometry["near_band_start_y"],
                geometry["near_band_end_y"],
            ),
            (0, 100, 130, 190),
        )
        self.assertEqual(
            geometry["far_band_end_y"] - geometry["far_band_start_y"],
            100,
        )
        self.assertEqual(
            geometry["near_band_end_y"] - geometry["near_band_start_y"],
            60,
        )
        self.assertEqual(
            geometry["near_band_start_y"] - geometry["far_band_end_y"],
            30,
        )

    def test_argument_has_priority_over_environment(self):
        with mock.patch.dict(os.environ, {"OBR_CAMERA_ROLE": "forward"}):
            profile = camera_line_frame.parse_camera_profile(
                ["--camera-role", "down"]
            )

        self.assertEqual(profile["role"], "down")

    def test_environment_selects_profile_without_argument(self):
        with mock.patch.dict(os.environ, {"OBR_CAMERA_ROLE": "down"}):
            profile = camera_line_frame.parse_camera_profile([])

        self.assertEqual(profile["role"], "down")

    def test_scaler_crop_tuple_is_serialized_for_status(self):
        values = camera_line_frame.rectangle_values((0, 2, 3280, 2460))

        self.assertEqual(
            values,
            {"x": 0, "y": 2, "width": 3280, "height": 2460},
        )

    def test_down_guidance_on_centered_straight_line(self):
        self.assert_control_preview(
            "down", True, 0.0, True, 0.0,
            (0.0, 0.0, 0.0, 0.66, 0.66),
        )

    def test_down_guidance_on_parallel_offset_straight_line(self):
        self.assert_control_preview(
            "down", True, 0.30, True, 0.30,
            (0.30, 2.0 / 9.0, 1.0 / 15.0, 0.67, 0.65),
        )

    def test_down_guidance_anticipates_left_curve(self):
        self.assert_control_preview(
            "down", True, 0.036, True, -0.356,
            (-0.3168, -0.2408888889, -0.0722666667, 0.65, 0.67),
        )

    def test_down_guidance_anticipates_right_curve(self):
        self.assert_control_preview(
            "down", True, -0.036, True, 0.356,
            (0.3168, 0.2408888889, 0.0722666667, 0.67, 0.65),
        )

    def test_down_guidance_preserves_near_when_far_is_invalid(self):
        self.assert_control_preview(
            "down", True, -0.30, False, 0.0,
            (-0.30, -2.0 / 9.0, -1.0 / 15.0, 0.65, 0.67),
        )

    def test_down_guidance_stays_zero_when_near_is_invalid(self):
        self.assert_control_preview(
            "down", False, 0.0, True, -0.80,
            (0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def test_down_guidance_clamps_combined_error(self):
        for near_error, far_error, expected_guidance in (
            (1.0, 3.0, 1.0),
            (-1.0, -3.0, -1.0),
        ):
            with self.subTest(expected_guidance=expected_guidance):
                result = camera_line_frame.calculate_control_preview(
                    camera_line_frame.CAMERA_PROFILES["down"]["vision"],
                    True,
                    near_error,
                    True,
                    far_error,
                )
                self.assertEqual(result[0], expected_guidance)
                self.assertGreaterEqual(result[1], -1.0)
                self.assertLessEqual(result[1], 1.0)

    def test_down_balanced_differential_mixer(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        cases = (
            (0.0, 0.0, 0.660, 0.660),
            (0.28, 0.06, 0.670, 0.650),
            (-0.28, -0.06, 0.650, 0.670),
            (0.55, 0.15, 0.670, 0.650),
            (-0.55, -0.15, 0.650, 0.670),
        )

        self.assertTrue(vision_profile["balanced_differential_mixing"])
        self.assertEqual(vision_profile["base_speed_preview"], 0.66)
        self.assertEqual(vision_profile["minimum_tracking_power"], 0.65)
        for guidance_error, correction, left, right in cases:
            with self.subTest(guidance_error=guidance_error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, guidance_error, False, 0.0
                )
                self.assertAlmostEqual(result[2], correction, places=6)
                self.assertAlmostEqual(result[3], left, places=6)
                self.assertAlmostEqual(result[4], right, places=6)
                self.assertAlmostEqual((result[3] + result[4]) / 2.0, 0.66)
                self.assertGreaterEqual(result[3], 0.65)
                self.assertGreaterEqual(result[4], 0.65)
                self.assertLessEqual(result[3], 1.0)
                self.assertLessEqual(result[4], 1.0)

    def test_forward_profile_ignores_far_for_control(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["forward"]["vision"]
        with_far = camera_line_frame.calculate_control_preview(
            vision_profile, True, 0.036, True, -0.356
        )
        without_far = camera_line_frame.calculate_control_preview(
            vision_profile, True, 0.036, False, 0.0
        )

        self.assertEqual(with_far, without_far)
        self.assertEqual(with_far, (0.036, 0.0, 0.0, 0.65, 0.65))

    def test_forward_profile_preserves_one_sided_mixer(self):
        self.assert_control_preview(
            "forward", True, 0.28, False, 0.0,
            (0.28, 0.20, 0.06, 0.71, 0.65),
        )

    def test_down_deadzone_uses_ten_percent(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]

        self.assertEqual(vision_profile["near_deadzone_ratio"], 0.10)
        for error in (0.08, -0.08):
            with self.subTest(error=error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, error, False, 0.0
                )
                self.assertEqual(result[1], 0.0)
                self.assertEqual(result[3:], (0.66, 0.66))
        for error in (0.11, -0.11):
            with self.subTest(error=error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, error, False, 0.0
                )
                self.assertNotEqual(result[1], 0.0)
                self.assertEqual(result[1] > 0.0, error > 0.0)

    def test_forward_deadzone_remains_ten_percent(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["forward"]["vision"]

        self.assertEqual(vision_profile["near_deadzone_ratio"], 0.10)
        for error in (0.08, -0.08):
            with self.subTest(error=error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, error, False, 0.0
                )
                self.assertEqual(result[1], 0.0)

    def test_profile_deadzone_pixel_limits_match_overlay_geometry(self):
        down_geometry = camera_line_frame.resolve_horizontal_deadzone(
            640,
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
        )
        forward_geometry = camera_line_frame.resolve_horizontal_deadzone(
            960,
            camera_line_frame.CAMERA_PROFILES["forward"]["vision"],
        )

        self.assertEqual(down_geometry, (32, 288, 352))
        self.assertEqual(forward_geometry, (48, 432, 528))

    def test_green_rgb888_array_uses_bgr_before_hsv(self):
        green_pixel = camera_line_frame.rgb_pixel_to_camera_array(0, 255, 0)
        red_pixel = camera_line_frame.rgb_pixel_to_camera_array(255, 0, 0)
        synthetic_frame = np.array([[green_pixel, red_pixel]], dtype=np.uint8)
        converted = object()

        with mock.patch.object(
            camera_line_frame.cv2,
            "cvtColor",
            return_value=converted,
        ) as convert:
            result = camera_line_frame.frame_to_hsv(synthetic_frame)

        self.assertIs(result, converted)
        self.assertEqual(green_pixel, (0, 255, 0))
        self.assertEqual(red_pixel, (0, 0, 255))
        convert.assert_called_once_with(
            synthetic_frame,
            camera_line_frame.cv2.COLOR_BGR2HSV,
        )

    def test_green_hsv_rejects_white_black_red_and_yellow(self):
        self.assertTrue(camera_line_frame.is_hsv_green(60, 255, 255))
        for name, hsv in (
            ("white", (0, 0, 255)),
            ("black", (0, 0, 0)),
            ("red", (0, 255, 255)),
            ("yellow", (30, 255, 255)),
        ):
            with self.subTest(color=name):
                self.assertFalse(camera_line_frame.is_hsv_green(*hsv))

    def test_green_capture_names_bgr_array_channels_as_rgb(self):
        frame = np.array([[[10, 80, 200], [30, 90, 210]]], dtype=np.uint8)

        red, green, blue = camera_line_frame.camera_array_rgb_channels(frame)

        self.assertEqual(red.tolist(), [[200, 210]])
        self.assertEqual(green.tolist(), [[80, 90]])
        self.assertEqual(blue.tolist(), [[10, 30]])
        dominance = green.astype(np.int16) - np.maximum(red, blue).astype(np.int16)
        self.assertEqual(dominance.tolist(), [[-120, -120]])

    def test_green_capture_lists_geometry_rejection_reasons(self):
        candidate = {
            "partial": False,
            "area": 20.0,
            "short_side": 3.0,
            "aspect_ratio": 0.10,
            "extent": 0.20,
        }

        reasons = camera_line_frame.green_geometry_rejection_reasons(candidate)

        self.assertEqual(reasons, [
            "area_below_minimum",
            "dimension_below_minimum",
            "aspect_ratio_outside_range",
            "extent_below_minimum",
        ])

    def test_green_capture_explains_no_line_ambiguity(self):
        candidate = synthetic_candidate((270, 200))
        invalid_axis = camera_line_frame.build_line_axis(None, None)
        topology = {"junction_valid": False, "confidence": 0.0}
        interpretation = camera_line_frame.interpret_green_candidates(
            [candidate], invalid_axis, topology
        )

        reasons = camera_line_frame.green_ambiguity_reasons(
            [candidate], invalid_axis, topology, interpretation
        )

        self.assertEqual(interpretation["interpretation"], "AMBIGUO")
        self.assertEqual(reasons, ["eixo local da linha inválido"])

    def test_green_capture_metadata_keeps_requested_keys_and_finite_values(self):
        metadata = camera_line_frame.json_safe_camera_metadata({
            "ExposureTime": 12000,
            "AnalogueGain": 1.5,
            "ColourGains": (1.2, 1.4),
            "ColourTemperature": float("nan"),
            "AwbEnable": True,
            "AeEnable": False,
        })

        self.assertEqual(metadata["ExposureTime"], 12000)
        self.assertEqual(metadata["ColourGains"], [1.2, 1.4])
        self.assertIsNone(metadata["ColourTemperature"])
        self.assertTrue(metadata["AwbEnable"])
        self.assertFalse(metadata["AeEnable"])

    def test_green_small_spot_is_rejected_as_noise(self):
        self.assertFalse(camera_line_frame.green_geometry_is_valid(
            area=12.0,
            short_side=4.0,
            aspect_ratio=0.90,
            extent=0.80,
            partial=False,
        ))

    def test_green_square_is_geometrically_accepted(self):
        self.assertTrue(camera_line_frame.green_geometry_is_valid(
            area=576.0,
            short_side=24.0,
            aspect_ratio=1.0,
            extent=0.92,
            partial=False,
        ))
        if CV2_AVAILABLE:
            frame = np.full((480, 640, 3), 255, dtype=np.uint8)
            frame[190:340, 160:310] = (0, 255, 0)
            line_mask = synthetic_line_mask(left_branch=True)
            _, candidates, rejected = camera_line_frame.find_green_candidates(
                frame,
                425,
                line_mask,
                synthetic_line_axis(),
            )
            self.assertEqual(len(candidates), 1)
            self.assertEqual(len(rejected), 0)

    def test_green_pipeline_does_not_modify_frame_or_line_mask(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        frame[190:340, 160:310] = (0, 255, 0)
        line_mask = synthetic_line_mask(left_branch=True)
        original_frame = frame.copy()
        original_line_mask = line_mask.copy()

        camera_line_frame.find_green_candidates(
            frame,
            425,
            line_mask,
            synthetic_line_axis(),
        )

        self.assertTrue(np.array_equal(frame, original_frame))
        self.assertTrue(np.array_equal(line_mask, original_line_mask))

    def test_green_spatially_rejected_component_leaves_candidate_list(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        frame[190:340, 10:160] = (0, 255, 0)
        line_mask = synthetic_line_mask(right_branch=True)

        _, candidates, rejected = camera_line_frame.find_green_candidates(
            frame,
            425,
            line_mask,
            synthetic_line_axis(),
        )

        self.assertEqual(len(candidates), 0)
        self.assertEqual(len(rejected), 1)

    def test_green_small_partial_marker_stays_without_confirmation(self):
        self.assertTrue(camera_line_frame.green_geometry_is_valid(
            area=40.0,
            short_side=4.0,
            aspect_ratio=0.50,
            extent=0.30,
            partial=True,
        ))
        candidate = synthetic_candidate(
            (270, 200),
            partial=True,
            area=40.0,
            short_side=4.0,
            extent=0.30,
        )
        result = camera_line_frame.interpret_green_candidates(
            [candidate], synthetic_line_axis(), synthetic_topology(left_branch=True)
        )
        tracker = camera_line_frame.GreenObservationTracker()
        tracker_result = None
        for sequence in (1, 2, 3):
            tracker_result = tracker.update(sequence, result["interpretation"])
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(tracker_result[1])

    def test_green_fragmented_square_is_counted_once(self):
        groups = camera_line_frame.group_fragment_boxes((
            (100, 100, 10, 20),
            (117, 101, 10, 19),
        ))
        self.assertEqual(len(groups), 1)
        self.assertEqual(sorted(groups[0]), [0, 1])

    def test_green_left_before_left_branch(self):
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate((270, 200))],
            synthetic_line_axis(),
            synthetic_topology(left_branch=True),
        )
        self.assertEqual(result["interpretation"], "ESQUERDA")
        self.assertTrue(result["left_seen"])

    def test_green_right_before_right_branch(self):
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate((370, 200))],
            synthetic_line_axis(),
            synthetic_topology(right_branch=True),
        )
        self.assertEqual(result["interpretation"], "DIREITA")
        self.assertTrue(result["right_seen"])

    def test_green_near_band_detects_only_candidates_inside_blue_roi(self):
        candidates = [
            synthetic_candidate((270, 209)),
            synthetic_candidate((370, 230)),
            synthetic_candidate((320, 270)),
        ]

        self.assertTrue(camera_line_frame.green_seen_in_vertical_band(
            candidates, 210, 270, synthetic_line_axis()
        ))
        self.assertFalse(camera_line_frame.green_seen_in_vertical_band(
            [candidates[0], candidates[2]], 210, 270, synthetic_line_axis()
        ))

    def test_green_near_band_accepts_large_partial_marker(self):
        candidate = synthetic_candidate((270, 230), partial=True)

        self.assertTrue(camera_line_frame.green_seen_in_vertical_band(
            [candidate], 210, 270, synthetic_line_axis()
        ))

    def test_green_near_band_rejects_unassociated_candidate(self):
        candidate = synthetic_candidate((370, 230), associated=False)

        self.assertFalse(camera_line_frame.green_seen_in_vertical_band(
            [candidate], 210, 270, synthetic_line_axis()
        ))

    def test_green_near_band_accepts_rectangle(self):
        candidate = synthetic_candidate((270, 230))
        candidate["aspect_ratio"] = 0.55

        self.assertTrue(camera_line_frame.green_seen_in_vertical_band(
            [candidate], 210, 270, synthetic_line_axis()
        ))

    def test_green_near_band_accepts_square_with_perspective(self):
        candidate = synthetic_candidate((270, 230))
        candidate["aspect_ratio"] = 0.72

        self.assertTrue(camera_line_frame.green_seen_in_vertical_band(
            [candidate], 210, 270, synthetic_line_axis()
        ))

    def test_green_reference_area_scales_to_useful_resolution(self):
        self.assertEqual(
            camera_line_frame.green_minimum_area(640, 425),
            17000.0,
        )

    def test_green_roi_coordinates_follow_rotated_box_midpoints(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")
        geometry = camera_line_frame.green_marker_roi_geometry(
            rectangle_contour(200, 250, 300, 350),
            640,
        )

        self.assertEqual(geometry["upper_roi"], (218, 186, 282, 250))
        self.assertEqual(geometry["left_roi"], (136, 268, 200, 332))
        self.assertEqual(geometry["right_roi"], (300, 268, 364, 332))

    def test_green_roi_requires_twenty_five_percent_black(self):
        mask = np.zeros((480, 640), dtype=np.uint8)
        roi = (100, 100, 164, 164)
        mask[100:164, 100:116] = 255
        self.assertTrue(camera_line_frame.measure_black_roi(mask, roi)["valid"])

        mask[:, :] = 0
        mask[100:164, 100:115] = 255
        self.assertFalse(camera_line_frame.measure_black_roi(mask, roi)["valid"])

    def test_green_roi_requires_half_of_nominal_area_inside_image(self):
        mask = np.full((480, 640), 255, dtype=np.uint8)
        self.assertTrue(camera_line_frame.measure_black_roi(
            mask, (-32, 100, 32, 164)
        )["valid"])
        self.assertFalse(camera_line_frame.measure_black_roi(
            mask, (-33, 100, 31, 164)
        )["valid"])

    def green_action_mask(self, contour, upper=True, left=False, right=False):
        mask = np.zeros((480, 640), dtype=np.uint8)
        geometry = camera_line_frame.green_marker_roi_geometry(contour, 640)
        for enabled, name in (
            (upper, "upper_roi"),
            (left, "left_roi"),
            (right, "right_roi"),
        ):
            if enabled:
                x1, y1, x2, y2 = geometry[name]
                mask[max(0, y1):min(mask.shape[0], y2),
                     max(0, x1):min(mask.shape[1], x2)] = 255
        return mask

    def test_green_marker_action_precedence(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")
        close = rectangle_contour(200, 250, 300, 350)
        distant = rectangle_contour(200, 100, 300, 200)

        self.assertEqual(
            camera_line_frame.detect_green_marker_action(
                [close], self.green_action_mask(close, upper=False, left=True)
            ),
            "SEGUIR_LINHA",
        )
        self.assertEqual(
            camera_line_frame.detect_green_marker_action(
                [distant], self.green_action_mask(distant)
            ),
            "APROXIMAR",
        )
        self.assertEqual(
            camera_line_frame.detect_green_marker_action(
                [close], self.green_action_mask(close, left=True)
            ),
            "VIRAR_DIREITA",
        )
        self.assertEqual(
            camera_line_frame.detect_green_marker_action(
                [close], self.green_action_mask(close, right=True)
            ),
            "VIRAR_ESQUERDA",
        )

    def test_two_upper_valid_markers_request_turnaround(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")
        first = rectangle_contour(100, 250, 180, 330)
        second = rectangle_contour(420, 250, 500, 330)
        mask = self.green_action_mask(first)
        mask |= self.green_action_mask(second)

        self.assertEqual(
            camera_line_frame.detect_green_marker_action(
                [first, second], mask
            ),
            "FAZER_180",
        )

    def test_left_roi_has_priority_when_both_laterals_are_black(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")
        contour = rectangle_contour(200, 250, 300, 350)
        mask = self.green_action_mask(contour, left=True, right=True)

        self.assertEqual(
            camera_line_frame.detect_green_marker_action([contour], mask),
            "VIRAR_DIREITA",
        )

    def test_near_connected_mask_discards_other_black_components(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")
        line_mask = np.zeros((480, 640), dtype=np.uint8)
        line_mask[20:190, 310:331] = 255
        line_mask[20:190, 50:90] = 255
        near_contour = rectangle_contour(310, 0, 330, 59)
        green_mask = np.zeros((425, 640), dtype=np.uint8)
        original_line = line_mask.copy()
        original_green = green_mask.copy()

        selected = camera_line_frame.select_near_connected_black_mask(
            line_mask, green_mask, near_contour, 130
        )

        self.assertGreater(np.count_nonzero(selected[:, 310:331]), 0)
        self.assertEqual(np.count_nonzero(selected[:, 50:90]), 0)
        self.assertTrue(np.array_equal(line_mask, original_line))
        self.assertTrue(np.array_equal(green_mask, original_green))

    def test_green_guidance_moves_target_to_requested_extreme(self):
        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[20:180, 100:121] = 255
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]

        left = camera_line_frame.calculate_green_guidance_preview(
            mask, "ESQUERDA", profile
        )

        self.assertGreater(left[0], 0.0)
        self.assertGreater(left[3], left[4])

        mask[:, :] = 0
        mask[20:180, 520:541] = 255
        right = camera_line_frame.calculate_green_guidance_preview(
            mask, "DIREITA", profile
        )
        self.assertLess(right[0], 0.0)
        self.assertLess(right[3], right[4])

    def test_green_approach_scales_both_motor_previews(self):
        scaled = camera_line_frame.scale_green_approach_preview(
            (0.2, 0.1, 0.03, 0.70, 0.56)
        )

        self.assertAlmostEqual(scaled[2], 0.03 * 50.0 / 70.0)
        self.assertAlmostEqual(scaled[3], 0.50)
        self.assertAlmostEqual(scaled[4], 0.40)

    def test_green_direction_is_retained_for_half_second(self):
        tracker = camera_line_frame.GreenObservationTracker()
        tracker.update(1, "ESQUERDA", 10.0)
        tracker.update(2, "ESQUERDA", 10.1)
        confirmed = tracker.update(3, "ESQUERDA", 10.2)
        retained = tracker.update(4, "SEM_DECISAO", 10.69)
        expired = tracker.update(5, "SEM_DECISAO", 10.70)

        self.assertEqual(confirmed, ("ESQUERDA", True, 3))
        self.assertEqual(retained[0], "ESQUERDA")
        self.assertEqual(expired, ("SEM_DECISAO", False, 0))

    def test_actionable_right_marker_does_not_require_junction_topology(self):
        candidate = synthetic_candidate((370, 230), side="DIREITA")

        result = camera_line_frame.interpret_actionable_green_candidates(
            [candidate]
        )

        self.assertEqual(result["interpretation"], "DIREITA")

    def test_actionable_left_marker_does_not_require_junction_topology(self):
        candidate = synthetic_candidate((270, 230), side="ESQUERDA")

        result = camera_line_frame.interpret_actionable_green_candidates(
            [candidate]
        )

        self.assertEqual(result["interpretation"], "ESQUERDA")

    def test_actionable_marker_on_each_side_requests_turnaround(self):
        candidates = [
            synthetic_candidate((270, 230), side="ESQUERDA"),
            synthetic_candidate((370, 230), side="DIREITA"),
        ]

        result = camera_line_frame.interpret_actionable_green_candidates(
            candidates
        )

        self.assertEqual(result["interpretation"], "RETORNO_180")

    def test_green_near_band_rejects_candidates_without_valid_line_axis(self):
        self.assertFalse(camera_line_frame.green_seen_in_vertical_band(
            [synthetic_candidate((270, 230))],
            210,
            270,
            {"valid": False},
        ))

    def test_empty_green_status_does_not_report_green_in_near_band(self):
        self.assertFalse(
            camera_line_frame.empty_green_status()["greenNearSeen"]
        )

    def test_green_diagonal_line_preserves_left_side(self):
        axis = camera_line_frame.build_line_axis((360, 280), (280, 180))
        marker = camera_line_frame.point_from_line_axis(axis, 60.0, -40.0)
        topology = synthetic_topology(left_branch=True)
        topology["junction_longitudinal"] = 120.0
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate(marker, axis)], axis, topology
        )
        self.assertEqual(result["interpretation"], "ESQUERDA")

    def test_green_diagonal_line_preserves_right_side(self):
        axis = camera_line_frame.build_line_axis((360, 280), (280, 180))
        marker = camera_line_frame.point_from_line_axis(axis, 60.0, 40.0)
        topology = synthetic_topology(right_branch=True)
        topology["junction_longitudinal"] = 120.0
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate(marker, axis)], axis, topology
        )
        self.assertEqual(result["interpretation"], "DIREITA")

    def test_green_after_junction_is_false_for_current_direction(self):
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate((270, 100))],
            synthetic_line_axis(),
            synthetic_topology(left_branch=True),
        )
        self.assertEqual(
            result["interpretation"],
            "VERDE_FALSO_NO_SENTIDO_ATUAL",
        )

    def test_green_before_junction_without_matching_branch_is_false(self):
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate((270, 200))],
            synthetic_line_axis(),
            synthetic_topology(),
        )
        self.assertEqual(
            result["interpretation"],
            "VERDE_FALSO_NO_SENTIDO_ATUAL",
        )

    def test_green_opposite_pair_before_line_end_is_return(self):
        result = camera_line_frame.interpret_green_candidates(
            [
                synthetic_candidate((270, 200)),
                synthetic_candidate((370, 202)),
            ],
            synthetic_line_axis(),
            synthetic_topology(),
        )
        self.assertEqual(result["interpretation"], "RETORNO_180")
        self.assertTrue(result["pair_compatible"])

    def test_green_return_pair_is_never_individually_false(self):
        result = camera_line_frame.interpret_green_candidates(
            [
                synthetic_candidate((270, 200)),
                synthetic_candidate((370, 200)),
            ],
            synthetic_line_axis(),
            synthetic_topology(),
        )
        self.assertNotEqual(
            result["interpretation"],
            "VERDE_FALSO_NO_SENTIDO_ATUAL",
        )
        self.assertEqual(result["interpretation"], "RETORNO_180")

    def test_green_two_fragments_cannot_produce_return(self):
        groups = camera_line_frame.group_fragment_boxes((
            (250, 190, 12, 22),
            (268, 191, 12, 21),
        ))
        candidates = [synthetic_candidate((270, 200)) for _ in groups]
        result = camera_line_frame.interpret_green_candidates(
            candidates,
            synthetic_line_axis(),
            synthetic_topology(left_branch=True),
        )
        self.assertEqual(len(candidates), 1)
        self.assertNotEqual(result["interpretation"], "RETORNO_180")

    def test_green_directional_resolution_cases(self):
        right_topology = synthetic_topology(right_branch=True)
        left_topology = synthetic_topology(left_branch=True)
        cases = (
            (
                "1 - principal e fragmento à direita",
                [
                    synthetic_candidate((370, 200), area=20000),
                    synthetic_candidate((380, 202), area=2000),
                ],
                right_topology,
                "DIREITA",
            ),
            (
                "2 - marcador fragmentado três vezes à direita",
                [
                    synthetic_candidate((365, 198), area=7000),
                    synthetic_candidate((375, 200), area=6000),
                    synthetic_candidate((385, 202), area=5000),
                ],
                right_topology,
                "DIREITA",
            ),
            (
                "3 - dois fragmentos à esquerda",
                [
                    synthetic_candidate((270, 200), area=10000),
                    synthetic_candidate((260, 202), area=2000),
                ],
                left_topology,
                "ESQUERDA",
            ),
            (
                "4 - verde duplo",
                [
                    synthetic_candidate((270, 200), area=2000),
                    synthetic_candidate((370, 202), area=20000),
                ],
                synthetic_topology(),
                "RETORNO_180",
            ),
            (
                "5 - direita válida e ruído espacial rejeitado",
                [
                    synthetic_candidate((370, 200), area=20000),
                    synthetic_candidate((220, 200), associated=False, area=2000),
                ],
                right_topology,
                "DIREITA",
            ),
            (
                "6 - somente ruído espacial rejeitado",
                [synthetic_candidate((220, 200), associated=False, area=2000)],
                right_topology,
                "SEM_DECISAO",
            ),
            (
                "7 - candidato realmente irresolúvel",
                [synthetic_candidate((320, 200), area=20000, side="UNKNOWN")],
                right_topology,
                "AMBIGUO",
            ),
            (
                "8 - fragmentos à direita em quadros consecutivos",
                [
                    synthetic_candidate((370, 200), area=20000),
                    synthetic_candidate((380, 202), area=2000),
                ],
                right_topology,
                "DIREITA",
            ),
        )

        for name, candidates, topology, expected in cases:
            with self.subTest(name=name):
                result, tracker_result, status = self.resolve_green_case(
                    name, candidates, topology
                )
                self.assertEqual(result["interpretation"], expected)
                self.assertEqual(tracker_result[0], expected)
                self.assertEqual(status["greenInterpretation"], expected)
                if expected in ("ESQUERDA", "DIREITA", "RETORNO_180"):
                    self.assertTrue(tracker_result[1])
                else:
                    self.assertFalse(tracker_result[1])
                if expected == "SEM_DECISAO":
                    self.assertEqual(result["observation_state"], "SEM_VERDE")

        fragmented_return = camera_line_frame.interpret_green_candidates(
            [
                synthetic_candidate((270, 200), area=2000),
                synthetic_candidate((370, 200), area=10000),
                synthetic_candidate((380, 202), area=2000),
            ],
            synthetic_line_axis(),
            synthetic_topology(),
        )
        self.assertEqual(fragmented_return["interpretation"], "RETORNO_180")
        self.assertTrue(fragmented_return["pair_compatible"])

    def test_green_intersection_without_marker_has_no_decision(self):
        result = camera_line_frame.interpret_green_candidates(
            [],
            synthetic_line_axis(),
            synthetic_topology(left_branch=True, right_branch=True),
        )
        self.assertEqual(result["observation_state"], "SEM_VERDE")
        self.assertEqual(result["interpretation"], "SEM_DECISAO")

    def test_green_insufficient_topology_is_ambiguous(self):
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate((270, 200))],
            synthetic_line_axis(),
            {"junction_valid": False, "confidence": 0.0},
        )
        self.assertEqual(result["interpretation"], "AMBIGUO")

    def test_green_three_new_frames_confirm_but_repeated_frame_does_not(self):
        tracker = camera_line_frame.GreenObservationTracker()
        first = tracker.update(10, "ESQUERDA")
        repeated = tracker.update(10, "ESQUERDA")
        second = tracker.update(11, "ESQUERDA")
        third = tracker.update(12, "ESQUERDA")

        self.assertEqual(first[2], 1)
        self.assertEqual(first[0], "SEM_DECISAO")
        self.assertEqual(repeated[2], 1)
        self.assertEqual(repeated[0], "SEM_DECISAO")
        self.assertEqual(second[2], 2)
        self.assertEqual(second[0], "SEM_DECISAO")
        self.assertFalse(second[1])
        self.assertEqual(third, ("ESQUERDA", True, 3))

    def test_green_disappearance_clears_after_retention(self):
        tracker = camera_line_frame.GreenObservationTracker()
        tracker.update(1, "DIREITA", 20.0)
        tracker.update(2, "DIREITA", 20.1)
        tracker.update(3, "DIREITA", 20.2)

        first_missing = tracker.update(4, "SEM_DECISAO", 20.69)
        expired = tracker.update(5, "SEM_DECISAO", 20.70)

        self.assertEqual(first_missing[0], "DIREITA")
        self.assertTrue(first_missing[1])
        self.assertEqual(expired, ("SEM_DECISAO", False, 0))

    def test_green_changes_do_not_modify_line_preview_calculation(self):
        self.assert_control_preview(
            "down", True, 0.036, True, -0.356,
            (-0.3168, -0.2408888889, -0.0722666667, 0.65, 0.67),
        )
        self.assert_control_preview(
            "forward", True, 0.28, False, 0.0,
            (0.28, 0.20, 0.06, 0.71, 0.65),
        )

    def test_green_fast_status_contains_only_finite_numbers(self):
        candidate = synthetic_candidate((270, 200))
        candidate["centroid"] = (float("nan"), float("inf"))
        candidate["area"] = float("nan")
        interpretation = {
            "observation_state": "UM_CANDIDATO",
            "interpretation": "AMBIGUO",
            "left_seen": True,
            "right_seen": False,
            "pair_compatible": False,
        }
        status = camera_line_frame.build_green_status(
            [candidate],
            0,
            interpretation,
            {"junction_valid": False},
            ("AMBIGUO", False, 1),
            float("nan"),
        )

        for value in status.values():
            if isinstance(value, float):
                self.assertTrue(np.isfinite(value))


if __name__ == "__main__":
    unittest.main()
