import importlib.util
import json
import math
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


def synthetic_line_axis(robot_side=(320, 260), forward=(320, 160)):
    return camera_line_frame.build_line_axis(robot_side, forward)


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


def synthetic_trajectory_mask(lateral_from_forward, selected_samples=None):
    """Desenha uma centerline na escala normalizada do visual pursuit."""

    height = 425
    width = 640
    center_x = width / 2.0
    mask = np.zeros((height, width), dtype=np.uint8)
    if selected_samples is None:
        y_values = range(height)
    else:
        sample_rows = np.linspace(
            height - 1,
            0,
            camera_line_frame.TRAJECTORY_SAMPLE_COUNT,
        )
        y_values = []
        for sample_index in selected_samples:
            center_y = int(round(sample_rows[sample_index]))
            y_values.extend(range(max(0, center_y - 2), min(height, center_y + 3)))

    for pixel_y in y_values:
        forward = (height - pixel_y) / height
        lateral = float(lateral_from_forward(forward))
        pixel_x = int(round(center_x + lateral * height))
        left_x = max(0, pixel_x - 10)
        right_x = min(width, pixel_x + 11)
        mask[pixel_y, left_x:right_x] = 255
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
    def analyze_synthetic_gap(self, mask):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        geometry = camera_line_frame.resolve_vision_geometry(480, profile)
        if CV2_AVAILABLE:
            return camera_line_frame.analyze_gap_geometry(
                mask,
                geometry,
                profile,
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
            )

    def test_gap_detector_accepts_centered_endpoint(self):
        result = self.analyze_synthetic_gap(synthetic_gap_mask())

        self.assertTrue(result["candidate"])
        self.assertTrue(result["alignment_valid"])
        self.assertAlmostEqual(result["alignment_error"], 0.0, places=3)
        self.assertFalse(result["return_valid"])

    def test_gap_detector_accepts_endpoint_with_near_empty(self):
        # A ponta em y=155 fica fora da NEAR, mas o componente inferior ainda
        # deve identificar o gap sem depender da banda de controle.
        result = self.analyze_synthetic_gap(
            synthetic_gap_mask(endpoint_y=155),
        )

        self.assertTrue(result["candidate"])
        self.assertTrue(result["alignment_valid"])
        self.assertAlmostEqual(result["alignment_error"], 0.0, places=3)

    def test_gap_detector_rejects_continuous_line_curve_and_crossing(self):
        continuous = synthetic_gap_mask(continuous=True)
        curve = np.zeros((425, 640), dtype=np.uint8)
        for y in range(130, 425):
            progress = (425 - y) / 295.0
            center_x = 320 + int(round(140 * progress * progress))
            curve[y, center_x - 10:center_x + 11] = 255
        crossing = synthetic_gap_mask(branch=True)

        self.assertFalse(self.analyze_synthetic_gap(continuous)["candidate"])
        self.assertFalse(self.analyze_synthetic_gap(curve)["candidate"])
        self.assertFalse(self.analyze_synthetic_gap(crossing)["candidate"])

    def test_gap_detector_keeps_alignment_error_and_ignores_noise(self):
        mask = synthetic_gap_mask(left_x=390, right_x=410)
        mask[20:23, 40:43] = 255
        result = self.analyze_synthetic_gap(mask)

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
                    [[0, 0, 0], [0, 255, 0]],
                    [[0, 255, 0], [0, 0, 0]],
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

    def test_green_saturation_rejects_light_floor_tint(self):
        self.assertTrue(camera_line_frame.is_hsv_green(75, 255, 90))
        self.assertFalse(camera_line_frame.is_hsv_green(75, 136, 146))

    def test_camera_hides_unconfirmed_rejected_and_unresolved_green(self):
        line_axis = synthetic_line_axis()
        valid = synthetic_candidate((370, 200))
        rejected = synthetic_candidate((370, 200), associated=False)
        unresolved = synthetic_candidate((320, 200))

        self.assertEqual(
            camera_line_frame.select_visible_green_candidates(
                [valid], line_axis, "DIREITA", "SEM_DECISAO", False
            ),
            [],
        )
        visible = camera_line_frame.select_visible_green_candidates(
            [valid, rejected, unresolved],
            line_axis,
            "DIREITA",
            "DIREITA",
            True,
        )
        self.assertEqual(visible, [valid])

    @unittest.skipUnless(CV2_AVAILABLE, "OpenCV não está disponível")
    def test_strong_green_overlay_is_symbolic_without_text(self):
        line_axis = synthetic_line_axis()
        candidate = synthetic_candidate((370, 200))
        candidate["contour"] = rectangle_contour(355, 185, 385, 215)
        frame = np.zeros((360, 480, 3), dtype=np.uint8)

        strong = camera_line_frame.select_strong_green_candidates(
            [candidate], line_axis, "DIREITA"
        )
        self.assertEqual(strong, [candidate])

        # O vídeo de produção não pode pagar o custo de renderização de texto.
        with mock.patch.object(camera_line_frame.cv2, "putText") as put_text:
            camera_line_frame.draw_green_candidate_overlays(
                frame, strong, "DIREITA", False
            )
            camera_line_frame.draw_line_mode_green_overlays(
                frame, strong, "DIREITA", True
            )
        put_text.assert_not_called()

    def test_camera_hides_stale_or_ambiguous_green_decision(self):
        line_axis = synthetic_line_axis()
        candidate = synthetic_candidate((370, 200))

        for raw_interpretation, published_interpretation in (
            ("SEM_DECISAO", "DIREITA"),
            ("AMBIGUO", "AMBIGUO"),
            ("VERDE_FALSO_NO_SENTIDO_ATUAL", "VERDE_FALSO_NO_SENTIDO_ATUAL"),
        ):
            with self.subTest(
                raw=raw_interpretation,
                published=published_interpretation,
            ):
                self.assertEqual(
                    camera_line_frame.select_visible_green_candidates(
                        [candidate],
                        line_axis,
                        raw_interpretation,
                        published_interpretation,
                        True,
                    ),
                    [],
                )

    def test_green_occlusion_uses_only_recent_line_axis_and_rejects_bad_pair(self):
        line_axis = synthetic_line_axis()
        left = synthetic_candidate((270, 130), associated=False)
        for candidate in (left,):
            candidate.update({
                "long_side": 25.0,
                "geometry_valid": True,
                "frameHeight": 480,
            })

        camera_line_frame.apply_green_occlusion_axis(
            [left], line_axis, 21.0
        )
        decision = camera_line_frame.interpret_occluded_green_candidates(
            [left], line_axis, 97, 306
        )
        self.assertEqual(decision["interpretation"], "ESQUERDA")
        self.assertTrue(decision["path_black_valid"])

        right_farther = synthetic_candidate((370, 220), associated=False)
        right_farther.update({
            "long_side": 25.0,
            "geometry_valid": True,
            "frameHeight": 480,
        })
        camera_line_frame.apply_green_occlusion_axis(
            [left, right_farther], line_axis, 21.0
        )
        ambiguous = camera_line_frame.interpret_occluded_green_candidates(
            [left, right_farther], line_axis, 97, 306
        )
        self.assertEqual(ambiguous["interpretation"], "AMBIGUO")
        self.assertFalse(ambiguous["path_black_valid"])

    def test_green_decision_uses_spatial_axis_when_fixed_front_roi_is_empty(self):
        line_axis = synthetic_line_axis()
        candidate = synthetic_candidate((270, 200))
        topology = synthetic_topology(left_branch=True)

        decision = camera_line_frame.resolve_green_decision(
            [candidate],
            line_axis,
            topology,
            False,
            97,
            306,
        )

        self.assertEqual(decision["interpretation"], "ESQUERDA")
        self.assertTrue(decision["path_black_valid"])

    def test_green_occlusion_accepts_near_marker_from_recent_line_axis(self):
        line_axis = synthetic_line_axis()
        candidate = synthetic_candidate((270, 220), associated=False)
        candidate.update({
            "long_side": 25.0,
            "geometry_valid": True,
            "frameHeight": 480,
        })
        camera_line_frame.apply_green_occlusion_axis(
            [candidate], line_axis, 21.0
        )

        decision = camera_line_frame.resolve_green_decision(
            [candidate],
            line_axis,
            {"junction_valid": False, "confidence": 0.0},
            True,
            97,
            306,
        )

        self.assertEqual(decision["interpretation"], "ESQUERDA")
        self.assertTrue(decision["path_black_valid"])

    def test_green_rois_validate_marker_with_wide_structural_black_region(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        black_mask = np.zeros((319, 480), dtype=np.uint8)
        black_mask[:80, :312] = 255
        black_mask[:302, 190:312] = 255
        contour = np.array(
            [[[88, 81]], [[199, 81]], [[199, 218]], [[88, 218]]],
            dtype=np.int32,
        )

        result = camera_line_frame.analyze_green_marker_contours(
            [contour], black_mask
        )

        self.assertTrue(result["path_black_valid"])
        self.assertEqual(result["interpretation"], "ESQUERDA")

    def test_green_roi_overlay_uses_only_geometric_primitives(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        frame = np.zeros((319, 480, 3), dtype=np.uint8)
        contour = np.array(
            [[[88, 81]], [[199, 81]], [[199, 218]], [[88, 218]]],
            dtype=np.int32,
        )
        geometry = camera_line_frame.green_marker_roi_geometry(contour, 480)
        interpretation = {
            "markers": [{
                "geometry": geometry,
                "upper": {"valid": True},
                "right": {"valid": True},
            }],
        }

        with mock.patch.object(camera_line_frame.cv2, "putText") as put_text:
            camera_line_frame.draw_green_roi_overlays(
                frame, interpretation, "ESQUERDA"
            )

        put_text.assert_not_called()
        self.assertGreater(np.count_nonzero(frame), 0)

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
        expected,
        far_valid=False,
        far_error=0.0,
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

        self.assertEqual(profile["main_size"], (480, 360))
        self.assertEqual(profile["sensor_size"], (1640, 1232))
        self.assertEqual(profile["sensor_bit_depth"], 10)
        self.assertEqual(profile["target_fps"], 30)

    def test_down_profile_uses_local_contrast_after_light_removal(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]

        self.assertEqual(profile["line_background_kernel_size"], 201)
        self.assertEqual(profile["line_max_background_ratio_percent"], 68)
        self.assertEqual(profile["line_max_brightness"], 110)
        self.assertEqual(profile["open_kernel_shape"], "ellipse")
        self.assertEqual(profile["open_kernel_size"], 17)
        self.assertEqual(profile["close_kernel_size"], 7)
        self.assertEqual(profile["full_line_max_area_ratio"], 0.30)
        self.assertEqual(
            profile["line_band_fallback_max_row_width_ratio"],
            0.40,
        )
        self.assertEqual(
            profile["line_band_fallback_min_row_coverage_ratio"],
            0.25,
        )

    def test_down_profile_scales_reference_geometry_and_kernels_at_480x360(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        geometry = camera_line_frame.resolve_vision_geometry(360, profile)

        self.assertEqual(geometry["structural_end_y"], 319)
        self.assertEqual(geometry["far_band_end_y"], 72)
        self.assertEqual(geometry["near_band_start_y"], 198)
        self.assertEqual(geometry["near_band_end_y"], 317)
        self.assertEqual(
            camera_line_frame.scaled_odd_kernel_size(201, 360), 151
        )
        self.assertEqual(
            camera_line_frame.scaled_odd_kernel_size(17, 360), 13
        )
        self.assertEqual(
            camera_line_frame.scaled_odd_kernel_size(7, 360), 5
        )

    def test_down_line_mask_rejects_dim_white_and_keeps_black_tape(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        width = 640
        height = 480
        brightness_gradient = np.linspace(55, 220, width, dtype=np.uint8)
        gray_frame = np.tile(brightness_gradient, (height, 1))
        gray_frame[:, 300:341] = 20
        frame = np.repeat(gray_frame[:, :, None], 3, axis=2)

        mask, _ = camera_line_frame.create_filtered_line_mask(
            frame,
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
        )

        tape_coverage = np.count_nonzero(mask[:425, 300:341]) / (425 * 41)
        floor_pixels = np.concatenate((
            mask[:425, :280].ravel(),
            mask[:425, 361:].ravel(),
        ))
        floor_coverage = np.count_nonzero(floor_pixels) / floor_pixels.size
        self.assertGreater(tape_coverage, 0.95)
        self.assertLess(floor_coverage, 0.01)

    def test_down_line_mask_finds_tape_inside_deep_shadow(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        gray_frame = np.full((480, 640), 205, dtype=np.uint8)
        gray_frame[:, :320] = 65
        gray_frame[:, 120:161] = 35
        frame = np.repeat(gray_frame[:, :, None], 3, axis=2)

        mask, _ = camera_line_frame.create_filtered_line_mask(
            frame,
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
        )

        tape_coverage = np.count_nonzero(mask[:425, 120:161]) / (425 * 41)
        shadow_floor = np.concatenate((
            mask[:425, :100].ravel(),
            mask[:425, 181:300].ravel(),
        ))
        shadow_coverage = np.count_nonzero(shadow_floor) / shadow_floor.size
        self.assertGreater(tape_coverage, 0.95)
        self.assertLess(shadow_coverage, 0.01)

    def test_down_line_mask_removes_thin_gap_connected_to_tape(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        gray_frame = np.full((480, 640), 205, dtype=np.uint8)
        gray_frame[:, 285:356] = 20
        camera_line_frame.cv2.line(
            gray_frame,
            (50, 390),
            (590, 210),
            20,
            14,
            camera_line_frame.cv2.LINE_8,
        )
        frame = np.repeat(gray_frame[:, :, None], 3, axis=2)

        mask, _ = camera_line_frame.create_filtered_line_mask(
            frame,
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
        )

        gap_region = mask.copy()
        gap_region[:, 260:381] = 0
        tape_coverage = np.count_nonzero(mask[:425, 300:341]) / (425 * 41)
        self.assertEqual(np.count_nonzero(gap_region), 0)
        self.assertGreater(tape_coverage, 0.95)

    def test_down_line_mask_preserves_twenty_pixel_diagonal_tape(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        gray_frame = np.full((480, 640), 205, dtype=np.uint8)
        expected_tape = np.zeros((480, 640), dtype=np.uint8)
        for target, color in ((gray_frame, 20), (expected_tape, 255)):
            camera_line_frame.cv2.line(
                target,
                (180, 400),
                (450, 20),
                color,
                20,
                camera_line_frame.cv2.LINE_8,
            )
        frame = np.repeat(gray_frame[:, :, None], 3, axis=2)

        mask, _ = camera_line_frame.create_filtered_line_mask(
            frame,
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
        )

        expected_pixels = expected_tape[:425] > 0
        retained_pixels = np.count_nonzero(mask[:425][expected_pixels])
        self.assertGreater(retained_pixels / np.count_nonzero(expected_pixels), 0.95)

    def test_repairs_small_clear_hole_inside_black_tape(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        frame = np.full((360, 480, 3), 220, dtype=np.uint8)
        frame[:, 210:250] = (20, 20, 20)
        frame[180:188, 226:234] = (255, 255, 255)
        raw_mask = camera_line_frame.create_line_binary_mask(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
        )

        repaired_mask, status = camera_line_frame.repair_small_specular_holes(
            raw_mask,
            frame,
            "RGB888",
        )

        self.assertEqual(status["specularRepairComponents"], 1)
        self.assertEqual(status["specularRepairPixels"], 64)
        self.assertTrue(np.all(repaired_mask[180:188, 226:234] == 255))

    def test_repair_rejects_large_gap_and_saturated_green(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        frame = np.full((360, 480, 3), 220, dtype=np.uint8)
        frame[:, 210:250] = (20, 20, 20)
        frame[170:182, 210:250] = (255, 255, 255)
        gap_mask = camera_line_frame.create_line_binary_mask(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
        )
        repaired_gap, gap_status = camera_line_frame.repair_small_specular_holes(
            gap_mask, frame, "RGB888")
        self.assertEqual(gap_status["specularRepairPixels"], 0)
        self.assertTrue(np.all(repaired_gap[170:182, 210:250] == 0))

        green_frame = np.full((360, 480, 3), 220, dtype=np.uint8)
        green_frame[:, 210:250] = (20, 20, 20)
        green_frame[180:188, 226:234] = (0, 255, 0)
        green_mask_before = camera_line_frame.create_green_mask(
            green_frame, 319, "RGB888")
        green_line_mask = camera_line_frame.create_line_binary_mask(
            cv2.cvtColor(green_frame, cv2.COLOR_BGR2GRAY),
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
        )
        repaired_green, green_status = camera_line_frame.repair_small_specular_holes(
            green_line_mask, green_frame, "RGB888")
        green_mask_after = camera_line_frame.create_green_mask(
            green_frame, 319, "RGB888")
        self.assertEqual(green_status["specularRepairPixels"], 0)
        self.assertTrue(np.all(repaired_green[180:188, 226:234] == 0))
        self.assertTrue(np.array_equal(green_mask_before, green_mask_after))

    def test_line_candidate_mask_rejects_giant_dark_component(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        structural_mask = np.zeros((425, 640), dtype=np.uint8)
        structural_mask[:, :280] = 255
        structural_mask[:, 320:361] = 255

        candidate_mask = camera_line_frame.create_line_candidate_mask(
            structural_mask,
            profile,
        )

        self.assertEqual(np.count_nonzero(candidate_mask[:, :280]), 0)
        self.assertGreater(np.count_nonzero(candidate_mask[:, 320:361]), 0)

    def test_down_uses_far_and_near_percentages_of_roi(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        geometry = camera_line_frame.resolve_vision_geometry(480, profile)

        self.assertEqual(
            (
                geometry["far_band_start_y"],
                geometry["far_band_end_y"],
                geometry["near_band_start_y"],
                geometry["near_band_end_y"],
                geometry["green_observation_start_y"],
                geometry["green_observation_end_y"],
            ),
            (0, 96, 264, 422, 130, 190),
        )
        self.assertEqual(
            geometry["far_band_end_y"] - geometry["far_band_start_y"],
            96,
        )
        self.assertEqual(
            geometry["near_band_end_y"] - geometry["near_band_start_y"],
            158,
        )
        self.assertEqual(
            (geometry["gap_anchor_start_y"], geometry["gap_anchor_end_y"]),
            (350, 425),
        )
        self.assertEqual(
            (geometry["gap_endpoint_start_y"], geometry["gap_endpoint_end_y"]),
            (19, 180),
        )
        self.assertEqual(
            set(geometry),
            {
                "structural_end_y",
                "ignored_start_y",
                "far_band_start_y",
                "far_band_end_y",
                "near_band_start_y",
                "near_band_end_y",
                "green_observation_start_y",
                "green_observation_end_y",
                "gap_anchor_start_y",
                "gap_anchor_end_y",
                "gap_endpoint_start_y",
                "gap_endpoint_end_y",
                "pixel_scale",
            },
        )

    def test_band_percentages_are_relative_to_roi_height(self):
        geometry = camera_line_frame.resolve_vision_geometry(
            400,
            {
                "line_roi_start_ratio": 0.25,
                "far_band_start_ratio": 0.00,
                "far_band_end_ratio": 0.20,
                "near_band_start_ratio": 0.65,
                "near_band_end_ratio": 0.85,
            },
        )

        self.assertEqual(
            (
                geometry["far_band_start_y"],
                geometry["far_band_end_y"],
                geometry["near_band_start_y"],
                geometry["near_band_end_y"],
            ),
            (100, 160, 295, 355),
        )

    def test_band_center_uses_median_of_rows_against_local_branch(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[312:408, 300:321] = 255
        mask[350:354, 300:521] = 255
        observation = camera_line_frame.analyze_line_band(
            mask, 312, 408
        )

        self.assertTrue(observation["valid"])
        self.assertAlmostEqual(observation["x"], 310.0, places=1)

    def test_band_fallback_recovers_track_connected_to_giant_component(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        structural_mask = np.zeros((319, 480), dtype=np.uint8)
        structural_mask[:160, :] = 255
        structural_mask[160:, 200:280] = 255
        candidate_mask = camera_line_frame.create_line_candidate_mask(
            structural_mask,
            profile,
        )

        self.assertEqual(np.count_nonzero(candidate_mask), 0)
        observation = camera_line_frame.analyze_line_band(
            candidate_mask,
            234,
            306,
            structural_fallback_mask=structural_mask,
            fallback_max_row_width_ratio=profile[
                "line_band_fallback_max_row_width_ratio"
            ],
            fallback_min_row_coverage_ratio=profile[
                "line_band_fallback_min_row_coverage_ratio"
            ],
        )

        self.assertTrue(observation["valid"])
        self.assertEqual(observation["source"], "structural_fallback")
        self.assertAlmostEqual(observation["x"], 239.5, places=1)

    def test_band_fallback_rejects_wide_dark_region(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        structural_mask = np.full((319, 480), 255, dtype=np.uint8)
        candidate_mask = camera_line_frame.create_line_candidate_mask(
            structural_mask,
            profile,
        )
        observation = camera_line_frame.analyze_line_band(
            candidate_mask,
            234,
            306,
            structural_fallback_mask=structural_mask,
            fallback_max_row_width_ratio=profile[
                "line_band_fallback_max_row_width_ratio"
            ],
            fallback_min_row_coverage_ratio=profile[
                "line_band_fallback_min_row_coverage_ratio"
            ],
        )

        self.assertFalse(observation["valid"])
        self.assertEqual(observation["source"], "none")

    def test_structural_fallback_recovers_fit_from_giant_component(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        structural_mask = np.zeros((319, 480), dtype=np.uint8)
        # A área superior grande força a rejeição global, mas a fita que chega
        # à NEAR continua estreita e conectada ao mesmo componente estrutural.
        cv2.rectangle(structural_mask, (0, 0), (479, 105), 255, -1)
        cv2.line(structural_mask, (240, 318), (300, 90), 255, 20)
        candidate_mask = camera_line_frame.create_line_candidate_mask(
            structural_mask,
            profile,
        )
        self.assertEqual(np.count_nonzero(candidate_mask), 0)

        near = camera_line_frame.analyze_line_band(
            candidate_mask,
            198,
            317,
            structural_fallback_mask=structural_mask,
            fallback_max_row_width_ratio=profile[
                "line_band_fallback_max_row_width_ratio"
            ],
            fallback_min_row_coverage_ratio=profile[
                "line_band_fallback_min_row_coverage_ratio"
            ],
        )
        self.assertTrue(near["valid"])
        self.assertEqual(near["source"], "structural_fallback")

        trajectory = camera_line_frame.analyze_visual_trajectory(
            candidate_mask,
            0,
            319,
            240.0,
            True,
            near["x"],
            structural_mask=structural_mask,
            near_center=near["center"],
            near_observation=near,
            fit_source_mask=structural_mask,
            trajectory_source="structural_fallback",
        )
        self.assertTrue(trajectory["trajectory_valid"])
        self.assertEqual(trajectory["trajectory_source"], "structural_fallback")
        self.assertFalse(trajectory["corner90_candidate"])

    def test_extreme_curve_fields_remain_inactive_for_compatibility(self):
        trajectory = camera_line_frame.empty_trajectory_result()
        trajectory["extreme_curve_candidate"] = True
        trajectory["extreme_curve_direction"] = "RIGHT"
        trajectory["extreme_curve_curvature"] = 2.0

        status = camera_line_frame.trajectory_status_fields(trajectory)
        self.assertFalse(status["extremeCurveCandidate"])
        self.assertEqual(status["extremeCurveDirection"], "NONE")
        self.assertEqual(status["extremeCurveState"], "idle")

    def test_single_near_band_estimates_line_axis_internally(self):
        if not CV2_AVAILABLE:
            self.skipTest("OpenCV não está disponível")

        contour = np.array(
            [[[300, 0]], [[340, 0]], [[360, 99]], [[320, 99]]],
            dtype=np.int32,
        )
        axis = camera_line_frame.build_single_band_line_axis(
            (100, 640), contour, 0
        )

        self.assertTrue(axis["valid"])
        self.assertLess(axis["forward"][1], 0.0)
        self.assertLess(axis["forward"][0], 0.0)

    def test_green_proximity_uses_its_separate_band(self):
        geometry = camera_line_frame.resolve_vision_geometry(
            480,
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
        )
        inside = synthetic_candidate((270, 150))
        outside = synthetic_candidate((270, 90))

        self.assertTrue(camera_line_frame.green_seen_in_vertical_band(
            [inside],
            geometry["green_observation_start_y"],
            geometry["green_observation_end_y"],
            synthetic_line_axis(),
        ))
        self.assertFalse(camera_line_frame.green_seen_in_vertical_band(
            [outside],
            geometry["green_observation_start_y"],
            geometry["green_observation_end_y"],
            synthetic_line_axis(),
        ))

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

    def test_fast_json_publishes_far_near_and_visual_errors(self):
        control_terms = camera_line_frame.calculate_control_terms(
            True,
            -0.0625,
            True,
            -0.1875,
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            status_path = os.path.join(temporary_directory, "line.json")
            temporary_path = os.path.join(temporary_directory, "line.tmp.json")
            with (
                mock.patch.object(
                    camera_line_frame, "LINE_STATUS_PATH", status_path
                ),
                mock.patch.object(
                    camera_line_frame,
                    "TEMP_LINE_STATUS_PATH",
                    temporary_path,
                ),
            ):
                camera_line_frame.save_line_status(
                    near_valid=True,
                    near_x=300.0,
                    near_error=-0.0625,
                    far_valid=True,
                    far_x=260.0,
                    far_error=-0.1875,
                    lateral_error=-0.0625,
                    heading_error=-0.125,
                    adaptive_preview=control_terms["adaptive_preview"],
                    preview_error=control_terms["preview_error"],
                    p_term=control_terms["p_term"],
                    filtered_derivative=control_terms["filtered_derivative"],
                    d_term=control_terms["d_term"],
                    control_error=control_terms["control_error"],
                    correction=-0.03,
                    target_correction=-0.08,
                    applied_correction=-0.03,
                    steer_rate_used=2.5,
                    left_preview=0.69,
                    right_preview=0.71,
                    gap_candidate=False,
                    gap_alignment_valid=False,
                    gap_alignment_error=0.0,
                    gap_return_valid=False,
                    gap_return_error=0.0,
                    line_timestamp=10.0,
                    line_sequence=7,
                )
            with open(status_path, "r", encoding="utf-8") as status_file:
                status = json.load(status_file)

        self.assertTrue(status["nearValid"])
        self.assertTrue(status["farValid"])
        self.assertEqual(status["nearX"], 300.0)
        self.assertEqual(status["farX"], 260.0)
        self.assertEqual(status["lateralError"], -0.0625)
        self.assertEqual(status["headingError"], -0.125)
        self.assertEqual(
            status["adaptivePreview"],
            control_terms["adaptive_preview"],
        )
        self.assertEqual(status["previewError"], control_terms["preview_error"])
        self.assertEqual(status["pTerm"], control_terms["p_term"])
        self.assertEqual(status["filteredDerivative"], 0.0)
        self.assertEqual(status["dTerm"], 0.0)
        self.assertEqual(status["controlError"], control_terms["control_error"])
        self.assertEqual(status["targetCorrection"], -0.08)
        self.assertEqual(status["appliedCorrection"], -0.03)
        self.assertEqual(status["steerRateUsed"], 2.5)
        self.assertEqual(status["preview"], control_terms["adaptive_preview"])
        self.assertEqual(status["kControl"], 1.60)
        self.assertAlmostEqual(
            status["kNear"],
            1.60 * (1.0 - control_terms["adaptive_preview"]),
        )
        self.assertAlmostEqual(
            status["kFar"],
            1.60 * control_terms["adaptive_preview"],
        )
        self.assertFalse(status["trajectoryValid"])
        self.assertEqual(status["trajectoryMode"], "fallback_far_near")
        self.assertEqual(status["fitSampleCount"], 0)
        self.assertEqual(status["lookaheadX"], 0.0)
        self.assertEqual(status["curvature"], 0.0)

    def test_visual_trajectory_fits_centered_straight_line(self):
        trajectory = camera_line_frame.analyze_visual_trajectory(
            synthetic_trajectory_mask(lambda _forward: 0.0),
            0,
            425,
            320.0,
            True,
            320.0,
        )

        self.assertTrue(trajectory["trajectory_valid"])
        self.assertEqual(trajectory["trajectory_mode"], "quadratic")
        self.assertGreaterEqual(
            trajectory["fit_sample_count"],
            camera_line_frame.QUADRATIC_FIT_MIN_SAMPLES,
        )
        self.assertAlmostEqual(trajectory["fit_a"], 0.0, places=4)
        self.assertAlmostEqual(trajectory["fit_b"], 0.0, places=4)
        self.assertAlmostEqual(trajectory["fit_c"], 0.0, places=4)
        self.assertAlmostEqual(
            trajectory["lookahead_y"],
            camera_line_frame.VISUAL_PURSUIT_LOOKAHEAD,
            places=4,
        )
        self.assertAlmostEqual(trajectory["curvature"], 0.0, places=4)

    def test_visual_trajectory_curve_generates_continuous_right_arc(self):
        trajectory = camera_line_frame.analyze_visual_trajectory(
            synthetic_trajectory_mask(lambda forward: 0.25 * forward * forward),
            0,
            425,
            320.0,
            True,
            320.0,
        )
        preview = camera_line_frame.calculate_visual_pursuit_preview(
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
            trajectory["curvature"],
        )

        self.assertTrue(trajectory["trajectory_valid"])
        self.assertGreater(trajectory["fit_a"], 0.20)
        self.assertGreater(trajectory["lookahead_x"], 0.0)
        self.assertGreater(trajectory["curvature"], 0.0)
        self.assertGreater(preview[2], 0.0)
        self.assertGreater(preview[3], preview[4])
        self.assertAlmostEqual(preview[3] - preview[4], preview[2], places=6)

    def test_visual_trajectory_curve_preserves_left_sign(self):
        trajectory = camera_line_frame.analyze_visual_trajectory(
            synthetic_trajectory_mask(lambda forward: -0.20 * forward * forward),
            0,
            425,
            320.0,
            True,
            320.0,
        )
        preview = camera_line_frame.calculate_visual_pursuit_preview(
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
            trajectory["curvature"],
        )

        self.assertTrue(trajectory["trajectory_valid"])
        self.assertLess(trajectory["curvature"], 0.0)
        self.assertLess(preview[2], 0.0)
        self.assertLess(preview[3], preview[4])

    def test_visual_trajectory_uses_linear_fallback_with_few_centers(self):
        mask = synthetic_trajectory_mask(
            lambda forward: 0.10 * forward,
            selected_samples=(0, 2, 4, 6, 8, 10),
        )
        trajectory = camera_line_frame.analyze_visual_trajectory(
            mask,
            0,
            425,
            320.0,
            True,
            320.0,
        )

        self.assertTrue(trajectory["trajectory_valid"])
        self.assertEqual(trajectory["trajectory_mode"], "linear_fallback")
        self.assertEqual(trajectory["fit_degree"], 1)
        self.assertEqual(trajectory["fit_a"], 0.0)
        self.assertGreaterEqual(
            trajectory["fit_sample_count"],
            camera_line_frame.LINEAR_FIT_MIN_SAMPLES,
        )

    def test_visual_trajectory_does_not_invent_missing_line(self):
        trajectory = camera_line_frame.analyze_visual_trajectory(
            np.zeros((425, 640), dtype=np.uint8),
            0,
            425,
            320.0,
            True,
            320.0,
        )

        self.assertFalse(trajectory["trajectory_valid"])
        self.assertEqual(trajectory["trajectory_mode"], "fallback_far_near")
        self.assertEqual(trajectory["fit_sample_count"], 0)

    def test_corner90_component_detects_connected_right_branch(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        cv2.rectangle(mask, (232, 200), (248, 306), 255, -1)
        cv2.rectangle(mask, (232, 200), (400, 216), 255, -1)

        corner = camera_line_frame.analyze_corner90_component(
            mask,
            0,
            (240, 270),
            0.0,
        )

        self.assertTrue(corner["corner90_candidate"])
        self.assertEqual(corner["corner90_direction"], "RIGHT")
        self.assertEqual(corner["corner90_angle"], 90.0)

    def test_corner90_component_detects_distant_connected_branch_at_zero_gate(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        cv2.rectangle(mask, (232, 75), (248, 306), 255, -1)
        cv2.rectangle(mask, (232, 75), (400, 91), 255, -1)

        corner = camera_line_frame.analyze_corner90_component(
            mask,
            0,
            (240, 270),
            0.0,
        )

        self.assertTrue(corner["corner90_candidate"])
        self.assertEqual(corner["corner90_direction"], "RIGHT")

    def test_corner90_bands_detects_distant_left_branch(self):
        corner = camera_line_frame.analyze_corner90_bands(
            {"valid": True, "x": 240.0},
            {
                "valid": True,
                "x": 149.0,
                "bounding_width_px": 210,
                "bounding_height_px": 70,
            },
            480,
        )

        self.assertTrue(corner["corner90_candidate"])
        self.assertEqual(corner["corner90_direction"], "LEFT")
        self.assertEqual(corner["corner90_angle"], -90.0)

    def test_corner90_bands_confirms_centered_straight_exit(self):
        corner = camera_line_frame.analyze_corner90_bands(
            {"valid": True, "x": 240.0},
            {
                "valid": True,
                "x": 244.0,
                "bounding_width_px": 20,
                "bounding_height_px": 70,
            },
            480,
        )

        self.assertFalse(corner["corner90_candidate"])
        self.assertTrue(corner["corner90_exit_alignment"])

    def test_corner90_bands_accepts_tolerant_exit_alignment(self):
        corner = camera_line_frame.analyze_corner90_bands(
            {"valid": True, "x": 285.0},
            {
                "valid": True,
                "x": 289.0,
                "bounding_width_px": 20,
                "bounding_height_px": 70,
            },
            480,
        )

        self.assertFalse(corner["corner90_candidate"])
        self.assertTrue(corner["corner90_exit_alignment"])

    def test_corner90_visual_handoff_requires_route_not_just_near(self):
        aligned_trajectory = {
            "trajectory_valid": True,
            "fit_a": 0.0,
            "fit_b": 0.0,
            "fit_sample_count": 8,
            "lookahead_y": 0.65,
        }
        tilted_trajectory = {
            **aligned_trajectory,
            "fit_b": 0.50,
        }
        self.assertTrue(camera_line_frame.corner90_visual_handoff_ready(
            False, True, 0.45, aligned_trajectory,
        ))
        self.assertFalse(camera_line_frame.corner90_visual_handoff_ready(
            False, True, 0.0, tilted_trajectory,
        ))
        self.assertFalse(camera_line_frame.corner90_visual_handoff_ready(
            True, True, 0.0, aligned_trajectory,
        ))
        self.assertFalse(camera_line_frame.corner90_visual_handoff_ready(
            False, True, 0.451, aligned_trajectory,
        ))
        self.assertFalse(camera_line_frame.corner90_visual_handoff_ready(
            False, False, 0.0, aligned_trajectory,
        ))
        self.assertFalse(camera_line_frame.corner90_visual_handoff_ready(
            False, True, 0.0, {"trajectory_valid": False},
        ))

    def test_quadratic_fit_rejects_isolated_center(self):
        y_values = np.linspace(0.05, 0.95, 15)
        points = [
            (0.12 * forward * forward, forward)
            for forward in y_values
        ]
        points[7] = (points[7][0] + 0.30, points[7][1])

        fit = camera_line_frame.fit_trajectory_polynomial(points, 2)

        self.assertIsNotNone(fit)
        self.assertFalse(bool(fit["inliers"][7]))
        self.assertAlmostEqual(fit["coefficients"][0], 0.12, places=4)
        self.assertAlmostEqual(fit["rms_error"], 0.0, places=4)

    def test_visual_pursuit_uses_small_deadband_and_run_floor(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        self.assertEqual(vision_profile["tracking_deadzone_ratio"], 0.025)
        self.assertEqual(vision_profile["visual_tracking_minimum_power"], 0.61)
        straight = camera_line_frame.calculate_visual_pursuit_preview(
            vision_profile,
            0.02,
        )
        light_curve = camera_line_frame.calculate_visual_pursuit_preview(
            vision_profile,
            0.05,
        )
        strong_curve = camera_line_frame.calculate_visual_pursuit_preview(
            vision_profile,
            5.0,
        )

        self.assertEqual(straight[1:], (0.0, 0.0, 0.70, 0.70))
        self.assertGreater(light_curve[1], 0.0)
        self.assertGreater(light_curve[3], 0.70)
        self.assertLess(light_curve[4], 0.70)
        self.assertEqual(
            strong_curve[2],
            camera_line_frame.MAX_CORRECTION_PREVIEW,
        )
        self.assertAlmostEqual(strong_curve[3], 0.86, places=6)
        self.assertAlmostEqual(strong_curve[4], 0.61, places=6)

    def test_steering_slew_limiter_uses_real_dt_and_crosses_zero(self):
        limiter = camera_line_frame.SteeringSlewRateLimiter()
        samples = [
            limiter.update(0.22, timestamp)[0]
            for timestamp in (0.00, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16)
        ]
        expected = (0.00, 0.036, 0.072, 0.108, 0.144, 0.180, 0.216, 0.22, 0.22)
        for actual, target in zip(samples, expected):
            self.assertAlmostEqual(actual, target, places=6)

        applied, release_rate = limiter.update(0.0, 0.18)
        self.assertAlmostEqual(release_rate, 2.5)
        self.assertAlmostEqual(applied, 0.17, places=6)

        limiter.reset()
        limiter.update(0.15, 1.00)
        self.assertAlmostEqual(limiter.update(0.15, 1.02)[0], 0.036, places=6)
        self.assertAlmostEqual(limiter.update(-0.15, 1.04)[0], 0.00, places=6)
        self.assertAlmostEqual(limiter.update(-0.15, 1.06)[0], -0.036, places=6)

    def test_corner90_geometry_accepts_visual_turns_from_30_to_140_degrees(self):
        def points_for_headings(near_degrees, far_degrees):
            near_slope = math.tan(math.radians(near_degrees))
            far_slope = math.tan(math.radians(far_degrees))
            near_points = [
                (near_slope * forward, forward)
                for forward in (0.05, 0.15, 0.25)
            ]
            near_end_x = near_slope * 0.25
            far_points = [
                (near_end_x + far_slope * (forward - 0.25), forward)
                for forward in (0.35, 0.45, 0.55, 0.65, 0.75, 0.85)
            ]
            return near_points + far_points

        for near_degrees, far_degrees in ((0.0, 30.0), (0.0, 45.0),
                                          (0.0, 90.0), (-52.0, 88.0)):
            with self.subTest(near=near_degrees, far=far_degrees):
                corner = camera_line_frame.analyze_corner90_geometry(
                    points_for_headings(near_degrees, far_degrees),
                    0.0,
                )
                self.assertTrue(corner["corner90_candidate"])
                self.assertEqual(corner["corner90_direction"], "RIGHT")

    def test_corner90_geometry_rejects_heading_change_above_140_degrees(self):
        near_slope = math.tan(math.radians(-55.0))
        far_slope = math.tan(math.radians(88.0))
        points = [
            (near_slope * forward, forward)
            for forward in (0.05, 0.15, 0.25)
        ]
        near_end_x = near_slope * 0.25
        points.extend(
            (near_end_x + far_slope * (forward - 0.25), forward)
            for forward in (0.35, 0.45, 0.55, 0.65, 0.75, 0.85)
        )
        corner = camera_line_frame.analyze_corner90_geometry(points, 0.0)
        self.assertFalse(corner["corner90_candidate"])

    def test_corner90_geometry_rejects_line_too_far_from_center(self):
        straight_then_right = [
            (0.0, 0.05), (0.0, 0.15), (0.0, 0.25),
            (0.30, 0.35), (0.70, 0.45), (1.10, 0.55),
            (1.50, 0.65), (1.90, 0.75), (2.30, 0.85),
        ]
        corner = camera_line_frame.analyze_corner90_geometry(
            straight_then_right,
            0.31,
        )

        self.assertFalse(corner["corner90_candidate"])

    def test_black_line_geometry_confidence_requires_agreement_or_strong_source(self):
        regular_left = {
            "corner90_candidate": True,
            "corner90_direction": "LEFT",
            "corner90_angle": -60.0,
            "black_line_geometry_source_strength": 0.55,
        }
        regular_left_component = {
            "corner90_candidate": True,
            "corner90_direction": "LEFT",
            "corner90_angle": -90.0,
            "black_line_geometry_source_strength": 0.55,
        }
        combined = camera_line_frame.combine_black_line_geometry(
            regular_left,
            regular_left_component,
        )
        self.assertTrue(combined["candidate"])
        self.assertEqual(combined["direction"], "LEFT")
        self.assertGreaterEqual(
            combined["confidence"],
            camera_line_frame.BLACK_LINE_GEOMETRY_MIN_CONFIDENCE,
        )

        conflicting_right = dict(regular_left_component)
        conflicting_right["corner90_direction"] = "RIGHT"
        self.assertFalse(camera_line_frame.combine_black_line_geometry(
            regular_left,
            conflicting_right,
        )["candidate"])

        strong_right = dict(regular_left_component)
        strong_right.update({
            "corner90_direction": "RIGHT",
            "corner90_angle": 90.0,
            "black_line_geometry_source_strength": 0.80,
        })
        self.assertTrue(camera_line_frame.combine_black_line_geometry(
            strong_right,
        )["candidate"])

    def test_corner90_tracker_confirms_one_frame_then_exit_alignment(self):
        tracker = camera_line_frame.Corner90ConfirmationTracker()
        trajectory = camera_line_frame.empty_trajectory_result()
        trajectory.update({
            "corner90_candidate": True,
            "corner90_direction": "LEFT",
        })
        tracker.update(trajectory, True)
        self.assertEqual(trajectory["corner90_confirm_frames"], 1)
        self.assertEqual(trajectory["corner90_state"], "confirmed")

        tracker.update(trajectory, True)
        self.assertEqual(trajectory["corner90_confirm_frames"], 1)
        self.assertEqual(trajectory["corner90_state"], "confirmed")

        # A geometria antiga pode continuar visível enquanto o robô gira.
        # Ela mantém o candidato até desaparecer; só então a saída pode procurar
        # a nova reta, evitando encerrar o pivot pela linha de chegada.
        trajectory.update({
            "corner90_candidate": False,
            "corner90_exit_alignment": False,
        })
        tracker.update(trajectory, True)
        self.assertFalse(trajectory["corner90_candidate"])
        self.assertEqual(trajectory["corner90_state"], "pivoting")

        trajectory.update({
            "corner90_candidate": True,
            "corner90_exit_alignment": True,
        })
        tracker.update(trajectory, True)
        self.assertEqual(trajectory["corner90_state"], "pivoting")
        self.assertFalse(trajectory["corner90_exit_alignment"])

        trajectory.update({
            "corner90_candidate": False,
            "corner90_exit_alignment": True,
        })
        tracker.update(trajectory, True)
        self.assertEqual(trajectory["corner90_state"], "exit_aligned")

    def test_corner90_tracker_rearms_after_geometry_clears(self):
        tracker = camera_line_frame.Corner90ConfirmationTracker()
        trajectory = camera_line_frame.empty_trajectory_result()
        trajectory.update({
            "corner90_candidate": True,
            "corner90_direction": "RIGHT",
        })
        for _ in range(camera_line_frame.CORNER90_CONFIRM_FRAMES):
            tracker.update(trajectory, True)

        trajectory.update({
            "corner90_candidate": False,
            "corner90_exit_alignment": False,
        })
        tracker.update(trajectory, True)
        trajectory["corner90_exit_alignment"] = True
        for _ in range(camera_line_frame.CORNER90_EXIT_ALIGNMENT_FRAMES):
            tracker.update(trajectory, True)

        trajectory.update({
            "corner90_candidate": True,
            "corner90_direction": "RIGHT",
            "corner90_exit_alignment": False,
        })
        tracker.update(trajectory, True)
        self.assertEqual(trajectory["corner90_state"], "rearming")
        self.assertFalse(trajectory["corner90_candidate"])

        trajectory.update({
            "corner90_candidate": False,
            "corner90_exit_alignment": False,
        })
        for _ in range(camera_line_frame.CORNER90_REARM_CLEAR_FRAMES - 1):
            tracker.update(trajectory, True)
            self.assertEqual(trajectory["corner90_state"], "rearming")

        tracker.update(trajectory, True)
        self.assertEqual(trajectory["corner90_state"], "idle")

        trajectory.update({
            "corner90_candidate": True,
            "corner90_direction": "RIGHT",
        })
        tracker.update(trajectory, True)
        self.assertEqual(trajectory["corner90_state"], "confirmed")
        self.assertEqual(trajectory["corner90_confirm_frames"], 1)

        tracker.update(trajectory, True)
        self.assertEqual(trajectory["corner90_state"], "confirmed")
        self.assertEqual(trajectory["corner90_confirm_frames"], 1)

    def test_corner90_tracker_allows_opposite_corner_while_rearming(self):
        tracker = camera_line_frame.Corner90ConfirmationTracker()
        tracker.direction = "LEFT"
        tracker.rearm_active = True
        trajectory = camera_line_frame.empty_trajectory_result()
        trajectory.update({
            "corner90_candidate": True,
            "corner90_direction": "RIGHT",
        })

        tracker.update(trajectory, True)

        self.assertFalse(tracker.rearm_active)
        self.assertEqual(trajectory["corner90_state"], "confirmed")
        self.assertEqual(trajectory["corner90_direction"], "RIGHT")

        tracker.update(trajectory, True)
        self.assertEqual(trajectory["corner90_state"], "confirmed")

    def test_corner90_status_preserves_broad_confirmation_geometry(self):
        trajectory = camera_line_frame.empty_trajectory_result()
        trajectory.update({
            "corner90_candidate": False,
            "corner90_confirmation_candidate": True,
            "corner90_confirmation_direction": "LEFT",
            "corner90_confirmation_angle": -90.0,
        })

        status = camera_line_frame.trajectory_status_fields(trajectory)

        self.assertFalse(status["blackLineGeometryCandidate"])
        self.assertEqual(status["blackLineGeometryDirection"], "NONE")
        self.assertEqual(status["blackLineGeometryAngleDegrees"], 0.0)

    def test_slew_limited_visual_mixing_preserves_target_authority(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        left_preview, right_preview = camera_line_frame.mix_visual_pursuit_correction(
            vision_profile,
            0.06,
        )
        self.assertAlmostEqual(left_preview, 0.73, places=6)
        self.assertAlmostEqual(right_preview, 0.67, places=6)

        left_preview, right_preview = camera_line_frame.mix_visual_pursuit_correction(
            vision_profile,
            camera_line_frame.MAX_CORRECTION_PREVIEW,
        )
        self.assertAlmostEqual(left_preview, 0.86, places=6)
        self.assertAlmostEqual(right_preview, 0.61, places=6)

    def test_fast_json_publishes_valid_trajectory(self):
        trajectory = camera_line_frame.analyze_visual_trajectory(
            synthetic_trajectory_mask(lambda forward: 0.15 * forward * forward),
            0,
            425,
            320.0,
            True,
            320.0,
        )
        fields = camera_line_frame.trajectory_status_fields(trajectory)

        self.assertTrue(fields["trajectoryValid"])
        self.assertEqual(fields["trajectoryMode"], "quadratic")
        self.assertGreater(fields["fitSampleCount"], 0)
        self.assertGreater(fields["lookaheadX"], 0.0)
        self.assertGreater(fields["lookaheadY"], 0.0)
        self.assertGreater(fields["curvature"], 0.0)

    def test_down_guidance_on_centered_straight_line(self):
        self.assert_control_preview(
            "down", True, 0.0,
            (0.0, 0.0, 0.0, 0.70, 0.70),
        )

    def test_down_guidance_on_parallel_offset_straight_line(self):
        self.assert_control_preview(
            "down",
            True,
            0.30,
            (
                1.60 * (0.27 / 0.97),
                (1.60 * (0.27 / 0.97) - 0.10) / 0.90,
                0.30 * (1.60 * (0.27 / 0.97) - 0.10) / 0.90,
                0.70 + 0.15 * (1.60 * (0.27 / 0.97) - 0.10) / 0.90,
                0.70 - 0.15 * (1.60 * (0.27 / 0.97) - 0.10) / 0.90,
            ),
            far_valid=True,
            far_error=0.30,
        )

    def test_down_guidance_reacts_to_left_curve_in_near(self):
        self.assert_control_preview(
            "down", True, -0.356,
            (-0.356, -0.2844444444, -0.0853333333,
             0.6573333333, 0.7426666667),
        )

    def test_down_guidance_separates_preview_and_control_gain(self):
        self.assertEqual(camera_line_frame.PREVIEW_MIN, 0.20)
        self.assertEqual(camera_line_frame.PREVIEW_MAX, 0.70)
        self.assertEqual(camera_line_frame.K_CONTROL, 1.60)
        self.assertEqual(camera_line_frame.MAX_CORRECTION_PREVIEW, 0.25)
        self.assertAlmostEqual(
            camera_line_frame.calculate_preview_error(0.10, 0.50),
            0.38,
        )
        expected_control = 1.60 * (0.38 - 0.03) / 0.97
        self.assert_control_preview(
            "down",
            True,
            0.10,
            (
                expected_control,
                (expected_control - 0.10) / 0.90,
                0.30 * (expected_control - 0.10) / 0.90,
                0.70 + 0.15 * (expected_control - 0.10) / 0.90,
                0.70 - 0.15 * (expected_control - 0.10) / 0.90,
            ),
            far_valid=True,
            far_error=0.50,
        )

    def test_down_guidance_anticipates_before_lateral_error_grows(self):
        result = camera_line_frame.calculate_control_preview(
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
            True,
            0.02,
            True,
            -0.38,
        )

        self.assertAlmostEqual(
            camera_line_frame.calculate_preview_error(0.02, -0.38),
            -0.26,
            places=6,
        )
        self.assertAlmostEqual(
            result[0],
            -1.60 * (0.26 - 0.03) / 0.97,
            places=6,
        )
        self.assertLess(result[2], 0.0)

    def test_heading_error_is_diagnostic_and_clamped(self):
        result = camera_line_frame.calculate_control_preview(
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
            True,
            0.90,
            True,
            -0.90,
        )

        # O heading bruto fica limitado apenas para o overlay e a telemetria.
        self.assertAlmostEqual(
            result[0],
            -1.60 * (0.36 - 0.03) / 0.97,
            places=6,
        )
        self.assertEqual(
            camera_line_frame.calculate_heading_error(0.90, -0.90),
            -1.0,
        )

    def test_adaptive_preview_uses_near_on_straight_and_far_on_curve(self):
        self.assertEqual(camera_line_frame.calculate_adaptive_preview(0.0), 0.20)
        self.assertEqual(camera_line_frame.calculate_adaptive_preview(0.03), 0.20)
        self.assertAlmostEqual(
            camera_line_frame.calculate_adaptive_preview(0.115),
            0.45,
        )
        self.assertEqual(camera_line_frame.calculate_adaptive_preview(0.20), 0.70)
        self.assertEqual(camera_line_frame.calculate_adaptive_preview(-0.80), 0.70)

    def test_error_deadband_is_zero_and_continuous_at_boundary(self):
        self.assertEqual(camera_line_frame.apply_error_deadband(0.03), 0.0)
        self.assertEqual(camera_line_frame.apply_error_deadband(-0.03), 0.0)
        self.assertLess(
            abs(camera_line_frame.apply_error_deadband(0.030001)),
            0.000002,
        )
        self.assertAlmostEqual(
            camera_line_frame.apply_error_deadband(0.10),
            0.07 / 0.97,
        )

    def test_near_derivative_is_filtered_and_reduces_falling_error(self):
        derivative_filter = camera_line_frame.NearDerivativeFilter()

        self.assertEqual(derivative_filter.update(True, 0.20, 10.0), 0.0)
        filtered = derivative_filter.update(True, 0.10, 10.1)
        self.assertAlmostEqual(filtered, -0.25)

        terms = camera_line_frame.calculate_control_terms(
            True,
            0.10,
            True,
            0.20,
            filtered,
        )
        self.assertAlmostEqual(terms["d_term"], -0.005)
        self.assertLess(terms["control_error"], terms["p_term"])

    def test_near_derivative_resets_after_invalid_or_stale_interval(self):
        derivative_filter = camera_line_frame.NearDerivativeFilter()
        derivative_filter.update(True, 0.0, 1.0)
        derivative_filter.update(True, 0.10, 1.1)
        self.assertEqual(derivative_filter.update(False, 0.0, 1.2), 0.0)
        self.assertEqual(derivative_filter.update(True, 0.20, 2.0), 0.0)
        self.assertEqual(derivative_filter.update(True, 0.30, 2.5), 0.0)

    def test_down_guidance_uses_near_only_fallback_without_weight_reduction(self):
        self.assert_control_preview(
            "down", True, 0.30,
            (0.30, 2.0 / 9.0, 1.0 / 15.0, 11.0 / 15.0, 2.0 / 3.0),
        )

    def test_down_guidance_uses_far_only_conservative_reference(self):
        self.assert_control_preview(
            "down",
            False,
            0.0,
            (-0.30, -2.0 / 9.0, -1.0 / 15.0, 2.0 / 3.0, 11.0 / 15.0),
            far_valid=True,
            far_error=-0.30,
        )

    def test_down_guidance_reacts_to_right_curve_in_near(self):
        self.assert_control_preview(
            "down", True, 0.356,
            (0.356, 0.2844444444, 0.0853333333,
             0.7426666667, 0.6573333333),
        )

    def test_down_guidance_uses_only_near(self):
        self.assert_control_preview(
            "down", True, -0.30,
            (-0.30, -2.0 / 9.0, -1.0 / 15.0, 2.0 / 3.0, 11.0 / 15.0),
        )

    def test_down_guidance_stays_zero_when_near_is_invalid(self):
        self.assert_control_preview(
            "down", False, 0.0,
            (0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def test_down_guidance_clamps_combined_error(self):
        for near_error, expected_guidance in (
            (3.0, 1.0),
            (-3.0, -1.0),
        ):
            with self.subTest(expected_guidance=expected_guidance):
                result = camera_line_frame.calculate_control_preview(
                    camera_line_frame.CAMERA_PROFILES["down"]["vision"],
                    True,
                    near_error,
                )
                self.assertEqual(result[0], expected_guidance)
                self.assertGreaterEqual(result[1], -1.0)
                self.assertLessEqual(result[1], 1.0)

    def test_down_balanced_differential_mixer(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        cases = (
            (0.0, 0.0, 0.700, 0.700),
            (0.28, 0.06, 0.730, 0.670),
            (-0.28, -0.06, 0.670, 0.730),
            (0.55, 0.15, 0.775, 0.625),
            (-0.55, -0.15, 0.625, 0.775),
        )

        self.assertTrue(vision_profile["balanced_differential_mixing"])
        self.assertEqual(vision_profile["base_speed_preview"], 0.70)
        self.assertEqual(vision_profile["minimum_tracking_power"], 0.69)
        for guidance_error, correction, left, right in cases:
            with self.subTest(guidance_error=guidance_error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, guidance_error
                )
                self.assertAlmostEqual(result[2], correction, places=6)
                self.assertAlmostEqual(result[3], left, places=6)
                self.assertAlmostEqual(result[4], right, places=6)
                self.assertAlmostEqual((result[3] + result[4]) / 2.0, 0.70)
                self.assertGreaterEqual(result[3], 0.61)
                self.assertGreaterEqual(result[4], 0.61)
                self.assertLessEqual(result[3], 1.0)
                self.assertLessEqual(result[4], 1.0)

    def test_every_nonzero_camera_preview_respects_motor_floor(self):
        scenarios = (
            ("down", True, 0.0, True, 0.0),
            ("down", True, -0.50, True, -0.10),
            ("down", True, 0.50, True, 0.10),
            ("down", True, 0.30, False, 0.0),
            ("down", False, 0.0, True, -0.30),
            ("forward", True, -0.50, True, -0.10),
            ("forward", True, 0.50, False, 0.0),
        )
        for role, near_valid, near_error, far_valid, far_error in scenarios:
            with self.subTest(role=role, near=near_error, far=far_error):
                result = camera_line_frame.calculate_control_preview(
                    camera_line_frame.CAMERA_PROFILES[role]["vision"],
                    near_valid,
                    near_error,
                    far_valid,
                    far_error,
                )
                for power in result[3:]:
                    if power != 0.0:
                        expected_floor = (
                            camera_line_frame.TRACKING_RUN_MINIMUM_MOTOR_PREVIEW
                            if camera_line_frame.CAMERA_PROFILES[role]["vision"].get(
                                "balanced_differential_mixing", False
                            )
                            else camera_line_frame.MINIMUM_MOTOR_PREVIEW
                        )
                        self.assertGreaterEqual(abs(power), expected_floor)

    def test_forward_profile_uses_near_for_control(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["forward"]["vision"]
        result = camera_line_frame.calculate_control_preview(
            vision_profile, True, 0.036
        )

        self.assertEqual(result, (0.036, 0.0, 0.0, 0.69, 0.69))

    def test_forward_profile_preserves_one_sided_mixer(self):
        self.assert_control_preview(
            "forward", True, 0.28,
            (0.28, 0.20, 0.06, 0.75, 0.69),
        )

    def test_down_deadzone_uses_ten_percent(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]

        self.assertEqual(vision_profile["near_deadzone_ratio"], 0.10)
        for error in (0.08, -0.08):
            with self.subTest(error=error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, error
                )
                self.assertEqual(result[1], 0.0)
                self.assertEqual(result[3:], (0.70, 0.70))
        for error in (0.11, -0.11):
            with self.subTest(error=error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, error
                )
                self.assertNotEqual(result[1], 0.0)
                self.assertEqual(result[1] > 0.0, error > 0.0)

    def test_forward_deadzone_remains_ten_percent(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["forward"]["vision"]

        self.assertEqual(vision_profile["near_deadzone_ratio"], 0.10)
        for error in (0.08, -0.08):
            with self.subTest(error=error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, error
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
        self.assertAlmostEqual(scaled[3], 0.79)
        self.assertAlmostEqual(scaled[4], 0.69)

    def test_green_direction_is_retained_for_half_second(self):
        tracker = camera_line_frame.GreenObservationTracker()
        tracker.update(1, "ESQUERDA", 10.0)
        tracker.update(2, "ESQUERDA", 10.1)
        confirmed = tracker.update(3, "ESQUERDA", 10.2)
        retained = tracker.update(4, "SEM_DECISAO", 10.69)
        expired = tracker.update(5, "SEM_DECISAO", 10.70)

        self.assertEqual(confirmed, ("ESQUERDA", True, 2))
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

    def test_green_two_new_frames_confirm_but_repeated_frame_does_not(self):
        tracker = camera_line_frame.GreenObservationTracker()
        first = tracker.update(10, "ESQUERDA")
        repeated = tracker.update(10, "ESQUERDA")
        second = tracker.update(11, "ESQUERDA")

        self.assertEqual(first[2], 1)
        self.assertEqual(first[0], "SEM_DECISAO")
        self.assertEqual(repeated[2], 1)
        self.assertEqual(repeated[0], "SEM_DECISAO")
        self.assertEqual(second[2], 2)
        self.assertEqual(second, ("ESQUERDA", True, 2))

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
            "down", True, -0.356,
            (-0.356, -0.2844444444, -0.0853333333,
             0.6573333333, 0.7426666667),
        )
        self.assert_control_preview(
            "forward", True, 0.28,
            (0.28, 0.20, 0.06, 0.75, 0.69),
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

    def test_green_status_exposes_reference_roi_evidence(self):
        interpretation = {
            "observation_state": "UM_CANDIDATO",
            "interpretation": "DIREITA",
            "left_seen": False,
            "right_seen": True,
            "pair_compatible": False,
            "markers": [{
                "upper": {"valid": True, "black_ratio": 0.42},
                "left": {"valid": True, "black_ratio": 0.31},
                "right": {"valid": False, "black_ratio": 0.08},
            }],
        }
        status = camera_line_frame.build_green_status(
            [],
            0,
            interpretation,
            {"junction_valid": False},
            ("DIREITA", True, 2),
            0.0,
        )

        self.assertEqual(status["greenMarkerCount"], 1)
        self.assertEqual(status["greenValidatedMarkerCount"], 1)
        self.assertTrue(status["greenFrontRoiValid"])
        self.assertAlmostEqual(status["greenFrontBlackRatio"], 0.42)
        self.assertTrue(status["greenLeftRoiValid"])
        self.assertFalse(status["greenRightRoiValid"])


if __name__ == "__main__":
    unittest.main()
