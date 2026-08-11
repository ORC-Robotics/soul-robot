"""Testes offline da geometria do seguidor de linha.

Execute com:
    python3 scripts/test_vision_path.py

Os testes usam somente máscaras sintéticas e não acessam câmera, GPIO ou motores.
"""

import numpy as np

from vision_path import analyze_primary_path


WIDTH = 320
HEIGHT = 144
LINE_WIDTH = 15
CENTER_X = WIDTH // 2


def new_mask():
    return np.zeros((HEIGHT, WIDTH), dtype=np.uint8)


def draw_thick_segment(mask, start, end, width=LINE_WIDTH):
    """Desenha um segmento espesso sem depender do OpenCV no computador de teste."""

    start_x, start_y = start
    end_x, end_y = end
    delta_x = float(end_x - start_x)
    delta_y = float(end_y - start_y)
    length_squared = delta_x * delta_x + delta_y * delta_y
    grid_y, grid_x = np.ogrid[: mask.shape[0], : mask.shape[1]]
    if length_squared <= 0.0:
        projection = np.zeros(mask.shape, dtype=np.float64)
    else:
        projection = np.clip(
            ((grid_x - start_x) * delta_x + (grid_y - start_y) * delta_y)
            / length_squared,
            0.0,
            1.0,
        )
    closest_x = start_x + projection * delta_x
    closest_y = start_y + projection * delta_y
    radius = width * 0.5
    mask[(grid_x - closest_x) ** 2 + (grid_y - closest_y) ** 2 <= radius**2] = 255


def fill_rectangle(mask, left, top, right, bottom):
    """Preenche um retângulo inclusivo nas coordenadas usadas pelos testes."""

    mask[max(0, top) : min(mask.shape[0], bottom + 1),
         max(0, left) : min(mask.shape[1], right + 1)] = 255


def draw_path(points, width=LINE_WIDTH):
    mask = new_mask()
    for start, end in zip(points, points[1:]):
        draw_thick_segment(mask, start, end, width)
    return mask


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def require_close(value, expected, tolerance, message):
    require(abs(value - expected) <= tolerance, f"{message}: valor={value:.2f}")


def test_centered_straight():
    observation = analyze_primary_path(draw_path([(CENTER_X, HEIGHT - 1), (CENTER_X, 0)]))
    require(observation.line_detected, "A reta central deveria ser detectada")
    require_close(observation.position_error_pixels, 0.0, 2.0, "Erro lateral da reta central")
    require_close(observation.heading_error_degrees, 0.0, 2.0, "Heading da reta central")
    require(not observation.corner.detected, "Reta central não pode virar corner")


def test_shifted_straights():
    left = analyze_primary_path(draw_path([(CENTER_X - 38, HEIGHT - 1), (CENTER_X - 38, 0)]))
    right = analyze_primary_path(draw_path([(CENTER_X + 42, HEIGHT - 1), (CENTER_X + 42, 0)]))
    require(left.position_error_pixels < -30.0, "Reta à esquerda deve gerar erro negativo")
    require(right.position_error_pixels > 30.0, "Reta à direita deve gerar erro positivo")
    require_close(left.heading_error_degrees, 0.0, 2.0, "Heading da reta esquerda")
    require_close(right.heading_error_degrees, 0.0, 2.0, "Heading da reta direita")


def test_smooth_curve():
    points = [
        (CENTER_X, HEIGHT - 1),
        (CENTER_X, 120),
        (CENTER_X + 3, 95),
        (CENTER_X + 9, 70),
        (CENTER_X + 19, 45),
        (CENTER_X + 34, 10),
    ]
    observation = analyze_primary_path(draw_path(points))
    require(observation.line_detected, "Curva suave deveria manter CurrentPath válido")
    require(not observation.corner.detected, "Curva suave não deve criar evento especial")
    require(observation.heading_error_degrees > 0.0, "Curva suave à direita deve gerar heading positivo")


def test_symmetric_widening_does_not_create_corner():
    mask = draw_path(
        [
            (CENTER_X + 18, HEIGHT - 1),
            (CENTER_X + 17, 120),
            (CENTER_X + 11, 100),
            (CENTER_X - 2, 75),
            (CENTER_X - 20, 35),
        ]
    )
    # Simula o alargamento frontal causado por perspectiva ou sombra sobre a
    # própria faixa. Como os dois lados crescem juntos, não existe direção de corner.
    fill_rectangle(mask, CENTER_X - 13, 94, CENTER_X + 35, 108)

    observation = analyze_primary_path(mask)
    require(observation.current_path_valid, "A linha alargada ainda deveria ser utilizável")
    require(
        not observation.corner.detected,
        "Alargamento aproximadamente simétrico não pode virar corner esquerdo",
    )


def distant_corner(direction):
    branch_y = 30
    branch_x = 275 if direction == "RIGHT" else 45
    return analyze_primary_path(
        draw_path([(CENTER_X, HEIGHT - 1), (CENTER_X, branch_y), (branch_x, branch_y)])
    )


def test_distant_right_corner_does_not_steer_early():
    observation = distant_corner("RIGHT")
    require(observation.line_detected, "Trecho atual do corner direito distante deve existir")
    require(observation.corner.detected, "Corner direito distante deveria aparecer no Preview")
    require(observation.corner.direction == "RIGHT", "Direção do corner distante deveria ser direita")
    require(observation.corner.proximity < 0.40, "Corner distante deve ter proximidade baixa")
    require_close(observation.position_error_pixels, 0.0, 3.0, "Corner distante alterou posição atual")
    require_close(observation.heading_error_degrees, 0.0, 3.0, "Corner distante alterou heading atual")
    require(
        all(abs(sample.x - CENTER_X) < 8.0 for sample in observation.current_path),
        "CurrentPath incorporou prematuramente o trecho horizontal direito",
    )


def test_distant_left_corner_does_not_steer_early():
    observation = distant_corner("LEFT")
    require(observation.corner.detected, "Corner esquerdo distante deveria aparecer no Preview")
    require(observation.corner.direction == "LEFT", "Direção do corner distante deveria ser esquerda")
    require_close(observation.position_error_pixels, 0.0, 3.0, "Corner esquerdo alterou posição atual")
    require_close(observation.heading_error_degrees, 0.0, 3.0, "Corner esquerdo alterou heading atual")


def test_near_right_corner_enters_action_zone():
    branch_y = 108
    observation = analyze_primary_path(
        draw_path([(CENTER_X, HEIGHT - 1), (CENTER_X, branch_y), (280, branch_y)])
    )
    require(observation.corner.detected, "Corner direito próximo deveria ser detectado")
    require(observation.corner.direction == "RIGHT", "Corner próximo deveria apontar para direita")
    require(observation.corner.proximity >= 0.72, "Corner próximo deveria alcançar a zona de ação")


def test_wide_corner_blob_stays_out_of_current_path():
    approach_x = CENTER_X - 15
    mask = new_mask()
    draw_thick_segment(mask, (approach_x, HEIGHT - 1), (approach_x, 106))
    # Reproduz a expansão preta larga vista no robô perto da base da imagem.
    # O centro dessa região fica muito à direita, mas sua origem continua na reta.
    fill_rectangle(mask, approach_x - 7, 106, 295, 122)

    observation = analyze_primary_path(
        mask, start_x=float(approach_x), expected_root_width=float(LINE_WIDTH)
    )
    require(observation.current_path_valid, "A linha de aproximação deveria continuar válida")
    require(observation.control_sample_count >= 4, "A regressão deveria usar samples próximos")
    require_close(
        observation.position_error_pixels,
        float(approach_x - CENTER_X),
        4.0,
        "Blob largo contaminou o erro da linha de aproximação",
    )
    require_close(observation.heading_error_degrees, 0.0, 4.0, "Blob largo contaminou o heading")
    require(
        all(abs(sample.x - approach_x) < 6.0 for sample in observation.current_path),
        "A região larga entrou no CurrentPath",
    )
    require(
        any(sample.rejection_reason == "width" for sample in observation.rejected_samples),
        "A expansão larga deveria aparecer como sample rejeitado por largura",
    )
    require(observation.corner.detected, "A expansão larga ainda deveria produzir Preview/corner")
    require(observation.corner.direction == "RIGHT", "O corner deveria permanecer à direita")
    require_close(
        observation.corner.anchor_x,
        float(approach_x),
        6.0,
        "A âncora do corner deveria nascer na linha de aproximação",
    )
    require(
        observation.corner.anchor_y
        < max(sample.y for sample in observation.rejected_samples) - 3.0,
        "A âncora Y não pode ser apenas a borda inferior mais próxima do blob",
    )
    require_close(
        observation.corner.proximity,
        observation.corner.anchor_y / float(HEIGHT - 1),
        0.001,
        "A proximidade deveria derivar exclusivamente da âncora Y",
    )


def test_distant_or_lateral_geometry_cannot_become_root():
    mask = new_mask()
    fill_rectangle(mask, 245, 108, 319, HEIGHT - 1)
    observation = analyze_primary_path(
        mask, start_x=float(CENTER_X), expected_root_width=float(LINE_WIDTH)
    )

    require(not observation.current_path_valid, "Geometria lateral não pode virar CurrentPath")
    require(not observation.line_detected, "Sem raiz próxima, lineDetected deve permanecer falso")
    require(not observation.current_path, "Preview distante não pode substituir o CurrentPath")
    require_close(observation.position_error_pixels, 0.0, 0.1, "Geometria sem raiz gerou erro lateral")
    require(
        any(
            sample.rejection_reason in ("root_center", "root_width")
            for sample in observation.rejected_samples
        ),
        "O debug deveria explicar por que a geometria lateral foi rejeitada",
    )


def test_corrupted_previous_hint_recovers_centered_root():
    approach_x = CENTER_X - 10
    side_branch_x = 285
    mask = new_mask()
    draw_thick_segment(mask, (approach_x, HEIGHT - 1), (approach_x, 72))
    draw_thick_segment(mask, (approach_x, 72), (side_branch_x, 72))
    draw_thick_segment(mask, (side_branch_x, 72), (side_branch_x, HEIGHT - 1), 32)

    observation = analyze_primary_path(
        mask,
        start_x=float(side_branch_x),
        expected_root_width=float(LINE_WIDTH),
    )
    require(observation.current_path_valid, "A raiz central deveria recuperar uma previsão contaminada")
    require_close(
        observation.root_x,
        float(approach_x),
        5.0,
        "A branch lateral foi reutilizada como raiz por causa do histórico",
    )


def test_stale_nearby_hint_recovers_centered_root():
    mask = draw_path([(CENTER_X, HEIGHT - 1), (CENTER_X, 0)])
    observation = analyze_primary_path(
        mask,
        start_x=float(CENTER_X + 55),
        expected_root_width=float(LINE_WIDTH),
    )
    require(
        observation.current_path_valid,
        "Uma previsão antiga ainda próxima do centro não pode esconder a linha frontal",
    )
    require_close(
        observation.root_x,
        float(CENTER_X),
        4.0,
        "A recuperação deveria preferir a faixa central realmente visível",
    )


def test_recovery_accepts_wide_centered_root_from_perspective():
    mask = new_mask()
    for y in range(HEIGHT):
        # A faixa ocupa cerca de 90 px junto ao robô e afina gradualmente para
        # frente, reproduzindo a perspectiva observada na imagem real.
        half_width = 11 + int(34 * y / max(1, HEIGHT - 1))
        fill_rectangle(mask, CENTER_X - half_width, y, CENTER_X + half_width, y)

    observation = analyze_primary_path(
        mask,
        start_x=float(CENTER_X),
        expected_root_width=float(LINE_WIDTH),
        allow_root_recovery=True,
    )
    require(
        observation.current_path_valid,
        "Uma faixa central larga por perspectiva não pode permanecer como ROOT_REJECTED",
    )
    require(
        observation.control_sample_count >= 4,
        "A raiz larga deveria começar cedo o bastante para alimentar a regressão",
    )
    require_close(
        observation.position_error_pixels,
        0.0,
        3.0,
        "A largura da raiz central não deveria criar erro lateral",
    )


def test_partial_root_at_image_edge_stabilizes_before_width_check():
    mask = new_mask()
    for y in range(HEIGHT):
        # As duas primeiras bandas vistas bottom-up contêm só a ponta estreita;
        # logo acima aparece a largura completa da mesma faixa central.
        half_width = 12 if y >= HEIGHT - 12 else 38
        fill_rectangle(mask, CENTER_X - half_width, y, CENTER_X + half_width, y)

    observation = analyze_primary_path(
        mask,
        start_x=float(CENTER_X),
        allow_root_recovery=True,
    )
    require(
        observation.current_path_valid,
        "A ponta estreita na borda não pode invalidar o restante da mesma linha",
    )
    require(
        observation.control_sample_count >= 4,
        "A largura deveria estabilizar antes de separar CurrentPath e Preview",
    )
    require(
        not observation.corner.detected,
        "Entrada parcial pela borda inferior não pode criar um corner falso",
    )


def test_recovery_accepts_centered_line_that_starts_ahead():
    mask = draw_path([(CENTER_X, 92), (CENTER_X, 0)])
    normal = analyze_primary_path(mask)
    recovered = analyze_primary_path(mask, allow_root_recovery=True)

    require(
        not normal.current_path_valid,
        "A aquisição normal deve continuar exigindo linha próxima do robô",
    )
    require(
        recovered.current_path_valid,
        "A recuperação deveria aceitar uma linha central visível um pouco mais à frente",
    )


def test_line_lost_and_isolated_noise():
    lost = analyze_primary_path(new_mask())
    require(not lost.line_detected, "Máscara vazia não pode detectar linha")

    noise = new_mask()
    fill_rectangle(noise, 20, 15, 35, 28)
    fill_rectangle(noise, 260, 45, 274, 57)
    noisy_observation = analyze_primary_path(noise)
    noisy_recovery = analyze_primary_path(noise, allow_root_recovery=True)
    require(not noisy_observation.line_detected, "Ruído distante não pode virar CurrentPath")
    require(not noisy_observation.corner.detected, "Ruído distante não pode virar corner")
    require(
        not noisy_recovery.line_detected,
        "A recuperação profunda não pode transformar ruído isolado em linha",
    )


def test_zigzag_tracks_only_first_corner():
    points = [
        (CENTER_X, HEIGHT - 1),
        (CENTER_X, 104),
        (255, 104),
        (255, 58),
        (80, 58),
        (80, 15),
    ]
    observation = analyze_primary_path(draw_path(points))
    require(observation.corner.detected, "Zig-zag deveria expor o primeiro corner")
    require(observation.corner.direction == "RIGHT", "Somente o primeiro corner do zig-zag deve ser rastreado")
    require_close(observation.position_error_pixels, 0.0, 3.0, "Segundo corner contaminou o CurrentPath")


def main():
    tests = [
        test_centered_straight,
        test_shifted_straights,
        test_smooth_curve,
        test_symmetric_widening_does_not_create_corner,
        test_distant_right_corner_does_not_steer_early,
        test_distant_left_corner_does_not_steer_early,
        test_near_right_corner_enters_action_zone,
        test_wide_corner_blob_stays_out_of_current_path,
        test_distant_or_lateral_geometry_cannot_become_root,
        test_corrupted_previous_hint_recovers_centered_root,
        test_stale_nearby_hint_recovers_centered_root,
        test_recovery_accepts_wide_centered_root_from_perspective,
        test_partial_root_at_image_edge_stabilizes_before_width_check,
        test_recovery_accepts_centered_line_that_starts_ahead,
        test_line_lost_and_isolated_noise,
        test_zigzag_tracks_only_first_corner,
    ]
    for test in tests:
        test()
        print(f"OK  {test.__name__}")
    print(f"{len(tests)} testes de visão concluídos com sucesso.")


if __name__ == "__main__":
    main()
