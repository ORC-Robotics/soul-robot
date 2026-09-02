"""Extrai e projeta a trajetória geométrica da faixa inferior."""

import math
import time

import cv2  # type: ignore
import numpy as np

from .virtual_sensors import VIRTUAL_NEAR_Y1


# Reaquisição geométrica após gaps.
#
# Quando não existe faixa no NEAR, procura uma continuação válida
# mais à frente antes de declarar a trajetória completamente perdida.
GEOMETRIC_GAP_FORWARD_MAX_FRAMES = 45
GEOMETRIC_GAP_REACQUIRE_FRAMES = 2
GEOMETRIC_GAP_SEARCH_STEP_PX = 6
GEOMETRIC_GAP_MAX_SEARCH_PX = 120

# Mantém por poucos quadros a evidência de que o NEAR-C estava visível.
# A memória permite reconhecer o início de um GAP sem depender de uma única
# amostra geométrica na altura exata usada pelo rastreador preservado.
GAP_NEAR_HISTORY_FRAMES = 3

# Uma faixa encontrada após o gap precisa continuar também nesta
# distância para não aceitarmos um pequeno blob isolado como caminho.
GEOMETRIC_GAP_CONFIRM_OFFSET_PX = 12
GEOMETRIC_GAP_PROJECTION_POINTS = 5
# Limita quanto o centro da faixa pode mudar entre a primeira
# detecção após o gap e sua amostra de confirmação.
GEOMETRIC_GAP_MAX_CENTER_SHIFT_PX = 50
# Geometria preservada exclusivamente para detectar e reaquistar GAP.
# Estes parâmetros não antecipam curvas nem alteram o controle LINE normal.
GEOMETRIC_PATH_BAND_HALF_HEIGHT = 2
GEOMETRIC_PATH_LOCAL_HEADING_POINTS = 5

# Distância aproximada entre pontos consecutivos da trajetória.
GEOMETRIC_TRACE_STEP_PX = 10.0

# Permite ao rastreador procurar o centro da faixa um pouco
# antes ou depois da distância nominal de avanço.
GEOMETRIC_TRACE_SEARCH_RANGE_PX = 5.0

GEOMETRIC_TRACE_MAX_BACKTRACK_Y_PX = 10.0

# Permite inclusive uma mudança de 90 graus entre dois passos.
# Nunca permite continuar para trás.
GEOMETRIC_TRACE_MAX_TURN_DEG = 90.0

# Limite de segurança contra caminhos que entrem em ciclos.
GEOMETRIC_TRACE_MAX_POINTS = 80

# Peso dado ao progresso em direção a uma saída lateral
# inequívoca do componente da faixa.
GEOMETRIC_TRACE_TARGET_PROGRESS_GAIN = 2.0

# Pequena margem usada para considerar que a faixa alcançou
# uma borda da imagem.
GEOMETRIC_TRACE_EXIT_MARGIN_PX = 3

# FAR e NEAR continuam presos às alturas extremas escolhidas.
GEOMETRIC_PATH_FAR_Y_RATIO = 0.00
GEOMETRIC_PATH_NEAR_Y_RATIO = VIRTUAL_NEAR_Y1


def find_active_band_segments(processed_line_mask, y):
    """
    Encontra os segmentos horizontais da faixa preta em uma pequena
    banda ao redor de uma determinada altura da imagem.

    Cada segmento retornado contém o centro horizontal ponderado
    pela quantidade real de pixels da máscara.
    """

    height, width = processed_line_mask.shape[:2]

    y0 = max(
        0,
        y - GEOMETRIC_PATH_BAND_HALF_HEIGHT,
    )
    y1 = min(
        height,
        y + GEOMETRIC_PATH_BAND_HALF_HEIGHT + 1,
    )

    band = processed_line_mask[y0:y1, :]

    if band.size == 0:
        return []

    active_columns = np.any(band > 0, axis=0)
    active_x = np.flatnonzero(active_columns)

    if active_x.size == 0:
        return []

    split_indices = np.where(
        np.diff(active_x) > 1
    )[0] + 1

    groups = np.split(
        active_x,
        split_indices,
    )

    segments = []

    for group in groups:
        if group.size == 0:
            continue

        x0 = int(group[0])
        x1 = int(group[-1])

        column_weights = np.count_nonzero(
            band[:, x0:x1 + 1],
            axis=0,
        ).astype(np.float32)

        total_weight = float(column_weights.sum())

        if total_weight <= 0.0:
            continue

        columns = np.arange(
            x0,
            x1 + 1,
            dtype=np.float32,
        )

        center_x = float(
            np.sum(columns * column_weights)
            / total_weight
        )

        segments.append({
            "x0": x0,
            "x1": x1,
            "centerX": center_x,
        })

    return segments

def find_geometric_lateral_exit(
    processed_line_mask,
    start_point,
):
    """
    Detecta uma saída lateral inequívoca do mesmo componente
    conectado que contém o início da trajetória.

    Se o componente alcançar o topo, mantém o comportamento
    geométrico normal.

    Se tocar somente a esquerda ou somente a direita, retorna
    um alvo naquela borda.

    Se houver ambiguidade, não interfere no rastreamento.
    """

    height, width = processed_line_mask.shape[:2]

    binary_mask = (
        processed_line_mask > 0
    ).astype(np.uint8)

    _, labels = cv2.connectedComponents(
        binary_mask,
        connectivity=8,
    )

    start_x = max(
        0,
        min(
            width - 1,
            int(round(start_point[0])),
        ),
    )

    start_y = max(
        0,
        min(
            height - 1,
            int(round(start_point[1])),
        ),
    )

    component_label = int(
        labels[start_y, start_x]
    )

    if component_label == 0:
        return None

    margin = GEOMETRIC_TRACE_EXIT_MARGIN_PX

    top_region = labels[
        0:min(height, margin + 1),
        :
    ]

    # Se a própria faixa chega ao topo, o tracer normal já
    # possui uma continuação natural para frente.
    if np.any(
        top_region == component_label
    ):
        return None

    left_region = labels[
        :,
        0:min(width, margin + 1),
    ]

    right_region = labels[
        :,
        max(0, width - margin - 1):width,
    ]

    left_positions = np.argwhere(
        left_region == component_label
    )

    right_positions = np.argwhere(
        right_region == component_label
    )

    touches_left = (
        left_positions.size > 0
    )

    touches_right = (
        right_positions.size > 0
    )

    # Nenhuma saída lateral ou duas saídas:
    # geometria ambígua, então não escolhemos por conta própria.
    if touches_left == touches_right:
        return None

    if touches_left:
        target_y = float(
            np.mean(left_positions[:, 0])
        )

        return (
            0.0,
            target_y,
        )

    target_y = float(
        np.mean(right_positions[:, 0])
    )

    return (
        float(width - 1),
        target_y,
    )

def find_geometric_gap_start(
    processed_line_mask,
    near_y,
):
    """
    Procura uma continuação válida da faixa à frente quando o
    NEAR está vazio.

    A primeira banda encontrada só é aceita se existir uma segunda
    amostra coerente um pouco mais à frente. Isso ajuda a rejeitar
    manchas pretas isoladas.

    Não cria pixels nem preenche o gap. Apenas encontra onde a
    faixa real reaparece.
    """

    height, width = processed_line_mask.shape[:2]

    image_center_x = (
        float(width - 1) / 2.0
    )

    minimum_y = max(
        0,
        near_y - GEOMETRIC_GAP_MAX_SEARCH_PX,
    )

    search_y = (
        near_y - GEOMETRIC_GAP_SEARCH_STEP_PX
    )

    while search_y >= minimum_y:
        segments = find_active_band_segments(
            processed_line_mask,
            search_y,
        )

        if not segments:
            search_y -= (
                GEOMETRIC_GAP_SEARCH_STEP_PX
            )
            continue

        confirmation_y = max(
            0,
            search_y
            - GEOMETRIC_GAP_CONFIRM_OFFSET_PX,
        )

        confirmation_segments = (
            find_active_band_segments(
                processed_line_mask,
                confirmation_y,
            )
        )

        if not confirmation_segments:
            search_y -= (
                GEOMETRIC_GAP_SEARCH_STEP_PX
            )
            continue

        valid_candidates = []

        for segment in segments:
            center_x = float(
                segment["centerX"]
            )

            confirmation_segment = min(
                confirmation_segments,
                key=lambda candidate: abs(
                    float(candidate["centerX"])
                    - center_x
                ),
            )

            confirmation_x = float(
                confirmation_segment["centerX"]
            )

            center_shift = abs(
                confirmation_x - center_x
            )

            if (
                center_shift
                <= GEOMETRIC_GAP_MAX_CENTER_SHIFT_PX
            ):
                valid_candidates.append(
                    segment
                )

        if valid_candidates:
            selected_segment = min(
                valid_candidates,
                key=lambda segment: abs(
                    float(segment["centerX"])
                    - image_center_x
                ),
            )

            start_point = (
                float(
                    selected_segment["centerX"]
                ),
                float(search_y),
            )

            return start_point

        search_y -= (
            GEOMETRIC_GAP_SEARCH_STEP_PX
        )

    return None

def find_next_geometric_path_point(
    distance_map,
    current_point,
    direction,
    target_point=None,
):
    """
    Procura o próximo ponto central da faixa ao redor do ponto atual.

    A busca acontece em uma coroa circular, permitindo que a
    trajetória avance em qualquer direção até 90 graus em relação
    à direção atual.

    O distance transform favorece pontos mais distantes das bordas,
    ou seja, próximos do eixo central da faixa.
    """

    height, width = distance_map.shape[:2]

    current_x = float(current_point[0])
    current_y = float(current_point[1])

    step = GEOMETRIC_TRACE_STEP_PX
    search_range = GEOMETRIC_TRACE_SEARCH_RANGE_PX

    minimum_step = max(
        2.0,
        step - search_range,
    )

    maximum_step = (
        step + search_range
    )

    search_radius = int(
        math.ceil(maximum_step)
    )

    center_x = int(round(current_x))
    center_y = int(round(current_y))

    x0 = max(
        0,
        center_x - search_radius,
    )
    x1 = min(
        width,
        center_x + search_radius + 1,
    )

    y0 = max(
        0,
        center_y - search_radius,
    )
    y1 = min(
        height,
        center_y + search_radius + 1,
    )

    local_distance = distance_map[
        y0:y1,
        x0:x1,
    ]

    if local_distance.size == 0:
        return None

    local_y, local_x = np.nonzero(
        local_distance > 0.0
    )

    if local_x.size == 0:
        return None

    candidate_x = (
        local_x.astype(np.float32)
        + float(x0)
    )

    candidate_y = (
        local_y.astype(np.float32)
        + float(y0)
    )

    delta_x = (
        candidate_x - current_x
    )

    delta_y = (
        candidate_y - current_y
    )

    radial_distance = np.hypot(
        delta_x,
        delta_y,
    )

    valid_distance = (
        (radial_distance >= minimum_step)
        & (radial_distance <= maximum_step)
    )

    safe_distance = np.maximum(
        radial_distance,
        1e-6,
    )

    alignment = (
        delta_x * float(direction[0])
        + delta_y * float(direction[1])
    ) / safe_distance

    minimum_alignment = math.cos(
        math.radians(
            GEOMETRIC_TRACE_MAX_TURN_DEG
        )
    )

    # Tolerância numérica para que uma mudança exatamente
    # perpendicular seja realmente aceita quando o limite é 90°.
    alignment_tolerance = 1e-6

    valid_direction = (
        alignment >= minimum_alignment - alignment_tolerance
    )

    valid = (
        valid_distance
        & valid_direction
    )

    if not np.any(valid):
        return None

    center_strength = local_distance[
        local_y,
        local_x,
    ].astype(np.float32)

    # A distância ao contorno é medida em pixels e pode crescer
    # muito dentro de regiões pretas largas. Normalizamos somente
    # entre os candidatos geometricamente válidos para que uma
    # grande massa preta não domine a continuidade da trajetória.
    maximum_center_strength = float(
        np.max(
            center_strength[valid]
        )
    )

    if maximum_center_strength > 0.0:
        center_strength = (
            center_strength
            / maximum_center_strength
        )

    # Prioridades:
    #
    # 1. manter continuidade com a direção atual da faixa;
    # 2. preferir o centro físico entre caminhos coerentes;
    # 3. manter aproximadamente o passo nominal.
    score = (
        center_strength
        + alignment * 2.0
        - np.abs(
            radial_distance - step
        ) * 0.10
    )

    if target_point is not None:
        target_x = float(
            target_point[0]
        )

        target_y = float(
            target_point[1]
        )

        current_target_distance = (
            math.hypot(
                target_x - current_x,
                target_y - current_y,
            )
        )

        candidate_target_distance = (
            np.hypot(
                target_x - candidate_x,
                target_y - candidate_y,
            )
        )

        # Positivo = candidato aproxima da saída correta.
        # Negativo = candidato se afasta dela.
        target_progress = (
            current_target_distance
            - candidate_target_distance
        )

        score += (
            target_progress
            * GEOMETRIC_TRACE_TARGET_PROGRESS_GAIN
        )

    score[~valid] = -np.inf

    best_index = int(
        np.argmax(score)
    )

    next_x = float(
        candidate_x[best_index]
    )

    next_y = float(
        candidate_y[best_index]
    )

    movement_x = (
        next_x - current_x
    )

    movement_y = (
        next_y - current_y
    )

    movement_length = math.hypot(
        movement_x,
        movement_y,
    )

    if movement_length <= 0.0:
        return None

    next_direction = (
        movement_x / movement_length,
        movement_y / movement_length,
    )

    return (
        (next_x, next_y),
        next_direction,
    )

def estimate_geometric_initial_direction(
    processed_line_mask,
    near_point,
):
    """
    Estima a direção inicial real da faixa a partir do NEAR.

    Usa duas pequenas amostras à frente do NEAR e acompanha
    o segmento mais próximo entre elas. Depois ajusta uma reta
    aos pontos encontrados.

    Caso não exista informação suficiente, mantém o fallback
    seguro apontando para a frente da câmera.
    """

    height, _ = processed_line_mask.shape[:2]

    near_x = float(near_point[0])
    near_y = float(near_point[1])

    direction_points = [
        (
            near_x,
            near_y,
        )
    ]

    previous_x = near_x

    for multiplier in (
        1.0,
        2.0,
    ):
        sample_y = int(round(
            near_y
            - GEOMETRIC_TRACE_STEP_PX
            * multiplier
        ))

        if sample_y < 0:
            break

        if sample_y >= height:
            continue

        segments = find_active_band_segments(
            processed_line_mask,
            sample_y,
        )

        if not segments:
            break

        selected_segment = min(
            segments,
            key=lambda segment: abs(
                segment["centerX"]
                - previous_x
            ),
        )

        sample_x = float(
            selected_segment["centerX"]
        )

        direction_points.append(
            (
                sample_x,
                float(sample_y),
            )
        )

        previous_x = sample_x

    if len(direction_points) < 2:
        return (
            0.0,
            -1.0,
        )

    fit_points = np.asarray(
        direction_points,
        dtype=np.float32,
    ).reshape(-1, 1, 2)

    vx, vy, _, _ = cv2.fitLine(
        fit_points,
        cv2.DIST_L2,
        0,
        0.01,
        0.01,
    ).flatten()

    vx = float(vx)
    vy = float(vy)

    reference_x = (
        direction_points[-1][0]
        - near_x
    )

    reference_y = (
        direction_points[-1][1]
        - near_y
    )

    # O cv2.fitLine não possui sentido definido.
    # Orienta o vetor do NEAR em direção à faixa à frente.
    if (
        vx * reference_x
        + vy * reference_y
    ) < 0.0:
        vx = -vx
        vy = -vy

    length = math.hypot(
        vx,
        vy,
    )

    if length <= 0.0:
        return (
            0.0,
            -1.0,
        )

    return (
        vx / length,
        vy / length,
    )

def project_geometric_gap_to_near(
    path_points,
    near_y,
    frame_width,
):
    """
    Projeta a direção local da faixa reaparecida através do gap
    até a altura fixa do NEAR.

    O ponto retornado é apenas uma estimativa geométrica.
    Ele não representa pixels realmente observados na máscara.
    """

    point_count = min(
        len(path_points),
        GEOMETRIC_GAP_PROJECTION_POINTS,
    )

    if point_count < 2:
        return None

    local_points = np.asarray(
        path_points[:point_count],
        dtype=np.float32,
    ).reshape(-1, 1, 2)

    vx, vy, _, _ = cv2.fitLine(
        local_points,
        cv2.DIST_L2,
        0,
        0.01,
        0.01,
    ).flatten()

    vx = float(vx)
    vy = float(vy)

    first_point = path_points[0]
    last_point = path_points[point_count - 1]

    reference_x = (
        float(last_point[0])
        - float(first_point[0])
    )

    reference_y = (
        float(last_point[1])
        - float(first_point[1])
    )

    # O cv2.fitLine não define o sentido do vetor.
    # Orienta a direção do começo da faixa para o FAR.
    if (
        vx * reference_x
        + vy * reference_y
    ) < 0.0:
        vx = -vx
        vy = -vy

    # Uma direção praticamente horizontal não possui uma
    # interseção estável com a altura fixa do NEAR.
    if abs(vy) <= 1e-6:
        return None

    start_x = float(first_point[0])
    start_y = float(first_point[1])

    scale = (
        float(near_y) - start_y
    ) / vy

    projected_x = (
        start_x + vx * scale
    )

    if not math.isfinite(projected_x):
        return None

    # Se a continuação atingiria o NEAR fora da imagem,
    # não fingimos possuir uma referência utilizável.
    if (
        projected_x < 0.0
        or projected_x > float(frame_width - 1)
    ):
        return None

    return (
        projected_x,
        float(near_y),
    )

def calculate_geometric_far_heading(path_points):
    """
    Calcula a direção local no final da trajetória disponível.

    Não exige que a trajetória alcance o FAR superior:
    também funciona quando ela sai lateralmente da imagem.

    0 graus representa seguir para a frente.
    Valor positivo aponta para a direita.
    Valor negativo aponta para a esquerda.
    """

    point_count = min(
        len(path_points),
        GEOMETRIC_PATH_LOCAL_HEADING_POINTS,
    )

    if point_count < 2:
        return None

    local_points = path_points[
        -point_count:
    ]

    fit_points = np.asarray(
        local_points,
        dtype=np.float32,
    ).reshape(-1, 1, 2)

    vx, vy, _, _ = cv2.fitLine(
        fit_points,
        cv2.DIST_L2,
        0,
        0.01,
        0.01,
    ).flatten()

    vx = float(vx)
    vy = float(vy)

    first_point = local_points[0]
    last_point = local_points[-1]

    reference_x = (
        float(last_point[0])
        - float(first_point[0])
    )

    reference_y = (
        float(last_point[1])
        - float(first_point[1])
    )

    # O fitLine não possui sentido definido.
    # Orienta o vetor no mesmo sentido em que a trajetória
    # foi percorrida, do NEAR em direção ao futuro.
    if (
        vx * reference_x
        + vy * reference_y
    ) < 0.0:
        vx = -vx
        vy = -vy

    return float(
        math.degrees(
            math.atan2(
                vx,
                -vy,
            )
        )
    )

def extract_geometric_line_path(processed_line_mask):
    """
    Rastreia a faixa somente para travessia e reaquisição de GAP.

    Diferentemente da versão baseada em bandas horizontais,
    os pontos intermediários não possuem Y fixo. O caminho pode
    avançar verticalmente, diagonalmente ou lateralmente.

    NEAR e FAR continuam presos às alturas de referência.
    """

    height, width = processed_line_mask.shape[:2]

    far_y = int(round(
        height * GEOMETRIC_PATH_FAR_Y_RATIO
    ))

    near_y = int(round(
        height * GEOMETRIC_PATH_NEAR_Y_RATIO
    ))

    far_y = max(
        0,
        min(height - 1, far_y),
    )

    near_y = max(
        0,
        min(height - 1, near_y),
    )

    # No início, escolhe a faixa mais próxima do centro físico
    # da câmera. Depois disso, a própria continuidade geométrica
    # decide o caminho.
    image_center_x = (
        float(width - 1) / 2.0
    )

    near_segments = find_active_band_segments(
        processed_line_mask,
        near_y,
    )

    gap_reacquired = False
    if near_segments:
        near_segment = min(
            near_segments,
            key=lambda segment: abs(
                float(segment["centerX"])
                - image_center_x
            ),
        )

        start_point = (
            float(near_segment["centerX"]),
            float(near_y),
        )

        # Há observação real exatamente no NEAR.
        near_point = start_point

    else:
        gap_start = find_geometric_gap_start(
            processed_line_mask,
            near_y,
        )

        if gap_start is None:
            return {
                "nearPoint": None,
                "farHeadingDeg": None,
                "virtualNearPoint": None,
                "lateralExitTarget": None,
            }

        start_point = gap_start

        gap_reacquired = True

        # Não fingimos que existe uma medição no NEAR.
        # A trajetória começa onde a faixa reaparece.
        near_point = None

    lateral_exit_target = (
        find_geometric_lateral_exit(
            processed_line_mask,
            start_point,
        )
    )

    # Quanto maior o valor, mais longe este pixel está das bordas
    # da faixa. Os máximos locais formam aproximadamente seu eixo.
    distance_map = cv2.distanceTransform(
        processed_line_mask,
        cv2.DIST_L2,
        3,
    )

    path_points = [
        start_point
    ]

    current_point = start_point

    # A direção inicial é medida na própria geometria da faixa.
    # Depois do primeiro passo, o rastreador continua atualizando
    # a direção normalmente pelos pontos encontrados.
    direction = estimate_geometric_initial_direction(
        processed_line_mask,
        start_point,
    )

    maximum_step = (
        GEOMETRIC_TRACE_STEP_PX
        + GEOMETRIC_TRACE_SEARCH_RANGE_PX
    )

    virtual_near_point = None

    for _ in range(
        GEOMETRIC_TRACE_MAX_POINTS - 1
    ):
        current_x = float(
            current_point[0]
        )

        current_y = float(
            current_point[1]
        )

        if lateral_exit_target is not None:
            distance_to_lateral_exit = math.hypot(
                float(lateral_exit_target[0])
                - current_x,
                float(lateral_exit_target[1])
                - current_y,
            )

            if distance_to_lateral_exit <= maximum_step:
                if distance_to_lateral_exit > 1.0:
                    path_points.append(
                        lateral_exit_target
                    )

                break

        # Quando a trajetória chega suficientemente perto do FAR,
        # tenta conectar ao centro real da faixa exatamente no Y
        # superior de referência.
        if current_y <= (
            far_y + maximum_step
        ):
            far_segments = (
                find_active_band_segments(
                    processed_line_mask,
                    far_y,
                )
            )

            if far_segments:
                selected_far_segment = min(
                    far_segments,
                    key=lambda segment: abs(
                        segment["centerX"]
                        - current_x
                    ),
                )

                candidate_far_point = (
                    float(
                        selected_far_segment[
                            "centerX"
                        ]
                    ),
                    float(far_y),
                )

                distance_to_far = math.hypot(
                    (
                        candidate_far_point[0]
                        - current_x
                    ),
                    (
                        candidate_far_point[1]
                        - current_y
                    ),
                )

                if distance_to_far <= (
                    maximum_step * 2.0
                ):
                    if distance_to_far > 1.0:
                        path_points.append(
                            candidate_far_point
                        )

                    break

        next_result = (
            find_next_geometric_path_point(
                distance_map,
                current_point,
                direction,
                lateral_exit_target,
            )
        )

        if next_result is None:
            break

        next_point, next_direction = (
            next_result
        )

        path_points.append(
            next_point
        )

        current_point = (
            next_point
        )

        direction = (
            next_direction
        )

    if gap_reacquired:
        virtual_near_point = (
            project_geometric_gap_to_near(
                path_points,
                near_y,
                width,
            )
        )

    far_heading = calculate_geometric_far_heading(
        path_points,
    )

    return {
        "nearPoint": near_point,
        "farHeadingDeg": far_heading,
        "virtualNearPoint": virtual_near_point,
        "lateralExitTarget": lateral_exit_target,
    }


def extract_gap_geometric_guidance(
    processed_line_mask,
    gap_forward_active,
    green_direction,
    near_center_visible,
    forward_control_trusted=True,
):
    """Executa a geometria somente quando ela pode participar do GAP."""

    geometry_required = (
        forward_control_trusted
        and (
            gap_forward_active
            or (
                green_direction == "NENHUMA"
                and not near_center_visible
            )
        )
    )
    if not geometry_required:
        return {
            "nearPoint": None,
            "farHeadingDeg": None,
            "virtualNearPoint": None,
            "lateralExitTarget": None,
            "processingMs": 0.0,
        }

    geometric_started = time.perf_counter()
    geometric_guidance = extract_geometric_line_path(
        processed_line_mask
    )
    geometric_guidance["processingMs"] = (
        time.perf_counter() - geometric_started
    ) * 1000.0
    return geometric_guidance

