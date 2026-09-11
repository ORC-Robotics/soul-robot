"""Configuração e perfis das câmeras do robô."""

import os


try:
    import RPi.GPIO as GPIO  # type: ignore
except ImportError:
    GPIO = None

try:
    from libcamera import Transform  # type: ignore[import]
    from picamera2 import Picamera2  # type: ignore[import]
except ImportError:
    Transform = None
    Picamera2 = None


FRAME_PATH = "/tmp/obr_camera_frame.jpg"
TEMP_FRAME_PATH = "/tmp/obr_camera_frame.tmp.jpg"
STATUS_PATH = "/tmp/obr_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_camera_status.tmp.json"
LINE_STATUS_PATH = "/dev/shm/obr_line_status.json"
TEMP_LINE_STATUS_PATH = "/dev/shm/obr_line_status.tmp.json"
GREEN_CAPTURE_REQUEST_PATH = "/dev/shm/obr_green_capture_request"
GREEN_CAPTURE_RAW_PATH = "/dev/shm/obr_green_raw.png"
GREEN_CAPTURE_HSV_MASK_PATH = "/dev/shm/obr_green_hsv_mask.png"
GREEN_CAPTURE_FINAL_MASK_PATH = "/dev/shm/obr_green_final_mask.png"
GREEN_CAPTURE_CANDIDATES_PATH = "/dev/shm/obr_green_candidates.png"
GREEN_CAPTURE_STATS_PATH = "/dev/shm/obr_green_stats.json"

LIGHT_PIN_BOARD = 40
MJPEG_STREAM_PORT = 8090
MJPEG_STREAM_PATH = "/stream.mjpg"
MJPEG_STREAM_FPS = 30
SNAPSHOT_FRAME_FPS = 2
STATUS_FPS = 5
JPEG_QUALITY = 82
CAMERA_PIXEL_FORMATS = ("RGB888",)
VIRTUAL_ROW_MIN_ACTIVATION = 0.10

# O nome RGB888 segue a convenção do libcamera. No array retornado pelo
# Picamera2, cada pixel fica em ordem B, G, R, que é a ordem nativa do OpenCV.
CAMERA_ARRAY_COLOR_ORDER = "BGR"
DEFAULT_CAMERA_INDICES = {
    "down": 0,
    "forward": 1,
}
CAMERA_INDEX_ENVIRONMENT = {
    "down": "OBR_DOWNWARD_CAMERA_INDEX",
    "forward": "OBR_FORWARD_CAMERA_INDEX",
}
DISPLAY_MODE_REAL = "real"
DISPLAY_MODE_LINE = "line"
DISPLAY_MODE_GREEN = "green"
DISPLAY_MODES = (
    DISPLAY_MODE_REAL,
    DISPLAY_MODE_LINE,
)


def environment_flag(name, default):
    """Lê uma flag booleana de ambiente sem aceitar valores ambíguos."""

    value = os.environ.get(name)
    if value is None:
        return bool(default)
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"{name} deve ser 0/1, true/false, yes/no ou on/off.")


def configured_camera_indices(environment=None):
    """Lê os dois índices físicos sem permitir fallback ou papéis duplicados."""

    environment = os.environ if environment is None else environment
    indices = {}
    for role, variable_name in CAMERA_INDEX_ENVIRONMENT.items():
        raw_value = environment.get(
            variable_name,
            str(DEFAULT_CAMERA_INDICES[role]),
        )
        try:
            camera_index = int(raw_value)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{variable_name} deve ser um índice inteiro não negativo."
            ) from error
        if camera_index < 0:
            raise ValueError(
                f"{variable_name} deve ser um índice inteiro não negativo."
            )
        indices[role] = camera_index

    if indices["down"] == indices["forward"]:
        raise ValueError(
            "As câmeras inferior e frontal não podem usar o mesmo índice."
        )
    return indices


def camera_number(camera_info, fallback_index):
    """Obtém o número enumerado sem deduzir o papel pelo modelo do sensor."""

    value = camera_info.get("Num", fallback_index)
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(fallback_index)


def resolve_camera_assignments(camera_infos, camera_indices):
    """Associa índices configurados aos papéis sem trocar câmeras ausentes."""

    enumerated = {
        camera_number(camera_info, fallback_index): camera_info
        for fallback_index, camera_info in enumerate(camera_infos)
    }
    return {
        role: {
            "role": role,
            "index": camera_index,
            "available": camera_index in enumerated,
            "info": enumerated.get(camera_index),
        }
        for role, camera_index in camera_indices.items()
    }


def require_camera_assignment(assignments, role):
    """Falha claramente se a câmera exigida para o papel não foi enumerada."""

    assignment = assignments[role]
    if not assignment["available"]:
        raise RuntimeError(
            f"Câmera {role} configurada no índice {assignment['index']} "
            "não foi encontrada; não haverá troca automática de papel."
        )
    return assignment


def camera_role_publishes_line_status(role):
    """Restringe o IPC do segue-faixa ao papel físico da câmera inferior."""

    return role == "down"


def log_camera_inventory(camera_infos, assignments):
    """Registra inventário, identificador e papel configurado de cada câmera."""

    if not camera_infos:
        print("Nenhuma câmera foi enumerada pelo Picamera2.", flush=True)
    for fallback_index, camera_info in enumerate(camera_infos):
        camera_index = camera_number(camera_info, fallback_index)
        print(
            "Câmera enumerada: "
            f"índice={camera_index}, modelo={camera_info.get('Model', '')}, "
            f"identificador={camera_info.get('Id', '')}, "
            f"localização={camera_info.get('Location', '')}, "
            f"rotação={camera_info.get('Rotation', '')}.",
            flush=True,
        )
    for role in ("down", "forward"):
        assignment = assignments[role]
        availability = "disponível" if assignment["available"] else "indisponível"
        print(
            f"Papel configurado: {role}=índice {assignment['index']} "
            f"({availability}).",
            flush=True,
        )


# Esta chave permite desativar apenas o diagnóstico verde sem alterar a câmera.
GREEN_PROCESSING_ENABLED = environment_flag("GREEN_PROCESSING_ENABLED", True)

# Mantém o classificador de prata ativo na câmera inferior. Quatro positivos
# consecutivos publicam o marcador cinza; a missão principal apenas para o robô.
# A variável permite desligar a inferência durante diagnóstico da câmera.
SILVER_DETECTION_ENABLED = environment_flag("SILVER_DETECTION_ENABLED", True)

# O diagnóstico legado é opt-in porque o extractor por scanlines e seus
# desenhos aumentam o custo e escondem o vetor Fusion-style no uso normal.
# Ativar esta flag não muda o controle; apenas restaura cálculo e overlay antigos.
LEGACY_LINE_DEBUG_ENABLED = environment_flag(
    "LEGACY_LINE_DEBUG_ENABLED",
    False,
)

# A câmera da pista mostrou que o branco sob iluminação esverdeada chega a
# saturação 136. Exigir 140 preserva o cartão verde saturado e bloqueia esse
# falso positivo antes de qualquer geometria ou decisão autônoma.
GREEN_HUE_MIN = 40
GREEN_HUE_MAX = 90
GREEN_SATURATION_MIN = 140
GREEN_VALUE_MIN = 60
GREEN_OPEN_KERNEL_SIZE = 5
GREEN_CLOSE_KERNEL_SIZE = 5
GREEN_OPEN_ITERATIONS = 2
GREEN_CLOSE_ITERATIONS = 2
# O detector de referência exige mais de 3.000 pixels verdes em 320×200.
# Este limite aceita o marcador oficial observado na pista, enquanto os filtros
# de HSV, formato e associação com a faixa continuam bloqueando falsos verdes.
LINE_MIN_COMPONENT_AREA_PX = 120
LINE_MIN_COMPONENT_THICKNESS_PX = 11.0
LINE_MIN_COMPONENT_CORE_RATIO = 0.15
GREEN_MIN_AREA_RATIO = 1900.0 / (320.0 * 200.0)
GREEN_MIN_AREA_PX = 80.0
GREEN_MIN_DIMENSION_PX = 6.0
GREEN_ASPECT_RATIO_MIN = 0.35
GREEN_ASPECT_RATIO_MAX = 1.0
GREEN_MIN_EXTENT = 0.35
GREEN_PARTIAL_BORDER_TOLERANCE_PX = 4
GREEN_PARTIAL_AREA_FACTOR = 0.40
GREEN_PARTIAL_DIMENSION_FACTOR = 0.50
GREEN_PARTIAL_ASPECT_RATIO_MIN = 0.20
GREEN_PARTIAL_EXTENT_MIN = 0.20
GREEN_FRAGMENT_MERGE_DISTANCE_PX = 12
GREEN_CONFIRMATION_FRAMES = 2
GREEN_SINGLE_OBSERVATION_FRAMES = 2
GREEN_CLEAR_HYSTERESIS_FRAMES = 2
# Mantém por no máximo dois frames o último comando Fusion já aceito enquanto
# um candidato verde aguarda confirmação. O limite evita um hold indefinido.
GREEN_CANDIDATE_HOLD_MAX_FRAMES = 2
# Mantém por no máximo dois frames o último target Fusion GREEN válido.
# Depois disso, a recuperação existente assume no lado indicado pelo marcador.
GREEN_FUSION_TARGET_HOLD_MAX_FRAMES = 2
# Esta medida-base equivale a 5% da largura do frame e dimensiona as duas ROIs
# sem prender o detector a uma resolução específica.
GREEN_ROI_HALF_SIZE_DIVISOR = 20
# A ROI superior recebe 25% a mais de largura para encontrar a faixa em curvas
# ou pequenos desalinhamentos. A altura e a ROI horizontal não são alteradas.
GREEN_UPPER_ROI_HALF_WIDTH_SCALE = 1.25
# Pelo menos metade da ROI nominal deve existir dentro da imagem. Uma amostra
# menor poderia aceitar ruído de borda como se fosse a faixa preta.
GREEN_ROI_MIN_VISIBLE_RATIO = 0.50
# Fração mínima de preto na ROI superior, que valida a associação com a faixa.
GREEN_ROI_MIN_BLACK_RATIO = 0.25
# A ROI lateral aceita uma fração menor porque a faixa ocupa uma área triangular
# quando o robô chega inclinado. Isso altera somente a direção do verde.
GREEN_SIDE_ROI_MIN_BLACK_RATIO = 0.18
# Mantém a orientação durante meio segundo depois da última leitura válida.
GREEN_DIRECTION_RETENTION_SECONDS = 0.3
# Dois marcadores só representam retorno quando estão na mesma altura local.
# A tolerância usa a maior altura observada para acompanhar a perspectiva.
GREEN_PAIR_MAX_VERTICAL_DISTANCE_HEIGHTS = 1.5

# O LED físico pode criar pequenos reflexos brancos dentro da fita preta. Este
# reparo atua somente em ilhas claras completamente cercadas pela máscara preta;
# jamais fecha uma abertura ligada ao fundo, pois ela pode ser uma interrupção real.
SPECULAR_REPAIR_REFERENCE_FRAME_HEIGHT = 360.0
SPECULAR_REPAIR_MAX_DIAMETER_PX = 12.0
SPECULAR_REPAIR_MIN_VALUE = 180
SPECULAR_REPAIR_MAX_SATURATION = 60

VIRTUAL_HEADING_FULL_SCALE_DEG = 30.0
VIRTUAL_HEADING_GAIN = 0.55

# A posição fina corrige apenas pequenos desvios que ainda cabem no sensor
# CENTER. O limite impede que essa correção alcance sozinha STRONG ou PIVOT.
VIRTUAL_FINE_CENTER_GAIN = 0.15
# A deadband elimina ruído perto do centro. O remapeamento contínuo fora dela
# preserva o alcance completo de -1,0 a +1,0 sem criar um salto no limite.
VIRTUAL_FINE_CENTER_DEADBAND = 0.09
VIRTUAL_FINE_CENTER_MAX_CORRECTION = 0.9

# A correção normal alcança toda a diferença de potência em 0,36.
# Somente MEDIUM a partir de 0,25 libera a faixa forte entre 0,36 e 0,45.
NORMAL_FULL_STEERING_ERROR = 0.36

# Potências do mapper NORMAL compartilhado pelo seguidor inferior e pela
# publicação frontal. Manter uma única função evita que a câmera auxiliar crie
# outra curva de potência ou alcance PIVOT, SPIN e ré.
NORMAL_BASE_POWER = 0.75
NORMAL_MAX_POWER = 0.82
NORMAL_INNER_MIN_POWER = 0.66

# Endpoint compartilhado pelos pivots existentes e pela curva contínua Fusion.
# A leve componente de avanço favorece a reacquisição sem liberar potência
# fora do intervalo normalizado aceito pelos motores.
PIVOT_OUTER_POWER = 0.78
PIVOT_INNER_POWER = -0.72

# O Fusion-style ignora desvios de até dois graus e alcança toda a autoridade
# NORMAL em dez graus. A curva cúbica mantém suavidade perto do centro e cresce
# depois sem criar memória entre frames.
FUSION_STEERING_DEADBAND_DEG = 3.5
FUSION_FULL_NORMAL_STEERING_DEG = 10.0

# A curva Fusion mantém ambas as rodas positivas até vinte e cinco graus, chega
# perto de zero aos quarenta, aplica ré leve aos cinquenta e cinco e só alcança
# o pivot máximo aos setenta graus. Cada trecho usa smoothstep e não cria estado.
FUSION_STRONG_POSITIVE_END_DEG = 25.0
FUSION_NEAR_ZERO_END_DEG = 40.0
FUSION_LIGHT_REVERSE_END_DEG = 55.0
FUSION_FULL_PIVOT_STEERING_DEG = 70.0

# Pontos da curva: erro angular, potência externa e potência interna. Estes
# valores são os pontos de calibração do Soul; a interpolação entre eles é
# contínua e os comandos finais permanecem limitados ao intervalo normalizado.
FUSION_POWER_CURVE = (
    (0.0, NORMAL_BASE_POWER, NORMAL_BASE_POWER),
    (
        FUSION_STEERING_DEADBAND_DEG,
        NORMAL_BASE_POWER,
        NORMAL_BASE_POWER,
    ),
    (
        FUSION_FULL_NORMAL_STEERING_DEG,
        NORMAL_MAX_POWER,
        NORMAL_INNER_MIN_POWER,
    ),
    (FUSION_STRONG_POSITIVE_END_DEG, 0.85, 0.30),
    (FUSION_NEAR_ZERO_END_DEG, 0.83, 0.02),
    (FUSION_LIGHT_REVERSE_END_DEG, 0.81, -0.20),
    (
        FUSION_FULL_PIVOT_STEERING_DEG,
        PIVOT_OUTER_POWER,
        PIVOT_INNER_POWER,
    ),
)

# A histerese impede alternância rápida entre a faixa forte e o pivot.
# A entrada exige erro 0,45; a saída ocorre somente após cair até 0,35.
PIVOT_ENTER_THRESHOLD = 0.45
PIVOT_EXIT_THRESHOLD = 0.35

# O MEDIUM promove a urgência da curva somente no seguimento LINE normal.
# Em 0,45, NEAR válido autoriza PIVOT; NEAR perdido autoriza SPIN. A saída em
# 0,30 evita alternância sem reter direção quando a leitura muda ou desaparece.
VIRTUAL_MEDIUM_STRONG_THRESHOLD = 0.25
VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD = 0.45
VIRTUAL_MEDIUM_SPIN_EXIT_THRESHOLD = 0.30
VIRTUAL_MEDIUM_SPIN_POWER = 0.72
# Abaixo deste valor, o MEDIUM ainda pode estar ativo, mas não fornece uma
# direção lateral confiável durante uma ação crítica já iniciada.
VIRTUAL_MEDIUM_DIRECTION_LOST_THRESHOLD = 0.15
# Uma ação crítica já iniciada tolera dez frames sem direção útil do MEDIUM.
# Depois disso, nenhuma direção antiga pode impedir o normal ou o recovery.
VIRTUAL_MEDIUM_CRITICAL_INVALID_MAX_FRAMES = 10

# O hard corner devolve o controle após a faixa permanecer centralizada no
# NEAR e no MEDIUM ou após o FAR reaparecer por dois quadros consecutivos.
VIRTUAL_HARD_CORNER_NEAR_RECOVERY_THRESHOLD = 0.18
VIRTUAL_HARD_CORNER_MEDIUM_RECOVERY_THRESHOLD = 0.20
VIRTUAL_HARD_CORNER_RECOVERY_FRAMES = 2
VIRTUAL_HARD_CORNER_FAR_RECOVERY_FRAMES = 2
# Em 30 FPS, este limite encerra o giro após aproximadamente um segundo.
# Ao expirar, o recovery existente volta a decidir sem iniciar outra busca.
VIRTUAL_HARD_CORNER_MAX_FRAMES = 30

PIVOT_STATE_NONE = "NONE"
PIVOT_STATE_LEFT = "LEFT"
PIVOT_STATE_RIGHT = "RIGHT"

GREEN_OBSERVATION_STATES = {
    "SEM_VERDE",
    "UM_CANDIDATO",
    "DOIS_CANDIDATOS",
    "MULTIPLOS_AMBIGUOS",
}
GREEN_INTERPRETATIONS = {
    "SEM_DECISAO",
    "ESQUERDA",
    "DIREITA",
    "RETORNO_180",
    "VERDE_FALSO",
    "AMBIGUO",
}
VISIBLE_GREEN_INTERPRETATIONS = {
    "ESQUERDA",
    "DIREITA",
    "RETORNO_180",
}

# A câmera inferior corrige sua montagem pelo Picamera2. A frontal mantém a
# captura neutra e aplica sua rotação no próprio processo, pois a OV5647 não
# entregou o flip de captura de forma consistente nos testes reais.
# Manter a transformação no Picamera2 evita rotacionar cada frame no OpenCV.
CAMERA_ROTATION_DEGREES = 180

# Ajustes básicos da imagem recebida pela segmentação e pela visualização.
# Alterá-los muda o contraste e as cores usados pelos detectores.
CAMERA_SHARPNESS = 1.2
CAMERA_CONTRAST = 1.05
CAMERA_SATURATION = 1.0
CAMERA_EXPOSURE_VALUE = 0.4

# Referência da geometria inferior validada em 640×480. Os kernels e limites
# em pixels são redimensionados pela altura real do frame para não ficarem
# presos a uma resolução específica.
DOWNWARD_REFERENCE_FRAME_HEIGHT = 480

# Cada papel define de forma independente a captura e os parâmetros visuais.
# O perfil inferior não herda ROIs nem limites em pixels da câmera frontal.
CAMERA_PROFILES = {
    "forward": {
        "role": "forward",
        "rotation_degrees": 0,
        "main_size": (960, 540),
        "sensor_size": (1920, 1080),
        "sensor_bit_depth": 10,
        "target_fps": 30,
        "vision": {
            "line_roi_start_ratio": 0.0,
            "line_threshold": 100,
            "open_kernel_size": 3,
            "close_kernel_size": 5,
            "full_line_min_short_side_ratio": 50.0 / 540.0,
            "overlay_line_thickness": 2,
            "overlay_thin_line_thickness": 1,
            "debug_text_overlay": False,
        },
    },
    "down": {
        "role": "down",
        "rotation_degrees": CAMERA_ROTATION_DEGREES,
        "main_size": (480, 360),
        "sensor_size": (1640, 1232),
        "sensor_bit_depth": 10,
        "target_fps": 30,
        "vision": {
            "line_roi_start_ratio": 0.0,
            # O fechamento grande estima a claridade do piso ao redor da fita
            # sob os LEDs. A comparação local reduz a aceitação de sombras.
            # Valor de referência em 640×480. A criação da máscara escala para
            # a altura recebida: em 480×360, 201 vira 151.
            "line_background_kernel_size": 201,
            # O limiar relativo fica 39% abaixo do fundo local, respeitando
            # os limites de cinza abaixo. Calibração: calibration/report/RESULTS.md.
            # Aumentar este valor aceita linhas com menos contraste, mas também
            # aumenta o risco de aceitar sombras como parte da linha.
            "line_max_background_ratio_percent": 61,
            # Piso do limiar em cinza de 8 bits. Preserva a fita escura quando
            # ela ocupa a borda do campo e contamina a estimativa do fundo.
            # Aumentar demais aceita sujeira; zero desativa essa proteção.
            "line_min_threshold": 40,
            # Compensa o padrão fixo de sombra causado pelo robô e pelos LEDs.
            # O ganho atua somente no cinza da segmentação preta; RGB, verde,
            # prata e controles de câmera continuam recebendo o frame original.
            "line_illumination_correction_enabled": True,
            "line_illumination_reference_path": (
                "calibration/down_line_illumination_480x360.png"
            ),
            # O hash impede usar silenciosamente um mapa alterado ou corrompido.
            "line_illumination_reference_sha256": (
                "21db98d4f1af631b1648fcf0dbabdb5b"
                "4663f2671878dfc7a7b48dd2d46e1608"
            ),
            "line_illumination_target_percentile": 85.0,
            # Ganhos acima de 2× amplificam ruído sem melhorar o replay atual.
            "line_illumination_max_gain": 2.0,
            # A área acima do FAR não participa do controle da linha.
            "line_illumination_useful_start_ratio": 0.13,
            # O overlay inclui a penumbra e oito pixels de margem, sem excluir
            # nenhum pixel da segmentação ou proibir fita real nessa região.
            "line_illumination_overlay_gain_threshold": 1.10,
            "line_illumination_overlay_margin_px": 8,
            # Retira o verde já detectado antes de filtrar componentes pretos.
            # A máscara estrutural original e o detector de verde são preservados.
            "line_exclude_green": True,
            # Mesmo com contraste local, tons acima deste limite não são pretos.
            # A unidade é o nível de cinza de 8 bits, entre 0 e 255. Aumentar o
            # limite aceita sombras; reduzir demais pode perder uma fita clara.
            "line_max_brightness": 190,
            # Valores de referência em 640×480; são sempre escalados para
            # kernels ímpares antes da morfologia (17 vira 13 e 11 vira 9).
            "open_kernel_shape": "ellipse",
            "open_kernel_size": 17,
            # O fechamento efetivo 9×9 preenche pequenas falhas. Aumentá-lo
            # pode unir sujeira à fita ou apagar a separação de um gap.
            "close_kernel_size": 11,
            # Vinte pixels mantêm aproximadamente a mesma espessura angular
            # mínima do perfil frontal após o aumento de campo de visão.
            "full_line_min_short_side_ratio": 20.0 / 480.0,
            # Fração máxima da imagem ocupada por um componente aceito.
            # O valor 1,0 mantém esse limite desativado; reduzi-lo pode rejeitar
            # interseções ou a fita muito próxima da câmera.
            "full_line_max_area_ratio": 1.0,
            # As coordenadas usam o frame de referência 640×480 validado.
            # A conversão centralizada mantém a mesma geometria proporcional se
            # a altura real do frame for diferente durante um diagnóstico.
            "geometry_reference": {
                "frame_height": 480,
                "structural_end_y": 480,
                # O verde mantém a ROI vertical usada em sua calibração.
                # Ampliar a máscara preta não deve mudar seus filtros.
                "green_end_y": 400,
            },
            "green_detection_enabled": True,
            "overlay_line_thickness": 2,
            "overlay_thin_line_thickness": 1,
            "debug_text_overlay": False,
        },
    },
}

VIRTUAL_FAR_Y0 = 0.13
VIRTUAL_FAR_Y1 = 0.43

"""rois virtuais para o seguidor de linha, em coordenadas normalizadas"""
VIRTUAL_NEAR_Y0 = 0.82
VIRTUAL_NEAR_Y1 = 1.0

# As duas novas bandas dividem somente a inteligência de curva. O FAR legado
# acima continua cobrindo 0,00–0,54 diretamente e não é reconstruído por elas.
VIRTUAL_FAR_BAND_Y0 = 0.13
VIRTUAL_FAR_BAND_Y1 = 0.27
VIRTUAL_MEDIUM_Y0 = 0.43
# Os três blocos superiores terminam juntos antes do corredor do NEAR-C.
VIRTUAL_MEDIUM_Y1 = 0.82
# As asas laterais observam curvas fechadas sem cobrir o corredor do NEAR-C.
VIRTUAL_MEDIUM_WING_Y0 = 0.82
VIRTUAL_MEDIUM_WING_Y1 = 0.94

# Divisão horizontal preservada pelo FAR BAND e pelo NEAR-C.
#
# Existe uma pequena sobreposição entre L/C e C/R.
#
# 0.00                                      1.00
# ├──────── L ────────┤
#              ├──────── C ────────┤
#                           ├──────── R ────────┤ - isx art

VIRTUAL_LEFT_X0 = 0.04
VIRTUAL_LEFT_X1 = 0.385

VIRTUAL_CENTER_X0 = 0.385
VIRTUAL_CENTER_X1 = 0.615

VIRTUAL_RIGHT_X0 = 0.615
VIRTUAL_RIGHT_X1 = 0.96

# Na parte superior do MEDIUM, os três blocos se encontram sem lacunas.
# O CENTER estreito mede continuidade; LEFT e RIGHT medem direção local.
VIRTUAL_MEDIUM_LEFT_X0 = 0.04
VIRTUAL_MEDIUM_LEFT_X1 = 0.43

VIRTUAL_MEDIUM_CENTER_X0 = 0.43
VIRTUAL_MEDIUM_CENTER_X1 = 0.57

VIRTUAL_MEDIUM_RIGHT_X0 = 0.57
VIRTUAL_MEDIUM_RIGHT_X1 = 0.96

# O FAR legado usa quase toda a largura útil da imagem para antecipar a faixa
# sem reutilizar a geometria horizontal exclusiva do MEDIUM.
VIRTUAL_FAR_LEFT_X0 = 0.00
VIRTUAL_FAR_LEFT_X1 = 0.385

VIRTUAL_FAR_CENTER_X0 = 0.385
VIRTUAL_FAR_CENTER_X1 = 0.615

VIRTUAL_FAR_RIGHT_X0 = 0.615
VIRTUAL_FAR_RIGHT_X1 = 1.0

# A espessura transversal continua sendo a principal evidência. A consistência
# recebe peso moderado e todos os pesos permanecem fáceis de calibrar.
VIRTUAL_LINE_CONFIDENCE_THICKNESS_WEIGHT = 0.60
VIRTUAL_LINE_CONFIDENCE_CONSISTENCY_WEIGHT = 0.20
VIRTUAL_LINE_CONFIDENCE_CONTINUITY_WEIGHT = 0.15
VIRTUAL_LINE_CONFIDENCE_AREA_WEIGHT = 0.05

# A mediana do quarto superior dos raios evita que os pixels de borda reduzam
# a medida e que um único pico isolado infle a espessura do componente.
VIRTUAL_LINE_THICKNESS_CORE_PERCENTILE = 75.0

# Uma dispersão robusta igual a metade da espessura mediana zera o score.
# Valores menores produzem uma transição linear até a consistência máxima.
VIRTUAL_LINE_CONSISTENCY_MAX_RELATIVE_DISPERSION = 0.50

# Larguras esperadas da fita no frame 480x360. A interpolação entre topo e
# base compensa a perspectiva e deixa os dois extremos fáceis de calibrar.
VIRTUAL_LINE_EXPECTED_THICKNESS_TOP_PX = 12.0
VIRTUAL_LINE_EXPECTED_THICKNESS_BOTTOM_PX = 32.0
VIRTUAL_LINE_CONFIDENCE_REFERENCE_WIDTH_PX = 480.0

# Distância vertical entre amostras no frame 480x360. O passo acompanha a
# altura real do frame para manter custo e densidade semelhantes em testes.
VIRTUAL_LINE_CONFIDENCE_REFERENCE_HEIGHT_PX = 360.0
VIRTUAL_LINE_CONFIDENCE_SAMPLE_STEP_PX = 4.0

# A trajetória NORMAL experimental usa várias scanlines entre NEAR e FAR.
# O passo acompanha a altura do frame para manter densidade e custo previsíveis.
NORMAL_TRAJECTORY_REFERENCE_HEIGHT_PX = 360.0
NORMAL_TRAJECTORY_SCAN_STEP_PX = 6.0
NORMAL_TRAJECTORY_BAND_HALF_HEIGHT_PX = 1.0

# Cada segmento deve ter largura compatível com a fita esperada naquela altura.
# A margem superior aceita inclinação e perspectiva, mas rejeita regiões largas
# demais, como sombras, cruzamentos inteiros ou partes do chassi.
NORMAL_TRAJECTORY_MIN_WIDTH_FACTOR = 0.35
NORMAL_TRAJECTORY_MAX_WIDTH_FACTOR = 4.0

# A continuação pode deslocar-se lateralmente até três pixels por pixel de
# avanço vertical. Esse limite aceita curvas fortes sem saltar para outro ramo.
NORMAL_TRAJECTORY_MAX_SLOPE_X_PER_Y = 3.0
NORMAL_TRAJECTORY_MAX_SHIFT_WIDTH_RATIO = 0.18
NORMAL_TRAJECTORY_MAX_MISSING_SCANLINES = 2

# Um caminho curto permanece visível no diagnóstico, mas não recebe o estado
# válido. Isso evita apresentar um pequeno fragmento como trajetória completa.
NORMAL_TRAJECTORY_MIN_POINTS = 8
NORMAL_TRAJECTORY_MIN_VERTICAL_COVERAGE = 0.25

# O diagnóstico inspirado no FusionZero resume o topo do contorno em poucas
# amostras. A âncora inferior mantém 1/20 da imagem, enquanto o target usa uma
# faixa moderadamente menor para representar antes a extremidade futura.
FUSION_STYLE_NEAR_BAND_HEIGHT_RATIO = 1.0 / 20.0
FUSION_STYLE_TARGET_BAND_HEIGHT_RATIO = 1.0 / 24.0

# A busca pode avançar até um quarto da imagem para sair da ponta afilada.
# Uma variação de até 10% na largura ainda representa uma faixa estável.
FUSION_STYLE_TARGET_SEARCH_HEIGHT_RATIO = 1.0 / 4.0
FUSION_STYLE_TRANSVERSE_WIDTH_STABILITY_RATIO = 0.10

# O guard só existe durante pivot extremo. Ele bloqueia uma inversão súbita de
# lado causada pela rotação da imagem e libera no primeiro target frontal claro.
FUSION_EXTREME_PIVOT_GUARD_ENTER_ERROR_DEG = 55.0
FUSION_EXTREME_PIVOT_GUARD_RELEASE_ERROR_DEG = 25.0

# Estes limites classificam apenas a consistência da observação; não filtram o
# ângulo nem participam da seleção do target. Valores menores recuperam a
# velocidade mais devagar após uma mudança.
FUSION_TARGET_CONSISTENCY_ANGLE_SCALE_DEG = 60.0
FUSION_TARGET_CONSISTENCY_SHIFT_WIDTH_RATIO = 0.25
FUSION_TARGET_HISTORY_MAX_MISSED_FRAMES = 2
FUSION_TARGET_STABLE_ANGLE_DELTA_DEG = 12.0
FUSION_TARGET_STABLE_SHIFT_WIDTH_RATIO = 0.12
FUSION_TARGET_SHORT_LENGTH_RATIO = 0.35
FUSION_TARGET_ESTABLISHED_LENGTH_RATIO = 0.75
FUSION_TARGET_STABLE_FRAMES_FOR_FULL_SPEED = 6
FUSION_STRONG_CORRECTION_ERROR_DEG = 25.0

# Uma curva que deixa o NEAR pela lateral pode trocar de nearCenter para
# deepestFallback sem representar um GAP. Só conservamos essa continuação
# quando o target já vinha estável e permaneceu geometricamente consistente.
FUSION_LATERAL_CONTINUATION_MIN_STABLE_FRAMES = 3
FUSION_LATERAL_CONTINUATION_MIN_CONSISTENCY = 0.75

# A recuperação começa no menor avanço que move as duas rodas e cresce até a
# potência NORMAL. Este limite atua somente na recuperação da velocidade; o
# diferencial contínuo do mapper Fusion permanece inalterado.
FUSION_MIN_FORWARD_POWER = 0.69
FUSION_MIN_FORWARD_SPEED_SCALE = (
    FUSION_MIN_FORWARD_POWER / NORMAL_BASE_POWER
)

# Os limiares de trust bloqueiam candidatos fracos antes que FAR ou MEDIUM
# participem do controle. Cada fileira permanece calibrável separadamente.
FAR_TRUST_MIN_CONFIDENCE = 0.75
FAR_TRUST_MIN_THICKNESS_PX = 18.0

MEDIUM_TRUST_MIN_CONFIDENCE = 0.75
MEDIUM_TRUST_MIN_THICKNESS_PX = 22.0

# Este veto continua valendo mesmo se os demais limiares forem reduzidos em
# uma calibração futura. Componentes tão finos não podem dirigir o robô.
LINE_TRUST_ABSOLUTE_THIN_VETO_PX = 10.0

# O scan usa somente uma direção clara do MEDIUM e termina após três frames.
VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD = 0.20
VIRTUAL_MEDIUM_SCAN_MAX_FRAMES = 3

# MEDIUM e FAR BAND precisam concordar claramente antes do pivot de recovery.
VIRTUAL_REORIENT_DIRECTION_THRESHOLD = 0.20
VIRTUAL_REORIENT_CONFIRMATION_FRAMES = 4

# O steering normal precisa reaparecer em dois frames antes de encerrar o
# estado persistente, mas recebe autoridade já no primeiro frame válido.
VIRTUAL_REORIENT_RECOVERY_FRAMES = 2

VIRTUAL_STATE_NORMAL = "NORMAL"
VIRTUAL_STATE_REORIENT_LEFT = "REORIENT_LEFT"
VIRTUAL_STATE_REORIENT_RIGHT = "REORIENT_RIGHT"

# Controle da prioridade de direção após um verde confirmado.
#
# A direção permanece memorizada durante toda a curva.
# ESQUERDA e DIREITA usam os sensores virtuais para selecionar
# somente o ramo permitido da interseção.
#
# Após terminar a curva, novos verdes continuam bloqueados até que nenhum
# candidato verde seja visto por esta quantidade de quadros consecutivos.
# Durante a manobra ativa, estes quadros não podem ser acumulados.
QUADROS_PARA_REARMAR_VERDE = 5

# A curva é considerada iniciada quando a posição fina local se desloca
# suficientemente para o lado escolhido, com presença confirmada no NEAR-C.
LIMIAR_CURVA_VERDE_INICIADA = 0.20

# Limita o pivô obrigatório usado apenas para iniciar a entrada no ramo verde.
# Em 30 FPS, oito quadros correspondem a aproximadamente 270 milissegundos.
GREEN_ENTRY_PIVOT_MAX_FRAMES = 8
# O alvo FAR precisa apontar claramente para o ramo escolhido antes de receber
# o controle. Valores menores ainda podem representar a faixa reta do cruzamento.
GREEN_ENTRY_FUSION_MIN_STEERING = 0.10

# Após a curva ter começado, o retorno da posição fina para esta região central
# indica que o robô entrou e se alinhou com a nova faixa.
LIMIAR_CENTRALIZACAO_VERDE = 0.18

# Evita encerrar a prioridade por uma leitura central isolada.
QUADROS_CENTRALIZADO_PARA_CONCLUIR = 3

# Limite lateral aceito em uma fileira trusted para confirmar que o ramo
# escolhido pelo verde já migrou para a região central.
GREEN_TRUSTED_POSITION_CENTER_LIMIT = 0.30

# A manobra verde não pode manter a máscara de controle indefinidamente.
# Em 30 FPS, 45 frames correspondem a aproximadamente 1,5 segundo. Este
# timeout é apenas uma proteção; a conclusão normal depende da geometria.
GREEN_MANEUVER_TIMEOUT_FRAMES = 45

# A busca cega começa no último lado confiável por uma janela curta e depois
# varre o lado oposto por mais tempo. O ciclo se repete até a linha reaparecer.
VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES = 35
VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES = 50
# Antes da busca automática em LINE, uma ré curta afasta o robô do ponto em
# que perdeu a faixa. A potência usa o menor valor confiável para mover o Soul.
VIRTUAL_BLIND_SEARCH_BACKUP_FRAMES = 5
VIRTUAL_BLIND_SEARCH_BACKUP_POWER = -0.69
# Dois frames sem orientação evitam entrar em busca por uma perda isolada.
# GREEN e GAP continuam usando start() diretamente e não recebem este atraso.
VIRTUAL_BLIND_SEARCH_CONFIRMATION_FRAMES = 2

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

# O gate de GAP observa toda a largura do NEAR. Como recebe a máscara final já
# filtrada, qualquer pixel aceito nessa faixa ainda representa linha disponível
# para o Fusion e impede a entrada prematura em GAP. O NEAR-C usado no steering,
# na centralização e no verde mantém sua geometria estreita original.
NEAR_VIRTUAL_SENSOR_CONFIG = {
    "roi_y0": VIRTUAL_NEAR_Y0,
    "roi_y1": VIRTUAL_NEAR_Y1,
    "roi_x0": 0.0,
    "roi_x1": 1.0,
    # A filtragem de componentes ocorre antes deste gate. Exigir ausência total
    # evita classificar como GAP uma ponta legítima que acabou de entrar no NEAR.
    "minimum_active_pixels": 1,
}

# A frontal confirma fita presente, mesmo inclinada ou lateral após uma curva.
# A continuidade interna importa; acertar a extrapolação inferior não é requisito.
FORWARD_PRESENCE_CONFIG = {
    "bands": 7, "min_bands": 3, "max_missing_bands": 1,
    "min_width": 0.008, "max_width": 0.15,  # Frações da largura frontal.
    "min_extent": 0.07, "min_elongation": 1.4,
    "clipped_elongation": 0.75,  # Não exige comprimento invisível além da borda.
    "max_slope": 3.0,
    "min_area": 120.0 / (960 * 960), "max_components": 16,
    "uncertain_threshold": 0.60,
    "near_fraction": 0.60,  # Prioriza os 60% mais próximos da ROI frontal.
    "near_bands": 2,  # Exige suporte próximo, além de um único pixel na base.
}

# Validação frontal: posições usam a largura inteira de cada imagem, com zero
# no eixo do robô. As escalas são projeções heurísticas, não uma homografia medida.
FORWARD_PATH_CONFIG = {
    "bottom_x_scale": 1.0,  # Ajustar se os FOVs tiverem escalas laterais diferentes.
    "projection_near_offset": 0.0,  # Distância além do topo inferior, em sua escala.
    "projection_depth_scale": 0.45,  # Avanço frontal na escala vertical inferior.
}

# Janelas em segundos. Decisões usam relógio monotônico; idades do IPC usam Unix.
# Frames duplicados não confirmam GAP. Uma posição FAR inferior trusted pode
# manter a travessia enquanto guia o robô; a frontal sozinha respeita o teto.
GAP_VALIDATION_CONFIG = {
    "source_timeout": 0.125,  # Mesmo prazo do IPC frontal consumido pelo C++.
    "reference_timeout": 2.0,  # Expira a trajetória anterior depois de perda longa.
    "near_history_seconds": 0.30,  # Memória local curta; não usa validade do Fusion.
    "near_present_frames": 2,  # Arma a detecção após presença real em frames novos.
    "near_loss_frames": 2,  # Uma única imagem vazia não inicia possível GAP.
    "bottom_far_present_frames": 2,  # Qualquer posição FAR trusted confirma continuação.
    "bottom_fusion_reacquire_frames": 2,  # FAR + MEDIUM estáveis devolvem o controle ao Fusion.
    # Procura o label escolhido pelo Fusion ao redor do seu ponto distante.
    # A margem cobre arredondamento do centro da fita, sem unir componentes.
    "bottom_fusion_target_radius_px": 3,
    # Meio segundo ainda limita uma perda acidental, mas permite avançar além
    # dos oito frames observados nas falhas reais antes de iniciar a busca.
    "confirmation_seconds": 0.50,
    "forward_present_frames": 2,  # Confirmação por frames frontais distintos.
    "evidence_grace_seconds": 0.20,  # Tolera um frame vazio após confirmar GAP.
    "max_gap_seconds": 1.5,  # Teto quando não existe continuação inferior no frame atual.
}

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
