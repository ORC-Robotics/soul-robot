"""Validação numérica compartilhada pelos componentes de controle visual."""

import math


# Altura usada para calibrar kernels e distâncias da câmera inferior.
DOWNWARD_REFERENCE_FRAME_HEIGHT = 480.0


def scaled_odd_kernel_size(reference_size, frame_height):
    """Escala um kernel de referência e mantém o tamanho ímpar mínimo de 3."""

    scaled = int(round(
        float(reference_size) * float(frame_height)
        / DOWNWARD_REFERENCE_FRAME_HEIGHT
    ))
    scaled = max(3, scaled)
    return scaled if scaled % 2 == 1 else scaled + 1


def scaled_reference_pixels(reference_value, frame_height, minimum=1.0):
    """Escala uma distância de referência para o frame processado."""

    scaled = float(reference_value) * float(frame_height) / (
        DOWNWARD_REFERENCE_FRAME_HEIGHT
    )
    return max(float(minimum), scaled)


def finite_virtual_position(value):
    """Converte uma posição virtual finita ou devolve None."""

    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None
