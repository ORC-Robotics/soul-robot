"""Validação numérica compartilhada pelos componentes de controle visual."""

import math


def finite_virtual_position(value):
    """Converte uma posição virtual finita ou devolve None."""

    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None
