# Validação da candidata congelada

Cada comparação usa os mesmos RGB. Todos os frames são posteriores ao congelamento.

| Captura | Frames | FP original → candidata, px/frame | Fita original → candidata | Jitter centro original → candidata, px | Score v3 original → candidata |
|---|---:|---:|---:|---:|---:|
| holdout_shadow_removed_candidate_01 | 90 | 8591.8444 → 322.6889 | 1.0000 → 1.0000 | 0.2410 → 0.0100 | 77.98 → 96.88 |
| holdout_shadow_removed_original_01 | 90 | 9054.7444 → 580.7667 | 1.0000 → 1.0000 | 0.3110 → 0.0050 | 76.08 → 96.90 |

Os indicadores completos, mínimos por frame e configurações estão em `holdout.json`.
Isso não é teste com motores em movimento nem comprovação de uma run completa.
