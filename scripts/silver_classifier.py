from dataclasses import dataclass
from pathlib import Path
import time

import cv2
import numpy as np

try:
    from ai_edge_litert.interpreter import Interpreter
except ImportError:
    Interpreter = None


CLASS_NAMES = ("black", "other", "silver")


@dataclass(frozen=True)
class SilverClassification:
    black: float
    other: float
    silver: float
    preprocess_ms: float
    inference_ms: float
    total_ms: float

    @property
    def scores(self):
        return {
            "black": self.black,
            "other": self.other,
            "silver": self.silver,
        }

    @property
    def label(self):
        return max(self.scores, key=self.scores.get)

    @property
    def confidence(self):
        return self.scores[self.label]


class SilverClassifier:
    def __init__(self, model_path, num_threads=2):
        if Interpreter is None:
            raise RuntimeError(
                "LiteRT não está instalado. Ative a .venv-ai e instale ai-edge-litert."
            )

        self.model_path = Path(model_path)

        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"Modelo TFLite não encontrado: {self.model_path}"
            )

        self.interpreter = Interpreter(
            model_path=str(self.model_path),
            num_threads=num_threads,
        )
        self.interpreter.allocate_tensors()

        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()

        if len(input_details) != 1:
            raise RuntimeError(
                f"O classificador deve possuir exatamente 1 entrada, recebeu {len(input_details)}."
            )

        if len(output_details) != 1:
            raise RuntimeError(
                f"O classificador deve possuir exatamente 1 saída, recebeu {len(output_details)}."
            )

        self.input_detail = input_details[0]
        self.output_detail = output_details[0]

        input_shape = tuple(int(value) for value in self.input_detail["shape"])

        if len(input_shape) != 4 or input_shape[0] != 1 or input_shape[3] != 3:
            raise RuntimeError(
                f"Entrada inválida do modelo: {input_shape}. Esperado [1, altura, largura, 3]."
            )

        self.input_height = input_shape[1]
        self.input_width = input_shape[2]
        self.input_dtype = self.input_detail["dtype"]
        self.output_dtype = self.output_detail["dtype"]

        output_shape = tuple(int(value) for value in self.output_detail["shape"])

        if int(np.prod(output_shape)) != len(CLASS_NAMES):
            raise RuntimeError(
                f"Saída inválida do modelo: {output_shape}. Esperadas {len(CLASS_NAMES)} classes."
            )

    @property
    def input_size(self):
        return self.input_width, self.input_height

    def classify(self, frame_bgr):
        total_started = time.perf_counter()
        preprocess_started = time.perf_counter()

        input_tensor = self._prepare_input(frame_bgr)

        preprocess_ms = (
            time.perf_counter() - preprocess_started
        ) * 1000.0

        inference_started = time.perf_counter()

        self.interpreter.set_tensor(
            self.input_detail["index"],
            input_tensor,
        )
        self.interpreter.invoke()

        raw_output = self.interpreter.get_tensor(
            self.output_detail["index"]
        )

        inference_ms = (
            time.perf_counter() - inference_started
        ) * 1000.0

        output = self._dequantize_output(raw_output)
        probabilities = self._to_probabilities(output)

        total_ms = (
            time.perf_counter() - total_started
        ) * 1000.0

        scores = dict(zip(CLASS_NAMES, probabilities))

        return SilverClassification(
            black=float(scores["black"]),
            other=float(scores["other"]),
            silver=float(scores["silver"]),
            preprocess_ms=preprocess_ms,
            inference_ms=inference_ms,
            total_ms=total_ms,
        )

    def _prepare_input(self, frame_bgr):
        if not isinstance(frame_bgr, np.ndarray):
            raise TypeError("frame_bgr deve ser um numpy.ndarray.")

        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError(
                f"Frame inválido: shape={frame_bgr.shape}. Esperado HxWx3."
            )

        resized = cv2.resize(
            frame_bgr,
            (self.input_width, self.input_height),
            interpolation=cv2.INTER_AREA,
        )

        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        real_input = rgb.astype(np.float32)[np.newaxis, ...]

        if np.issubdtype(self.input_dtype, np.floating):
            return real_input.astype(self.input_dtype)

        if np.issubdtype(self.input_dtype, np.integer):
            scale, zero_point = self.input_detail["quantization"]

            if scale <= 0:
                raise RuntimeError(
                    "Modelo inteiro possui parâmetros de quantização inválidos."
                )

            quantized = np.round(real_input / scale + zero_point)
            limits = np.iinfo(self.input_dtype)
            quantized = np.clip(
                quantized,
                limits.min,
                limits.max,
            )

            return quantized.astype(self.input_dtype)

        raise RuntimeError(
            f"Tipo de entrada TFLite não suportado: {self.input_dtype}."
        )

    def _dequantize_output(self, raw_output):
        output = np.asarray(raw_output).reshape(-1)

        if np.issubdtype(self.output_dtype, np.floating):
            return output.astype(np.float32)

        if np.issubdtype(self.output_dtype, np.integer):
            scale, zero_point = self.output_detail["quantization"]

            if scale <= 0:
                raise RuntimeError(
                    "Saída inteira possui parâmetros de quantização inválidos."
                )

            return (
                output.astype(np.float32) - zero_point
            ) * scale

        raise RuntimeError(
            f"Tipo de saída TFLite não suportado: {self.output_dtype}."
        )

    @staticmethod
    def _to_probabilities(values):
        values = np.asarray(values, dtype=np.float32)

        if not np.all(np.isfinite(values)):
            raise RuntimeError(
                "O modelo produziu NaN ou infinito."
            )

        already_probabilities = (
            np.all(values >= 0.0)
            and np.all(values <= 1.0)
            and np.isclose(float(values.sum()), 1.0, atol=1e-3)
        )

        if already_probabilities:
            return values

        shifted = values - np.max(values)
        exponentials = np.exp(shifted)
        denominator = exponentials.sum()

        if denominator <= 0:
            raise RuntimeError(
                "Não foi possível normalizar a saída do modelo."
            )

        return exponentials / denominator