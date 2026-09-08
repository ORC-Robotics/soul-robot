"""Captura exclusiva da segunda câmera com Picamera2."""

from dataclasses import dataclass
import os

try:
    from libcamera import Transform  # type: ignore[import]
    from picamera2 import Picamera2  # type: ignore[import]
except ImportError:
    Transform = None
    Picamera2 = None


@dataclass(frozen=True)
class CameraConfig:
    """Reúne a configuração da câmera frontal usada para as bolas."""

    camera_index: int = 1
    downward_camera_index: int = 0
    width: int = 960
    height: int = 540
    sensor_width: int = 1920
    sensor_height: int = 1080
    sensor_bit_depth: int = 10
    target_fps: int = 30

    @classmethod
    def from_environment(cls):
        """Lê apenas o índice frontal, sem permitir acesso implícito à CAM0."""

        raw_index = os.environ.get("OBR_FORWARD_CAMERA_INDEX", "1")
        raw_downward_index = os.environ.get("OBR_DOWNWARD_CAMERA_INDEX", "0")
        try:
            camera_index = int(raw_index)
            downward_camera_index = int(raw_downward_index)
        except ValueError as error:
            raise ValueError(
                "Os índices das câmeras devem ser inteiros não negativos."
            ) from error
        config = cls(
            camera_index=camera_index,
            downward_camera_index=downward_camera_index,
        )
        config.validate()
        return config

    def validate(self):
        """Impede que o detector abra a câmera reservada ao segue-faixa."""

        if self.camera_index < 0 or self.downward_camera_index < 0:
            raise ValueError(
                "Os índices das câmeras devem ser inteiros não negativos."
            )
        if self.camera_index == self.downward_camera_index:
            raise ValueError(
                "A câmera frontal não pode usar o índice reservado ao segue-faixa."
            )
        dimensions = (
            self.width,
            self.height,
            self.sensor_width,
            self.sensor_height,
            self.sensor_bit_depth,
            self.target_fps,
        )
        if any(value <= 0 for value in dimensions):
            raise ValueError("Resolução, bit depth e FPS devem ser positivos.")


class FrontCamera:
    """Abre, captura e libera somente a câmera frontal do robô."""

    def __init__(self, config=None):
        self.config = config or CameraConfig.from_environment()
        self.config.validate()
        self._camera = None

    def open(self):
        """Configura a CAM1 e informa claramente conflitos de propriedade."""

        if self._camera is not None:
            return
        if Picamera2 is None or Transform is None:
            raise RuntimeError(
                "Picamera2/libcamera indisponível. Execute este programa na Raspberry Pi."
            )

        frame_duration_us = int(1_000_000 / self.config.target_fps)
        camera = None
        try:
            camera = Picamera2(self.config.camera_index)
            camera_config = camera.create_video_configuration(
                main={
                    "size": (self.config.width, self.config.height),
                    "format": "RGB888",
                },
                sensor={
                    "output_size": (
                        self.config.sensor_width,
                        self.config.sensor_height,
                    ),
                    "bit_depth": self.config.sensor_bit_depth,
                },
                controls={
                    "FrameDurationLimits": (
                        frame_duration_us,
                        frame_duration_us,
                    )
                },
                # As duas câmeras estão montadas invertidas no robô.
                transform=Transform(hflip=True, vflip=True),
                buffer_count=4,
            )
            camera.configure(camera_config)
            camera.start()
            self._camera = camera
        except Exception as error:
            if camera is not None:
                try:
                    camera.close()
                except Exception:
                    pass
            raise RuntimeError(
                "Não foi possível abrir a segunda câmera. Desative a câmera "
                "frontal no dashboard antes de iniciar o detector de bolas. "
                f"Detalhe: {error}"
            ) from error

    def capture_frame(self):
        """Captura um frame BGR compatível com as funções do OpenCV."""

        if self._camera is None:
            raise RuntimeError("A câmera frontal ainda não foi aberta.")
        frame = self._camera.capture_array("main")
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise RuntimeError("A câmera frontal retornou um frame inválido.")
        # Neste projeto, RGB888 do Picamera2 chega no array em ordem B, G, R.
        return frame

    def close(self):
        """Para a captura antes de liberar o dispositivo da câmera."""

        camera = self._camera
        self._camera = None
        if camera is None:
            return
        try:
            camera.stop()
        except Exception:
            pass
        camera.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exception_type, exception, traceback):
        del exception_type, exception, traceback
        self.close()
