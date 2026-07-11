"""Camera configuration class."""

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(kw_only=True)
class CameraConfig:
    """Default camera configuration."""

    camera_color_image_topic: str
    camera_color_info_topic: str

    camera_name: str = "camera"
    camera_frame: str = "camera_link"

    max_image_delay: float = 1.0

    # NOTE: (HEIGHT, WIDTH) order — Camera unpacks `target_h, target_w = resolution`
    # and the camera-info fallback stores (msg.height, msg.width). All shipped
    # configs are square, which is why a (w, h) reading never bit anyone.
    resolution: list[int, int] | None = None
    crop_width: list[int | float, int | float] | None = None
    crop_height: list[int | float, int | float] | None = None

    @classmethod
    def from_yaml(cls, yaml_path: Path, **overrides) -> "CameraConfig":  # noqa: ANN003
        """Load config from YAML file with optional overrides.

        Args:
            yaml_path: Path to the YAML configuration file
            **overrides: Additional parameters to override YAML values

        Returns:
            CameraConfig: Configured camera instance
        """
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}

        # Apply overrides
        data.update(overrides)

        return cls(**data)

    def __post_init__(self):
        """Post-initialization to validate resolution and cropping."""
        if self.resolution is not None:
            if not (isinstance(self.resolution, (list, tuple)) and len(self.resolution) == 2):
                raise ValueError(
                    "Resolution must be a list or tuple of (HEIGHT, WIDTH) — the "
                    "code unpacks it as (h, w) and the camera-info fallback stores "
                    f"(msg.height, msg.width). Got: {self.resolution} of type {type(self.resolution)}"
                )

        if self.crop_width is not None:
            if not (isinstance(self.crop_width, (list, tuple)) and len(self.crop_width) == 2):
                raise ValueError("Crop width must be a list or tuple of [start, end].")
            if not all(isinstance(x, (int, float)) for x in self.crop_width):
                raise ValueError("Crop width values must be int or float.")
            # Check if using floats (relative cropping)
            if any(isinstance(x, float) for x in self.crop_width):
                if not all(isinstance(x, float) for x in self.crop_width):
                    raise ValueError(
                        "Crop width values must be either all int or all float, not mixed."
                    )
                if not all(0.0 <= x <= 1.0 for x in self.crop_width):
                    raise ValueError(
                        "Float crop width values must be between 0.0 and 1.0 (relative cropping)."
                    )
            # Check if using ints (absolute cropping)
            else:
                if not all(x >= 0 for x in self.crop_width):
                    raise ValueError("Integer crop width values must be non-negative.")
            # Check ordering
            if self.crop_width[0] >= self.crop_width[1]:
                raise ValueError("Crop width start must be less than end.")

        if self.crop_height is not None:
            if not (isinstance(self.crop_height, (list, tuple)) and len(self.crop_height) == 2):
                raise ValueError("Crop height must be a list or tuple of [start, end].")
            if not all(isinstance(x, (int, float)) for x in self.crop_height):
                raise ValueError("Crop height values must be int or float.")
            # Check if using floats (relative cropping)
            if any(isinstance(x, float) for x in self.crop_height):
                if not all(isinstance(x, float) for x in self.crop_height):
                    raise ValueError(
                        "Crop height values must be either all int or all float, not mixed."
                    )
                if not all(0.0 <= x <= 1.0 for x in self.crop_height):
                    raise ValueError(
                        "Float crop height values must be between 0.0 and 1.0 (relative cropping)."
                    )
            # Check if using ints (absolute cropping)
            else:
                if not all(x >= 0 for x in self.crop_height):
                    raise ValueError("Integer crop height values must be non-negative.")
            # Check ordering
            if self.crop_height[0] >= self.crop_height[1]:
                raise ValueError("Crop height start must be less than end.")


@dataclass(kw_only=True)
class DummyCameraConfig(CameraConfig):
    """Dummy camera configuration class for testing purposes.

    Subclasses CameraConfig so it carries ALL fields the Camera code reads
    (crop_width/crop_height included — the old plain class lacked them, so the
    default Camera(config=None) raised AttributeError on the first image) and
    runs the same __post_init__ validation.
    """

    camera_color_image_topic: str = "dummy_camera/color/image_raw"
    camera_color_info_topic: str = "dummy_camera/color/camera_info"
    resolution: list[int, int] | None = None
    camera_name: str = "dummy_camera"
    camera_frame: str = "dummy_camera_link"

    def __post_init__(self):
        """Default the resolution then run CameraConfig validation."""
        if self.resolution is None:
            self.resolution = [640, 480]
        super().__post_init__()
