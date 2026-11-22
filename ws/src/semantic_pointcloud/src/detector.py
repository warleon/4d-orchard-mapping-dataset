from types import SimpleNamespace
from typing import Optional, Tuple

from ultralytics.engine.results import Boxes, Masks

import torch
import torch.nn.functional as F
from torchvision.ops import nms
from ultralytics import YOLO

from utils.detector_config import DetectorConfig


def roundToStride(size: int, stride: int = 32) -> int:
    return max(stride, (size // stride) * stride)


def gridWindows(width: int, height: int, tileSize: int, overlap: float) -> list[tuple[int, int]]:
    """Deterministic sliding-window grid covering (width, height) edge-to-edge.

    Same logic as the tiling used for training/offline inference elsewhere
    in this project (see entrenamiento/tiled_inference.py's _grid_windows) -
    copied rather than imported, since that lives in a separate repo with
    its own dependencies.
    """
    stride = max(1, round(tileSize * (1 - overlap)))

    def starts(size: int) -> list[int]:
        if size <= tileSize:
            return [0]
        points = list(range(0, size - tileSize + 1, stride))
        if points[-1] != size - tileSize:
            points.append(size - tileSize)
        return points

    return [(x0, y0) for y0 in starts(height) for x0 in starts(width)]


class Detector:
    def __init__(self, config: DetectorConfig, device: str) -> None:
        self.device = device
        self.model = YOLO(config.yolo_model_path)
        self.model.to(self.device)
        self.confidence = config.yolo_confidence
        self.classes = config.yolo_classes
        self.tileSize = config.tile_size
        self.tileOverlap = config.tile_overlap
        self.tileNmsIou = config.tile_nms_iou
        # class index -> class name, as reported by the loaded model
        self.classNames = self.model.names

    def detect(self, image: torch.Tensor) -> Tuple[Boxes, Optional[Masks]]:
        """Run YOLO inference on an RGB CxHxW float image.

        The image is resized to the nearest multiple of the model's stride
        (32) in each dimension, since YOLO refuses tensor inputs otherwise.
        Both the returned boxes and masks are in that resized frame, not the
        original image's pixel coordinates.

        Returns tuple:
            Boxes:
                (N, 6) tensor of [x1, y1, x2, y2, confidence, class_id] in the
                resized frame's pixel coordinate system (empty if nothing was
                detected). This is ultralytics' own packed Boxes.data layout.
            Masks:
                (N,H,W) binary masks of the detected objects, H/W matching the
                resized frame, or None if nothing was detected (ultralytics'
                own convention - an empty-but-present Masks is never returned).
        """
        _, height, width = image.shape
        resized = F.interpolate(
            image.unsqueeze(0),
            size=(roundToStride(height), roundToStride(width)),
            mode="bilinear",
            align_corners=False,
        ).squeeze(0)

        if self.tileSize is None:
            results = self.model.predict(
                resized.unsqueeze(0),
                conf=self.confidence,
                classes=self.classes,
                device=self.device,
                verbose=False,
            )
            return results[0].boxes, results[0].masks

        return self._detectTiled(resized)

    def _detectTiled(self, image: torch.Tensor) -> Tuple[SimpleNamespace, Optional[SimpleNamespace]]:
        """Grid-tile `image` into tileSize x tileSize windows (tileOverlap
        fraction of overlap between neighbors, full coverage edge-to-edge -
        see gridWindows), run one batched YOLO pass over every tile, then
        merge each tile's local-coordinate detections back into the full
        image's own coordinate frame. Overlapping tiles can each detect the
        same real object; per-class NMS over the combined global boxes keeps
        only the highest-confidence copy of each.

        Needed for small objects (e.g. grape berries) that would shrink past
        recognizability if the whole frame were downscaled into one pass -
        tiling keeps every tile at native pixel density instead.
        """
        _, height, width = image.shape
        windows = gridWindows(width, height, self.tileSize, self.tileOverlap)
        tiles = torch.stack(
            [image[:, y0 : y0 + self.tileSize, x0 : x0 + self.tileSize] for x0, y0 in windows]
        )

        results = self.model.predict(
            tiles,
            conf=self.confidence,
            classes=self.classes,
            device=self.device,
            verbose=False,
        )

        globalBoxRows: list[torch.Tensor] = []
        localMasks: list[torch.Tensor] = []
        maskOffsets: list[tuple[int, int]] = []
        maskDtype = None
        for (x0, y0), result in zip(windows, results):
            if result.masks is None:
                continue
            maskDtype = result.masks.data.dtype
            offset = torch.tensor(
                [x0, y0, x0, y0, 0.0, 0.0], device=result.boxes.data.device
            )
            globalBoxRows.append(result.boxes.data + offset)
            for i in range(result.masks.data.shape[0]):
                localMasks.append(result.masks.data[i])
                maskOffsets.append((x0, y0))

        if not globalBoxRows:
            return SimpleNamespace(data=torch.empty((0, 6), device=self.device)), None

        globalBoxes = torch.cat(globalBoxRows, dim=0)
        keepIndices: list[int] = []
        for classId in globalBoxes[:, 5].unique().tolist():
            classMask = (globalBoxes[:, 5] == classId).nonzero(as_tuple=True)[0]
            classKeep = nms(
                globalBoxes[classMask, :4],
                globalBoxes[classMask, 4],
                self.tileNmsIou,
            )
            keepIndices.extend(classMask[classKeep].tolist())

        keptBoxes = globalBoxes[keepIndices]
        canvas = torch.zeros(
            (len(keepIndices), height, width), dtype=maskDtype, device=self.device
        )
        for canvasIndex, sourceIndex in enumerate(keepIndices):
            x0, y0 = maskOffsets[sourceIndex]
            canvas[canvasIndex, y0 : y0 + self.tileSize, x0 : x0 + self.tileSize] = localMasks[
                sourceIndex
            ]

        return SimpleNamespace(data=keptBoxes), SimpleNamespace(data=canvas)
