import rospy
import torch
from pytorch3d.io import IO
from pytorch3d.structures import Pointclouds

# qualitative palette (tab20); cycles via modulo for models with more classes
_CLASS_PALETTE = torch.tensor(
    [
        [31, 119, 180],
        [174, 199, 232],
        [255, 127, 14],
        [255, 187, 120],
        [44, 160, 44],
        [152, 223, 138],
        [214, 39, 40],
        [255, 152, 150],
        [148, 103, 189],
        [197, 176, 213],
        [140, 86, 75],
        [196, 156, 148],
        [227, 119, 194],
        [247, 182, 210],
        [127, 127, 127],
        [199, 199, 199],
        [188, 189, 34],
        [219, 219, 141],
        [23, 190, 207],
        [158, 218, 229],
    ],
    dtype=torch.uint8,
)


def classIdsToColors(classIds: torch.Tensor) -> torch.Tensor:
    palette = _CLASS_PALETTE.to(classIds.device)
    indices = classIds.long().clamp(min=0) % palette.shape[0]
    return palette[indices]


def voxelDownsample(
    points: torch.Tensor, classIds: torch.Tensor, voxelSize: float
) -> tuple[torch.Tensor, torch.Tensor]:
    """One output point per occupied voxel: mean position, majority-vote class."""
    voxelIndex = torch.floor(points / voxelSize).long()
    uniqueVoxels, inverse = torch.unique(voxelIndex, dim=0, return_inverse=True)
    numVoxels = uniqueVoxels.shape[0]

    sums = torch.zeros((numVoxels, 3), dtype=points.dtype, device=points.device)
    sums.index_add_(0, inverse, points)
    counts = torch.zeros(numVoxels, dtype=points.dtype, device=points.device)
    counts.index_add_(0, inverse, torch.ones_like(inverse, dtype=points.dtype))
    meanPoints = sums / counts.unsqueeze(-1)

    uniqueClasses = classIds.unique()
    votes = torch.zeros((numVoxels, uniqueClasses.shape[0]), device=points.device)
    for i, cls in enumerate(uniqueClasses):
        mask = classIds == cls
        votes[:, i].index_add_(
            0, inverse[mask], torch.ones(mask.sum(), device=points.device)
        )
    voxelClasses = uniqueClasses[votes.argmax(dim=1)]

    return meanPoints, voxelClasses


class DetectionExporter:
    def __init__(self, voxelSize: float) -> None:
        self.voxelSize = voxelSize
        self.points: list[torch.Tensor] = []
        self.classIds: list[torch.Tensor] = []

    def add(self, points: torch.Tensor, classIds: torch.Tensor) -> None:
        mask = classIds >= 0
        if mask.any():
            self.points.append(points[mask].cpu())
            self.classIds.append(classIds[mask].cpu())

    def export(self, path: str) -> None:
        if not self.points:
            rospy.logwarn("DetectionExporter: nothing accumulated, skipping %s", path)
            return

        points = torch.cat(self.points, dim=0)
        classIds = torch.cat(self.classIds, dim=0)
        rawCount = points.shape[0]
        points, classIds = voxelDownsample(points, classIds, self.voxelSize)
        colors = classIdsToColors(classIds)

        pointCloud = Pointclouds(points=[points], features=[colors])
        IO().save_pointcloud(pointCloud, path, colors_as_uint8=True)
        rospy.loginfo(
            "DetectionExporter: exported %d points (%d classes, deduped from %d "
            "with %gm voxels) to %s",
            points.shape[0],
            classIds.unique().numel(),
            rawCount,
            self.voxelSize,
            path,
        )
