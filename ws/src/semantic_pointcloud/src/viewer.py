import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import torch


class Viewer:
    def __init__(self, windowName: str = "image") -> None:
        self.figure, self.axes = plt.subplots(num=windowName)
        self.axes.axis("off")
        self.axesImage = None
        plt.show(block=False)

    def show(self, image: torch.Tensor) -> None:
        if image.ndim == 3 and image.shape[0] in (1, 3):
            image = image.permute(1, 2, 0)

        frame = image.detach().cpu()
        if frame.is_floating_point():
            frame = frame.clamp(0, 1)
        frame = frame.numpy()

        if self.axesImage is None:
            self.axesImage = self.axes.imshow(frame)
        else:
            self.axesImage.set_data(frame)

        # redraws just this figure and pumps its event loop, without blocking
        # for user input like plt.show()/plt.pause() would
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def close(self) -> None:
        plt.close(self.figure)
