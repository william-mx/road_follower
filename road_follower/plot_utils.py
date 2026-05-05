import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def plot_mae(predicted_angles, true_angles, pkg_path):

    pred = np.array(predicted_angles)
    gt = np.array(true_angles)

    mae = np.mean(np.abs(pred - gt))
    print(f"MAE: {mae}")

    plt.figure()
    plt.plot(pred, label="Predicted")
    plt.plot(gt, label="Ground Truth")
    plt.title(f"Steering Comparison (MAE={mae:.4f})")
    plt.xlabel("Frame")
    plt.ylabel("Steering angle")
    plt.legend()
    plt.grid(True)

    # Save instead of showing
    save_dir = Path(pkg_path) / "plots"

    if not save_dir.exists():
        save_dir.mkdir(parents=True)

    save_path = save_dir / "steering_plot.png"

    plt.savefig(save_path)
    print(f"Plot saved to: {save_path}")

    plt.close()