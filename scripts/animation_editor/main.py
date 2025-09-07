import pandas as pd
import matplotlib.pyplot as plt
import sys
from pathlib import Path

def plot_csv_timeseries(csv_path):
    """Read and plot CSV time series data from animation recordings."""
    
    # Read the CSV file
    df = pd.read_csv(csv_path)
    
    # Convert timestamp to seconds relative to start
    df['time'] = df['timestamp_sec'] - df['timestamp_sec'].iloc[0]
    
    # Get joint columns (all columns except timestamp columns)
    joint_columns = [col for col in df.columns if col not in ['timestamp_ns', 'timestamp_sec', 'time']]
    
    # Create subplots for each joint
    n_joints = len(joint_columns)
    n_cols = 3
    n_rows = (n_joints + n_cols - 1) // n_cols
    
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, n_rows * 3))
    fig.suptitle(f'Animation Recording: {Path(csv_path).stem}', fontsize=16)
    
    # Flatten axes array for easier iteration
    if n_rows == 1:
        axes = axes.reshape(1, -1)
    axes_flat = axes.flatten()
    
    # Plot each joint
    for idx, joint in enumerate(joint_columns):
        ax = axes_flat[idx]
        ax.plot(df['time'], df[joint], linewidth=1.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.set_title(joint)
        ax.grid(True, alpha=0.3)
    
    # Hide any unused subplots
    for idx in range(n_joints, len(axes_flat)):
        axes_flat[idx].set_visible(False)
    
    plt.tight_layout()
    
    # Save the plot
    output_path = Path(csv_path).with_suffix('.png')
    plt.savefig(output_path, dpi=100, bbox_inches='tight')
    print(f"Plot saved to: {output_path}")
    
    # Show interactive plot
    plt.show()

def main():
    # Get CSV path from command line
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        # Try to find CSV files in the animation_controller launch directory
        repo_root = Path(__file__).parent.parent.parent
        launch_dir = repo_root / "ros2_ws/src/animation_controller/launch"
        
        if launch_dir.exists():
            csv_files = list(launch_dir.glob("*.csv"))
            if csv_files:
                print("Available CSV files:")
                for i, f in enumerate(csv_files, 1):
                    print(f"  {i}. {f.name}")
                print(f"\nUsage: uv run python main.py <path_to_csv>")
                print(f"Example: uv run python main.py {csv_files[0]}")
                sys.exit(0)
            else:
                print("No CSV files found in animation_controller/launch/")
                print("Usage: uv run python main.py <path_to_csv>")
                sys.exit(1)
        else:
            print("Animation controller launch directory not found")
            print("Usage: uv run python main.py <path_to_csv>")
            sys.exit(1)
    
    # Check if file exists
    csv_path = Path(csv_path)
    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}")
        sys.exit(1)
    
    print(f"Plotting data from: {csv_path}")
    plot_csv_timeseries(csv_path)


if __name__ == "__main__":
    main()
