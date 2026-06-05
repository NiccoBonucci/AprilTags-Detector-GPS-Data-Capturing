from pathlib import Path
import shutil

root_folder = "/home/simulator/Desktop/neural_mpc_arlotta/paper_extension/apriltags_ws/src/cube_pose_estimator/apriltag_snapshots"

# Folder where the search starts
start_folder = Path(root_folder)

# Folder where matching PNG files will be copied
output_folder = Path("/home/simulator/Desktop/neural_mpc_arlotta/paper_extension/apriltags_ws/src/cube_pose_estimator/parking_images")
output_folder.mkdir(parents=True, exist_ok=True)

for folder in start_folder.iterdir():
    if folder.is_dir():
        for png_file in folder.glob("*.png"):
            if "color" in png_file.name.lower():
                destination = output_folder / png_file.name

                # Avoid overwriting files with the same name
                if destination.exists():
                    destination = output_folder / f"{png_file.stem}_{folder.name}{png_file.suffix}"

                shutil.copy2(png_file, destination)
                print(f"Copied: {png_file} -> {destination}")