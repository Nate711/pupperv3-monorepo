import argparse
import pathlib
import shutil

import wandb


def download_latest_model(
    project_name,
    entity_name,
    run_number: int,
    model_dir="launch",
    model_name="policy_latest.json",
):
    """
    Downloads the latest model from a W&B project.

    :param project_name: The name of the W&B project.
    :param entity_name: The W&B entity (username or team).
    :param model_dir: The directory where the model will be downloaded.
    :param model_name: The name to copy the model as.
    :return: None
    """

    # Initialize the API
    api = wandb.Api()

    # Fetch the latest run
    runs = api.runs(f"{entity_name}/{project_name}")

    # Check if there are any runs
    if not runs:
        print("No runs found in the project.")
        return

    # find the run whose names ends in -run_number
    if run_number is not None:
        runs = [run for run in runs if run.name.endswith(f"-{run_number}")]
        if not runs:
            print(f"No runs found with the number {run_number}.")
            return
        run = runs[0]
        print("Using run: ", run.name)
    else:
        # Get the latest run (assuming runs are sorted by start time by default)
        # sort runs by the number at the end of the name
        runs = sorted(runs, key=lambda run: int(run.name.split("-")[-1]))
        run = runs[-1]
        print(f"Latest run: {run.name}")

    # get the artifact with the name that contains .json
    artifacts = [art for art in run.logged_artifacts() if ".json" in art.name]
    if not artifacts:
        print("ERROR: No model .json files found in the run.")
        return
        
    art = artifacts[0]
    print("Using artifact: ", art.name)

    # remove the :[version] from the name
    base_name = art.name.split(":")[0]
    print("Base name: ", base_name)

    # get folder of this script
    script_dir = pathlib.Path(__file__).parent
    model_dir = pathlib.Path(model_dir)

    downloaded_filepath = script_dir / model_dir / pathlib.Path(base_name)
    save_filepath = script_dir / model_dir / model_name

    # Download the file
    art.download(root=script_dir / model_dir)
    print(f"Model downloaded to: {downloaded_filepath}")
    model_name = pathlib.Path(model_name)
    shutil.copyfile(downloaded_filepath, save_filepath)
    print(f"Model copied to: {save_filepath}")


if __name__ == "__main__":
    # Define your project and entity (username or team)
    project_name = "pupperv3-mjx-rl"
    entity_name = "hands-on-robotics"

    # declare argparse arg for model number
    argparser = argparse.ArgumentParser()
    # model number
    argparser.add_argument("--run_number", type=int, default=None)
    args = argparser.parse_args()

    # Call the function to download the latest model
    download_latest_model(
        project_name,
        entity_name,
        args.run_number,
        model_dir="launch",
        model_name="policy_latest.json",
    )
