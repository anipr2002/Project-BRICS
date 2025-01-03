import argparse
from ultralytics import YOLO
import yaml
import os

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Train YOLO model for different configurations.")
    parser.add_argument("--model", required=True, help="Model name (e.g., control, canny, etc.)")
    parser.add_argument("--dataset_path", required=True, help="Path to the dataset directory")
    args = parser.parse_args()

    model_name = args.model
    dataset_path = args.dataset_path

    # Update the YAML configuration
    yaml_data = {
        "path": f"{dataset_path}/{model_name}",
        "train": "train/images",
        "val": "val/images",
        "test": "test/images",
        "names": {
            0: "Unusable",
            1: "Usable",
        }
    }

    # Save the YAML configuration
    yaml_path = os.path.join(dataset_path, "data.yaml")
    with open(yaml_path, "w") as file:
        yaml.dump(yaml_data, file)
    
    project_dir = dataset_path.replace("datasets", "trains")
    # project_dir = project_dir.replace("data", "home")
    save_name = f"InitialPass_{''.join(model_name.split('/'))}"

    # Load and train the YOLO model
    model = YOLO("yolo11x-obb.yaml").load("yolo11x.pt")
    results = model.train(data=yaml_path, 
                          epochs=100, 
                          imgsz=640, 
                          project=project_dir, 
                          name=save_name)

if __name__ == "__main__":
    main()
