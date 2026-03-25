# Remember activate the virtual environment if needed.

from ultralytics import YOLO

# The source video name
video_name = "your_video_name.mp4"

# The pytorch model name
model_name = "your_model_pt"

# The model source
model = YOLO("your_model_location/weights/best.pt")
results = model(video_name, stream=True, verbose=False)

count = 0

with open("your_output_file.txt", "w") as f:
    for r in results:
        if count % 100 == 0:
            print("Running, current frame:", count)

        # https://docs.ultralytics.com/reference/engine/results/#ultralytics.engine.results.Boxes.id
        if r.boxes is not None:
            for box in r.boxes.xywh:
                x = box[0].item()
                y = box[1].item()

                f.write(f"{count} {x} {y}\n")

        count += 1