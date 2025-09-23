import io
import requests
import supervision as sv
from PIL import Image
from rfdetr import RFDETRBase

model = RFDETRBase()

# url = "https://media.roboflow.com/notebooks/examples/dog-2.jpeg"
path = "/Users/nathankau/pupperv3-monorepo/ai/playground/image-description-benchmark/src/image_description_benchmark/images/camera_image_raw_compressed-1755118546-420444431.jpg"

image = Image.open(path)
detections = model.predict(image, threshold=0.5)

annotated_image = image.copy()
annotated_image = sv.BoxAnnotator().annotate(annotated_image, detections)
annotated_image = sv.LabelAnnotator().annotate(annotated_image, detections)

sv.plot_image(annotated_image)
