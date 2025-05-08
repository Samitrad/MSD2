from picamera2 import Picamera2
import cv2
from ultralytics import YOLO

# Load trained YOLOv11 model
model = YOLO("Fire_Detection.pt")

# Initialize PiCamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (1280, 720)})
picam2.configure(config)
picam2.start()

print("Starting Fire Detection (press 'q' to quit)...")

try:
    while True:
        # Capture frame from Pi Camera
        frame = picam2.capture_array()

        # Convert BGR (from picamera2) to RGB for YOLO
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Run YOLOv8 inference
        results = model.predict(source=rgb_frame, save=False, conf=0.5, verbose=False)

        # Annotate detections on frame
        annotated_frame = results[0].plot()

        # Display the frame
        cv2.imshow("Fire Detection", annotated_frame)

        # Break loop on 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    cv2.destroyAllWindows()
    picam2.close()
