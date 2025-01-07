import cv2
import os

def capture_images(output_dir="src/yolo/captured", image_format="jpg"):
    # Ensure the output directory exists
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Set up the RealSense camera using OpenCV
    # Replace '0' with the appropriate device index if you have multiple cameras
    cap = cv2.VideoCapture(6)

    # Set the resolution to 720p (1280x720)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("Error: Could not open the camera.")
        return

    print("Press 'c' to capture an image, 'q' to quit.")

    count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Display the frame
        cv2.imshow("RealSense Camera - 720p", frame)

        # Wait for key press
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            # Save the captured image
            filename = os.path.join(output_dir, f"image_{count}.{image_format}")
            cv2.imwrite(filename, frame)
            print(f"Image saved: {filename}")
            count += 1

        elif key == ord('q'):
            # Quit the loop
            break

    # Release the camera and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_images()
