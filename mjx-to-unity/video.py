import cv2
import numpy as np

def show_video(images, output_file='output_video.avi', display=True, fps=24):
    """
    Create and display a video from a stack of NumPy ndarrays (RGB images).

    Parameters:
    - images: List of NumPy ndarrays representing RGB images.
    - output_file: Output video file name.
    - display: Boolean flag to control whether to display the video.

    Returns:
    - None
    """

    # Get the dimensions from the first image
    height, width, _ = images[0].shape
    fps = float(fps)
    # Create a video writer object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    # Iterate through the images and write them to the video file
    for frame in images:
        video_writer.write(frame)

    # Release the video writer
    video_writer.release()

    if display:
        # Optionally, display the video using OpenCV
        cap = cv2.VideoCapture(output_file)

        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                break

            cv2.imshow('Video', frame)

            # Adjust the delay (in milliseconds) based on your video frame rate
            if cv2.waitKey(int(1000/fps)) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Example usage:
    import numpy as np

    # Generate example RGB images (replace this with your actual images)
    num_images = 2000
    image_height, image_width = 200, 200
    images = [(np.random.randint(0, 256, (image_height, image_width, 3), dtype=np.uint8)* (np.sin(i/100)+1)/2).astype(np.uint8)  for i in range(num_images)]

    show_video(images)
