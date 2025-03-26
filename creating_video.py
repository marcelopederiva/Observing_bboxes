import cv2
import os
from tqdm import tqdm
import numpy as np

def find_closest_timestamp(target_timestamp, available_timestamps):
    closest_timestamp = min(available_timestamps, key=lambda x: abs(target_timestamp - x))
    return closest_timestamp

# Folder containing the images from folder_1
folder_1 = 'C:/Users/marped4128/Documents/Scripts/Light3_Noise_sv3_cap10p_hi_drive_short'

# Folder containing the images from images_folder

images_folder = '//ctag/data/DINN/058AC/052SS/VisionData/datasets/DIMS/Lidar+camera/ap1/hi-drive_short/LI-AP020X'  # Update this path to your actual images_folder

# images_folder = '//Innserver3/VisionData/datasets/DIMS/Lidar+camera/ap1/ctag_exterior/LI-AP020X'  # Update this path to your actual images_folder

# Get all image file names in folder_1 and sort them
images_folder1 = [img for img in os.listdir(folder_1) if img.endswith((".png", ".jpg", ".jpeg"))]
images_folder1.sort()  # Ensure the images are in correct order

# Get all image file names in images_folder and extract their timestamps
images_folder2 = [img for img in os.listdir(images_folder) if img.endswith((".png", ".jpg", ".jpeg"))]

# Create a dictionary mapping from timestamp to filename for images_folder images
timestamps_folder2 = {}
available_timestamps = [] 

for img_name in images_folder2:
    # Assuming filename is 'timestamp_number.jpg', extract timestamp
    timestamp_str = img_name.split('_')[0]  # Split at '_', take first part
    try:
        timestamp = int(timestamp_str)
        timestamps_folder2[timestamp] = img_name
        available_timestamps.append(timestamp)
    except ValueError:
        # Filename doesn't start with a timestamp
        continue

# Define the labels and colors
labels_and_colors = [
    ('Car', (0, 0, 1)),
    ('Truck', (0, 0.5, 0)),
    ('Construction Vehicle', (0.5, 1, 0.5)),
    ('Bus', (0.7, 0.4, 1)),
    ('Trailer', (1, 0.5, 0)),
    ('Barrier', (0, 1, 1)),
    ('Motorcycle', (0.2, 0.2, 0.2)),
    ('Bicycle', (0.25, 0.1, 0)),
    ('Pedestrian', (1, 0, 0)),
    ('Traffic Cone', (1.0, 1.0, 0))
]

# Function to convert colors from (R, G, B) 0-1 range to BGR 0-255
def convert_color(color_tuple):
    R = int(color_tuple[0] * 255)
    G = int(color_tuple[1] * 255)
    B = int(color_tuple[2] * 255)
    return (B, G, R)  # OpenCV uses BGR

# Define the video file name and its codec
video_name = 'Light3_Noise_sv3_car10p_hi_drive_short.avi'

# Read the first frame to get dimensions
frame = cv2.imread(os.path.join(folder_1, images_folder1[0]))

# Get the dimensions of the image
height, width, layers = frame.shape

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # You can use other codecs like 'MP4V'
video = cv2.VideoWriter(video_name, fourcc, 30.0, (width, height))

# Read each image and write it to the video
for image_name in tqdm(images_folder1[:7000]):
    img_path = os.path.join(folder_1, image_name)
    img = cv2.imread(img_path)

    # Extract timestamp from image_name
    timestamp_str = os.path.splitext(image_name)[0]  # Remove extension
    try:
        target_timestamp = int(timestamp_str)
    except ValueError:
        # Filename doesn't start with a timestamp
        continue

    # Find the closest timestamp from images_folder
    closest_timestamp = find_closest_timestamp(target_timestamp, available_timestamps)
    closest_image_name = timestamps_folder2[closest_timestamp]

    # Read the image from images_folder
    overlay_img_path = os.path.join(images_folder, closest_image_name)
    overlay_img = cv2.imread(overlay_img_path)

    # Resize the overlay image to be smaller (e.g., 20% of the main image size)
    overlay_height = int(height * 0.2)
    overlay_width = int(width * 0.2)
    overlay_img_resized = cv2.resize(overlay_img, (overlay_width, overlay_height))

    # Overlay the resized image onto the main image at position (0, 0)
    img[0:overlay_height, 0:overlay_width] = overlay_img_resized

    # Now, add the legend in the upper-right corner

    # Starting position for the legend
    x_start = width - 220  # Adjust as needed
    y_start = 20  # Start 20 pixels from the top edge
    line_height = 30  # Space between labels

    # Font settings
    font = cv2.FONT_HERSHEY_TRIPLEX
    font_scale = 0.6
    font_color = (0, 0, 0)
    thickness = 1

    # Draw a white rectangle as background for the legend
    legend_x1 = x_start - 10  # Slightly offset to include padding
    legend_y1 = y_start - 10
    legend_x2 = x_start + 200
    legend_y2 = y_start + 300
    cv2.rectangle(img, (legend_x1, legend_y1), (legend_x2, legend_y2), (255, 255, 255), -1)  # -1 fills the rectangle

    # Draw each label and color
    for idx, (label, color_tuple) in enumerate(labels_and_colors):
        y_position = y_start + idx * line_height

        # Draw filled circle
        circle_radius = 8
        circle_color = convert_color(color_tuple)
        circle_center = (x_start, y_position)

        cv2.circle(img, circle_center, circle_radius, circle_color, -1)  # -1 fills the circle

        # Put text next to circle
        text_position = (x_start + 20, y_position + 5)  # Adjust as needed
        cv2.putText(img, label, text_position, font, font_scale, font_color, thickness, cv2.LINE_AA)

    # Add the "Confidence: 0.12" box in the top middle

    # Text to display
    confidence_text = "Light3_Noise conf: 0.1 <Hi-drive short>"

    # Font settings for confidence text
    conf_font_scale = 0.6
    conf_thickness = 1

    # Get text size
    (text_width, text_height), baseline = cv2.getTextSize(confidence_text, font, conf_font_scale, conf_thickness)

    # Coordinates for the box
    box_width = text_width + 20  # Add padding
    box_height = text_height + 20  # Add padding

    box_x1 = (width - box_width) // 2  # Centered horizontally
    box_y1 = 20  # 20 pixels from the top
    box_x2 = box_x1 + box_width
    box_y2 = box_y1 + box_height

    # Draw filled white rectangle for the confidence box
    cv2.rectangle(img, (box_x1, box_y1), (box_x2, box_y2), (255, 255, 255), -1)

    # Put the confidence text on the rectangle
    text_x = box_x1 + 10  # Padding from the box edge
    text_y = box_y1 + box_height // 2 + text_height // 2 - baseline

    cv2.putText(img, confidence_text, (text_x, text_y), font, conf_font_scale, font_color, conf_thickness, cv2.LINE_AA)

    # Write the frame to the video
    video.write(img)

# Release the video writer
video.release()

print(f"Video saved as {video_name}")



'''
ffmpeg -i Light3_Noise3_hi_drive_short.avi -vcodec libx265 -crf 28 output_video.avi
'''