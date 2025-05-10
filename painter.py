import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QSlider, QWidget, QDial
)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QBrush, QCursor, QColor, QTransform, QFont
from PyQt5.QtCore import Qt, QPoint, QPointF, pyqtSignal
from PIL import Image
from collections import deque
import math 
import requests
from io import BytesIO
import threading
import time

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion, PoseStamped
from robot_localization.srv import FromLL
from visualization_msgs.msg import Marker, MarkerArray

# These are parameters that the user could choose but are hardcoded for now
# API_KEY = ""
API_KEY = ""
LAT = 25.959060
LONG = -80.244230
CENTER = f"{LAT}, {LONG}"
ZOOM = 20
SIZE = "600x600"
MAP_TYPE = "satellite"

# Variables
markerUID = 0
mapPosX = 0
mapPosY = 0
mapYawTheta = 0

class CompleteCoveragePathPlannerUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.angle = 0 # Mowing direction angle [-90, 90]
        self.setWindowTitle("Paint Area to Mow")
        self.setGeometry(100, 100, 600, 600)

        # Initialize variables
        self.image = None
        self.drawing = False
        self.brush_size = 10
        self.highlighted_pixels = set()

        # Create UI elements
        self.image_label = QLabel("Open an image to start", self)
        self.image_label.setAlignment(Qt.AlignCenter)

        self.slider = QSlider(Qt.Horizontal, self)
        self.slider.setMinimum(1)
        self.slider.setMaximum(50)
        self.slider.setValue(self.brush_size)
        self.slider.valueChanged.connect(self.update_brush_size)

        self.submit_button = QPushButton("Mow", self)
        self.submit_button.clicked.connect(self.submit_result)

        self.mow_dir_label = QLabel("Choose Mowing Direction", self)
        self.mow_dir_label.setAlignment(Qt.AlignCenter)
        # Set font for QLabel
        font = QFont("Arial", 16, QFont.Bold)  # Font family: Arial, size: 16, weight: Bold
        self.mow_dir_label.setFont(font)

        # Create the dial
        self.dial = QDial()
        self.dial.setRange(-90, 90)
        self.dial.setValue(0)
        self.dial.valueChanged.connect(self.updateCanvas)

        # Create the QLabel to display the image
        self.arrow_label = QLabel()
        self.arrow_label.setAlignment(Qt.AlignCenter)
        self.canvas_size = 300  # Fixed canvas size for the QLabel
        self.arrow_label.setFixedSize(self.canvas_size, self.canvas_size)

        # Load and set the initial image
        self.original_pixmap = QPixmap("arrow.png")  # Replace with the path to your image
        if not self.original_pixmap.isNull():
            self.original_pixmap = self.original_pixmap.scaled(
                self.canvas_size - 20, self.canvas_size - 20, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        self.updateCanvas()

        # Layouts
        mainLayout = QHBoxLayout()
        rlayout = QVBoxLayout()
        rlayout.addWidget(self.mow_dir_label)
        rlayout.addWidget(self.arrow_label)
        rlayout.addWidget(self.dial)

        llayout = QVBoxLayout()
        llayout.addWidget(self.image_label)

        controls = QHBoxLayout()
        controls.addWidget(self.submit_button)
        controls.addWidget(QLabel("Brush Size:"))
        controls.addWidget(self.slider)
        llayout.addLayout(controls)

        mainLayout.addLayout(llayout)
        mainLayout.addLayout(rlayout)

        container = QWidget()
        container.setLayout(mainLayout)
        self.setCentralWidget(container)
        self.open_image()

    def updateCanvas(self):
        painter_transform = QTransform()
        painter_transform.translate(self.canvas_size / 2, self.canvas_size / 2)
        painter_transform.rotate(self.dial.value())
        painter_transform.translate(-self.original_pixmap.width() / 2, -self.original_pixmap.height() / 2)

        rotated_pixmap = self.original_pixmap.transformed(painter_transform, Qt.SmoothTransformation)

        self.arrow_label.setPixmap(rotated_pixmap)
        self.angle = -self.dial.value()

    def open_image(self):
        self.image = self.download_google_map(API_KEY, CENTER, ZOOM, SIZE, MAP_TYPE)
        if self.image.format() != QImage.Format_ARGB32:
            self.image = self.image.convertToFormat(QImage.Format_ARGB32)

        # Initialize the transparent drawing layer
        self.drawing_layer = QImage(self.image.size(), QImage.Format_ARGB32)
        self.drawing_layer.fill(Qt.transparent)  # Fully transparent

        # Initialize the robot location layer
        self.robot_location_layer = QImage(self.image.size(), QImage.Format_ARGB32)
        self.robot_location_layer.fill(Qt.transparent)  # Fully transparent

        # Track which pixels have been modified
        self.highlighted_pixels.clear()
        self.update_image()

    def update_image(self):
        if not self.image or not self.drawing_layer:
            return

        # Create a temporary combined image
        combined_image = QImage(self.image.size(), QImage.Format_ARGB32)
        combined_image.fill(Qt.transparent)

        # Blend the original image and the drawing layer
        painter = QPainter(combined_image)
        painter.drawImage(0, 0, self.image)  # Draw the original image
        painter.drawImage(0, 0, self.drawing_layer)  # Overlay the drawing layer
        painter.drawImage(0, 0, self.robot_location_layer) # Overlay the layer with the robot location indicator
        painter.end()

        # Update the QLabel
        pixmap = QPixmap.fromImage(combined_image)
        self.image_label.setPixmap(pixmap)

    def update_brush_size(self):
        self.brush_size = self.slider.value()
        self.update_cursor()
    
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.image and self.drawing_layer:
            self.drawing = True
            self.draw(event.pos())

    def mouseMoveEvent(self, event):
        if self.drawing and self.image and self.drawing_layer:
            self.draw(event.pos())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drawing = False

    def draw(self, pos):
        if not self.image_label.pixmap():
            return

        # Map mouse position to QLabel's coordinate system
        label_pos = self.image_label.mapFromGlobal(self.mapToGlobal(pos))

        # Map the QLabel's position to the pixmap's coordinate system
        pixmap = self.image_label.pixmap()

        label_width = self.image_label.width()
        label_height = self.image_label.height()
        pixmap_width = pixmap.width()
        pixmap_height = pixmap.height()

        # Calculate scale factors to map QLabel dimensions to pixmap dimensions
        scale_x = pixmap_width / label_width
        scale_y = pixmap_height / label_height

        # Adjust for potential padding or margins
        x = int(label_pos.x() * scale_x)
        y = int(label_pos.y() * scale_y)

        if x < 0 or y < 0 or x >= self.image.width() or y >= self.image.height():
            return

        painter = QPainter(self.drawing_layer)
        painter.setRenderHint(QPainter.Antialiasing)

        # # Iterate over pixels in the brush area
        for dx in range(-self.brush_size // 2, self.brush_size // 2 + 1):
            for dy in range(-self.brush_size // 2, self.brush_size // 2 + 1):
                px = x + dx
                py = y + dy
                if (px, py) not in self.highlighted_pixels and 0 <= px < self.image.width() and 0 <= py < self.image.height():
                    # Blend yellow with transparency
                    brush_color = QColor(255, 255, 92, int(0.25 * 255))  # 25% opacity yellow
                    painter.setPen(Qt.NoPen)
                    painter.setBrush(brush_color)
                    painter.drawEllipse(QPoint(px, py), 1, 1)  # Draw a single pixel
                    self.highlighted_pixels.add((px, py))

        painter.end()
        self.update_image()

    def submit_result(self):
        if not self.image or not self.drawing_layer:
            return

        pilImage = self.qimageToPilImage(self.drawing_layer)

        box_size = (3, 2)  # Box size (width=3, height=2) (This is currently hardcoded to be approximately the size of the robot in pixels at this zoom level)
        step_size = (3, 1)  # Step size (horizontal=3, vertical=1)
        yellow_threshold = (255, 255, 92)  # Define what constitutes a chosen pixel (yellow)

        # Get dimensions of image
        orig_width, orig_height = pilImage.size

        # Rotate the image
        rotated_image = pilImage.rotate(self.angle, expand=True)
        rot_width, rot_height = rotated_image.size

        # Find yellow pixels
        yellow_pixels = self.find_yellow_pixels(rotated_image, rot_width, rot_height, box_size, step_size, yellow_threshold)

        # Prune path and build Queue (Remove all pixels that are in a straight line between points that require robot to turn to minimize waypoints)
        pruned_path = self.prune_path(yellow_pixels)

        original_image_size = (orig_width, orig_height)  # Original image dimensions

        # NOTE:  When rotating the coords of the pixel location back into the original frame, there is an offset that depends on the angle.  
        # I do not know where this offset is coming from but probably has something to do with the padding that is added when rotating an image 
        # with the PIL package.  The below equations adjust for this offset and were experimentally determiend.
        if self.angle >= 0 and self.angle <=90:
            offset_x = math.floor(50 * math.sin(math.pi*self.angle/45.0))
            offset_y = math.floor(175 * math.sin(math.pi*self.angle/90.0))
        elif self.angle < 0 and self.angle >=-90:
            offset_y = -math.floor(50 * math.sin(math.pi*self.angle/45.0))
            offset_x = -math.floor(175 * math.sin(math.pi*self.angle/90.0))
        else:
            # Error, the user has requested an angle of cut that is outside of the limits.
            raise Exception("Something went wrong, the angle of cut is outside the allowed limit.")
        
        # For each waypoint in the rotated image, transform them back into the original coordinates in the un rotated image and correct 
        # for any offset introduced from the padding/expanding operation
        derot = deque()
        for element in pruned_path:
            orig_pos = self.calculate_original_position(element, original_image_size)
            corrected_pos = ((orig_pos[0] - offset_x), ((orig_pos[1] - offset_y)))
            derot.append(corrected_pos)

        # Calculate the bounds of the GPS map image
        corners = self.calculate_corners(LAT, LONG, ZOOM, orig_width, orig_height)
        gps_corners = []
        for _, coords in corners.items():
            gps_corners.append((coords[0], coords[1]))

        # Convert image coordinates into GPS waypoints
        path = deque()
        for element in derot:
            latitude, longitude = self.interpolate_gps(element[0], element[1], (orig_width, orig_height), gps_corners)
            path.append((latitude, longitude))

        print(f'Path waypoint count: {len(path)}')

        # Display the result
        self.visualize(derot)

        # Send path to planner with Nav2 API in a separate thread so you don't block the UI
        def sendWaypoints():
            rclpy.init()
            gps_wpf = GpsWpCommander(self)
            for wp in path:
                gps_wpf.llPoint2WayPoint(wp[0], wp[1])
                gps_wpf.spin()

        thread = threading.Thread(target=sendWaypoints)
        thread.start()

        # Disable UI controls and begin to monitor mission progress
        self.submit_button.setEnabled(False)

        # For monitoring progress, we will map the location of the robot on the map image, over time from the feedback that is sent from the navigator and also
        # draw out the path the robot has taken on the map

    def enterEvent(self, event):
        """Set the brush cursor when the mouse enters the QLabel."""
        if self.image:
            self.update_cursor()
        super().enterEvent(event)

    def leaveEvent(self, event):
        """Reset to the default cursor when the mouse leaves the QLabel."""
        self.unsetCursor()
        super().leaveEvent(event)

    def update_cursor(self):
        """Update the cursor to represent the brush size."""
        if not self.image:
            return
        cursor_pixmap = QPixmap(self.brush_size, self.brush_size)
        cursor_pixmap.fill(Qt.transparent)

        painter = QPainter(cursor_pixmap)
        pen = QPen(Qt.black)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawRect(0, 0, self.brush_size - 1, self.brush_size - 1)
        painter.end()

        brush_cursor = QCursor(cursor_pixmap)
        self.setCursor(brush_cursor)

    def qimageToPilImage(self, qimage):
        """
        Convert a QImage to a PIL Image by iterating over each pixel.

        Parameters:
            qimage (QImage): The QImage to convert.

        Returns:
            Image: The resulting PIL Image.
        """
        if qimage.isNull():
            raise ValueError("The provided QImage is null.")
        
        # Get the dimensions of the QImage
        width, height = qimage.width(), qimage.height()
        
        # Create a blank PIL Image of the same size
        pil_image = Image.new("RGBA", (width, height))
        
        # Iterate over each pixel in the QImage
        for y in range(height):
            for x in range(width):
                # Get the pixel value as (R, G, B, A)
                color = qimage.pixelColor(x, y)
                r, g, b, a = color.red(), color.green(), color.blue(), color.alpha()
                
                # Set the pixel value in the PIL Image
                pil_image.putpixel((x, y), (r, g, b, a))
        
        return pil_image

    def interpolate_gps(self, x, y, image_size, gps_corners):
        """
        Interpolate the GPS coordinates for a given pixel (x, y) in the image.

        Parameters:
            x, y: Pixel coordinates in the image (0-based index).
            image_size: Tuple (width, height) of the image.
            gps_corners: Tuple of GPS coordinates for the four corners:
                        (top_left, top_right, bottom_left, bottom_right)
                        Each corner is a tuple of (latitude, longitude).
        
        Returns:
            (latitude, longitude) of the interpolated point.
        """
        # Unpack GPS corners
        lat_0 = gps_corners[0][0]
        lat_1 = gps_corners[2][0]
        lon_0 = gps_corners[0][1]
        lon_1 = gps_corners[1][1]

        # Interpolate Latitude and Longitude for the given pixel location
        lon = ((float(x) / float(image_size[0])) * (lon_1 - lon_0) ) + lon_0
        lat = (float(y) / float(image_size[1])) * (lat_1 - lat_0) + lat_0

        return lat, lon

    def find_yellow_pixels(self, image, width, height, box_size, step_size, yellow_threshold=(200, 200, 92)):
        """
        Iterates through the pixels of an image, and records the location
        of pixels with the value yellow_threshold and adds them to a queue to return.
        
        Parameters:
            image (PIL Image): Image of type PIL Image.
            width (int): Width size of image in pixels
            height (int): Height size of image in pixels
            box_size (int, int):  Size of sliding window
            step_size (int, int): Horizontal step size, Vertical step size
            yellow_threshold (tuple): Minimum (R, G, B) values to classify a pixel as yellow or chosen.
            
        Returns:
            deque: Queue of (x, y) coordinates of yellow pixels.
        """
        rotated_pixels = image.load()
    
        # Initialize a queue to store yellow pixel locations
        yellow_pixel_queue = deque()
        
        # Unpack box and step sizes
        box_width, box_height = box_size
        step_x, step_y = step_size

        # Back-and-forth motion direction
        direction = 1  # 1 for left-to-right, -1 for right-to-left
        
        # Iterate over rows with vertical step size
        for y in range(0, height - box_height + 1, step_y):
            if direction == 1:
                x_range = range(0, width - box_width + 1, step_x)
            else:
                x_range = range(width - box_width, -1, -step_x)
            
            for x in x_range:
                # Check the pixels in the current box
                has_yellow = False
                for i in range(box_height):
                    for j in range(box_width):
                        px, py = x + j, y + i
                        if px < width and py < height:  # Ensure bounds
                            r, g, b, _ = rotated_pixels[px, py]
                            if r >= yellow_threshold[0] and g >= yellow_threshold[1] and b <= yellow_threshold[2]:
                                has_yellow = True
                
                # If the box contains a yellow pixel, add the location to the queue and turn the box black
                if has_yellow:
                    yellow_pixel_queue.append((x, y))
                    for i in range(box_height):
                        for j in range(box_width):
                            px, py = x + j, y + i
                            if px < width and py < height:  # Ensure bounds
                                rotated_pixels[px, py] = (0, 0, 0)
            
            # Reverse the direction for the next row
            direction *= -1

        return yellow_pixel_queue

    # Prune path
    def prune_path(self, yellow_box_queue):
        """
        Processes the queue by removing intermediate elements between groups of items with the same `y` value.
        
        Parameters:
            yellow_box_queue (deque): A deque of (x, y) coordinates.
            
        Returns:
            deque: The modified queue after processing and eliminating all points in a straight line path while keeping the outer pixels.
        """
        if not yellow_box_queue:
            return deque()  # Return an empty queue if input is empty
        
        # Initialize the processed queue
        processed_queue = deque()
        
        # Get the first starting element
        starting_element = yellow_box_queue.popleft()
        processed_queue.append(starting_element)
        
        exitFlag = False
        while yellow_box_queue:
            # Iterate until finding an element with a different y-value
            found = None
            prevElement = None
            while yellow_box_queue:
                current_element = yellow_box_queue.popleft()
                # If y values are not equal, then the path has made a turn off straight line
                if current_element[1] != starting_element[1]:
                    found = current_element
                    break
                # Save previous element so when a turn is encountered both points are recorded
                prevElement = current_element

                # If we are on the last element in the dequeue, always append this value and exit the processing loop
                if len(yellow_box_queue) == 1:
                    processed_queue.append(yellow_box_queue.popleft())
                    exitFlag = True
                    
            if exitFlag:
                break

            # If we found a new y-level, update the starting element and also add the two end pixels to queue which represent the turning manuever
            if found:
                starting_element = found
                if prevElement is not None:
                    processed_queue.append(prevElement)
                processed_queue.append(found)
        
        return processed_queue

    def calculate_original_position(self, rotated_pos, original_image_size):
        """
        Calculates the position of a pixel in the original unrotated image from its position
        in the rotated image, considering the elimination of padding.
        
        Parameters:
            rotated_pos (tuple): (x', y') position in the rotated image.
            original_image_size (tuple): (width, height) of the original image.
        
        Returns:
            tuple: (x, y) position in the original unrotated image.
        """
        angle_rad = math.radians(self.angle)  # Convert to radians and reverse the angle
        cos_theta = math.cos(angle_rad)
        sin_theta = math.sin(angle_rad)
        
        # Center of the original image
        original_center = (original_image_size[0] / 2, original_image_size[1] / 2)
        
        # Adjust for the center of the rotated image
        x_prime, y_prime = rotated_pos
        x_prime -= original_center[0]
        y_prime -= original_center[1]
        
        # Apply the inverse rotation (to map to the original image)
        x = cos_theta * x_prime - sin_theta * y_prime
        y = sin_theta * x_prime + cos_theta * y_prime
        
        # Adjust back to the center of the original image
        x += original_center[0]
        y += original_center[1]
        
        return (round(x), round(y))

    def lat_lon_to_pixels(self, lat, lon, zoom):
        """
        Convert latitude and longitude to pixel coordinates.
        """
        sin_lat = math.sin(math.radians(lat))
        x = (lon + 180) / 360 * 256 * 2**zoom
        y = (0.5 - math.log((1 + sin_lat) / (1 - sin_lat)) / (4 * math.pi)) * 256 * 2**zoom
        return x, y

    def pixels_to_lat_lon(self, x, y, zoom):
        """
        Convert pixel coordinates to latitude and longitude.
        """
        map_size = 256 * 2**zoom
        lon = x / map_size * 360 - 180
        n = math.pi - 2 * math.pi * y / map_size
        lat = math.degrees(math.atan(math.sinh(n)))
        return lat, lon

    def calculate_corners(self, lat, lon, zoom, width, height):
        """
        Calculate the GPS coordinates of the corner pixels.

        Parameters:
            lat (float): Lattitude of center of map image in Decimal degrees
            lon (float): Longitude of center of map image in Decimal degrees
            zoom (int): Zoom level of map image (0-21 for Google Maps API)
            width (int): Width of the map image in pixels
            height (int): Height of the map image in pixels

        Returns:
            dict: {
                "top_left": (float, float)  Lattitude, Longitude (Decimal degrees)
                "top_right": (float, float)  Lattitude, Longitude (Decimal degrees)
                "bottom_left": (float, float)  Lattitude, Longitude (Decimal degrees)
                "bottom_right": (float, float)  Lattitude, Longitude (Decimal degrees)
            }
        """
        center_x, center_y = self.lat_lon_to_pixels(lat, lon, zoom)

        # Map size in pixels
        half_width = width / 2
        half_height = height / 2

        # Calculate corner pixels
        top_left_x = center_x - half_width
        top_left_y = center_y - half_height
        bottom_right_x = center_x + half_width
        bottom_right_y = center_y + half_height

        # Convert corner pixels back to lat/lon
        top_left = self.pixels_to_lat_lon(top_left_x, top_left_y, zoom)
        bottom_right = self.pixels_to_lat_lon(bottom_right_x, bottom_right_y, zoom)

        return {
            "top_left": top_left,
            "top_right": self.pixels_to_lat_lon(bottom_right_x, top_left_y, zoom),
            "bottom_left": self.pixels_to_lat_lon(top_left_x, bottom_right_y, zoom),
            "bottom_right": bottom_right,
        }

    def visualize(self, coord_deque):
        """
        Clears then sets the pixel of the displayed map image (self.drawing layer) at each coordinate in the deque to the color (255, 255, 92).
        
        Parameters:
            coord_deque (deque): A deque containing tuples of pixel coordinates (x, y).
        """
        self.drawing_layer.fill(Qt.transparent)  # Clear drawing layer

        painter = QPainter(self.drawing_layer)
        painter.setRenderHint(QPainter.Antialiasing)

        for x, y in coord_deque:
            # Blend yellow with transparency
            brush_color = QColor(255, 255, 92, int(0.25 * 255))  # 25% opacity yellow
            painter.setPen(Qt.NoPen)
            painter.setBrush(brush_color)
            painter.drawEllipse(QPoint(x, y), 1, 1)  # Draw a single pixel

        painter.end()
        self.update_image()

    def drawRobotLocation(self):
        # To convert from Gazebo Coords to Map image coords, you must scale the coords
        width, height = self.image.width(), self.image.height()
        scaleFactor = width / 58.12     # This value is calculated from the distance per pixel at the map zoom level and resolution
        offsetX = -10
        offsetY = 25
        posX = width//2 + scaleFactor * mapPosX + offsetX
        posY = height // 2 - scaleFactor * mapPosY + offsetY
        theta = mapYawTheta

        # First clear layer from previous arrow draw
        self.robot_location_layer.fill(Qt.transparent)  # Fully transparent

        # Parameters for arrow
        color = (0, 255, 0)
        width = 4
        arrowhead_length = 15
        arrowhead_angle = 30
        arrow_length = 30

        # Draw an arrow to indicate the robots location and heading
        painter = QPainter(self.robot_location_layer)
        painter.setRenderHint(QPainter.Antialiasing)
        pen = QPen(QColor(*color), width)
        painter.setPen(pen)
        painter.setBrush(QBrush(QColor(*color)))

        startPoint = (posX, posY)
        endPoint = (posX + arrow_length * math.cos(-theta), posY + arrow_length * math.sin(-theta))
        painter.drawLine(QPointF(*startPoint), QPointF(*endPoint))

        # Calculate points for arrow head
        angle1 = theta + math.radians(arrowhead_angle)
        angle2 = theta - math.radians(arrowhead_angle)
        x1 = endPoint[0] - arrowhead_length * math.cos(-angle1)
        y1 = endPoint[1] - arrowhead_length * math.sin(-angle1)
        x2 = endPoint[0] - arrowhead_length * math.cos(-angle2)
        y2 = endPoint[1] - arrowhead_length * math.sin(-angle2)

        # Draw arrow head
        painter.drawPolygon(
            QPointF(endPoint[0], endPoint[1]),
            QPointF(x1, y1),
            QPointF(x2, y2)
        )

        # End painting and update image
        painter.end()
        self.update_image()

    def download_google_map(self, api_key, center, zoom, size, map_type):
        """
        Downloads a map image from the Google Maps Static API and sets the displayed image to the map image.

        Parameters:
            api_key (str): Your Google Maps API key.
            center (str): The center of the map (e.g., "New York, NY") in Lattitude, Longitude Decimal degrees.
            zoom (int): Zoom level (0-21).
            size (str): Size of the map in "widthxheight" (e.g., "800x600") pixels.
            map_type (str): Map type ("roadmap", "satellite", "hybrid", "terrain").

        Returns:
            QImage: Image of the downloaded map at the specified zoom level, size, and location
        """
        base_url = "https://maps.googleapis.com/maps/api/staticmap"
        params = {
            "center": center,
            "zoom": zoom,
            "size": size,
            "maptype": map_type,
            "key": api_key,
        }

        response = requests.get(base_url, params=params)

        if response.status_code == 200:
            # Convert the content to a QImage
            image_data = BytesIO(response.content)
            qimage = QImage()
            if qimage.loadFromData(image_data.read()):
                return qimage
            else:
                raise ValueError("Failed to load the image into QImage format.")
        else:
            raise ValueError(f"Failed to download map: {response.status_code}, {response.text}")

class GpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received
    """
    def __init__(self, CompleteCoveragePathPlannerUIInstance):
        self.updateRobotLocation = CompleteCoveragePathPlannerUIInstance.drawRobotLocation
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        
        self.localizer = self.create_client(FromLL,  '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.client_futures = []

        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        # self.timer = self.create_timer(0.5, self.publish_spheres)
        # self.get_logger().info("Sphere square publisher started.")

        self.get_logger().info('Ready for waypoints...')

    def llPoint2WayPoint(self, lat: float, lon: float):
        """
        Sends received point to nav2 gps waypoint follower
        """    
        wps = [self.latLonYaw2Geopose(lat, lon)]

        for wp in wps:
            self.req = FromLL.Request()
            self.req.ll_point.longitude = wp.position.longitude
            self.req.ll_point.latitude = wp.position.latitude
            self.req.ll_point.altitude = wp.position.altitude

            self.get_logger().info("Waypoint added to conversion queue...")
            self.client_futures.append(self.localizer.call_async(self.req))

    def addMarker(self, point):
        global markerUID

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "spheres"
        marker.id = markerUID
        markerUID = markerUID + 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.25  # Sphere diameter in meters
        marker.scale.y = 0.25
        marker.scale.z = 0.25

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha value

        # marker_array.markers.append(marker)
        self.publisher.publish(marker)

    def command_send_cb(self, future):
        self.resp = PoseStamped()
        self.resp.header.frame_id = 'map'
        self.resp.header.stamp = self.get_clock().now().to_msg()
        self.resp.pose.position = future.result().map_point

        self.addMarker(future.result().map_point)
    
        self.navigator.goToPose(self.resp)

    def spin(self):
        global mapPosX
        global mapPosY
        global mapYawTheta
        commandSent = False
        while rclpy.ok():
            rclpy.spin_once(self)
            for f in self.client_futures:
                if f.done():
                    while not self.navigator.isTaskComplete():
                        currentPose = self.navigator.getFeedback().current_pose.pose
                        currentPosition = currentPose.position
                        currentOrientation = currentPose.orientation
                        curRollX, curPitchY, curYawZ = self.euler_from_quaternion(currentOrientation)
                        mapPosX = currentPosition.x
                        mapPosY = currentPosition.y
                        mapYawTheta = curYawZ

                        # Trigger Robot Location Indicator Update
                        self.updateRobotLocation()

                        # May need to add small delay here to prevent rapid requests for feedback

                    self.get_logger().info("Following converted waypoint...")
                    self.command_send_cb(f)
                    commandSent = True
                    self.client_futures = []

            if commandSent:
                break
                    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q

    def euler_from_quaternion(self, q: Quaternion):
        """
        Convert a quaternion into euler angles
        taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        """
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def latLonYaw2Geopose(self, latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
        """
        Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
        """
        geopose = GeoPose()
        geopose.position.latitude = latitude
        geopose.position.longitude = longitude
        geopose.orientation = self.quaternion_from_euler(0.0, 0.0, yaw)
        return geopose


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CompleteCoveragePathPlannerUI()
    window.show()
    sys.exit(app.exec_())
