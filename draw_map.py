import numpy as np
import math
import cv2
import yaml
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString

class drawMap():
    """
    A class to handle the drawing of a map based on sensor measurements and robot coordinates.

    Attributes:
        image (ndarray): The image of the map loaded from a file.
        origin_x (float): The x-coordinate of the origin of the map.
        origin_y (float): The y-coordinate of the origin of the map.
        resolution (float): The resolution of the map.
        map_height (int): The height of the map in pixels.
        measurements (dict): Sensor measurements and robot poses loaded from a YAML file.

    Methods:
        transform(x_bot, y_bot, angle, ss, bs, fs): Transforms robot's base coordinates and orientation into two points.
        process_measurements(): Processes measurements to compute lines and their pixel coordinates.
        draw_lines_with_custom_length(points, lengths): Draws lines between given points.
        map(tolerance): Groups and processes lines and measurements to draw a map with annotated line lengths.
    """
    def __init__(self, map_img_file_path, map_yaml_file_path, measurements_yaml_file_path):

        self.image = cv2.imread(map_img_file_path, cv2.IMREAD_GRAYSCALE)
        
        with open(map_yaml_file_path, 'r') as file:
            map_metadata = yaml.safe_load(file)

        self.origin_x = map_metadata["origin"][0]
        self.origin_y = map_metadata["origin"][1]
        self.resolution = map_metadata["resolution"]
        self.map_height, _ = self.image.shape

        with open(measurements_yaml_file_path, 'r') as file1:
            self.measurements = yaml.safe_load(file1)


    def transform(self, x_bot, y_bot, angle, ss, bs, fs):
        """
        Transforms robot's base coordinates and orientation into two points based on sensor measurements.

        Returns:
        - tuple: Coordinates of two points (x1, y1, x2, y2).
        """
        x_robot = x_bot
        y_robot = y_bot
        theta_robot = np.radians(angle)   

        x1 = round(x_robot + bs * math.cos(theta_robot) - ss * math.sin(theta_robot), 2)
        y1 = round(y_robot + bs * math.sin(theta_robot) + ss * math.cos(theta_robot), 2)

        x2 = round(x_robot + fs * math.cos(theta_robot) - ss * math.sin(theta_robot), 2)
        y2 = round(y_robot + fs * math.sin(theta_robot) + ss * math.cos(theta_robot), 2)
            
        return x1, y1, x2, y2 


    def process_measurements(self):
        """
        Processes measurements to compute lines and their pixel coordinates.

        Returns:
        - tuple: Lists of lines and lengths.
        """
        self.lines = []
        self.lines_px = []
        self.lengths = []

        for measure in self.measurements:
            if not measure:
                continue
            else:
                fs = measure["x1"]
                bs = measure["x2"]
                ss = measure["y1"]
                x = measure["robot_pose"]["x"]
                y = measure["robot_pose"]["y"]
                angle = measure["robot_pose"]["z"]
                
                x1, y1, x2, y2 = self.transform(x, y, angle, ss, bs, fs)
                # x1_px, y1_px, x2_px, y2_px = get_img_coords(x1, y1, x2, y2)
                length = fs - bs

                if all(not(math.isinf(x) or math.isnan(x)) for x in [x1, y1, x2, y2]):
                    self.lines.append(((x1, y1), (x2, y2)))
                    # lines_px.append(((x1_px, y1_px), (x2_px, y2_px)))
                    self.lengths.append(length)

        return self.lines, self.lengths


    def draw_lines(self, filtered_points):
        """
        Draws lines between given points.

        Parameters:
        - filtered_points: List of tuples, where each tuple contains two points ((x1, y1), (x2, y2))
        """
        plt.figure()
        
        # Plotting the filtered points and their lines
        for i, ((x1, y1), (x2, y2)) in enumerate(filtered_points):
            plt.plot([x1, x2], [y1, y2], 'k-', linewidth=1.5)  # Line between points
            plt.scatter([x1, x2], [y1, y2], color='b')  # Points
        
        # Setting the aspect of the plot to be equal
        plt.gca().set_aspect('equal', adjustable='box')
        
        # Adding grid
        plt.grid(True)

        # Showing the plot
        plt.show()

    def calculate_direction(self, line):
        x1, y1 = line[0]
        x2, y2 = line[1]
        dx = x2 - x1
        dy = y2 - y1
        return np.arctan2(dy, dx)

    def calculate_midpoint(self, line):
        x1, y1 = line[0]
        x2, y2 = line[1]
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def angle_between_vectors(self, v1, v2):
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        return np.arccos(dot_product / (norm_v1 * norm_v2))

    def filter_duplicate_lines(self, lines, tolerance=1e-1, angle_tolerance=0.2):
        unique_lines = []
        for line in lines:
            line_direction = self.calculate_direction(line)
            line_midpoint = self.calculate_midpoint(line)

            is_duplicate = False
            for unique_line in unique_lines:
                unique_line_direction = self.calculate_direction(unique_line)
                unique_line_midpoint = self.calculate_midpoint(unique_line)

                angle_diff = abs(line_direction - unique_line_direction)
                if angle_diff > np.pi:
                    angle_diff = 2 * np.pi - angle_diff

                midpoint_dist = np.linalg.norm(np.array(line_midpoint) - np.array(unique_line_midpoint))

                if angle_diff < angle_tolerance and midpoint_dist < tolerance:
                    is_duplicate = True
                    break

            if not is_duplicate:
                unique_lines.append(line)

        return unique_lines

    def map(self):
        """
        Main mapping function that processes measurements and draws the map.

        Returns:
        - list of lists of tuples: Final processed lines ready for display or further processing.
        """

        lines, _ = self.process_measurements()
        self.draw_lines(lines)

        unique_lines = self.filter_duplicate_lines(lines)
        self.draw_lines(unique_lines)

        return lines

def main():

    mapper = drawMap(map_img_file_path="SLAM/slam_map.pgm", 
                     map_yaml_file_path="SLAM/slam_map.yaml", 
                     measurements_yaml_file_path="measurements.yaml")
    
    lines = mapper.map()


if __name__ == "__main__":
    main()
