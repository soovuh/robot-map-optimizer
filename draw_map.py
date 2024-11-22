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

    def filter_duplicate_lines(self, lines, tolerance=0.2):
        unique_lines = []

        for line in lines:
            x1, y1 = line[0]
            x2, y2 = line[1]

            is_duplicate = False
            for unique_line in unique_lines:
                ux1, uy1 = unique_line[0]
                ux2, uy2 = unique_line[1]

                if (
                    (abs(x1 - ux1) <= tolerance and abs(y1 - uy1) <= tolerance and
                    abs(x2 - ux2) <= tolerance and abs(y2 - uy2) <= tolerance) or
                    (abs(x1 - ux2) <= tolerance and abs(y1 - uy2) <= tolerance and
                    abs(x2 - ux1) <= tolerance and abs(y2 - uy1) <= tolerance)
                ):
                    is_duplicate = True
                    break

            if not is_duplicate:
                unique_lines.append(line)

        return unique_lines

    def filter_sub_lines(self, lines, tolerance=0.2):
        def normalize_line(line):
            p1, p2 = sorted(line)
            return p1, p2

        def is_sub_line(small, large, tol):
            (sx1, sy1), (sx2, sy2) = small
            (lx1, ly1), (lx2, ly2) = large

            return (
                min(lx1, lx2) - tol <= sx1 <= max(lx1, lx2) + tol and
                min(ly1, ly2) - tol <= sy1 <= max(ly1, ly2) + tol and
                min(lx1, lx2) - tol <= sx2 <= max(lx1, lx2) + tol and
                min(ly1, ly2) - tol <= sy2 <= max(ly1, ly2) + tol
            )

        normalized_lines = [normalize_line(line) for line in lines]
        filtered_lines = []

        for i, line in enumerate(normalized_lines):
            is_subset = False
            for j, other_line in enumerate(normalized_lines):
                if i != j and is_sub_line(line, other_line, tolerance):
                    is_subset = True
                    break

            if not is_subset:
                filtered_lines.append(line)

        return filtered_lines

    def map(self):
        """
        Main mapping function that processes measurements and draws the map.

        Returns:
        - list of lists of tuples: Final processed lines ready for display or further processing.
        """

        lines, _ = self.process_measurements()
        unique_lines = self.filter_duplicate_lines(lines)
        unique_lines_without_sublines = self.filter_sub_lines(unique_lines)
        self.draw_lines(unique_lines_without_sublines)

        return lines


def main():

    mapper = drawMap(map_img_file_path="SLAM/slam_map.pgm", 
                     map_yaml_file_path="SLAM/slam_map.yaml", 
                     measurements_yaml_file_path="measurements.yaml")
    
    lines = mapper.map()


if __name__ == "__main__":
    main()
