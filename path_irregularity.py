import csv
import math

# Define a function to calculate the angle between two vectors
def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.atan2(dy, dx)

# Read the CSV file with X and Y positions
scenes = [5,6,7,8,9]
speeds = [1.5]
methods = ["unaware", "sf", "rvo", "sacadrl"]

x_values = []
y_values = []


for l in range(len(scenes)):
    for j in range(len(speeds)):

        csv_file = "annotations/model/scene"+str(scenes[l])+"_"+str(speeds[j])+"_0_robotPositionsFileName.csv" 
        visited_positions = set()  # To store unique positions

        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header row if it exists
            for row in reader:
                x = float(row[1])
                y = float(row[2])
                position = (x, y)

                # Check if the position has been visited before
                if position in visited_positions:
                    continue  # Skip duplicate positions

                x_values.append(x)
                y_values.append(y)
                visited_positions.add(position)

        # Initialize variables to store the total angle and point count
        total_angle = 0
        point_count = len(x_values)

        # Calculate the angle for each point and add it to the total
        for i in range(point_count - 1):
            angle = calculate_angle(x_values[i], y_values[i], x_values[i+1], y_values[i+1])
            total_angle += angle

        # Calculate the path irregularity (average angle)
        path_irregularity = total_angle / (point_count - 1)
        print(csv_file)
        print(f"Path Irregularity: {path_irregularity} radians")
        
        for k in range(len(methods)):

           
        
            csv_file = "annotations/tests/scene"+str(scenes[l])+"_"+str(speeds[j])+"_1_"+methods[k]+"_robotPositionsFileName.csv" 
            visited_positions = set()  # To store unique positions

            with open(csv_file, 'r') as file:
                reader = csv.reader(file)
                next(reader)  # Skip the header row if it exists
                for row in reader:
                    x = float(row[1])
                    y = float(row[2])
                    position = (x, y)

                    # Check if the position has been visited before
                    if position in visited_positions:
                        continue  # Skip duplicate positions

                    x_values.append(x)
                    y_values.append(y)
                    visited_positions.add(position)

            # Initialize variables to store the total angle and point count
            total_angle = 0
            point_count = len(x_values)

            # Calculate the angle for each point and add it to the total
            for i in range(point_count - 1):
                angle = calculate_angle(x_values[i], y_values[i], x_values[i+1], y_values[i+1])
                total_angle += angle

            # Calculate the path irregularity (average angle)
            path_irregularity = total_angle / (point_count - 1)
            print(csv_file)
            print(f"Path Irregularity: {path_irregularity} radians")
