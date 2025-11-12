#!/usr/bin/env python3

import sys
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import os


def load_map_and_path_data(filename):
    """
    Load map and path data from the binary file
    """
    file_size = os.path.getsize(filename)
    
    # Calculate expected size for the new format (with algorithm type)
    # Just read part of the file to determine format
    with open(filename, 'rb') as f:
        # Peek at the file to check if algorithm type is present
        try:
            # Try to read algorithm type (new format)
            algorithm_type = struct.unpack('i', f.read(4))[0]
        except struct.error:
            # If we can't read algorithm type, file is too short or corrupted
            # This could mean it's in old format or malformed, let's handle this better
            f.seek(0)  # Reset
            
            # Read first few values to determine the format based on plausibility
            try:
                # Attempt to read as if it's the old format (first item is resolution)
                resolution = struct.unpack('f', f.read(4))[0]
                x_min = struct.unpack('f', f.read(4))[0]
                x_max = struct.unpack('f', f.read(4))[0]
                y_min = struct.unpack('f', f.read(4))[0]
                y_max = struct.unpack('f', f.read(4))[0]
                rows = struct.unpack('Q', f.read(8))[0]
                cols = struct.unpack('Q', f.read(8))[0]
                
                # Check if these values are plausible for map parameters
                if (abs(resolution) <= 100 and abs(x_min) <= 100 and abs(x_max) <= 100 and 
                    abs(y_min) <= 100 and abs(y_max) <= 100 and rows <= 10000 and cols <= 10000):
                    # This looks like the old format, restart and process correctly
                    f.seek(0)
                    algorithm_type = 1  # Default to Hybrid A* for old format
                else:
                    # This might be an algorithm type instead of resolution, use the new format approach
                    # This is likely a complex case; let's try the new format approach
                    f.seek(0)
                    algorithm_type = struct.unpack('i', f.read(4))[0]
            except struct.error:
                raise ValueError("Invalid or truncated file")
        
        # Now continue with reading based on the algorithm type we identified
        # Read map parameters
        resolution = struct.unpack('f', f.read(4))[0]
        x_min = struct.unpack('f', f.read(4))[0]
        x_max = struct.unpack('f', f.read(4))[0]
        y_min = struct.unpack('f', f.read(4))[0]
        y_max = struct.unpack('f', f.read(4))[0]
        rows = struct.unpack('Q', f.read(8))[0]  # size_t (using Q for unsigned long long, 8 bytes)
        cols = struct.unpack('Q', f.read(8))[0]  # size_t (using Q for unsigned long long, 8 bytes)

        # Read map data size
        map_size = struct.unpack('Q', f.read(8))[0]  # size_t (using Q for unsigned long long, 8 bytes)

        # Read map data
        map_data = []
        for _ in range(map_size):
            value = struct.unpack('B', f.read(1))[0]  # uint8_t
            map_data.append(value)
        
        map_data = np.array(map_data, dtype=np.uint8).reshape((rows, cols))

        # Read path size
        path_size = struct.unpack('Q', f.read(8))[0]  # size_t (using Q for unsigned long long, 8 bytes)

        # Read path data
        path = []
        for _ in range(path_size):
            x = struct.unpack('f', f.read(4))[0]
            y = struct.unpack('f', f.read(4))[0]
            theta = struct.unpack('f', f.read(4))[0]
            path.append([x, y, theta])

        # Read vehicle parameters
        vehicle_wheelbase = struct.unpack('f', f.read(4))[0]
        vehicle_axle_to_front = struct.unpack('f', f.read(4))[0]
        vehicle_axle_to_rear = struct.unpack('f', f.read(4))[0]
        vehicle_width = struct.unpack('f', f.read(4))[0]

        return {
            'algorithm_type': algorithm_type,
            'map': map_data,
            'resolution': resolution,
            'x_min': x_min,
            'x_max': x_max,
            'y_min': y_min,
            'y_max': y_max,
            'rows': rows,
            'cols': cols,
            'path': np.array(path),
            'vehicle_wheelbase': vehicle_wheelbase,
            'vehicle_axle_to_front': vehicle_axle_to_front,
            'vehicle_axle_to_rear': vehicle_axle_to_rear,
            'vehicle_width': vehicle_width
        }


def draw_car(ax, x, y, theta, vehicle_params, color='red', alpha=0.7):
    """
    Draw a rectangular car model at position (x, y) with orientation theta
    """
    # Calculate car dimensions
    total_length = vehicle_params['vehicle_axle_to_front'] + vehicle_params['vehicle_axle_to_rear']
    half_width = vehicle_params['vehicle_width'] / 2.0
    half_length = total_length / 2.0
    
    # The vehicle position (x, y) in the path is the center of the rear axle
    # So to get the center of the car body, we need to move forward by half the total length
    center_x = x + (vehicle_params['vehicle_axle_to_front'] - vehicle_params['vehicle_axle_to_rear']) / 2.0 * np.cos(theta)
    center_y = y + (vehicle_params['vehicle_axle_to_front'] - vehicle_params['vehicle_axle_to_rear']) / 2.0 * np.sin(theta)
    
    # Create the rectangle for the car body
    car_rect = Rectangle(
        (-half_length, -half_width),  # bottom-left corner relative to center
        total_length,                 # width (length of car)
        vehicle_params['vehicle_width'],  # height (width of car)
        angle=np.degrees(theta),      # rotation angle in degrees
        rotation_point='center',      # rotate around the center
        facecolor=color,
        edgecolor='black',
        alpha=alpha
    )
    
    # Apply the transformation to move to the actual position
    t = plt.matplotlib.transforms.Affine2D().rotate(theta) + plt.matplotlib.transforms.Affine2D().translate(center_x, center_y)
    car_rect.set_transform(t + ax.transData)
    
    ax.add_patch(car_rect)
    
    # Optional: Draw a small indicator for the front of the car
    front_x = x + total_length * np.cos(theta)
    front_y = y + total_length * np.sin(theta)
    ax.plot(front_x, front_y, 'o', color='yellow', markersize=3)


def visualize_map_and_path(filename):
    """
    Visualize the map and path data
    """
    data = load_map_and_path_data(filename)
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Display the map with proper orientation
    # Flip the map vertically to match the coordinate system (image y-axis is inverted)
    ax.imshow(data['map'], cmap='gray', origin='lower', 
              extent=[data['x_min'], data['x_max'], data['y_min'], data['y_max']])
    
    # Draw the path if it exists
    if len(data['path']) > 0:
        path_x = data['path'][:, 0]
        path_y = data['path'][:, 1]
        path_theta = data['path'][:, 2]
        
        ax.plot(path_x, path_y, 'b-', linewidth=2, label='Path', alpha=0.7)
        ax.scatter(path_x[0], path_y[0], color='green', s=100, label='Start', zorder=5)
        ax.scatter(path_x[-1], path_y[-1], color='red', s=100, label='Goal', zorder=5)

        # Draw vehicle models along the path - only draw some to avoid clutter
        # For A*, we don't draw orientation-sensitive car models; for Hybrid A*, we do
        step = max(1, len(path_x) // 20)  # Draw about 20 vehicles max
        
        if data['algorithm_type'] == 1:  # Hybrid A*
            # Draw car models with orientation for Hybrid A*
            for i in range(0, len(path_x), step):
                draw_car(ax, path_x[i], path_y[i], path_theta[i], data, 
                        color='blue', alpha=0.5)

            # Draw car at start and end positions more prominently
            if len(path_x) > 0:
                draw_car(ax, path_x[0], path_y[0], path_theta[0], data, 
                        color='green', alpha=1.0)
                draw_car(ax, path_x[-1], path_y[-1], path_theta[-1], data, 
                        color='red', alpha=1.0)
        else:  # A*
            # For A* path, just draw circles since there's no orientation (theta = 0 for all points)
            for i in range(0, len(path_x), step):
                ax.plot(path_x[i], path_y[i], 's', color='orange', markersize=6, alpha=0.5)

            # Draw start and end positions more prominently for A*
            if len(path_x) > 0:
                ax.plot(path_x[0], path_y[0], 's', color='green', markersize=10, alpha=1.0)
                ax.plot(path_x[-1], path_y[-1], 's', color='red', markersize=10, alpha=1.0)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    
    # Set title based on algorithm type
    algorithm_name = "A*" if data['algorithm_type'] == 0 else "Hybrid A*"
    ax.set_title(f'{algorithm_name} Path Planning Visualization')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Save the image with the same name as the binary file but with .png extension
    png_filename = filename.replace('.bin', '.png')
    plt.savefig(png_filename, dpi=300, bbox_inches='tight')
    print(f"Visualization saved to {png_filename}")
    # plt.show()


def main():
    if len(sys.argv) != 2:
        print("Usage: python visualize.py <path_to_binary_file>")
        sys.exit(1)
    
    filename = sys.argv[1]
    if not os.path.exists(filename):
        print(f"Error: File {filename} does not exist.")
        sys.exit(1)
    
    if not filename.endswith('.bin'):
        print("Error: File must have .bin extension")
        sys.exit(1)
    
    visualize_map_and_path(filename)


if __name__ == "__main__":
    main()