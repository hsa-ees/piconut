import tkinter as tk
import colorsys
import os
import math

# Global dictionary to manage user-selected color entries
user_entries = {}



def rgb_to_hex(rgb):
    """Converts an RGB tuple to a hex string."""
    return '#{:02X}{:02X}{:02X}'.format(*rgb)

def hsv_to_rgb(h, s, v):
    """Adjusts color from HSV/HSB values and returns RGB tuple."""
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return int(r * 255), int(g * 255), int(b * 255)

def get_text_color(bg_hex):
    """Determines whether black or white text is better readable on given background."""
    r = int(bg_hex[1:3], 16)
    g = int(bg_hex[3:5], 16)
    b = int(bg_hex[5:7], 16)
    brightness = (r * 299 + g * 587 + b * 114) / 1000
    return 'black' if brightness > 128 else 'white'

def format_hsb(h, s, v):
    """Formats HSB values into a readable string."""
    hue_deg = int(h * 360)
    sat_percent = int(s * 100)
    val_percent = int(v * 100)
    return f'HSB=({hue_deg}°, {sat_percent}%, {val_percent}%)'

def hex_to_rgb(hex_color):
    """Converts a hex color to an RGB tuple."""
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

def rgb_to_hsv(rgb_color):
    """Converts an RGB tuple to HSV."""
    r, g, b = [x / 255.0 for x in rgb_color]
    return colorsys.rgb_to_hsv(r, g, b)

def euclidean_distance_rgb(color1, color2):
    """Calculates Euclidean distance between two RGB colors."""
    return math.sqrt(sum((c1 - c2) ** 2 for c1, c2 in zip(color1, color2)))

def euclidean_distance_hsv(color1, color2):
    """Calculates Euclidean distance between two HSV colors."""
    return math.sqrt(sum((c1 - c2) ** 2 for c1, c2 in zip(color1, color2)))

def rgb_to_hex(rgb_color):
    """Converts an RGB tuple to hex."""
    return '#{:02X}{:02X}{:02X}'.format(*rgb_color)

def find_closest_color(hex_color):
    """Finds the closest match to a given hex color in the master palette."""
    rgb_color = hex_to_rgb(hex_color)
    hsv_color = rgb_to_hsv(rgb_color)
    
    min_rgb_distance = float('inf')
    min_hsv_distance = float('inf')
    best_match_rgb = None
    best_match_hsv = None

    # Find the closest match in both HSV and RGB
    for master_color in create_palette_colors()[0]:
        master_rgb = hex_to_rgb(master_color)
        master_hsv = rgb_to_hsv(master_rgb)
        
        # Calculate Euclidean distances
        rgb_distance = euclidean_distance_rgb(rgb_color, master_rgb)
        hsv_distance = euclidean_distance_hsv(hsv_color, master_hsv)
        
        # Check if the current color is a better match
        if hsv_distance < min_hsv_distance:
            min_hsv_distance = hsv_distance
            best_match_hsv = master_color
        
        if rgb_distance < min_rgb_distance:
            min_rgb_distance = rgb_distance
            best_match_rgb = master_color
    
    return {
        'color': best_match_rgb,
        'hsv': best_match_hsv,
        'rgb': best_match_rgb
    }

# Number of grayscale levels
num_grayscales = 16
def create_palette_grayscale():
    """Creates only the grayscale palette."""
    colors = []
    hsb_infos = []
    for i in range(num_grayscales):
        gray_value = int(255 * i / (num_grayscales - 1))
        colors.append(rgb_to_hex((gray_value, gray_value, gray_value)))
        hsb_infos.append(f"Gray level {gray_value}")
    return colors, hsb_infos

def create_palette_colors():
    """Creates the color palette from red to magenta."""
    colors = []
    hsb_infos = []
    color_count = num_grayscales
    for i in range(27):
        hue_deg = i * (300 / 25)  # 0° to 300°, divided into 25 steps
        hue = hue_deg / 360

        for s in [1.0, 0.66, 0.33]:
            for v in [1.0, 0.66, 0.33]:
                # generate 256 colors only, will cut off the last three derivates from 250: #FF00CB
                if color_count >= 256:  
                    return colors, hsb_infos
                rgb = hsv_to_rgb(hue, s, v)
                colors.append(rgb_to_hex(rgb))
                hsb_infos.append(format_hsb(hue, s, v))
                color_count += 1

    return colors, hsb_infos

# Analyze text files logic:

def analyze_text_files():
    """Searches for .txt files in the current directory and analyzes them."""
    # Get all txt files in the current directory
    current_directory = os.path.dirname(os.path.abspath(__file__))
    text_files = [f for f in os.listdir(current_directory) if f.endswith('.txt')]
    print(f"Found {len(text_files)} text file(s): {', '.join(text_files)}\n")

    # Open a new window for each file
    for text_file in text_files:
        text_file_path = os.path.join(current_directory, text_file)
        with open(text_file_path, 'r') as file:
            original_colors = []
            for line in file:
                color = line.strip()
                if color.startswith('#') and len(color) == 7:
                    original_colors.append(color)
            
            # Open a new window to show the analysis results
            open_palette_window(text_file, original_colors)

def open_palette_window(filename, original_colors):
    """Creates a new window to display the color palette and matches."""
    window = tk.Toplevel()
    window.title(f"Palette: {filename}")

    # Create frames to organize the display
    left_frame = tk.Frame(window)
    left_frame.grid(row=0, column=0, padx=10)

    right_frame = tk.Frame(window)
    right_frame.grid(row=0, column=1, padx=10)

    # Display original colors on the left side
    for index, color in enumerate(original_colors):
        color = color.strip()
        if color:
            text_color = get_text_color(color)
            entry = tk.Entry(left_frame, font=("Consolas", 10), fg=text_color, width=20, bd=0, justify='center')
            entry.insert(0, color)
            entry.grid(row=index, column=0)
            entry.config(state='readonly', readonlybackground=color)

            # Find closest match in the master palette
            best_match = find_closest_color(color)
            matched_color = best_match['color']
            matched_color_hex = matched_color
            print(f"Matched color hex1: {matched_color_hex}")
            matched_color_hsv = best_match['hsv']
            matched_color_rgb = best_match['rgb']

            # Display the best match in the right frame
            entry_match = tk.Entry(right_frame, font=("Consolas", 10), fg='black', width=20, bd=0, justify='center')
            entry_match.insert(0, matched_color_hex)
            entry_match.grid(row=index, column=0)
            print(f"Matched color hex2: {matched_color_hex}")
            entry_match.config(state='readonly', readonlybackground=matched_color_hex)

            # Show the comparison result
            print(f"Original: {color} -> Best Match: {matched_color_hex} (HSV: {matched_color_hsv}, RGB: {matched_color_rgb})")

def append_corrections_to_file(filename, corrections, corrected_palette):
    """Appends corrections and the corrected palette at the end of the text file."""
    with open(filename, 'a') as file:
        # Print the corrections at the bottom
        file.write("\n# Corrections:\n")
        for correction in corrections:
            file.write(f"{correction}\n")
        
        # Write the final corrected palette
        file.write("\n# Corrected Palette:\n")
        for color in corrected_palette:
            file.write(f"{color}\n")


def build_gui_grayscale(frame, colors, hsb_infos):
    """Builds the GUI elements for grayscale entries."""
    for index, color in enumerate(colors):
        text_color = get_text_color(color)

        if index == 0 or index % 9 != 0:
            pady_value = (0, 0)
            # print(f'{index}: {color}', end=' ')
        else:
            pady_value = (0, 12)
            # print(f'{index}: {color}')

        entry = tk.Entry(frame, font=("Consolas", 10), fg=text_color, width=20, bd=0, justify='center')
        entry.insert(0, f"{index}: {color}")
        entry.grid(row=index, column=0, pady=pady_value)
        entry.config(state='readonly', readonlybackground=color) 

    print("Greyscale generated.")

def build_gui_colors(frame, colors, hsb_infos, start_index=0):
    """Builds the GUI elements for the colored entries."""
    color_counter = start_index
    for idx, color in enumerate(colors, start=start_index):
        text_color = get_text_color(color)

        # Arrange colors into columns
        column = 1 + (idx - color_counter) // 45  # Each column holds about 51 colors
        row = (idx - color_counter) % 45

        # format output for readability, newline after central RGB colors
        # if color in ['#FF0000', '#00FF00', '#0000FF', '#FFFF00', '#FF00FF', '#00FFFF']:
        #     print()

        # group output in 100% saturated color and derivatives
        if (idx-15) % 9 != 0:
            pady_value = (0, 0)
            # print(f'{idx}: {color}', end=' ')
        else:
            pady_value = (0, 12)
            # print(f'{idx}: {color}')

        entry = tk.Entry(frame, font=("Consolas", 10), fg=text_color, width=20, bd=0, justify='center')
        entry.insert(0, f"{idx}: {color}")
        entry.grid(row=row, column=column, pady=pady_value)
        entry.config(state='readonly', readonlybackground=color)

    print("Color palette generated.")


# -------------------
# Tkinter Main Part
# -------------------

root = tk.Tk()
root.title("8-Bit Color Palette")

frame = tk.Frame(root)
frame.pack()

# Create and display grayscale palette
grayscale_colors, grayscale_hsb_infos = create_palette_grayscale()
build_gui_grayscale(frame, grayscale_colors, grayscale_hsb_infos)

# Create and display color palette
color_palette_colors, color_palette_hsb_infos = create_palette_colors()
build_gui_colors(frame, color_palette_colors, color_palette_hsb_infos, start_index=len(grayscale_colors))

# Add the "Analyze Text Files" button below the grayscale palette
analyze_button = tk.Button(frame, text="Analyze Text File(s)", font=("Consolas", 10), command=analyze_text_files)
analyze_button.grid(row=len(grayscale_colors) + 1, column=0, pady=20)

root.mainloop()

