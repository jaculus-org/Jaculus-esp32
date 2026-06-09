import os
import glob
from PIL import Image, ImageEnhance

# --- CONFIGURATION ---
INPUT_DIR = "images"         # Folder containing your source images
OUTPUT_FILE = "sprites.ts"   # Output JS/TS file
TARGET_WIDTH = 64            # Desired width
TARGET_HEIGHT = 64           # Desired height
CONTRAST_FACTOR = 1.0        # 1.0 is original, >1.0 increases contrast
SATURATION_FACTOR = 1.0      # 1.0 is original, >1.0 increases saturation
# Groups similar colors to prevent massive palettes (1-255)
COLOR_BIN_SIZE = 64
# ---------------------


def rgb_to_rgb565(r, g, b):
    """Converts 8-bit RGB channels to 16-bit RGB565 integer."""
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


def process_images():
    if not os.path.exists(INPUT_DIR):
        print(f"Error: The directory '{INPUT_DIR}' does not exist.")
        return

    image_files = glob.glob(os.path.join(INPUT_DIR, "*.png")) + \
        glob.glob(os.path.join(INPUT_DIR, "*.jpg")) + \
        glob.glob(os.path.join(INPUT_DIR, "*.jpeg"))

    if not image_files:
        print(f"No images found in '{INPUT_DIR}'.")
        return

    # Character pool for mapping colors to letters
    chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"
    char_index = 0

    palette_dict = {}
    color_to_char = {}

    # Reserve pure transparent/black
    color_to_char["transparent"] = "B"
    palette_dict["B"] = 0x0000

    output_js = ""

    for img_path in image_files:
        filename = os.path.basename(img_path)
        base_name = os.path.splitext(filename)[0].upper().replace(
            " ", "_").replace("-", "_")

        # Load and convert to RGBA
        img = Image.open(img_path).convert("RGBA")

        # 1. Reduce Resolution
        # Using NEAREST to maintain sharp pixel art edges
        img = img.resize((TARGET_WIDTH, TARGET_HEIGHT),
                         Image.Resampling.NEAREST)

        # 2. Increase Contrast
        contrast_enhancer = ImageEnhance.Contrast(img)
        img = contrast_enhancer.enhance(CONTRAST_FACTOR)

        # 3. Increase Saturation
        color_enhancer = ImageEnhance.Color(img)
        img = color_enhancer.enhance(SATURATION_FACTOR)

        pixels = img.getdata()

        output_js += f"export const {base_name}_TEX = `\n"

        row_str = ""
        for i, pixel in enumerate(pixels):
            r, g, b, a = pixel

            # Handle transparency
            if a < 128:
                row_str += "B"
            else:
                # Bin colors to reduce palette size (e.g., floor to nearest multiple of 32)
                r_binned = (r // COLOR_BIN_SIZE) * COLOR_BIN_SIZE
                g_binned = (g // COLOR_BIN_SIZE) * COLOR_BIN_SIZE
                b_binned = (b // COLOR_BIN_SIZE) * COLOR_BIN_SIZE
                if (r_binned == 0 and g_binned == 0 and b_binned == 0):
                    r_binned, g_binned, b_binned = 8, 4, 8

                color_tuple = (r_binned, g_binned, b_binned)

                if color_tuple not in color_to_char:
                    # Assign next available character
                    if char_index < len(chars):
                        if chars[char_index] == 'B':  # Skip 'B' as it is reserved for transparent
                            char_index += 1
                        char = chars[char_index]
                    else:
                        # Fallback if we run out of standard chars
                        char = f"c{char_index}"

                    color_to_char[color_tuple] = char
                    palette_dict[char] = rgb_to_rgb565(
                        r_binned, g_binned, b_binned)
                    char_index += 1
                else:
                    char = color_to_char[color_tuple]

                row_str += char

            # End of row
            if (i + 1) % TARGET_WIDTH == 0:
                output_js += row_str + "\n"
                row_str = ""

        output_js += "`;\n\n"

    # Generate the shared palette
    output_js += "export const PALETTE: Record<string, number> = {\n"
    output_js += "    // Transparent\n"
    output_js += "    'B': 0x0000,\n\n"

    # Write colors
    for char, color_val in palette_dict.items():
        if char == 'B':
            continue
        output_js += f"    '{char}': 0x{color_val:04X},\n"

    output_js += "};\n"

    output_js += f'export const texW = {
        TARGET_WIDTH}; export const texH = {TARGET_HEIGHT};'

    # Save to file
    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        f.write(output_js)

    print(f"Success! Processed {len(image_files)} images.")
    print(f"Output saved to {OUTPUT_FILE}")
    print(f"Generated a shared palette with {len(palette_dict)} colors.")


if __name__ == "__main__":
    process_images()
