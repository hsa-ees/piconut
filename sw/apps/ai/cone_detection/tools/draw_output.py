#!/usr/bin/env python3

################################################################################
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2026 Johannes Hofmann <johannes.hofmann1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

#
# THIS FILE IS STILL UNDER CONSTRUCTION.
#

import os
from PIL import Image, ImageDraw


def draw_bounding_boxes(
    image_path, boxes, output_path="detected_objects.jpg", color="red", width=3
):
    """Draws bounding boxes on an image using PIL.

    :param image_path: Path to the input image file.
    :param boxes: A list of tuples/lists formatted as [(x0, y0, x1, y1), ...]
    :param output_path: Where to save the output image.
    :param color: Color of the box outline (e.g., 'red', 'green', 'blue', or an
    RGB tuple).
    :param width: Thickness of the box outline in pixels.
    """
    if not os.path.exists(image_path):
        print(f"Error: Image path '{image_path}' does not exist.")
        return

    # 1. Open the image
    img = Image.open(image_path)

    # 2. Create a drawing handle
    draw = ImageDraw.Draw(img)

    # 3. Loop through your array of boxes and draw them
    for box in boxes:
        x0, y0, x1, y1 = box

        # PIL expects coordinates as a list: [x0, y0, x1, y1]
        draw.rectangle([x0, y0, x1, y1], outline=color, width=width)

    # 4. Handle potential JPEG transparency bugs and save
    if img.mode in ("RGBA", "P"):
        img = img.convert("RGB")

    img.save(output_path, "JPEG", quality=95, optimize=True)
    print(f"Successfully drew {len(boxes)} boxes. Saved to: {output_path}")


# ==========================================
# EXAMPLE USAGE
# ==========================================
if __name__ == "__main__":
    # Your array of boxes formatted as (x0, y0, x1, y1)
    my_boxes = [
        (266, 360, 268, 366),
        (337, 358, 343, 368),
        (114, 368, 124, 378),
        (256, 366, 262, 376),
        (352, 363, 358, 373),
        (358, 369, 368, 383),
        (244, 369, 254, 383),
        (358, 370, 368, 382),
        (376, 377, 396, 401),
        (379, 375, 399, 397),
        (225, 380, 237, 402),
        (224, 380, 238, 402),
        (376, 378, 396, 400),
        (379, 378, 399, 400),
        (379, 378, 399, 400),
        (221, 381, 235, 401),
        (224, 381, 238, 401),
        (376, 378, 396, 400),
        (379, 378, 399, 400),
        (379, 378, 399, 400),
        (187, 401, 207, 433),
        (424, 397, 448, 433),
        (181, 401, 203, 433),
        (187, 401, 207, 433),
        (184, 401, 204, 433),
        (424, 401, 448, 433),
        (418, 401, 448, 433),
        (419, 401, 447, 433),
        (184, 400, 204, 430),
        (187, 400, 207, 430),
        (184, 400, 204, 430),
        (424, 396, 448, 428),
        (418, 397, 448, 433),
        (418, 397, 448, 433),
        (184, 403, 204, 431),
        (183, 405, 205, 435),
        (424, 399, 448, 435),
        (424, 399, 448, 435),
        (418, 399, 448, 435),
        (533, 457, 583, 523),
        (533, 455, 583, 525),
        (47, 460, 103, 536),
        (114, 368, 124, 378),
        (243, 372, 255, 386),
        (357, 373, 369, 385),
        (221, 380, 235, 398),
        (379, 380, 399, 398),
        (537, 453, 583, 523),
        (533, 454, 583, 526),
        (49, 461, 105, 535),
        (46, 461, 104, 535),
        (533, 455, 583, 525),
        (533, 455, 583, 525),
        (47, 466, 103, 536),
        (46, 461, 104, 535),
        (46, 461, 104, 535),
        (533, 457, 583, 523),
        (533, 457, 583, 523),
        (47, 462, 103, 534),
        (47, 462, 103, 534),
        (46, 462, 104, 534)
    ]

    # Target image
    input_image = "../output_image.jpg"

    # Run it!
    draw_bounding_boxes(
        image_path=input_image,
        boxes=my_boxes,
        output_path="output_with_boxes.jpg",
        color="red",  # You can use standard color names or RGB like (0, 255, 0)
        width=4,  # Thicker lines make it easier to see on high-res images
    )
