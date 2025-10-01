import cv2
import numpy as np

# --- Parameters ------------------------------------------------------------
IMG_PATH = 'top_view.png'
MIN_AREA = 500       # ignore tiny specks
BOWL_AREA = 2000     # anything above this → bowl

# approximate hue ranges for color classification (in degrees)
HUE_RANGES = {
    'blue':   (100, 130),
    'yellow': ( 20,  40),
    'green':  ( 40,  80),
}

WORLD_X_MIN, WORLD_X_MAX = -0.3, 0.3
WORLD_Y_MIN, WORLD_Y_MAX = -0.2, -0.8

# --- Helpers ---------------------------------------------------------------
def pixel_to_world(pt, img_shape):
    px, py = pt
    H, W = img_shape[:2]
    xw = px / W * (WORLD_X_MAX - WORLD_X_MIN) + WORLD_X_MIN
    yw = py / H * (WORLD_Y_MAX - WORLD_Y_MIN) + WORLD_Y_MIN
    return (xw, yw)

def is_bowl(area, thresh=BOWL_AREA):
    """Return True if the contour area corresponds to a bowl."""
    return area > thresh

def classify_color(hue, sat, val):
    """
    Given the mean HSV of an object, decide if it is blue, yellow, or green.
    Returns the color name or 'unknown'.
    """
    for name, (h_lo, h_hi) in HUE_RANGES.items():
        if h_lo <= hue <= h_hi:
            return name
    return 'unknown'

def show_mask(mask, title, show=False):
    if show:
        cv2.imshow(title, mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def localize_objects(img, show_masks=False):
    # 1) Load & remove dark background
    img = cv2.imread(IMG_PATH)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, fg_mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY) # drop dark pixels

    # show the foreground mask if requested
    show_mask(fg_mask, "Foreground Masks", show_masks)

    # 2) Find all contours in the cleaned foreground mask
    cnts, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    objects = dict()
    for idx, cnt in enumerate(cnts):
        # Ignore tiny contours
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        
        # Display contour if requested
        if show_masks:
            preview = img.copy()
            cv2.drawContours(preview, [cnt], -1, (0,255,0), 2)
            cv2.putText(preview,
                        f"Contour {idx}, area={int(area)}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255,255,255),
                        2,
                        cv2.LINE_AA)
            cv2.imshow(f"Inspect Contour {idx}", preview)
            cv2.waitKey(0)
            cv2.destroyWindow(f"Inspect Contour {idx}")

        # 3) Compute centroid
        M = cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        # 4) Classify shape by area
        kind = 'bowl' if is_bowl(area) else 'block'

        # 5) Create a mask for this contour, compute mean HSV inside it
        single_mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.drawContours(single_mask, [cnt], -1, 255, -1)
        mean_h, mean_s, mean_v, _ = cv2.mean(hsv, mask=single_mask)

        color = classify_color(mean_h, mean_s, mean_v)
        
        world_pt = pixel_to_world((cx, cy), img.shape)
        objects[color+ " " + kind] = [*world_pt, 0.05]

    return objects

if __name__ == "__main__":
    img = cv2.imread(IMG_PATH) 
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    objects = localize_objects(img, show_masks=True)
    print(objects)