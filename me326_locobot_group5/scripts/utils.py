import numpy as np
import cv2

### TRAJ FOLLOWING UTILS ###
def wrap_to_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi


### PERCEPTION UTILS ###
def projectPixelTo3d(camera, us, vs):
    """
    :param uv:        rectified pixel coordinates
    :type uv:         (u, v)
    Returns the unit vector which passes from the camera center to through rectified pixel (u, v),
    using the camera :math:`P` matrix.
    This is the inverse of :math:`project3dToPixel`.
    """
    x = (us - np.array(camera.cx())) / np.array(camera.fx())
    y = (vs - np.array(camera.cy())) / np.array(camera.fy())
    norm = np.sqrt(x*x + y*y + 1)
    # x = x / norm
    # y = y / norm
    # z = 1.0 / norm
    z = np.ones(x.shape)
    return np.stack([x, y, z], axis=-1)

def mask_grid(color_img, depth_img, meshgrid, camera, color_mask='r'):
    """Returns the black and white image with a color-mask for the specified color (white or the color where the color is, black everywhere else)
    
    Parameters
    ----------
    color_img : np.ndarray
        Raw input image of colored blocks on a table
    color_mask : string
        String indicating which color to draw the mask on; options 'r', 'g','b','y' (red, green, blue, yellow)
    
    Returns
    -------
    mask_img : np.ndarray
        Image with just the selected color shown (either with true color or white mask)
    """
    #Hint: one way would be to define threshold in HSV, leverage that to make a mask?
 
    #Step 1: Convert to HSV space; OpenCV uses - H: 0-179, S: 0-255, V: 0-255
    hsv_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2HSV)

    if color_mask == 'r':
        hsv_low = np.array([0, 100, 0])
        hsv_high = np.array([20, 255, 255])
    elif color_mask == 'g':
        hsv_low = np.array([40, 50, 0])
        hsv_high = np.array([80, 255, 255])
    elif color_mask == 'b':
        hsv_low = np.array([100, 170, 0])
        hsv_high = np.array([140, 255, 255])
    elif color_mask == 'y':
        hsv_low = np.array([20, 100, 0])
        hsv_high = np.array([40, 255, 255])
    else:
        raise NotImplementedError
    
    #Step 2: prep the mask
    mask = cv2.inRange(hsv_img, hsv_low, hsv_high)
    mask = np.array(mask.squeeze(), dtype=bool)

    #Step 3: Apply the mask; black region in the mask is 0, so when multiplied with original image removes all non-selected color 
    # masked = cv2.bitwise_and(color_img, color_img, mask=mask)
    # mask_xyz = cv2.bitwise_and(img_xyz, img_xyz, mask=mask)

    # Mask color
    mask_img = color_img[mask]

    # Mask depth
    mask_uv = meshgrid[mask]
    
    rays = np.array(projectPixelTo3d(camera, mask_uv[:, 0], mask_uv[:, 1]))

    mask_depth = depth_img.squeeze()[mask]

    mask_xyz = mask_depth[..., None] * rays

    return mask_img, mask_xyz