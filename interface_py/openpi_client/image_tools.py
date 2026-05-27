import numpy as np


def convert_to_uint8(image):
    arr = np.asarray(image)
    if arr.dtype == np.uint8:
        return arr
    if np.issubdtype(arr.dtype, np.floating):
        if arr.size and float(np.nanmax(arr)) <= 1.0:
            arr = arr * 255.0
        return np.clip(arr, 0, 255).astype(np.uint8)
    return np.clip(arr, 0, 255).astype(np.uint8)

