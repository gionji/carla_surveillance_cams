import requests
import cv2
import numpy as np

def get_boundary_from_content_type(content_type):
    """
    Extract the boundary string from the Content-Type header.

    Parameters:
    - content_type: The Content-Type header value.

    Returns:
    - The boundary string if found, None otherwise.
    """
    if 'boundary=' in content_type:
        boundary = content_type.split('boundary=')[1]
        # Handle cases where the boundary is quoted
        boundary = boundary.strip('"')
        return boundary
    return None

def fetch_first_image_from_stream(url):
    """
    Fetch the first image from a multipart HTTP stream.

    Parameters:
    - url: The URL from which to fetch the streamed image.

    Returns:
    - An image as a numpy array if successful, None otherwise.
    """

    response = requests.get(url)
    if response.status_code == 200:
        # Convert the response content into a NumPy array
        img_array = np.asarray(bytearray(response.content), dtype=np.uint8)

        # Decode the array into an image
        image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        return image
    else:
        print("Failed to fetch the image. Status code:", response.status_code)
        return None

# Example usage
url = "http://localhost:5000/crop_feed/rgb_image_01?id=1"
image = fetch_first_image_from_stream(url)
if image is not None:
    cv2.imshow('Image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("No image was fetched.")


