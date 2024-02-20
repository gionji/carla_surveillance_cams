
REST_API_ENDPOINT = "localhost/images/"
import requests


def upload_img(img, name, date_time, id):

    url = f'{REST_API_ENDPOINT}/{name}/{id}_crop.png'
    #response = requests.post(url, img)

    #if response.status_code == 200:
    print(f'Faked Uploaded image {id}_crop.png.')


