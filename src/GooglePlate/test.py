from google.cloud import vision

def detect_text(image):
    """Detects text in the file located in Google Cloud Storage or on the Web.
    """
    client = vision.ImageAnnotatorClient()
    

    response = client.text_detection(image=image)
    texts = response.text_annotations
    print('Texts:')

    for text in texts:
        print('\n"{}"'.format(text.description))

        vertices = (['({},{})'.format(vertex.x, vertex.y)
                    for vertex in text.bounding_poly.vertices])

        print('bounds: {}'.format(','.join(vertices)))

    if response.error.message:
        raise Exception(
            '{}\nFor more info on error messages, check: '
            'https://cloud.google.com/apis/design/errors'.format(
                response.error.message))

if __name__ == "__main__":
    import io
    import os

    # Imports the Google Cloud client library
    from google.cloud import vision
    from google.cloud.vision import types

    # Instantiates a client
    client = vision.ImageAnnotatorClient()

    # The name of the image file to annotate
    file_name = os.path.abspath('testimg.jpg')

    # Loads the image into memory
    with io.open(file_name, 'rb') as image_file:
        content = image_file.read()

    image = types.Image(content=content)

    # Performs label detection on the image file
    response = client.label_detection(image=image)
    labels = response.label_annotations
    detect_text(image)



    print('Labels:')
    for label in labels:
        print(label.description)