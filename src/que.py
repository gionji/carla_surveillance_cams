import numpy as np
from threading import Lock


class CropObject:
    def __init__(self, image, metadata, id):
        if not isinstance(image, np.ndarray):
            raise ValueError("Image must be a numpy array")
        self.image = image
        self.metadata = metadata
        self.id = id


class ImageQueue:
    def __init__(self, max_size,  name):
        self.max_size = max_size
        self.name = name
        self.lock = Lock()  # Protects accesses and modifications
        self.queue = []  # Stores (id, ImageData) tuples
        self.id_to_index = {}  # Maps id to index in self.queue for quick lookup

    def push(self, id, image_data):
        with self.lock:
            # Remove the oldest image if at max capacity
            if len(self.queue) >= self.max_size:
                oldest_id, _ = self.queue.pop(0)
                del self.id_to_index[oldest_id]

            # Append new image and update ID mapping
            self.queue.append((id, image_data))
            self.id_to_index[id] = len(self.queue) - 1

            #print(f"Adding object with id {id} to que {self.name}.")
            # Update indexes in id_to_index
            for i, (id, _) in enumerate(self.queue):
                self.id_to_index[id] = i

    def pop(self, image_id=-1):
        with self.lock:
            if image_id in self.id_to_index:
                index = self.id_to_index.pop(image_id)
                _, image_data = self.queue.pop(index)

                # Update indexes in id_to_index
                for i, (id, _) in enumerate(self.queue):
                    self.id_to_index[id] = i

                return image_data
            else:
                return None

    def is_empty(self):
        with self.lock:
            return len(self.queue) == 0


    def pop_oldest(self):
        with self.lock:
            if self.queue:
                oldest_id, obj = self.queue.pop(0)
                del self.id_to_index[oldest_id]

                # Update indexes in id_to_index
                for i, (id, _) in enumerate(self.queue):
                    self.id_to_index[id] = i

                return obj
            else:
                return None

    def inspect_flask(self):
        ret = []
        with self.lock:
            #print(f"Inspecting queue {self.name}.")
            for id, data in self.queue:
                item = {"ID": id, "Metadata": data.metadata}
                ret.append(item)

        return json.dumps(ret, cls=NumpyArrayEncoder)

    def inspect(self):
        ret = []
        with self.lock:
            #print(f"Inspecting queue {self.name}.")
            for id, data in self.queue:
                item = f"ID: {id}, data: {data}"
                ret.append(item)

        return ret
    def __len__(self):
        with self.lock:
            return len(self.queue)

import json
class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()  # Convert ndarray to list
        # Let the base class default method raise the TypeError
        return json.JSONEncoder.default(self, obj)

# Example Usage
if __name__ == "__main__":
    queue = ImageQueue(5, "gurka")

    # Simulate adding images with metadata in one thread
    for i in range(7):  # This will exceed the capacity, showing auto-removal
        image = np.random.rand(100, 100)  # Example image (100x100 random array)
        metadata = {"source": "camera_1", "timestamp": f"2024-03-14 12:{i:02}"}
        image_data = CropObject(image, metadata, id=i)
        queue.push(image_data)
        print(f"Pushed image_{i} with metadata: {metadata}")

    print(f"Queue size after pushes: {len(queue)}")

    # Simulate retrieving and removing an image by ID in another thread
    queue.inspect()
    image_id_to_retrieve = 3
    retrieved_image_data = queue.pop(image_id_to_retrieve)
    if retrieved_image_data is not None:
        print(f"Retrieved and removed {image_id_to_retrieve} with metadata: {retrieved_image_data.metadata}")
    else:
        print(f"{image_id_to_retrieve} not found")

    print(f"Queue size after retrieval: {len(queue)}")
    #queue.inspect()