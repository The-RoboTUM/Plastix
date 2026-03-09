from torch.utils.data import Dataset
import os
from PIL import Image

class SemanticSegmentationDataset(Dataset):

    """Image (semantic) segmentation dataset."""

    def __init__(self, root_dir, feature_extractor):
        self.root_dir = root_dir
        self.feature_extractor = feature_extractor

        # extracts the class names of the dataset
        self.classes_csv_file = os.path.join(self.root_dir, "_classes.csv")
        with open(self.classes_csv_file, 'r') as fid:
            data = [l.split(',') for i, l in enumerate(fid) if i != 0]
        self.id2label = {x[0]: x[1] for x in data} # convert id's to labels

        image_file_names = [f for f in os.listdir(self.root_dir) if '.jpg' in f] # list of images
        mask_file_names = [f for f in os.listdir(self.root_dir) if '.png' in f] # list of masks

        # sort images and masks based on their filenames (string)
        self.images = sorted(image_file_names)
        self.masks = sorted(mask_file_names)

    """Get the length of the Dataset"""
    def __len__(self):
        return len(self.images)

    """Get data and prepare"""
    def __getitem__(self, idx):
        image = Image.open(os.path.join(self.root_dir, self.images[idx]))
        segmentation_map = Image.open(os.path.join(self.root_dir, self.masks[idx]))

        encoded_inputs = self.feature_extractor(image, segmentation_map, return_tensors="pt") # prepare the image (return as tensor)

        for k, v in encoded_inputs.items():
            encoded_inputs[k].squeeze_() # reduce dimension (delete Batchsize)

        return encoded_inputs