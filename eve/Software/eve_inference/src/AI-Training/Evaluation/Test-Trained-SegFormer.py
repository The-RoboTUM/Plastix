import numpy as np
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
from PIL import Image
import torch
import pprint as pp
import matplotlib.pyplot as plt

from pathlib import Path
ROOT_DIR = Path(__file__).resolve().parents[2] # src is ROOT PATH
WEIGHTS = ROOT_DIR /"AI-Training/segformer_weights" # Path to the weights

#weights_path = "../Train/segformer_finetuned-weights/"
weights_path = WEIGHTS

feature_extractor = SegformerImageProcessor.from_pretrained("nvidia/segformer-b2-finetuned-ade-512-512")

# load model
model = SegformerForSemanticSegmentation.from_pretrained(
    weights_path,
    return_dict=True,
)

model.eval() # evaluation mode

# IMAGE
image = Image.open("HERE IS YOU IMAGE PATH FOR TESTING") # load image #TODO 
image = np.array(image)
pp.pprint(image.shape)
inputs = feature_extractor(images=image, return_tensors="pt")

# inference
with torch.no_grad():
    model_output = model(**inputs)
    print(model_output)
    logits = model_output.logits
    pred = torch.argmax(logits, dim=1).squeeze(0).cpu().numpy() # (H,W)
    pp.pprint(pred)

colors = np.array([
    [255, 0, 255],        # Hintergrund = schwarz
    [255, 165, 0],      # Klasse 1 = rot
    [0, 0, 255],      # Klasse 2 = grün
], dtype=np.uint8)

# Segmentierungs-Maske in RGB umwandeln
seg_rgb = colors[pred]
#
plt.imshow(seg_rgb)
plt.axis("off")
plt.show()







