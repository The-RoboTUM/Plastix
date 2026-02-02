import numpy as np
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
from PIL import Image
import torch
from ultralytics import YOLO
import os


####################
# Preprocess Stuff
###################
# Classes File
script_dir = os.path.dirname(__file__)  # Ordner des Skripts
classes_csv_file = os.path.join(script_dir, "_classes.csv")

# Convert Ids to Labels
with open(classes_csv_file, 'r') as fid:
    data = [l.split(',') for i, l in enumerate(fid) if i != 0]
    print(data)
    id2label = {x[0]: x[1][1:-1] for x in data}  # convert id's to labels
    print(id2label)

# Prepare Image
#image = Image.open("Evaluation_Image.jpg")


####################
# SEGMENTATION MODEL
####################
'''Classify the background'''
def inference_segmentation(img: Image) -> np.array:
    #Define Model
    seg_weights_path = "../Background_Classification/Train/segformer_finetuned-weights/" # path to the weights
    feature_extractor = SegformerImageProcessor.from_pretrained("nvidia/segformer-b2-finetuned-ade-512-512") # feature extractor (defined by Huggingface)
    seg_model = SegformerForSemanticSegmentation.from_pretrained( # pretrained model
        seg_weights_path,
        return_dict=True,
    )
    seg_model.eval() # evaluation mode

    inputs = feature_extractor(images=img, return_tensors="pt") # preprocess the image before forwarding

    # inference
    with torch.no_grad():
        seg_output = seg_model(**inputs)
        # print(seg_output)
        logits = seg_output.logits # get logits (raw output)
        pred = torch.argmax(logits, dim=1).squeeze(0).cpu().numpy() # (H,W) -> convert image in tensor, where each pixel value of the image is assigned to a class
        print(pred.device)

    # Define Colour Array -> for every Class a specific colour
    seg = Image.fromarray(pred.astype("uint8")).resize(img.size, Image.NEAREST) # resize the image
    seg = np.array(seg)

    colors = np.array([
        [0,0 ,0],        # Background = black
        [255, 165, 0],      # Class 1 -> Beach (ORANGE)
        [0, 0, 255],      # Class 2 -> Water (BLUE)
    ], dtype=np.uint8)
    seg_rgb = colors[seg] # assign every pixel a colour
    seg_bgr = seg_rgb[..., ::-1].copy()

    return seg, seg_bgr # np.array seg -> each pixel is assigned to a class, np.array seg_bgr -> each class has a unique colour 

# Print only Segmentation Model
# plt.imshow(seg_rgb)
# plt.axis("off")
# plt.show()

# ####################
# # YOLO MODEL
# ####################
'''Detect the Trash'''
def inference_yolo(img : Image, class_map: np.array, colour_map : np.array) -> list[list[float]]:
    
    yolo_weights_path = "../Trash Size Detection/fotogenic_v8s.pt" # path to the weights
    yolo_model = YOLO(yolo_weights_path) # load weights

    with torch.no_grad():
        yolo_output = yolo_model(img)

    # ####################
    # Determine Trash Background
    # ####################

        bboxes_list = [] # initialise list for saving the bounding boxes
        # get Bounding boxes
        for o in yolo_output:
            bboxes = o.boxes.data.cpu().numpy() # convert to numpy array and move to CPU
            o.orig_img = colour_map  # exchange the image with segmentation map
            
            # Calculate the Surface Area of the Trash
            for bbox in bboxes:
                xmin, ymin, xmax, ymax = bbox[:-2] # get Bounding Box Coordinates
                bboxes_list.append([xmin, ymin, xmax, ymax]) # add every bounding box to list
                roi = class_map[int(ymin):int(ymax), int(xmin):int(xmax)] # get the ROI -> Region Of Interest
                background_id = np.bincount(roi.flatten()).argmax() # flatten the array to one-dimensional array to determine the class
            label = id2label[str(background_id)]
            o.names[0] = label  # change the name rubbish -> Trash

        for i, r in enumerate(yolo_output):
            # Plot results image
            im_bgr = r.plot()  # BGR-order numpy array
            im_rgb = img.fromarray(im_bgr[..., ::-1])  # RGB-order PIL image

            # Show results to screen (in supported environments)
            r.show()

            # Save results to disk
            r.save(filename=f"results{i}.jpg")

        return bboxes_list


'''This function calculates the midpoint of the bounding box'''
def bbox_midpoint_calc (bboxes : list[list[float]]) -> list[(float, float)]:

    # midpoint of bounding box
    midpoints = []
    for bbox in bboxes:
        
        midpoint_y = (bboxes[1] + bboxes[3]) // 2
        midpoint_x = (bboxes[0] + bboxes[2]) // 2
        midpoints.append((midpoint_x, midpoint_y))
    
    return midpoints

'''Run a complete completet Inference'''
def complete_inference (img : Image) -> list[list[float]]:

    class_map, colour_map = inference_segmentation(img) # classify backgoround

    bboxes = inference_yolo(image=img, class_map=class_map, colour_map=colour_map) # detect trash with bounding boxes

    bboxes_midpoint = bbox_midpoint_calc(bboxes=bboxes) # calculate the midpoint of the bounding boxes

    return bboxes_midpoint












