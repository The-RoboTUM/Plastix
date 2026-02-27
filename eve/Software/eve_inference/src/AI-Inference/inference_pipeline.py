import numpy as np
from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
from multiprocessing import Process, Queue
from PIL import Image
import logging
import torch
from ultralytics import YOLO
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent  # adjust based on your folder structure
if str(ROOT_DIR) not in sys.path:
    sys.path.append(str(ROOT_DIR))

from project_paths import SEG_WEIGHTS, YOLO_WEIGHTS, SEGMENTATION_IMG, YOLO_IMG, COMBINED_IMG, CLASSES_FILE
from interface.interface_node import main as ros_main


####################
# Preprocess
###################
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Convert Ids to Labels
with open(CLASSES_FILE, 'r') as fid:
    data = [l.split(',') for i, l in enumerate(fid) if i != 0]
    print(data)
    id2label = {x[0]: x[1][1:-1] for x in data}  # convert id's to labels
    print(id2label)

####################
# SEGMENTATION MODEL
####################

# load and define segmentation model from pretrained weights
def load_segmentation_model():
    logging.info("Loading the Segmentation model")
    
    feature_extractor = SegformerImageProcessor.from_pretrained("nvidia/segformer-b2-finetuned-ade-512-512") # feature extractor (defined by Huggingface)
    seg_model = SegformerForSemanticSegmentation.from_pretrained( # pretrained model
        SEG_WEIGHTS,
        return_dict = True
    )
    
    seg_model.eval() # evaluation mode

    logging.info("Successfully loaded the segmentation model")

    return seg_model, feature_extractor

'''Classify the background'''
def inference_segmentation(feature_extractor, seg_model, img: Image, save_Image: bool = False) -> np.array:
    logging.info("Starting segmentation inference...")
    
    #Define Model
    inputs = feature_extractor(images=img, return_tensors="pt") # preprocess the image before forwarding

    # inference
    with torch.no_grad():
        seg_output = seg_model(**inputs)
        # print(seg_output)
        logits = seg_output.logits # get logits (raw output)
        pred = torch.argmax(logits, dim=1).squeeze(0).cpu().numpy() # (H,W) -> convert image in tensor, where each pixel value of the image is assigned to a class
        

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

    # save Image
    if save_Image:
        seg_image = Image.fromarray(seg_rgb)
        seg_image.save(f"{SEGMENTATION_IMG}segmentation.png")

    logging.info("Segmentation inferenced finished!")
    return seg, seg_bgr # np.array seg -> each pixel is assigned to a class, np.array seg_bgr -> each class has a unique colour 

# ####################
# # YOLO MODEL
# ####################

def load_yolo_model():
    logging.info("Loading Yolo Model...")
    yolo_model = YOLO(YOLO_WEIGHTS) # load weights
    
    logging.info("Successfully loaded Yolo Model...")

    return yolo_model

'''Detect the Trash'''
def inference_yolo(yolo_model, img : Image, class_map: np.array, colour_map : np.array, save_img: bool = False) -> list[list[float]]:
    logging.info("Starting Inference with Yolo Model...")

    with torch.no_grad():
        yolo_output = yolo_model(img)

    # ####################
    # Determine Trash Background
    # ####################

        bboxes_list = [] # initialise list for saving the bounding boxes
        # get Bounding boxes
        for o in yolo_output:
            bboxes = o.boxes.data.cpu().numpy() # convert to numpy array and move to CPU
            original_image = o.orig_img
            o.orig_img = colour_map  # exchange the image with segmentation map
            
            # Calculate the Surface Area of the Trash
            for bbox in bboxes:
                xmin, ymin, xmax, ymax = bbox[:-2] # get Bounding Box Coordinates
                bboxes_list.append([xmin, ymin, xmax, ymax]) # add every bounding box to list
                roi = class_map[int(ymin):int(ymax), int(xmin):int(xmax)] # get the ROI -> Region Of Interest
                background_id = np.bincount(roi.flatten()).argmax() # flatten the array to one-dimensional array to determine the class
                label = id2label[str(background_id)]
                #o.names[background_id] = label  # change the name rubbish -> Trash (optional #TODO use a trained AI Model with the appropriate class labels, 
                                            #this is only an example)
            
            if save_img:
                o.names[0] = "Trash"
                o.save(f"{COMBINED_IMG}combined.png")
                o.orig_img = original_image
                o.save(f"{YOLO_IMG}bounding_box.png")

        logging.info("Yolo inferenced finished!")

        return bboxes_list


'''This function calculates the midpoint of the bounding box'''
def bbox_midpoint_calc (bboxes : list[list[float]]) -> list[(float, float)]:
    logging.info("Calculating the Bounding Box Midpoints...")
    
    # midpoint of bounding box
    midpoints = []
    
    for bbox in bboxes:
        midpoint_y = (bbox[1] + bbox[3]) // 2
        midpoint_x = (bbox[0] + bbox[2]) // 2
        midpoints.append((midpoint_x, midpoint_y))
    
    logging.info("Successfully calculated the Bounding Box Midpoints!")
    
    return midpoints

'''Run a complete completet Inference'''
def complete_inference (feature_extractor, segmentation_model, yolo_model, img : Image) -> list[list[float]]:

    class_map, colour_map = inference_segmentation(feature_extractor, segmentation_model, img) # classify backgoround

    bboxes = inference_yolo(yolo_model=yolo_model, img=img, class_map=class_map, colour_map=colour_map) # detect trash with bounding boxes

    bboxes_midpoint = bbox_midpoint_calc(bboxes=bboxes) # calculate the midpoint of the bounding boxes

    return bboxes_midpoint

def ai_run(image_queue, coordinates_queue):
    
    # load the models
    seg_model, feature_extractor = load_segmentation_model()
    yolo_model = load_yolo_model()
    logging.info("Loaded both models!")

    while True:

        img = image_queue.get() # get image from queue
        midpoints = complete_inference(feature_extractor=feature_extractor, segmentation_model=seg_model,  yolo_model=yolo_model, img=img) # run AI inference

        coordinates_queue.put(midpoints)
        
if __name__ == "__main__":
    logging.info("Starting Program")

    image_queue = Queue(maxsize=20)  # max 20 Images in the Buffer #TODO
    coordinates_queue = Queue(maxsize=50)  # max 50 Images in the coordinate list Buffer #TODO 

    # start ROS2-Node as seperate Process
    ros_process = Process(target=ros_main, args=(image_queue, coordinates_queue))
    ros_process.daemon = True # shut down if AI-Inference is down
    ros_process.start()

    # AI-Inference is Main Process
    ai_run(image_queue, coordinates_queue)

    ros_process.join()  














