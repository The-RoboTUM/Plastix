Instructions:

running the code from terminal

1. Download the footage folder from https://nextcloud.itq.de/apps/files/files/2210?dir=/CirQmind%20Plastix%20%28S3%29/Additional%20Content/Eve/code/detect-and-localize and move it to the data folder. The files are kinda big, so I did not want to have them on Git

2. change directory to detect-and-localize folder  
    open an new terminal window (for example windows powershell).
    use `cd path/to/detect_adn_localize` 
    
3. start a new virtual enviornment (optional)  
    `python -m venv .venv`  
    `.venv\Scripts\activate`  

4. install requiremnts  
    `pip install --upgrade pip`  
    `pip install -r requirements.txt`  

5. run the programm  
    presets have been created to satisfy the following arguments:  
    --model (the yolo model you want to use)  
    --source: (videosource)  
    --thresh: (confidence threshhold)  
    --tags: (.csv file which defines the arrangement of apriltags used)  
    --yolo_frameskip (running yolo each frame can be intensive when )  
    to get your feet wet you can run the default preset by running python `main.py --preset default`
    if you want to try a live video you can use the `--preset engineering_5` which uses the built in laptop camera
    you can always override individual arguments, or use your own arguments example: `--preset default --model yolov8n.pt`

6. closing and deactivating  
    `Q` to close the window (video and opencv)  
    `deactivate` to deactivate the virtual enviornment  

