from torch.utils.data import DataLoader
from transformers import  SegformerImageProcessor #SegformerFeatureExtractor

from SemanticSegmentationDataset import SemanticSegmentationDataset
from SegformerFinetuner import SegformerFinetuner
import pytorch_lightning as pl
from pytorch_lightning.loggers import TensorBoardLogger
from pytorch_lightning.callbacks.early_stopping import EarlyStopping
from pytorch_lightning.callbacks.model_checkpoint import ModelCheckpoint
import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent.parent  # adjust based on your folder structure
if str(ROOT_DIR) not in sys.path:
    sys.path.append(str(ROOT_DIR))

from project_paths import TRAIN_DATASET, VALID_DATASET, LOGS_FILE, SEG_WEIGHTS

def main ():
    feature_extractor = SegformerImageProcessor.from_pretrained("nvidia/segformer-b2-finetuned-ade-512-512") #SegformerFeatureExtractor.from_pretrained("nvidia/segformer-b2-finetuned-ade-512-512")
    feature_extractor.reduce_labels = False
    feature_extractor.size = 512

    train_dataset = SemanticSegmentationDataset(TRAIN_DATASET, feature_extractor)
    val_dataset = SemanticSegmentationDataset(VALID_DATASET, feature_extractor)

    batch_size = 1
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=3, prefetch_factor=8)
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size, num_workers=3, prefetch_factor=8)

    segformer_finetuner = SegformerFinetuner(train_dataset.id2label, train_dataloader=train_dataloader,
                                             val_dataloader=val_dataloader, test_dataloader=None, metrics_interval=10)

    early_stop_callback = EarlyStopping(
        monitor="val_loss",
        min_delta=0.00,
        patience=10,
        verbose=False,
        mode="min",
    )

    checkpoint_callback = ModelCheckpoint(save_top_k=1, monitor="val_loss", save_weights_only= False)

    # create Tensorboardlogger
    logger = TensorBoardLogger(
        save_dir=LOGS_FILE,  # store logs
        name="segformer_training"
    )

    trainer = pl.Trainer(
        devices=1,
        accelerator="gpu",
        callbacks=[early_stop_callback, checkpoint_callback],
        max_epochs=500,
        val_check_interval=len(train_dataloader),
        logger=logger,
    )

    segformer_finetuner.train()
    trainer.fit(model=segformer_finetuner) # train model
    #segformer_finetuner.model.save_pretrained("segformer_finetuned-weights") # save directory of weights
    segformer_finetuner.model.save_pretrained(SEG_WEIGHTS) # save weights to corresponding directory

if __name__=="__main__":
    main()