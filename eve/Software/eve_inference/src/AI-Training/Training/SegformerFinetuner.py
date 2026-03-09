import pytorch_lightning as pl
from transformers import SegformerForSemanticSegmentation
import numpy as np
import evaluate
import torch
from torch import nn


"""Finetune SegFormer"""
class SegformerFinetuner(pl.LightningModule):

    def __init__(self, id2label, train_dataloader=None, val_dataloader=None, test_dataloader=None,
                 metrics_interval=100):
        super(SegformerFinetuner, self).__init__()
        self.id2label = id2label # assign labels from Dataset
        self.metrics_interval = metrics_interval

        # Define different Datasets
        self.train_dl = train_dataloader
        self.val_dl = val_dataloader
        self.test_dl = test_dataloader


        self.num_classes = len(id2label.keys()) # define the number of classes
        self.label2id = {v: k for k, v in self.id2label.items()} # assign labels to id

        # define the SegformerModel
        self.model = SegformerForSemanticSegmentation.from_pretrained(
            "nvidia/segformer-b0-finetuned-ade-512-512",
            return_dict=False,
            num_labels=self.num_classes,
            id2label=self.id2label,
            label2id=self.label2id,
            ignore_mismatched_sizes=True,
        )

        self.training_step_outputs = []
        self.validation_step_outputs = []
        self.test_step_outputs = []

        # load the metrics
        self.train_mean_iou = evaluate.load("mean_iou")
        self.val_mean_iou = evaluate.load("mean_iou")
        self.test_mean_iou = evaluate.load("mean_iou")

    """Pass the training data"""
    def forward(self, images, masks):
        outputs = self.model(pixel_values=images, labels=masks)
        return (outputs) # return the prediction

    """Train the model"""
    def training_step(self, batch, batch_nb):

        images, masks = batch['pixel_values'], batch['labels'] # seperate batches into images and masks

        outputs = self(images, masks) # pass them through model (forward pass)

        loss, logits = outputs[0], outputs[1] # get output (logit -> unnormalized score at the end of the model)

        self.training_step_outputs.append(loss)

        # logits should have same size as ground truth masks
        upsampled_logits = nn.functional.interpolate(
            logits,
            size=masks.shape[-2:], # -> [H, W]
            mode="bilinear",
            align_corners=False # don't distort the corners
        )

        predicted = upsampled_logits.argmax(dim=1) # take the class with the highest score

        # store predictions and labels for metrics
        self.train_mean_iou.add_batch(
            predictions=predicted.detach().cpu().numpy(),
            references=masks.detach().cpu().numpy()
        )

        # calculate metrics
        if batch_nb % self.metrics_interval == 0:

            metrics = self.train_mean_iou.compute(
                num_labels=self.num_classes,
                ignore_index=255,
                reduce_labels=False,
            )

            metrics = {'loss': loss, "mean_iou": metrics["mean_iou"], "mean_accuracy": metrics["mean_accuracy"]}

            for k, v in metrics.items():
                self.log(k, v) # logging for Tensor Board

            return (metrics)
        else:
            return ({'loss': loss})

    """Validate the model"""
    def validation_step(self, batch, batch_nb):

        images, masks = batch['pixel_values'], batch['labels']

        # pass through network
        outputs = self(images, masks)

        # get loss
        loss, logits = outputs[0], outputs[1]

        # upsample them
        upsampled_logits = nn.functional.interpolate(
            logits,
            size=masks.shape[-2:],
            mode="bilinear",
            align_corners=False
        )

        # get predictions
        predicted = upsampled_logits.argmax(dim=1)

        self.validation_step_outputs.append({'val_loss': loss, 'predictions': predicted, 'references': masks})

        # store metrics
        self.val_mean_iou.add_batch(
            predictions=predicted.detach().cpu().numpy(),
            references=masks.detach().cpu().numpy()
        )

        # return loss
        return ({'val_loss': loss})


    def on_validation_epoch_end(self):
        avg_val_loss = torch.stack([x["val_loss"] for x in self.validation_step_outputs]).mean()

        all_preds = np.concatenate([x["predictions"].cpu().numpy() for x in self.validation_step_outputs])
        all_refs = np.concatenate([x["references"].cpu().numpy() for x in self.validation_step_outputs])

        metrics = self.val_mean_iou.compute(predictions=all_preds, references=all_refs, num_labels=self.num_classes, ignore_index=255)

        self.log("val_loss", avg_val_loss)
        self.log("val_mean_iou", metrics["mean_iou"])
        self.log("val_mean_accuracy", metrics["mean_accuracy"])

        # Reset für nächste Epoche
        self.validation_step_outputs.clear()

    def test_step(self, batch, batch_nb):

        images, masks = batch['pixel_values'], batch['labels']

        outputs = self(images, masks)

        loss, logits = outputs[0], outputs[1]

        upsampled_logits = nn.functional.interpolate(
            logits,
            size=masks.shape[-2:],
            mode="bilinear",
            align_corners=False
        )

        predicted = upsampled_logits.argmax(dim=1)

        self.test_step_outputs.append({'test_loss': loss, 'predictions': predicted, 'references': masks})

        self.test_mean_iou.add_batch(
            predictions=predicted.detach().cpu().numpy(),
            references=masks.detach().cpu().numpy()
        )

        return ({'test_loss': loss})

    def on_test_epoch_end(self):
        avg_test_loss = torch.stack([x["test_loss"] for x in self.test_step_outputs]).mean()

        all_preds = np.concatenate([x["predictions"].cpu().numpy() for x in self.test_step_outputs])
        all_refs = np.concatenate([x["references"].cpu().numpy() for x in self.test_step_outputs])

        metrics = self.test_mean_iou.compute(predictions=all_preds, references=all_refs, num_labels=self.num_classes, ignore_index=255)

        self.log("test_loss", avg_test_loss)
        self.log("test_mean_iou", metrics["mean_iou"])
        self.log("test_mean_accuracy", metrics["mean_accuracy"])

        # Reset für nächste Epoche
        self.test_step_outputs.clear()


    def configure_optimizers(self):
        return torch.optim.Adam([p for p in self.parameters() if p.requires_grad], lr=2e-05, eps=1e-08)


    def train_dataloader(self):
        return self.train_dl


    def val_dataloader(self):
        return self.val_dl


    def test_dataloader(self):
        return self.test_dl