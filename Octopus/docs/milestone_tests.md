# Milestone Tests

## 1. Marker field transform

Success:

- camera sees all 4 markers
- system computes image-to-map transform
- clicking an image point returns x,y map coordinate
- error below 10-20 cm on a 3-5 m field

## 2. Map builder

Success:

- visible cells become covered
- mock trash detection updates one grid cell
- map patch is published for backend/dashboard

Example:

- input: trash at x=2.0, y=1.0, confidence=0.9
- output: matching cell has trash probability > 0.8

## 3. Backend/dashboard

Success:

- backend receives map patch
- dashboard shows coverage
- dashboard shows trash location
