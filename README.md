# C-Implementation of Object Tracking for Tally

## Description
This repository contains generic (portable) c code to parse/filter predictions from object detection model and track objects in consecutive frames.

## Main components
* Post processing pipeline with NMS, score and top-K filtering
* Tracking pipeline to associate objects with tracks
* Hungarian algorithm to solve object-tracklet assignment problem
* Predictive object position estimator using Kalman Filter
* Memory-optimizing preprocessor based Kalman Filter factory
* Algorithmically optimized matrix/matrix and matrix/vector operations
* C-based unit testing functions
* Matrix inverse using Cholesky decomposition

## References
This repository sources from https://github.com/sunsided/kalman-clib. Only kalman filter implementation remained (heavility modified), whereas tracking algorithm is developed by ETA.
