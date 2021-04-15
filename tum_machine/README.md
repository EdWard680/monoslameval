Download and unzip the file `LSD_machine.bag.zip` to this folder and rename it to `dataset.bag`

From: https://drive.google.com/drive/folders/1FaQMp_kAdjkhB8dxWW_nqdu_MB-gqbcU?usp=sharing

Note that this bag, unlike the ones we generated, has an uncompressed image topic `/image_raw`. To test with this dataset, use the following command.
```
roslaunch test_dataset.launch algorithm=<algorithm> dataset=tum_machine dataset_image_topic:=/image_raw decompress:=false
```

When running with LSD SLAM, note that the camera configuration is published on a topic, so the `_calib` argument should be deleted from `lsd_slam.sh` when running this file
