#!/bin/bash

# run tests for each video situation

# test 1
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/combination-cut.mp4 ./calibration.txt "IMPROVED" ./image.png 20 > results_SIFT_2/combination-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/scale-cut.mp4 ./calibration.txt "IMPROVED" ./image.png 20 > results_improved/scale-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/occlusion-cut.mp4 ./calibration.txt "IMPROVED" ./image.png 20 > results_improved/occlusion-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/plane_rotation-cut.mp4 ./calibration.txt "IMPROVED" ./image.png 20 > results_improved/plane_rotation-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/out_plane_rotation-cut.mp4 ./calibration.txt "IMPROVED" ./image.png 20 > results_improved/out_plane_rotation-cut.txt

./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/combination-cut.mp4 ./calibration.txt "IMAGE" ./image.png 20 > results_SIFT_2/combination-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/scale-cut.mp4 ./calibration.txt "IMAGE" ./image.png 20 > results_improved/scale-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/occlusion-cut.mp4 ./calibration.txt "IMAGE" ./image.png 20 > results_improved/occlusion-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/plane_rotation-cut.mp4 ./calibration.txt "IMAGE" ./image.png 20 > results_improved/plane_rotation-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/out_plane_rotation-cut.mp4 ./calibration.txt "IMAGE" ./image.png 20 > results_improved/out_plane_rotation-cut.txt

./out/build/x64-Debug/ARTracking.exe ./videos/old_video/cut/combination-cut.mp4 ./calibration.txt "IMPROVED" ./image.png 20 > results_improved_old_videos/combination-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/old_video/cut/scale-cut.mp4 ./calibration.txt "IMPROVED" ./image.png 20 > results_improved_old_videos/scale-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/old_video/cut/occlusion-cut.mp4 ./calibration.txt "IMPROVED" ./image.png 20 > results_improved_old_videos/occlusion-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/old_video/cut/plane_rotation-cut.mp4 ./calibration.txt "IMPROVED" ./image.png 20 > results_improved_old_videos/plane_rotation-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/old_video/cut/out_plane_rotation-cut.mp4 ./calibration.txt "IMPROVED" ./image.png 20 > results_improved_old_videos/out_plane_rotation-cut.txt