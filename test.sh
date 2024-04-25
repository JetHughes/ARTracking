#!/bin/bash

# run tests for each video situation

# test 1
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/combination-cut.mp4			./calibration.txt "IMPROVED" ./image.png 20 > results_improved/combination-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/scale-cut.mp4				./calibration.txt "IMPROVED" ./image.png 20 > results_improved/scale-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/occlusion-cut.mp4			./calibration.txt "IMPROVED" ./image.png 20 > results_improved/occlusion-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/plane_rotation-cut.mp4		./calibration.txt "IMPROVED" ./image.png 20 > results_improved/plane_rotation-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/out_plane_rotation-cut.mp4	./calibration.txt "IMPROVED" ./image.png 20 > results_improved/out_plane_rotation-cut.txt

./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/combination-cut.mp4			./calibration.txt "IMAGE" ./image.png 20 > results_baseline/combination-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/scale-cut.mp4				./calibration.txt "IMAGE" ./image.png 20 > results_baseline/scale-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/occlusion-cut.mp4			./calibration.txt "IMAGE" ./image.png 20 > results_baseline/occlusion-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/plane_rotation-cut.mp4		./calibration.txt "IMAGE" ./image.png 20 > results_baseline/plane_rotation-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/out_plane_rotation-cut.mp4	./calibration.txt "IMAGE" ./image.png 20 > results_baseline/out_plane_rotation-cut.txt