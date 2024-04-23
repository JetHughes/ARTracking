#! ./bin/bash

# run tests for each video situation

# test 1
#./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/combination-cut.mp4 ./calibration.txt "IMAGE" ./image.png 20 > results_SIFT_2/combination-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/scale-cut.mp4 ./calibration.txt "IMAGE" ./image.png 20 > results_SIFT_2/scale-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/occlusion-cut.mp4 ./calibration.txt "IMAGE" ./image.png 20 > results_SIFT_2/occlusion-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/plane_rotation-cut.mp4 ./calibration.txt "IMAGE" ./image.png 20 > results_SIFT_2/plane_rotation-cut.txt
./out/build/x64-Debug/ARTracking.exe ./videos/new_video/cut/out_plane_rotation-cut.mp4 ./calibration.txt "IMAGE" ./image.png 20 > results_SIFT_2/out_plane_rotation-cut.txt