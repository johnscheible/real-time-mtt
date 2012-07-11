==============================================
= multi-target tracking method by Wongun Choi
==============================================

Compile
* Require opencv and boost library to be installed
 - sudo apt-get install libboost-dev or see http://www.boost.org/
 - refer to http://opencv.willowgarage.com/wiki/InstallGuide
1. move into the standalone_tracker directory : cd standalone_tracker
2. cmake .
3. make

Run
1. How to run
 1) Download/prepare a video to be processed : e.g. wget http://www.vision.ee.ethz.ch/~aess/cvpr2008/seq02-img-left.tar.gz
 2) unzip the images.
 3) generate imagelist file using following command : ls seq02-img-left/*.jpg > seq02_imlist.txt
 4) run detector to get detection bboxes and confidence map
 5) generate conflist file : ls seq02-img-left/*.conf > seq02_conflist.txt
 6) try tracking with track.sh
