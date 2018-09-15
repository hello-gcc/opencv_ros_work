#########################################################################
# File Name: work_cam.sh
# Author: jones
# mail: kangjunderensheng@163.com
# Created Time: Sat 23 Jun 2018 09:15:43 AM CST
#########################################################################
#!/bin/bash

sudo rmmod uvcvideo

sudo insmod ./uvc/uvcvideo.ko quirks=128
