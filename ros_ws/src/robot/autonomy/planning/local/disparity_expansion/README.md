# README #

This package generates a world representation using disparity images. This enables planning in image space by applying C-space expansion in 2.5D disparity images.

It generates a pair expansion images: one for foreground expansion and one for background expansion.

Currently this package also has the following nodes

* `disparity_expansion`: Generate expanded disparity

* `disparity_conv`: Convert nerian disparity to float32

* `disparity_pcd`: Disparity image to point cloud.

### Who do I talk to? ###

* geetesh dubey (geeteshdubey@gmail.com)
## License 

BSD, see LICENSE