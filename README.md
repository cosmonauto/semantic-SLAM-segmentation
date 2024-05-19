# A Unique Semantic Segmentation & SLAM Project
=======
#This integrated semantic segmentation and SLAM mapping code was a graduation project.


Some results
=======
![result1][1]   
![result2][6]
![result3][2]
![result4][3]
![result5][4]
![result6][5]

[1]: https://github.com/cosmonauto/semantic-SLAM-segmentation/blob/master/001.png
[2]: https://github.com/cosmonauto/semantic-SLAM-segmentation/blob/master/2.png
[3]: https://github.com/cosmonauto/semantic-SLAM-segmentation/blob/master/3.png
[4]: https://github.com/cosmonauto/semantic-SLAM-segmentation/blob/master/0002.png
[5]: https://github.com/cosmonauto/semantic-SLAM-segmentation/blob/master/0002.jpg
[6]: https://github.com/cosmonauto/semantic-SLAM-segmentation/blob/master/000000.png

Models
====
>- Model files are not included, you can choose to modify this file:

```
//segnet.cpp
    std::string model_file = "../models/segnet_model_driving_webdemo.prototxt";
    std::string trained_file = "../models/segnet_weights_driving_webdemo.caffemodel";
    std::string label_file = "../models/semantic12.txt";
```


> - [ ] The semantic segmentation method is based on [Alex Kendall's work ](https://github.com/cosmonauto/caffe-segnet) 

>- [ ] Caffe Segnet is a modified version of **Caffe** which supports the **SegNet** architecture

>- [ ] The mapping architecture is based on GaoXiang's work and **ORB-SLAM**.


----
Getting Started
=======
If you would like to try out this code , you should satisfy these requirements first.

#OpenCV
Used for image and feature manipulation. Download and install in