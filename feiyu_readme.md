


## Computer vision
### 1. Camera Calibration

We only used Baxter's left hand camera. It's camera info is already there in the topic, so no need for calibration.

But we did calibrate its head camera using Python. See /camera_calibration.

### 2. Get the Pose of Table Surface

We used a chessboard to do this. Given the chessboard corners' pos in image, and their real pos in chessboard frame, we solve **PnP** to get the transformation from **camera frame** to **chessboard frame**. By left multiplying another matrix, we get the transformation from **Baxter base frame** to **chessboard frame**.

### 3. Locate Object Pos in Baxter Frame
Suppose we get the object pos in image. We know the object is on the **table surface (left side of equations)**, and it's also on a **beam (right side of equatiosn)** shooted from camera's focal point. We solve the equations (X1=X2, Y1=Y2, 0=Z2) to get X1, Y1. These are the object's pos in Chessboard frame. Transform it to the Baxter's frame.

### 4. Detect All Dices in Image

We used Graph Based Image Segmentation ([paper](http://cs.brown.edu/people/pfelzens/segment/), [code](https://github.com/luisgabriel/image-segmentation)) and square detection to find all dices. If the dice's color and the table's color are enough distinguishable, this can work well.

We resize the image to 320x200 and then calls this algorithm. It takes 4s to compute.

### 5. Detect One Dice in Image

If we know there is a dice in the middle of the image, we first define a potential region it might be in, and then use Grabcut to segment it out.

### 6. Dice Value
The number of dots on the dice surface is the dice value. We use opencv Blob Detection to detect number of dots. 

This algorithm usually works bad. The dots in in dice are small and unclear.

### 7. Dice Color
Currently we get the dice rgb/hsv color by the dice region's median value. (Though not implemented) We can then use kNN and a small training set to determine the dice color.

### 8. Problems

The current performance of our computer vision code doesn't perform as robust as expected. 

For detecting dice, sometimes not all dices are detected. In an ideal condition, with sufficient lighting and uniform table color, the algorithm should work well. However, images from Baxter are dark, and there are also shadows.

For locating dice, there are about 3cm error when the Baxter's hand is 20 cm above the table. It comes from two folds:  
    1. The detected square region in image is not the accurate countour of the real dice.  
    2. The sensor data of Baxter's camera pos might not be so accurate.


## Services 

### Services from Computer Vision Part
**/mycvGetObjectInImage**: Detect the dice in the middle of the image, return pos in image.

**/mycvGetAllObjectsInImage**: Detect all dices in image, return pos in image

**/mycvCalibChessboardPose**: Calibrate the pose of table's surface. If there is a chessboard in image, detect its pose wrt camera, and then get and save its pose wrt baxter's base. 

**/mycvGetObjectInBaxter**: Call /mycvGetObjectInImage, and then transform pixel pos to world pos.

**/mycvGetAllObjectsInBaxter**: Call /mycvGetAllObjectsInImage, and then transform all objects' pixel pos to world pos.


