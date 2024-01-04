目前可以选择的hmi包括：

* opencv viz3d,维护者较少，不更新了，文档也少，但是代码结构清晰；
* rviz依赖ros，不推荐；
* open 3d，持续更新，有python版本，可以利用cyber rt开发一个python版本的hmi；
* im gui，带gui的架构，但是对于显示系统状态来说有一点大了？
* osg，古老的api，最近不更新；
* opengl：比较原始，api少；
* opencv，开发简单，但是只能2d显示，且不能旋转缩放；
* pcl viz，文档多，持续更新，可以尝试；

现在dreamview/drewview plus对于不熟悉前后端开发者太不友好了，想调试自己的系统状态非常需要一个轻量级的c/c++/python hmi！
所以，这里会尝试使用pcl viz, 我更喜欢pcl代码风格, 而不是open 3d代码风格来开发3d 版本的hmi.