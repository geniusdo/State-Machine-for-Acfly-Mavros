#include "../include/camera.h"

int main()
{
    cv::Mat img = cv::imread("/home/icarus/vio/cam/cam0/1663993472.245903.jpg");
    cv::imshow("test1", img);
    cv::waitKey();
    camera left_cam = camera("cam1");
    bool load = left_cam.LoadFromYAML("/home/icarus/vio/cam/yaml/test.yaml");
    if (load == false)
    {
        printf("load fail\n");
        return 0;
    }
    left_cam.frame = img;
    bool dedist = left_cam.DeDistortion();
    if (dedist == false)
    {
        printf("dedist fail\n");
        return 0;
    }
    cv::waitKey();
    cv::imshow("test", left_cam.frame_undistort);
    cv::waitKey();
    return 0;
}