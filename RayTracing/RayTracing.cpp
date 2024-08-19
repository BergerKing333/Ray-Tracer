#include <opencv2/opencv.hpp>

#include "utils.h"

#include "hittable.h"
#include "hittable_list.h"
#include "sphere.h"
#include "camera.h"

using color = vec3;




int main()
{
    double aspect_ratio = 16.0 / 9.0;
    int image_width = 800;

    camera cam(aspect_ratio, image_width);

    int image_height = cam.image_height;

    hittable_list world;

    world.add(make_shared<sphere>(point3(0, 0, -1), 0.5));
    world.add(make_shared<sphere>(point3(0, -100.5, -1), 100));

    // Camera
    cv::Mat image(image_height, image_width, CV_8UC3, cv::Scalar(0, 0, 0));
	
    cam.render(image, world);

	cv::imshow("image", image);
	cv::waitKey(0);

	return 0;

}

