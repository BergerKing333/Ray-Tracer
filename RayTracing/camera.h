#pragma once

using color = vec3;

class camera {
public:
	double aspect_ratio;
	int image_width;
	int    image_height;   // Rendered image height

	camera(double aspect_ratio, int image_width) : aspect_ratio(aspect_ratio), image_width(image_width) {
        initialize();
    }

    void render(cv::Mat& image, const hittable& world) {
		for (int j = 0; j < image_height; j++) {
			for (int i = 0; i < image_width; i++) {
				vec3 pixel_center = pixel00_loc + i * pixel_delta_u + j * pixel_delta_v;
				vec3 ray_direction = pixel_center - center;
				ray r(center, ray_direction);

				color pixel_color = ray_color(r, world);
				writeImage(image, j, i, pixel_color);
			}
		}
	}


private:
    
    point3 center;         // Camera center
    point3 pixel00_loc;    // Location of pixel 0, 0
    vec3   pixel_delta_u;  // Offset to pixel to the right
    vec3   pixel_delta_v;  // Offset to pixel below

    void initialize() {
        image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        center = point3(0, 0, 0);

        // Determine viewport dimensions.
        auto focal_length = 1.0;
        auto viewport_height = 2.0;
        auto viewport_width = viewport_height * (double(image_width) / image_height);

        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        auto viewport_u = vec3(viewport_width, 0, 0);
        auto viewport_v = vec3(0, -viewport_height, 0);

        // Calculate the horizontal and vertical delta vectors from pixel to pixel.
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // Calculate the location of the upper left pixel.
        auto viewport_upper_left =
            center - vec3(0, 0, focal_length) - viewport_u / 2 - viewport_v / 2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);
    }

	void writeImage(cv::Mat image, int i, int j, color pixel_color) {
		int r = pixel_color.x() * 255.999;
		int g = pixel_color.y() * 255.999;
		int b = pixel_color.z() * 255.999;

		image.at<cv::Vec3b>(i, j) = cv::Vec3b(b, g, r);
	}

	color ray_color(const ray& r, const hittable& world) {
		hit_record rec;
		if (world.hit(r, interval(0, infinity), rec)) {
			return 0.5 * (rec.normal + color(1, 1, 1));
		}

		vec3 unit_direction = unit_vector(r.direction());
		auto a = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0);
	}
};