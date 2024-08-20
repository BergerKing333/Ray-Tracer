#pragma once

using color = vec3;
#include "material.h"

class camera {
public:
	double aspect_ratio;
	int image_width;
	int image_height;   // Rendered image height
	int samples_per_pixel = 10;
	int maxRayDepth = 10;

	double vfov = 90.0;
	point3 lookfrom = point3(0, 0, 0);
	point3 lookat = point3(0, 0, -1);
	vec3 vup = vec3(0, 1, 0);

	double defocus_angle = 0;
	double focus_dist = 10;

	camera() {}

    void render(cv::Mat& image, const hittable& world) {
		initialize();
		const int height = image.rows;
		const int width = image.cols;
		vec3** imageArray = new vec3*[height];
		for (int i = 0; i < height; i++) {
			imageArray[i] = new vec3[width];
			for (int j = 0; j < width; j++) {
				imageArray[i][j] = vec3(0, 0, 0);
			}
		}

		int totalRayCount = samples_per_pixel * image_height * image_width;

		for (int sample = 0; sample < samples_per_pixel; sample++) {
			for (int j = 0; j < image_height; j++) {
				if (j % 50 == 0) {
					cv::imshow("image", image);
					cv::waitKey(1);
					std::cout << "Scanlines remaining: " << totalRayCount - (j * width + sample * height * width) << '\n';
				}
				for (int i = 0; i < image_width; i++) {
					ray r = get_ray(i, j);
					imageArray[j][i] += ray_color(r, maxRayDepth, world);

					writeImage(image, j, i, imageArray[j][i] * (1.0 / (sample + 1)));
				}
			}
		}
	}


private:
	double pixel_samples_scale;
    point3 center;         // Camera center
    point3 pixel00_loc;    // Location of pixel 0, 0
    vec3   pixel_delta_u;  // Offset to pixel to the right
    vec3   pixel_delta_v;  // Offset to pixel below

	vec3 u, v, w;

	vec3 defocus_disk_u;
	vec3 defocus_disk_v;

    void initialize() {
        pixel_samples_scale = 1.0 / samples_per_pixel;
		
		image_height = int(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

		center = lookfrom;

        // Determine viewport dimensions.
		// auto focal_length = (lookfrom - lookat).length();
		auto theta = degrees_to_radians(vfov);
		auto h = std::tan(theta / 2);
		// auto viewport_height = 2 * h * focal_length;
		auto viewport_height = 2 * h * focus_dist;
        auto viewport_width = viewport_height * (double(image_width) / image_height);

		w = unit_vector(lookfrom - lookat);
		u = unit_vector(cross(vup, w));
		v = cross(w, u);

        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        auto viewport_u = viewport_width * u;
		auto viewport_v = viewport_height * -v;

        // Calculate the horizontal and vertical delta vectors from pixel to pixel.
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // Calculate the location of the upper left pixel.
		auto viewport_upper_left = center - (focus_dist * w) - viewport_u / 2 - viewport_v / 2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

		auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
		defocus_disk_u = defocus_radius * u;
		defocus_disk_v = defocus_radius * v;
    }

	void writeImage(cv::Mat image, int i, int j, color pixel_color) {
		static const interval intensity(0, .999);

		float r = linear_to_gamma(pixel_color.x());
		float g = linear_to_gamma(pixel_color.y());
		float b = linear_to_gamma(pixel_color.z());

		int rbyte = int(256 * intensity.clamp(r));
		int gbyte = int(256 * intensity.clamp(g));
		int bbyte = int(256 * intensity.clamp(b));

		image.at<cv::Vec3b>(i, j) = cv::Vec3b(bbyte, gbyte, rbyte);
	}

	inline double linear_to_gamma(double linear) {
		if (linear > 0) {
			return std::sqrt(linear);
		}
		return 0;
	}

	ray get_ray(int i, int j) {
		vec3 offset = vec3(random_double() - 0.5, random_double() - 0.5, 0);
		// If not using anti-aliasing:
		if (samples_per_pixel == 1) {
			offset = vec3(0, 0, 0);
		}

		vec3 pixel_sample = pixel00_loc + ((i + offset.x()) * pixel_delta_u) + ((j + offset.y()) * pixel_delta_v);
		
		vec3 ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
		vec3 direction = pixel_sample - ray_origin;

		return ray(ray_origin, direction);
	}

	point3 defocus_disk_sample() {
		auto p = random_in_unit_disk();
		return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
	}

	color ray_color(const ray& r, int depth, const hittable& world) {
		if (depth <= 0) {
			return color(0, 0, 0);
		}
		hit_record rec;
		if (world.hit(r, interval(0.001, infinity), rec)) {
			ray scattered;
			color attenuation;

			if (rec.mat->scatter(r, rec, attenuation, scattered)) {
				return attenuation * ray_color(scattered, depth - 1, world);
			}
			return color(0, 0, 0);

			// return 0.5 * (rec.normal + color(1, 1, 1));
		}

		vec3 unit_direction = unit_vector(r.direction());
		auto a = 0.5 * (unit_direction.y() + 1.0);
		return (1.0 - a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0);
	}
};