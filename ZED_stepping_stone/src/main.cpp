#include <iostream>
#include <algorithm>
#include <string>
#include <cmath>

// some of these includes might not be useful
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <setjmp.h>
#include <signal.h>
#include <dirent.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sl/Camera.hpp>
#include <sl/types.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

/* This function converts sl::Mat images to cv::Mat images
 * input - sl::Mat image
 */
cv::Mat slMat2cvMat(sl::Mat &input) {
    //Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

/* This function is used to send the estimated distance back to the robot
 * via TCP/IP connection.
 * confd - the file descriptor ID for the accpted client
 * distance - the distance to send back to the robot */
void tcp_sendback(int connfd, double distance) {
    bool use_meter = true;
    if (use_meter) {
	distance = distance / 1000.0;
    }

    // This chunk of code is used to convert the 'double' type distance
    // into bits characterized by 8 char type array (64 bits in total)
    // Because this old library code dictates that things need to be in C fashion strings.
    char buf[8];
    long data_tmp;
    data_tmp = *reinterpret_cast<long*>(&distance);
    for (int i = 7; i >= 0; i--) {
	buf[i] = data_tmp % (1 << 8);
        data_tmp = data_tmp >> 8;
    }

    write(connfd, buf, 8);
}

/* This function is imported from an old library code. It is used to listen 
 * for client connections.
 * port -  the specified port number. */
int open_listenfd(char *port)
{
    struct addrinfo hints, *listp, *p;
    int listenfd, optval=1;

    /* Get a list of potential server addresses */
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_socktype = SOCK_STREAM;             /* Accept connections */
    hints.ai_flags = AI_PASSIVE | AI_ADDRCONFIG; /* ... on any IP address */
    hints.ai_flags |= AI_NUMERICSERV;            /* ... using port number */
    getaddrinfo(NULL, port, &hints, &listp);

    /* Walk the list for one that we can bind to */
    for (p = listp; p; p = p->ai_next) {
        /* Create a socket descriptor */
        if ((listenfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) < 0)
            continue;  /* Socket failed, try the next */

        /* Eliminates "Address already in use" error from bind */
        setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR,    //line:netp:csapp:setsockopt
                   (const void *)&optval , sizeof(int));

        /* Bind the descriptor to the address */
        if (bind(listenfd, p->ai_addr, p->ai_addrlen) == 0)
            break; /* Success */
        close(listenfd); /* Bind failed, try the next */
    }

    /* Clean up */
    freeaddrinfo(listp);
    if (!p) /* No address worked */
        return -1;

    /* Make it a listening socket ready to accept connection requests */
    if (listen(listenfd, 1024) < 0) {
        close(listenfd);
        return -1;
    }
    return listenfd;
}

/* This function judges if a pixel belongs to the stepping stones. (Not very generalizable)
 * x,y,z - the coordinates of the point in the world that the pixel corresponds to
 * color_tmp - the encoded color value (decoding is done in this function) */
bool isValidPixel(float x, float y, float z, float color_tmp) {
    float min_height = -100;
    float max_height = 250;
    float dist_threshold = 3000;
    unsigned int color = *reinterpret_cast<unsigned int*>(&color_tmp);

    // If the pixel's corresponding point location has a valid height and not very far away
    // note: z are all negative values
    if ((y > min_height) && (y < max_height) && (abs(z) < dist_threshold)) {
        // the color is stored as a 32 bit float
        // first 8 bit is alpha, second 8 bit is blue, third 8 bit is green, last 8 bit is red
        // The following three lines of codes decode the color and normalize 
        // them to values between 0 and 1
        float blue = ((color % (1 << 24)) >> 16) / 255.0;
        float green = ((color % (1 << 16)) >> 8) / 255.0;
        float red = (color % (1 << 8)) / 255.0;

        // the following chunk of codes convert RGB valuse to HSV values
        // HSV values are easier for color identification
	float cl_max, cl_min, cl_diff, hue, saturation, value;
        // find the min and max of the RGB channel values
	if (red > green) {
	    if (red > blue) {
		cl_max = red;
		if (green > blue) {
		    cl_min = blue;
		} else {
		    cl_min = green;
		}   
	    } else {
		cl_max = blue;
		cl_min = green;
	    }
	} else if (green > blue) {
	    cl_max = green;
	    if (blue > red) {
		cl_min = red;
	    } else {
		cl_min = blue;
	    }   
	} else {
	    cl_max = blue;
	    cl_min = red;
	}
        // get the Hue, Saturation and Value values
	cl_diff = cl_max - cl_min;
	value = cl_max;
	if (cl_max > 0) {
	    saturation = cl_diff / cl_max;
	} else {
	    saturation = 0;
	    hue = 0;
	}
	if (cl_max == red) {hue = (green-blue)/cl_diff;}
	if (cl_max == green) {hue = (blue - red)/cl_diff + 2;}
	if (cl_max == blue) {hue = (red - green)/cl_diff + 4;}
	hue = hue * 60.0;
	if (hue < 0) {hue = hue + 360.0;}

        // judges if the color is the stepping stone color
	if (((hue < 20) || (hue > 340)) && (saturation > 0.2) && (value > 0.2)) {//(saturation < 0.475) && (value > 0.25) && (value < 0.5)) {
	    return true;
	}   
    }
    return false;
}

/* the main function takes in the port number */
int main (int argc, char** argv) {
    
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.coordinate_units = sl::UNIT_MILLIMETER;
    init_params.camera_fps = 90; // NOTE: 90 means 100 in ZED SDK (Are you kidding me!?)
    init_params.camera_resolution = sl::RESOLUTION_VGA;

    // the coordinate system is based on the center of left camera
    // with right hand rule applied and y pointing up.
    init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    //init_params.depth_minimum_distance = 1000; // default value is 700
    init_params.depth_mode = sl::DEPTH_MODE_QUALITY;
    init_params.sdk_verbose = true;

    // significantly improves speed by setting this to 0
    init_params.camera_buffer_count_linux = 0; 
    
    // open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::SUCCESS) {
        cout << errorCode2str(err) << endl;
        zed.close();
        return EXIT_FAILURE;
    }
    zed.disableTracking();

    sl::Mat zed_image(zed.getResolution(), sl::MAT_TYPE_8U_C4);
    sl::Mat point_cloud;
    int img_height, img_width;
    float x, y, z, color;

    int area_threshold = 1600; // area threshold value can be adjusted here

    sl::float4 point_val;

    vector<unsigned int> equivalances(1, 0);
    
    // grab a frame from ZED initially to get the size information
    err = zed.grab();
    if (err == sl::SUCCESS) {
        zed.retrieveImage(zed_image, sl::VIEW_LEFT);
        img_height = zed_image.getHeight();
        img_width = zed_image.getWidth();
    }
    int file_counter = 0;
    int tmp_up, tmp_left;
    int counter = 0;

    int listenfd1, listenfd2, connfd1, connfd2, clientlen;
    char* port1;
    char* port2;
    struct sockaddr_in clientaddr;
    struct hostent *hp;
    char *haddrp;
    if (argc != 3) {
        cout << "need to specify a port! and a port only!" << endl;
        exit(0);
    }
    port1 = argv[1];
    port2 = argv[2];

    // open the sever to listen for clients
    listenfd1 = open_listenfd(port1);
    listenfd2 = open_listenfd(port2);

    while (1) { // we want the server to keep running when the client leaves (just in case)

	    clientlen = sizeof(clientaddr);
	    // acccept a client
	    connfd1 = accept(listenfd1, (struct sockaddr*)&clientaddr, (socklen_t*)&clientlen);
	    connfd2 = accept(listenfd2, (struct sockaddr*)&clientaddr, (socklen_t*)&clientlen);
   
	    while (1) { // this is THE loop

		err = zed.grab(); // grab a frame

		// read camera angle from client(robot) and parse the data
		double cam_angle, cam_height;
	        long data_tmp;
		char buf[8];
		while (read(connfd1, buf, 64) == 0) {
		   cout << "No data received!" << endl;
		   sleep(1);
		}
	        data_tmp = 0;
	        for (int j = 0; j < 8; j++) {
	            data_tmp = data_tmp << 8;
                    data_tmp = data_tmp | buf[j];
	        }
	        cam_angle = *reinterpret_cast<double *>(&data_tmp);
		cam_height = cam_angle / 10;
		if (cam_height > 0) {
		    cam_height = floor(cam_height);
		} else {
		    cam_height = ceil(cam_height);
		}
		cam_angle = cam_angle - cam_height * 10;
		cam_height = abs(cam_height);
		cout << cam_height << "  ||  " << cam_angle << "    ++++++++++++" << endl; 

		// the camera's pose in the world frame
		float rotate_x = -30/180.0*3.14159265;
		//float rotate_x = -cam_angle/180.0*3.14159265;
		float rotate_y = 0;
		float rotate_z = 0;
		float translate_x = 0;
		float translate_y = 890;
		float translate_z = 0;

		// initialize the rotation and translation matrices as arrays
		float xdata[] = {1, 0, 0, 0, cos(rotate_x), -sin(rotate_x), 0, sin(rotate_x), cos(rotate_x)};
		float ydata[] = {cos(rotate_y), 0, sin(rotate_y), 0, 1, 0, -sin(rotate_y), 0, cos(rotate_y)};
		float zdata[] = {cos(rotate_z), -sin(rotate_z), 0, sin(rotate_z), cos(rotate_z), 0, 0, 0, 1};
		float tdata[] = {1, 0, 0, translate_x, 0, 1, 0, translate_y, 0, 0, 1, translate_z, 0, 0, 0, 1};
		// conver to matrices
		sl::Matrix3f rotate_xmat(xdata);
		sl::Matrix3f rotate_ymat(ydata);
		sl::Matrix3f rotate_zmat(zdata);
		sl::Matrix4f translate(tdata);
		// combine (multiply) to get the ultimate matrix
		sl::Matrix4f rotate = translate.identity(); 
		rotate.setSubMatrix3f(rotate_xmat * rotate_ymat * rotate_zmat, 0, 0);
		sl::Matrix4f transform = translate * rotate;

		// matrix to store the data point in the local frame
		sl::Matrix4f data_point_mat = transform.identity(); 
		// matrix to store the data point in the world frame
		sl::Matrix4f product;
    
		if (err == sl::SUCCESS) {
		    // uncomment these to visualize the camera images (also the last few lines at the end of the loop)
		    zed.retrieveImage(zed_image, sl::VIEW_LEFT);
		    //cv::Mat cv_image = slMat2cvMat(zed_image);
		    //cv::imshow("camera_image_view", cv_image);

		    // get the point cloud image
		    zed.retrieveMeasure(point_cloud, sl::MEASURE_XYZRGBA);
		    // judge is a mask image where 0 means not part of the segment, vice versa
		    cv::Mat judge(img_height, img_width, CV_8UC1, cv::Scalar(0));

		    for (int i = 1; i < judge.rows; i++) {
			for (int j = 1; j < judge.cols; j++) {

			    // get the information at this pixel location and decode into locations and value
			    point_cloud.getValue(j, i, &point_val);
			    color = point_val.w;
			    point_val.w = 1;
			    data_point_mat.setSubVector4f(point_val, 3);
			    product = transform * data_point_mat; // transform locations into world frame
			    x = product.tx;
			    y = product.ty;
			    z = product.tz;

			    // if it is judged as valid pixel
			    if (isValidPixel(x, y, z, color)) {
				judge.at<uchar>(i, j) = 1;
			    } else {
				judge.at<uchar>(i, j) = 0;
			    }
			}
		    }

		    // get rid of tiny noises, then fill in the tiny holes and gaps within the segments
		    int num_iter = 3;
		    cv::Mat judge_tmp;
	 	    cv::Mat element = cv::Mat(); // by default a 3x3 mask, (-1, -1) points to the center of the mask
		    cv::dilate(judge, judge_tmp, element, cv::Point(-1, -1), num_iter);
		    cv::erode(judge_tmp, judge, element, cv::Point(-1, -1), num_iter);
		    for (int i = 1; i < judge.rows; i++) {
			for (int j = 1; j < judge.cols; j++) {
			    judge.at<uchar>(i, j) = (judge.at<uchar>(i, j) != 0);
			}
		    }

		    // double raster segmentation
		    for (int i = 1; i < judge.rows; i++) {
			for (int j = 1; j < judge.cols; j++) {
			    if (judge.at<uchar>(i, j) == 1) {
				tmp_up = judge.at<uchar>(i - 1, j);
				tmp_left = judge.at<uchar>(i, j - 1);
				// if up and left both 0, viewed as new segment discovered
				// counter is used to label this newly discovered segment
				if ((tmp_up == 0) && (tmp_left == 0)) {
				    counter++;
				    equivalances.push_back(counter);
				    judge.at<uchar>(i, j) = counter;
				// if either left or up is non zero (but not both)
				// then this pixel belongs to the segment left or up
				} else if ((tmp_up != 0) && (tmp_left == 0)) {
				    judge.at<uchar>(i, j) = tmp_up;
				} else if ((tmp_up == 0) && (tmp_left != 0)) {
				    judge.at<uchar>(i, j) = tmp_left;
				// if both are non zero, then these two segments are essentially the
				// same segment. Note the equivalence and then pick any label
				// equivalences are noted by using array indexes, for example:
				// 1 2 2 3 5, then we have 1 is 1, 2 is 2, 3 is 2, 4 is 3 then is 2, 5 is 5
			  	// the 0th index is ignored
				} else {
				    if (tmp_up < tmp_left) {
					judge.at<uchar>(i, j) = tmp_up;
					equivalances[tmp_left] = tmp_up;
				    } else if (tmp_up == tmp_left) {
					judge.at<uchar>(i, j) = tmp_up;
				    }
				    else {
					judge.at<uchar>(i, j) = tmp_left;
					equivalances[tmp_up] = tmp_left;
				    }
				}
			    }
			}
		    }

		    int eq_length = 0;
		    int temp;
		    // resolving equivalences by making 1 2 2 3 5, 1 2 2 2 5
		    for (int i = 0; i < equivalances.size(); i++) {
			temp = equivalances[i];
			while (equivalances[temp] != temp) {
			    temp = equivalances[temp];
			}
			equivalances[i] = temp;
			if (temp > eq_length) {
			    eq_length = temp;
			}
		    }
		    eq_length++; // this is essentially counter+1, but needs to be tested

		    // some more initializations
		    long area[eq_length];
		    long x_tot[eq_length];
		    long y_tot[eq_length];
		    for (int i = 0; i < eq_length; i++) {
			area[i] = 0;
			x_tot[i] = 0;
			y_tot[i] = 0;
		    }
		    int x_center;
		    int y_center;
		    // estimate the area for each segment and calculate the sum of x and y coordinates
		    for (int i = 0; i < judge.rows; i++) {
			for (int j = 0; j < judge.cols; j++) {
			    temp = judge.at<uchar>(i, j);
			    if (temp > 0) {
				area[equivalances[temp]]++;
				x_tot[equivalances[temp]] += i;
				y_tot[equivalances[temp]] += j;
			    }
			    judge.at<uchar>(i, j) = (temp > 1);
			}
		    }

		    vector<double> horizontal_distances;
		    for (int i = 0; i < eq_length; i++) {
			// if the saegment is too small then it's not a stepping-stone
			if (area[i] > area_threshold) {
			    // get the geometric centers of the segments
			    x_center = x_tot[i] / area[i];
			    y_center = y_tot[i] / area[i];
			    // get the point cloud value and transform the location into world frame
			    point_cloud.getValue(y_center, x_center, &point_val);
			    point_val.w = 1;
			    data_point_mat.setSubVector4f(point_val, 3);
			    product = transform * data_point_mat;

			    // estimate the distance to the center of the stepping-stone
			    horizontal_distances.push_back(sqrt(product.tz * product.tz + product.tx * product.tx));
			    cout << i << " || " << area[i] << " || " << x_center << "  ||  " << y_center << "  ||  " << (sqrt(product.tz * product.tz + product.tx * product.tx)) << endl;
			}
		    }
		    cout << "======================================================" << endl;

		    // larger stepping-stone appearances mean closer distance
		    sort(horizontal_distances.begin(), horizontal_distances.end());
		    int stone_id = 0;
		    if (horizontal_distances.size() <= stone_id) { // if no stepping-stones are detected
			tcp_sendback(connfd2, 0);
		    } else {
			tcp_sendback(connfd2, horizontal_distances[stone_id]);
		    }

		    horizontal_distances.clear();
		    equivalances.clear();
		    equivalances.push_back(0);
		    counter = 0;
		    eq_length = 0;

		    // uncomment these too to visualize the filtered results
		    /*
		    cv::Mat new_judge;
		    cv::Mat temp_img[4] = {judge, judge, judge, judge};
		    cv::merge(temp_img, 4, new_judge);
		    cv::imshow("camera_filtered_results", cv_image.mul(new_judge));
		    cv::waitKey(10);
		    */
		}

	    }

        close(connfd1);
        close(connfd2);
    }

    zed.close();
    return EXIT_SUCCESS;

}
