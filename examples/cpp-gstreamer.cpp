// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>

#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>

#include <Eigen/Core>
#include <SimpleRansac.h>
#include <PlaneModel.h>

#pragma pack(push, 1)
struct rgb_pixel
{
    uint8_t r,g,b; 
};
#pragma pack(pop)

////////////////////////////////////////////////////////////////////////////////
//
// OpenCV stuff
//

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;

std::vector<Point2f> src;
std::vector<Point2f> dst;

// im: identity matrix
Mat im = Mat::eye(3,3,CV_32FC1);
// pm: perspective matrix
Mat pm = im;

Mat calcPerspective() {

  Mat result;

  dst.push_back(Point2f(   0,   0));
  dst.push_back(Point2f(1920,   0));
  dst.push_back(Point2f(1920,1080));
  dst.push_back(Point2f(   0,1080));

  result = getPerspectiveTransform(src,dst);

  src.clear();
  dst.clear();

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
// GStreamer stuff
//

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/navigation.h>

#include <stdint.h>

#include <immintrin.h>

bool find_plane = false;

GstElement *gpipeline, *appsrc, *conv, *videosink;

gboolean pad_event(GstPad *pad, GstObject *parent, GstEvent *event) {

  if (GST_EVENT_TYPE (event) != GST_EVENT_NAVIGATION)
    return gst_pad_event_default(pad,parent,event);

  double x,y; int b;
  const gchar* key;

  switch (gst_navigation_event_get_type(event)) {

    case GST_NAVIGATION_EVENT_MOUSE_BUTTON_RELEASE:
      gst_navigation_event_parse_mouse_button_event(event,&b,&x,&y);
      src.push_back(Point2f(x,y));
      break;

    case GST_NAVIGATION_EVENT_KEY_PRESS:
      gst_navigation_event_parse_key_event(event,&key);
      if (key == std::string("space"))
        pm = im;
      if (key == std::string("p"))
        find_plane = true;
      if (key == std::string("q"))
        exit(1);
      break;

    default:
      return false;
  }

  /*if (src.size() == 4) {
    Mat r = calcPerspective();
    pm = r; //set_matrix(perspective,r.ptr<gdouble>(0));
  }*/

  return true;
}

void buffer_destroy(gpointer data) {
  free(data);
}

GstFlowReturn prepare_buffer(GstAppSrc* appsrc, uint32_t* color_data) {

  guint size = 1920 * 1080 * 4;
  GstBuffer *buffer = gst_buffer_new_wrapped_full( (GstMemoryFlags)0, (gpointer)(color_data), size, 0, size, color_data, buffer_destroy );

  return gst_app_src_push_buffer(appsrc, buffer);
}

void gstreamer_init(gint argc, gchar *argv[]) {

  /* init GStreamer */
  gst_init (&argc, &argv);

  /* setup pipeline */
  gpipeline = gst_pipeline_new ("pipeline");
  appsrc = gst_element_factory_make ("appsrc", "source");

  // attach event listener to appsrc pad
  GstPad* srcpad = gst_element_get_static_pad (appsrc, "src");
  gst_pad_set_event_function( srcpad, (GstPadEventFunction)pad_event );

  // create pipeline from string
  const char* pipe_desc = (argc > 1) ? argv[1] : "videoconvert ! fpsdisplaysink sync=false";
  videosink = gst_parse_bin_from_description(pipe_desc,TRUE,NULL);

  // TODO: output only 1280x720 post-perspective
  /* setup */
  g_object_set (G_OBJECT (appsrc), "caps",
    gst_caps_new_simple ("video/x-raw",
				     "format", G_TYPE_STRING, "RGBA",
				     "width", G_TYPE_INT, 1920,
				     "height", G_TYPE_INT, 1080,
				     "framerate", GST_TYPE_FRACTION, 0, 1,
				     NULL), NULL);
  gst_bin_add_many (GST_BIN (gpipeline), appsrc, videosink, NULL);
  gst_element_link_many (appsrc, videosink, NULL);

  /* setup appsrc */
  g_object_set (G_OBJECT (appsrc),
		"stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
		"format", GST_FORMAT_TIME,
    "is-live", TRUE,
    "block", TRUE,
    "do-timestamp", TRUE,
    NULL);

  /* play */
  gst_element_set_state (gpipeline, GST_STATE_PLAYING);
}

////////////////////////////////////////////////////////////////////////////////
//
// realsense stuff
//

// plane parameters
PlaneModel<float> plane;
float scale = 32;
rs::intrinsics color_intrinsics;
rs::intrinsics depth_intrinsics;
rs::extrinsics depth_to_color;

void deproject_all(uint16_t* depth_data) {
  int w = depth_intrinsics.width;
  int h = depth_intrinsics.height;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      Eigen::Vector3f point;
      uint16_t raw_depth = depth_data[w * y + x];
      if (raw_depth == 0) continue;
      rs::float2 pixel = { (float) x, (float) y };
      *((rs::float3*)(&point)) = depth_intrinsics.deproject(pixel, raw_depth*scale);
      // FIXME: fixed threshold of 5 cm
      if (fabs(plane.n.dot(point) - plane.d) < 0.05) depth_data[w * y + x] = 0;
    }
  }
}

/*
template<class GET_DEPTH, class TRANSFER_PIXEL> void align_images(const rs_intrinsics & depth_intrin, const rs_extrinsics & depth_to_other, const rs_intrinsics & other_intrin, GET_DEPTH get_depth, TRANSFER_PIXEL transfer_pixel)
    {
        // check if the target image is significantly larger than the source image
        int filter_half_x = std::round(0.5 * (float)other_intrin.width  / (float)depth_intrin.width );
        int filter_half_y = std::round(0.5 * (float)other_intrin.height / (float)depth_intrin.height);

        if (other_intrin.width  == depth_intrin.width ) filter_half_x = 0;
        if (other_intrin.height == depth_intrin.height) filter_half_y = 0;

        // Iterate over the pixels of the depth image    
#pragma omp parallel for schedule(dynamic)
        for(int depth_y = 0; depth_y < depth_intrin.height; ++depth_y)
        {
            int depth_pixel_index = depth_y * depth_intrin.width;
            for(int depth_x = 0; depth_x < depth_intrin.width; ++depth_x, ++depth_pixel_index)
            {
                // Skip over depth pixels with the value of zero, we have no depth data so we will not write anything into our aligned images
                if(float depth = get_depth(depth_pixel_index))
                {
                    // Map the top-left corner of the depth pixel onto the other image
                    float depth_pixel[2] = {(float)depth_x, (float)depth_y}, depth_point[3], other_point[3], other_pixel[2];
                    rs_deproject_pixel_to_point(depth_point, &depth_intrin, depth_pixel, depth);
                    rs_transform_point_to_point(other_point, &depth_to_other, depth_point);
                    rs_project_point_to_pixel(other_pixel, &other_intrin, other_point);
                    const int other_x0 = static_cast<int>(other_pixel[0] + 0.5f);
                    const int other_y0 = static_cast<int>(other_pixel[1] + 0.5f);

                    if(other_x0 < filter_half_x || other_y0 < filter_half_y || other_x0 >= other_intrin.width-filter_half_x || other_y0 >= other_intrin.height-filter_half_y) continue;

                    // Transfer between the depth pixels and the pixels inside the rectangle on the other image
                    int index = (other_y0-filter_half_y) * other_intrin.width + other_x0-filter_half_x;

                    // creates rectangular patch of size filter_width_{x,y}*2+1 around other0_{x,y}
                    for (int ty = -filter_half_y; ty <= filter_half_y; ty++) {
                        for (int tx = -filter_half_x; tx <= filter_half_x; tx++) {
                            transfer_pixel(depth_pixel_index, index);
                            index += 1;
                        }
                        index += other_intrin.width - (2*filter_half_x+1);
                   }
                }
            }
        }    
    }

*/

int main(int argc, char * argv[]) try
{
    gstreamer_init(argc,argv);
    int threshold = -32; // ~ 1 mm

    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    printf("\nUsing device 0, an %s\n", dev.get_name());
    printf("    Serial number: %s\n", dev.get_serial());
    printf("    Firmware version: %s\n\n", dev.get_firmware_version());

    dev.enable_stream(rs::stream::depth, rs::preset::best_quality); // 640,  480, rs::format::z16,  30);
    dev.enable_stream(rs::stream::color, 1920, 1080, rs::format::rgb8, 30);
    dev.start();

    scale = rs_get_device_depth_scale((const rs_device*)&dev, NULL);
    color_intrinsics = dev.get_stream_intrinsics(rs::stream::color);
    depth_intrinsics = dev.get_stream_intrinsics(rs::stream::depth);
    depth_to_color = dev.get_extrinsics(rs::stream::depth,rs::stream::color);

    while (1)
    {
        // Wait for new images
        dev.wait_for_frames();

        // TODO: should use rectified streams?
        uint8_t* color_data = (uint8_t*)dev.get_frame_data(rs::stream::color);
        uint16_t* depth_data = (uint16_t*)dev.get_frame_data(rs::stream::depth);

        // use RANSAC to compute a plane out of sparse point cloud
        std::vector<Eigen::Vector3f> points;
        if (find_plane) {
        for (int y = 0; y < depth_intrinsics.height; y+=3) {
          for (int x = 0; x < depth_intrinsics.width; x+=3) {
            Eigen::Vector3f point;
            uint16_t raw_depth = depth_data[depth_intrinsics.width * y + x];
            if (raw_depth == 0) continue;
            rs::float2 pixel = { (float) x, (float) y };
            *((rs::float3*)(&point)) = depth_intrinsics.deproject(pixel, raw_depth*scale);
            points.push_back( point );
          }
        }

        std::cout << "3D point count: " << points.size() << std::endl;
        plane = ransac<PlaneModel<float>>( points, 0.05, 50 );
        std::cout << "Ransac computed plane: n=" << plane.n.transpose() << " d=" << plane.d << std::endl;
        find_plane = false;
        }

        // set all depth pixels to zero which are within threshold distance of plane
        deproject_all( depth_data );

        // now project the _modified_ depth data onto the color stream
        depth_data = (uint16_t*)dev.get_frame_data(rs::stream::depth_aligned_to_color);

        // calloc is never slower, and often _much_ faster, than malloc+memset
        uint32_t* new_frame = (uint32_t*)calloc( sizeof(uint32_t), 1920*1080 );
        uint32_t* persp_frame = (uint32_t*)calloc( sizeof(uint32_t), 1920*1080 );

        // TODO: optimize with SSE/AVX?

        for (int i = 0; i < 1920*1080; i++) {
          if (depth_data[i] != 0)
            new_frame[i] = *((uint32_t*)(color_data+3*i));
        }
 
        Mat input(1080,1920,CV_8UC4,new_frame);
        Mat output(1080,1920,CV_8UC4,persp_frame);
        warpPerspective(input,output,pm,input.size(),INTER_NEAREST);

        prepare_buffer((GstAppSrc*)appsrc,persp_frame);
        g_main_context_iteration(g_main_context_default(),FALSE);
        free(new_frame);
    }

    /* clean up */
    gst_element_set_state (gpipeline, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (gpipeline));

    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
