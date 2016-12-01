// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>

#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <Vector.h>

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

    const float scale = rs_get_device_depth_scale((const rs_device*)&dev, NULL);
    rs::intrinsics color_intrinsics = dev.get_stream_intrinsics(rs::stream::color);
    rs::intrinsics depth_intrinsics = dev.get_stream_intrinsics(rs::stream::depth);

    // plane parameters
    float d,l_n;
    _Vector<float> n;

    while (1)
    {
        // Wait for new images
        dev.wait_for_frames();

        // TODO: should use rectified streams?
        uint8_t* color_data = (uint8_t*)dev.get_frame_data(rs::stream::color);
        uint16_t* depth_data = (uint16_t*)dev.get_frame_data(rs::stream::depth);

        // TODO: set all depth pixels to zero which are within threshold distance of plane

        // now project the _modified_ depth data onto the color stream
        depth_data = (uint16_t*)dev.get_frame_data(rs::stream::depth_aligned_to_color);

        if (src.size() == 3) {
          _Vector<float> point[3];
          //rs::float3 point[3];
          for (int i = 0; i < 3; i++) {
            float depth = depth_data[1920*(int)round(src[i].y)+(int)round(src[i].x)] * scale;
            rs::float2 pixel = { src[i].x, src[i].y };
            *((rs::float3*)(&point[i])) = color_intrinsics.deproject(pixel, depth);
            //std::cout << point[i] << " ";
          }
          //std::cout << std::endl;

          _Vector<float> d1,d2;

          d1 = point[1] - point[0];
          d2 = point[2] - point[0];
          n = d1 & d2;

          d = n * point[0];
          l_n = n.length();

          std::cout << n << " " << l_n << " " << d << std::endl;
        }

        if (src.size() > 3) {
          for (int i = 3; i < src.size(); i++) {
            _Vector<float> point;
            float depth = depth_data[1920*(int)round(src[i].y)+(int)round(src[i].x)] * scale;
            rs::float2 pixel = { src[i].x, src[i].y };
            *((rs::float3*)(&point)) = color_intrinsics.deproject(pixel, depth);
            std::cout << point << " ";
            float tmp = (n * point) - d;
            std::cout << tmp/l_n << " ";
          }
          std::cout << std::endl;
        }

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
