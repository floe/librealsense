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
Mat im = (Mat_<float>(3,3) << 1280.0/1920.0, 0, 0, 0, 720.0/1080.0, 0, 0, 0, 1 );
// pm: perspective matrix
Mat pm = im;

Mat calcPerspective() {

  Mat result;

  dst.push_back(Point2f(   0,  0));
  dst.push_back(Point2f(1280,  0));
  dst.push_back(Point2f(1280,720));
  dst.push_back(Point2f(   0,720));

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

bool find_plane = true;
bool filter = true;
bool quit = false;

int distance = 5;

GstElement *gpipeline, *appsrc, *conv, *videosink;

gboolean pad_event(GstPad *pad, GstObject *parent, GstEvent *event) {

  if (GST_EVENT_TYPE (event) != GST_EVENT_NAVIGATION)
    return gst_pad_event_default(pad,parent,event);

  double x,y; int b;
  const gchar* key;

  switch (gst_navigation_event_get_type(event)) {

    case GST_NAVIGATION_EVENT_MOUSE_BUTTON_RELEASE:
      gst_navigation_event_parse_mouse_button_event(event,&b,&x,&y);
      src.push_back(Point2f(1920.0*x/1280.0,1080.0*y/720.0));
      break;

    case GST_NAVIGATION_EVENT_KEY_PRESS:
      gst_navigation_event_parse_key_event(event,&key);
      if (key == std::string("space"))
        pm = im;
      if (key == std::string("p"))
        find_plane = true;
      if (key == std::string("f"))
        filter = !filter;
      if (key == std::string("q"))
        quit = true;
      if (key == std::string( "plus")) distance++;
      if (key == std::string("minus")) distance--;
      break;

    default:
      return false;
  }

  if (src.size() == 4) {
    Mat r = calcPerspective();
    pm = r; //set_matrix(perspective,r.ptr<gdouble>(0));
  }

  return true;
}

void buffer_destroy(gpointer data) {
  free(data);
}

GstFlowReturn prepare_buffer(GstAppSrc* appsrc, uint32_t* color_data) {

  guint size = 1280 * 720 * 4;
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

  /* setup */
  g_object_set (G_OBJECT (appsrc), "caps",
    gst_caps_new_simple ("video/x-raw",
				     "format", G_TYPE_STRING, "RGBA",
				     "width", G_TYPE_INT, 1280,
				     "height", G_TYPE_INT, 720,
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

uint32_t* deproject_all(uint16_t* depth_data, uint8_t* color_data) {
  
  const int dw = depth_intrinsics.width;
  const int dh = depth_intrinsics.height;

  const int cw = color_intrinsics.width;
  const int ch = color_intrinsics.height;

  const int filter_half_x = 2;
  const int filter_half_y = 1;

  // calloc is never slower, and often _much_ faster, than malloc+memset
  uint32_t* new_frame = (uint32_t*)calloc( sizeof(uint32_t), 1920*1080 );

  for (int y = 0; y < dh; y++) {
    for (int x = 0; x < dw; x++) {

      rs::float3 point;
      uint16_t raw_depth = depth_data[y * dw + x];
      if (raw_depth == 0) continue;

      rs::float2 pixel = { (float) x, (float) y };
      point = depth_intrinsics.deproject(pixel, raw_depth*scale);
      
      // FIXME: fixed threshold of 5 cm
      if (filter) if (fabs(plane.n.dot(*((Eigen::Vector3f*)(&point))) - plane.d) < distance*0.01) continue; // depth_data[w * y + x] = 0;
      
      rs::float3 point2 = depth_to_color.transform(point);
      rs::float2 pixel2 = color_intrinsics.project(point2);

      const int other_x0 = static_cast<int>(pixel2.x + 0.5f);
      const int other_y0 = static_cast<int>(pixel2.y + 0.5f);

      if (other_x0 < filter_half_x || other_y0 < filter_half_y || other_x0 >= cw-filter_half_x || other_y0 >= ch-filter_half_y) continue;

      // Transfer between the depth pixels and the pixels inside the rectangle on the other image
      int index = (other_y0-filter_half_y) * cw + other_x0-filter_half_x;

      // creates rectangular patch of size filter_width_{x,y}*2+1 around other0_{x,y}
      for (int ty = -filter_half_y; ty <= filter_half_y; ty++) {
        for (int tx = -filter_half_x; tx <= filter_half_x; tx++) {
          new_frame[index] = *((uint32_t*)(color_data+3*index));
          index += 1;
        }
        index += cw - (2*filter_half_x+1);
      }
    }
  }

  return new_frame;
}


int main(int argc, char * argv[]) try
{
    gstreamer_init(argc,argv);

    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    printf("\nUsing device 0, an %s\n", dev.get_name());
    printf("    Serial number: %s\n", dev.get_serial());
    printf("    Firmware version: %s\n\n", dev.get_firmware_version());

    dev.enable_stream(rs::stream::depth,  640,  480, rs::format::z16,  30);
    dev.enable_stream(rs::stream::color, 1920, 1080, rs::format::rgb8, 30);
    dev.start();

    scale = rs_get_device_depth_scale((const rs_device*)&dev, NULL);
    color_intrinsics = dev.get_stream_intrinsics(rs::stream::color);
    depth_intrinsics = dev.get_stream_intrinsics(rs::stream::depth);
    depth_to_color = dev.get_extrinsics(rs::stream::depth,rs::stream::color);

    while (!quit)
    {
        // Wait for new images
        dev.wait_for_frames();

        // TODO: should use rectified streams?
        uint8_t* color_data = (uint8_t*)dev.get_frame_data(rs::stream::color);
        uint16_t* depth_data = (uint16_t*)dev.get_frame_data(rs::stream::depth);

        // use RANSAC to compute a plane out of sparse point cloud
        std::vector<Eigen::Vector3f> points;
        if (find_plane) {
        for (int y = 0; y < depth_intrinsics.height; y++) {
          for (int x = 0; x < depth_intrinsics.width; x++) {
            Eigen::Vector3f point;
            uint16_t raw_depth = depth_data[depth_intrinsics.width * y + x];
            if (raw_depth == 0) continue;
            rs::float2 pixel = { (float) x, (float) y };
            *((rs::float3*)(&point)) = depth_intrinsics.deproject(pixel, raw_depth*scale);
            points.push_back( point );
          }
        }

        std::cout << "3D point count: " << points.size() << std::endl;
        plane = ransac<PlaneModel<float>>( points, distance*0.01, 100 );
        std::cout << "Ransac computed plane: n=" << plane.n.transpose() << " d=" << plane.d << std::endl;
        find_plane = false;
        }

        uint32_t* new_frame = deproject_all( depth_data, color_data );
        uint32_t* persp_frame = (uint32_t*)malloc( sizeof(uint32_t) * 1280*720 );

        Mat input(1080,1920,CV_8UC4,new_frame);
        Mat output(720,1280,CV_8UC4,persp_frame);
        warpPerspective(input,output,pm,output.size(),INTER_NEAREST);

        prepare_buffer((GstAppSrc*)appsrc,persp_frame);
        g_main_context_iteration(g_main_context_default(),FALSE);
        free(new_frame);
    }

    /* clean up */
    gst_element_set_state (gpipeline, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (gpipeline));

    dev.stop();

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
