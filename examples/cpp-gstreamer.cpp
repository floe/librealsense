// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>

#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>

#pragma pack(push, 1)
struct rgb_pixel
{
    uint8_t r,g,b; 
};
#pragma pack(pop)

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

/* Initialize a 2D perspective matrix, you can use
 * cvGetPerspectiveTransform() from OpenCV to build it
 * from a quad-to-quad transformation */
gdouble pm[9] = {
  2.0, 0.83, -400,
  0.0, 2.0,  0.0,
  0.0, 0.0,  1.0
};

gdouble im[9] = {
  1.0, 0.0, 0.0,
  0.0, 1.0, 0.0,
  0.0, 0.0, 1.0
};

void set_matrix(GstElement *element, gdouble m[9])
{
	GValueArray *va;
	GValue v = G_VALUE_INIT;
	guint i;

	va = g_value_array_new(1);

	g_value_init(&v, G_TYPE_DOUBLE);
	for (i = 0; i < 9; i++) {
		g_value_set_double(&v, m[i]);
		g_value_array_append(va, &v);
		g_value_reset(&v);
	}
	g_object_set(G_OBJECT(element), "matrix", va, NULL);
	g_value_array_free(va);
}

gboolean pad_event(GstPad *pad, GstObject *parent, GstEvent *event) {

  if (GST_EVENT_TYPE (event) != GST_EVENT_NAVIGATION)
    return gst_pad_event_default(pad,parent,event);

  double x,y; int b;

  switch (gst_navigation_event_get_type(event)) {

    case GST_NAVIGATION_EVENT_MOUSE_BUTTON_PRESS:
    case GST_NAVIGATION_EVENT_MOUSE_BUTTON_RELEASE:
      gst_navigation_event_parse_mouse_button_event(event,&b,&x,&y);
      break;

    case GST_NAVIGATION_EVENT_KEY_PRESS:
      //gst_navigation_event_parse_key_event (GstEvent *event,
      break;

    default:
      return false;
  }

  std::cout << b << " " << x << "," << y << std::endl;

  return true;
}

void buffer_destroy(gpointer data) {
  //delete done;
}

GstFlowReturn prepare_buffer(GstAppSrc* appsrc, uint8_t* color_data) {

  guint size = 1920 * 1080 * 3;
  GstBuffer *buffer = gst_buffer_new_wrapped_full( (GstMemoryFlags)0, (gpointer)(color_data), size, 0, size, color_data, buffer_destroy );

  return gst_app_src_push_buffer(appsrc, buffer);
}

void gstreamer_init(gint argc, gchar *argv[]) {

  /* init GStreamer */
  gst_init (&argc, &argv);

  /* setup pipeline */
  gpipeline = gst_pipeline_new ("pipeline");
  appsrc = gst_element_factory_make ("appsrc", "source");

	GstElement* perspective = gst_element_factory_make("perspective", "persp");
	set_matrix(perspective,im);

  // attach event listener to appsrc pad
  GstPad* srcpad = gst_element_get_static_pad (appsrc, "src");
  gst_pad_set_event_function( srcpad, (GstPadEventFunction)pad_event );

  // create pipeline from string
  const char* pipe_desc = (argc > 1) ? argv[1] : "videoconvert ! fpsdisplaysink";
  videosink = gst_parse_bin_from_description(pipe_desc,TRUE,NULL);

  /* setup */
  g_object_set (G_OBJECT (appsrc), "caps",
    gst_caps_new_simple ("video/x-raw",
				     "format", G_TYPE_STRING, "RGB",
				     "width", G_TYPE_INT, 1920,
				     "height", G_TYPE_INT, 1080,
				     "framerate", GST_TYPE_FRACTION, 0, 1,
				     NULL), NULL);
  gst_bin_add_many (GST_BIN (gpipeline), appsrc, perspective, videosink, NULL);
  gst_element_link_many (appsrc, perspective, videosink, NULL);

  /* setup appsrc */
  g_object_set (G_OBJECT (appsrc),
		"stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
		"format", GST_FORMAT_TIME,
    "is-live", TRUE,
    "min-latency", 0,
    "max-latency", gst_util_uint64_scale_int (1, GST_SECOND, 30),
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

    while (1)
    {
        // Wait for new images
        dev.wait_for_frames();

        uint8_t* color_data = (uint8_t*)dev.get_frame_data(rs::stream::color);
        const uint16_t* depth_data = (const uint16_t*)dev.get_frame_data(rs::stream::depth_aligned_to_color);

        // TODO: depth filtering, i.e. background subtraction + threshold

        // TODO: optimize with SSE/AVX?

        for (int i = 0; i < 1920*1080; i++) {
          if (depth_data[i] == 0) {
            color_data[3*i+0] = 0;
            color_data[3*i+1] = 0;
            color_data[3*i+2] = 0;
          }
        }

        prepare_buffer((GstAppSrc*)appsrc,color_data);
        g_main_context_iteration(g_main_context_default(),FALSE);
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
