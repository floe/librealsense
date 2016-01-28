// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>
#include "example.hpp"

#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>

texture_buffer buffers[6];

#pragma pack(push, 1)
struct rgb_pixel
{
    uint8_t r,g,b; 
};
#pragma pack(pop)

// GStreamer stuff

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <stdint.h>

#include <immintrin.h>

GstElement *gpipeline, *appsrc, *conv, *videosink;

/*void buffer_destroy(gpointer data) {
  //libfreenect2::Frame* done = (libfreenect2::Frame*)data;
  //delete done;
}

GstFlowReturn prepare_buffer(GstAppSrc* appsrc, libfreenect2::Frame* frame) {

  guint size = 1920 * 1080 * 4;
  GstBuffer *buffer = gst_buffer_new_wrapped_full( (GstMemoryFlags)0, (gpointer)(frame->data), size, 0, size, frame, buffer_destroy );

  return gst_app_src_push_buffer(appsrc, buffer);
}*/

void gstreamer_init(gint argc, gchar *argv[]) {

  /* init GStreamer */
  gst_init (&argc, &argv);

  /* setup pipeline */
  gpipeline = gst_pipeline_new ("pipeline");
  appsrc = gst_element_factory_make ("appsrc", "source");

  const char* pipe_desc = argv[2] ? argv[2] : "videoconvert ! autovideosink";
  videosink = gst_parse_bin_from_description(pipe_desc,TRUE,NULL);

  /* setup */
  g_object_set (G_OBJECT (appsrc), "caps",
    gst_caps_new_simple ("video/x-raw",
				     "format", G_TYPE_STRING, "BGRA",
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
    "min-latency", 0,
    "max-latency", gst_util_uint64_scale_int (1, GST_SECOND, 30),
    "do-timestamp", TRUE,
    NULL);

  /* play */
  gst_element_set_state (gpipeline, GST_STATE_PLAYING);
}

// realsense stuff

int main(int argc, char * argv[]) try
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    printf("\nUsing device 0, an %s\n", dev.get_name());
    printf("    Serial number: %s\n", dev.get_serial());
    printf("    Firmware version: %s\n", dev.get_firmware_version());

    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, 1920, 1080, rs::format::rgb8, 30);
    try { dev.enable_stream(rs::stream::infrared2, rs::preset::best_quality); } catch(...) {}
    dev.start();

    // Open a GLFW window
    glfwInit();
    std::ostringstream ss; ss << "CPP Image Alignment Example (" << dev.get_name() << ")";
    GLFWwindow * win = glfwCreateWindow( 1920, 1080, ss.str().c_str(), 0, 0);
    glfwMakeContextCurrent(win);

    while (!glfwWindowShouldClose(win))
    {
        // Wait for new images
        glfwPollEvents();
        dev.wait_for_frames();

        // mouse handling
        if (glfwGetMouseButton(win,GLFW_MOUSE_BUTTON_LEFT)) {
          double x,y; glfwGetCursorPos(win,&x,&y);
          std::cout << "click @ " << x << "," << y << std::endl;
        }

        // Clear the framebuffer
        int w,h;
        glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw the images        
        glPushMatrix();
        glfwGetWindowSize(win, &w, &h);
        glOrtho(0, w, h, 0, -1, +1);
        int s = w / (dev.is_stream_enabled(rs::stream::infrared2) ? 3 : 2);

        uint8_t* color_data = (uint8_t*)dev.get_frame_data(rs::stream::color);
        const uint16_t* depth_data = (const uint16_t*)dev.get_frame_data(rs::stream::depth_aligned_to_color);

        for (int i = 0; i < 1920*1080; i++) {
          if (depth_data[i] == 0) {
            color_data[3*i+0] = 0;
            color_data[3*i+1] = 0;
            color_data[3*i+2] = 0;
          }
        }

        buffers[2].show(color_data, 1920, 1080, rs::format::rgb8, "aligned (HD)", 0, 0, w, h);
        //buffers[2].show(dev, rs::stream::depth_aligned_to_color, 0, 0, w, h);

        glPopMatrix();
        glfwSwapBuffers(win);
    }

    glfwDestroyWindow(win);
    glfwTerminate();
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
