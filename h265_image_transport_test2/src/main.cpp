#include <iostream>

#include <libde265/de265.h>
#include <libde265/en265.h>
#include <libde265/image.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char * argv[])
{
  de265_init();

  en265_encoder_context * ectx = en265_new_encoder();

  en265_start_encoder(ectx, 0);

  auto image = cv::imread("/home/ahcorde/lenna.png", cv::IMREAD_COLOR);
  // auto image = cv::imread("/home/ahcorde/TRI/performance_transport_ws/src/performance_transport/resources/image.bmp", cv::IMREAD_COLOR);

  std::cout << "image " << image.cols << " " << image.rows << std::endl;

  struct de265_image * img = new struct de265_image () ;

    img->alloc_image(
    image.cols, image.rows, de265_chroma_444, NULL, false,
    NULL, /*NULL,*/ 0, NULL, false);

  uint8_t * p;
  int stride;

  for (int c = 0; c < 3; c++) {
    int h265channel;
    switch (c) {
      case 0: h265channel = 2; break; // R
      case 1: h265channel = 0; break; // G
      case 2: h265channel = 1; break; // B
    }

    p = img->get_image_plane(h265channel);
    stride = img->get_image_stride(h265channel);

    cv::Mat ch1;
    cv::extractChannel(image, ch1, c); // extract specific channel

    cv::imshow("img", ch1);
    cv::waitKey(0);

    for (int y = 0; y < image.rows; y++) {
      memcpy(p, &ch1.data[y * image.rows], image.cols);
      p += stride;
    }
  }

  {
    cv::Mat restored_image(512, 512, CV_8UC3);

    // int stride;
    // const uint8_t* data;
    // data = de265_get_image_plane(img, 0, &stride);

    // for (int y = 0; y < 512; y++) {
    //   memcpy(&restored_image.data[y], data + y*stride, 512);
    // }

    std::vector<cv::Mat> ch;
    ch.resize(3);

    for (int c = 0; c < 3; c++) {
      int h265channel;
      switch (c) {
        case 0: h265channel = 2; break;   // R
        case 1: h265channel = 0; break;   // G
        case 2: h265channel = 1; break;   // B
      }

      p = img->get_image_plane(h265channel);
      stride = img->get_image_stride(h265channel);

      cv::extractChannel(restored_image, ch[c], c);   // extract specific channel

      for (int y = 0; y < restored_image.rows; y++) {
        memcpy(&ch[c].data[y * restored_image.cols], p, restored_image.cols);
        p += stride;
      }
    }
    // cv::Mat dst;
    cv::merge(ch, restored_image);
    cv::imwrite("salida2.png", restored_image);
  }


  en265_push_image(ectx, img);

  en265_encode(ectx);


  // std::cout << "en265_number_of_queued_packets: " << en265_number_of_queued_packets(ectx) << std::endl;

//   en265_number_of_queued_packets(ectx);


//////////////////////////////////////////

  de265_decoder_context * ctx_de = de265_new_decoder();

  de265_set_parameter_int(ctx_de, DE265_DECODER_PARAM_ACCELERATION_CODE, de265_acceleration_SSE4);

  de265_set_verbosity(3);

  // de265_start_worker_threads(ctx_de, 10);

  de265_flush_data(ctx_de);

  while (en265_number_of_queued_packets(ectx) > 0) {
    en265_packet * pck = en265_get_packet(ectx, 0);

    // decode input data
    auto err = de265_push_NAL(ctx_de, pck->data, pck->length, 0, (void *)2);
    if (err != DE265_OK) {
      std::cout << "Error" << std::endl;
    }

    std::cout << "size: " << pck->length << std::endl;
    en265_free_packet(ectx, pck);
  }

  int more = 1;
  while (more) {
    more = 0;

    // decode some more

    auto err = de265_decode(ctx_de, &more);

    if (err != DE265_OK) {
      // if (quiet<=1) fprintf(stderr,"ERROR: %s\n", de265_get_error_text(err));

      // if (check_hash && err == DE265_ERROR_CHECKSUM_MISMATCH) {
      //   stop = 1;
      // }

      std::cout << "err " << err << std::endl;
      more = 0;
      return -1;
    }

    const de265_image * img_decode = de265_get_next_picture(ctx_de);
    if (img_decode) {

      int width = de265_get_image_width(img_decode, 0);
      int height = de265_get_image_height(img_decode, 0);
      std::cout << "width " << width << " height: " << height << std::endl;

      cv::Mat restored_image(height, width, CV_8UC3);

      // int stride;
      // const uint8_t* data;
      // data = de265_get_image_plane(img_decode, 0, &stride);

      // for (int y = 0; y < height; y++) {
      //   memcpy(&restored_image.data[y], data + y*stride, width);
      // }

      std::vector<cv::Mat> ch;
      ch.resize(3);

      for (int c = 0; c < 3; c++) {
        int h265channel;
        switch (c) {
          case 0: h265channel = 2; break; // R
          case 1: h265channel = 0; break; // G
          case 2: h265channel = 1; break; // B
        }

        const uint8_t * p = img_decode->get_image_plane(h265channel);
        stride = img_decode->get_image_stride(h265channel);

        cv::extractChannel(restored_image, ch[c], c); // extract specific channel

        for (int y = 0; y < restored_image.rows; y++) {
          memcpy(&ch[c].data[y * restored_image.rows], p, restored_image.cols);
          p += stride;
        }

        cv::imshow("img", ch[c]);
        cv::waitKey(0);
      }

      for (;; ) {
        de265_error warning = de265_get_warning(ctx_de);
        if (warning == DE265_OK) {
          break;
        }

        fprintf(stderr, "WARNING: %s\n", de265_get_error_text(warning));
      }

      cv::merge(ch, restored_image);

      cv::imwrite("salida.png", restored_image);
    } else {
      std::cout << "no lol!" << std::endl;
    }

  }


  // // decode input data
  // auto err = de265_push_data(ctx_de, buf, n, pos, (void*)2);
  // if (err != DE265_OK) {
  //     std::cout << "Error" << std::endl;
  // }


  de265_free_decoder(ctx_de);


//////////////////////////////////////////


//   const uint8_t* data;
//   int   length;


  en265_free_encoder(ectx);

  de265_free();

  return 0;
}
