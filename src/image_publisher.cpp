extern "C" {
    #include <libavformat/avformat.h>
    #include <libavcodec/avcodec.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h> 
}

#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>

// 视频流地址
const char* videoStreamUrl = "tcp://192.168.128.5:2020";
std::string device_ip;


int main(int argc, char **argv) {
    
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    
    std::string param_name = "/devices/" + std::string(argv[1]) + "/ip";
    std::string topic_name = std::string(argv[1])+"/image_raw";

    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>(topic_name, 10);
    
    nh.getParam(param_name, device_ip);
    
    // 初始化FFmpeg库
    avformat_network_init();
    AVFormatContext* formatContext = avformat_alloc_context();

    // 打开输入流
    if (avformat_open_input(&formatContext, videoStreamUrl, nullptr, nullptr) != 0) {
        std::cerr << "Error: Cannot open video stream" << std::endl;
        return -1;
    }

    // 查找流信息
    if (avformat_find_stream_info(formatContext, nullptr) < 0) {
        std::cerr << "Error: Cannot find stream information" << std::endl;
        return -1;
    }

    // 找到视频流的索引
    int videoStreamIndex = -1;
    for (unsigned int i = 0; i < formatContext->nb_streams; i++) {
        if (formatContext->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            videoStreamIndex = i;
            break;
        }
    }
    if (videoStreamIndex == -1) {
        std::cerr << "Error: Cannot find video stream" << std::endl;
        return -1;
    }

    // 获取解码器上下文
    AVCodecParameters* codecParams = formatContext->streams[videoStreamIndex]->codecpar;
    AVCodec* codec = avcodec_find_decoder(codecParams->codec_id);
    AVCodecContext* codecContext = avcodec_alloc_context3(codec);
    if (avcodec_parameters_to_context(codecContext, codecParams) != 0) {
        std::cerr << "Error: Cannot create codec context" << std::endl;
        return -1;
    }
    if (avcodec_open2(codecContext, codec, nullptr) < 0) {
        std::cerr << "Error: Cannot open codec" << std::endl;
        return -1;
    }

    // 初始化SWS转换上下文
    SwsContext* swsContext = sws_getContext(
        codecContext->width, codecContext->height, codecContext->pix_fmt,
        codecContext->width, codecContext->height, AV_PIX_FMT_BGR24,
        SWS_BILINEAR, nullptr, nullptr, nullptr
    );

    AVPacket* packet = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    AVFrame* frameBGR = av_frame_alloc();

    // 分配BGR图像缓冲区
    int numBytes = av_image_get_buffer_size(AV_PIX_FMT_BGR24, codecContext->width, codecContext->height, 1);
    uint8_t* buffer = (uint8_t*)av_malloc(numBytes * sizeof(uint8_t));
    av_image_fill_arrays(frameBGR->data, frameBGR->linesize, buffer, AV_PIX_FMT_BGR24, codecContext->width, codecContext->height, 1);

    // 读取帧并解码
    while (av_read_frame(formatContext, packet) >= 0 && ros::ok()) {

        if (packet->stream_index == videoStreamIndex) {
            if (avcodec_send_packet(codecContext, packet) == 0) {
                while (avcodec_receive_frame(codecContext, frame) == 0) {
                    // 将解码后的帧转换为BGR格式
                    sws_scale(swsContext, frame->data, frame->linesize, 0, codecContext->height, frameBGR->data, frameBGR->linesize);

                    // 将BGR数据复制到OpenCV的Mat
                    cv::Mat img(codecContext->height, codecContext->width, CV_8UC3, frameBGR->data[0]);
                    
                    // 发布图像
                    sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                    img_pub.publish(msg);
                    cv::imshow("Original Video", img);
                    cv::waitKey(1);
                    
                }
            }
        }
        av_packet_unref(packet);
    }

    // 释放资源
    av_free(buffer);
    av_frame_free(&frame);
    av_frame_free(&frameBGR);
    av_packet_free(&packet);
    avcodec_close(codecContext);
    avformat_close_input(&formatContext);
    sws_freeContext(swsContext);
    cv::destroyAllWindows();

    return 0;
}
