#include "calib.hpp"


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
        return 1;
    }

    // 读取图像
    cv::Mat image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Failed to load image" << std::endl;
        return 1;
    }

    // 创建 AprilTag 检测器
    apriltag_family_t* tf = tag36h11_create();
    apriltag_detector_t* td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // 设置检测器参数
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;
    td->nthreads = 1;
    td->debug = 0;
    td->refine_edges = 1;

    // 将 OpenCV 图像转换为 AprilTag 所需的格式
    image_u8_t im = { image.cols, image.rows, image.cols, image.data };

    // 检测 AprilTag
    zarray_t* detections = apriltag_detector_detect(td, &im);

    // 绘制检测结果
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        // 绘制边界框
        cv::line(image, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(0, 255, 0), 2);
        cv::line(image, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0, 255, 0), 2);
        cv::line(image, cv::Point(det->p[1][0], det->p[1][1]), cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0, 255, 0), 2);
        cv::line(image, cv::Point(det->p[2][0], det->p[2][1]), cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0, 255, 0), 2);

        // 绘制 ID
        std::stringstream ss;
        ss << det->id;
        std::string text = ss.str();
        int fontface = cv::FONT_HERSHEY_SIMPLEX;
        double fontscale = 0.5;
        int baseline;
        cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
        cv::putText(image, text, cv::Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2), fontface, fontscale, cv::Scalar(0, 0, 255), 2);
    }

    // 销毁检测结果
    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);

    // 显示结果
    cv::imshow("AprilTag Detections", image);
    cv::waitKey(0);

    return 0;
}