#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>

int main() {
    // Create a RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    // Wait for the first set of frames to arrive
    rs2::frameset frames = pipe.wait_for_frames();

    // Get a frame from the frameset
    rs2::video_frame color_frame = frames.get_color_frame();
    
    // Get the intrinsic parameters of the frame
    rs2_intrinsics intrinsics = color_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    
    // Print the intrinsic parameters
    std::cout << "Width: " << intrinsics.width << std::endl;
    std::cout << "Height: " << intrinsics.height << std::endl;
    std::cout << "PPX: " << intrinsics.ppx << std::endl;
    std::cout << "PPY: " << intrinsics.ppy << std::endl;
    std::cout << "FX: " << intrinsics.fx << std::endl;
    std::cout << "FY: " << intrinsics.fy << std::endl;

    // Intrinsic matrix
    float intrinsic_matrix[3][3] = {
        { intrinsics.fx, 0, intrinsics.ppx },
        { 0, intrinsics.fy, intrinsics.ppy },
        { 0, 0, 1 }
    };

    std::cout << "Intrinsic Matrix: " << std::endl;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            std::cout << intrinsic_matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
