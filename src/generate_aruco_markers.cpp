/**
 * @file generate_aruco_markers.cpp
 * @brief C++ ArUco marker generator using OpenCV
 * 
 * This program generates ArUco markers for use in Gazebo simulations.
 * Uses OpenCV's stable C++ ArUco implementation instead of the problematic Python bindings.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

class ArucoMarkerGenerator {
private:
    int marker_size_;
    std::string output_dir_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    
public:
    /**
     * @brief Constructor
     * @param marker_size Size of the marker in pixels (default: 200)
     * @param output_dir Output directory for generated markers
     */
    ArucoMarkerGenerator(int marker_size = 200, const std::string& output_dir = "../textures")
        : marker_size_(marker_size), output_dir_(output_dir) {
        
        // Use DICT_4X4_50 - a stable, well-tested dictionary
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        
        // Create output directory if it doesn't exist
        fs::create_directories(output_dir_);
        
        std::cout << "ArUco Marker Generator initialized:" << std::endl;
        std::cout << "  Dictionary: DICT_4X4_50" << std::endl;
        std::cout << "  Marker size: " << marker_size_ << "x" << marker_size_ << " pixels" << std::endl;
        std::cout << "  Output directory: " << output_dir_ << std::endl;
    }
    
    /**
     * @brief Generate a single ArUco marker
     * @param marker_id ID of the marker (0-49 for DICT_4X4_50)
     * @return true if successful, false otherwise
     */
    bool generateMarker(int marker_id) {
        if (marker_id < 0 || marker_id >= 50) {
            std::cerr << "Error: Marker ID " << marker_id << " is out of range for DICT_4X4_50 (0-49)" << std::endl;
            return false;
        }
        
        try {
            // Generate the marker
            cv::Mat marker_img;
            cv::aruco::drawMarker(dictionary_, marker_id, marker_size_, marker_img, 1);
            
            // Create filename with proper zero-padding
            std::string filename = output_dir_ + "/aruco_" + 
                                 std::string(3 - std::to_string(marker_id).length(), '0') + 
                                 std::to_string(marker_id) + ".png";
            
            // Save the marker
            bool success = cv::imwrite(filename, marker_img);
            
            if (success) {
                std::cout << "Generated ArUco marker " << marker_id << " -> " << filename << std::endl;
            } else {
                std::cerr << "Error: Failed to save marker " << marker_id << " to " << filename << std::endl;
            }
            
            return success;
            
        } catch (const cv::Exception& e) {
            std::cerr << "OpenCV error generating marker " << marker_id << ": " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Error generating marker " << marker_id << ": " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Generate multiple ArUco markers
     * @param start_id Starting marker ID
     * @param count Number of markers to generate
     * @return Number of successfully generated markers
     */
    int generateMarkers(int start_id = 0, int count = 10) {
        int successful = 0;
        
        std::cout << "\nGenerating " << count << " ArUco markers (IDs " << start_id 
                  << " to " << (start_id + count - 1) << ")..." << std::endl;
        
        for (int i = 0; i < count; ++i) {
            int marker_id = start_id + i;
            if (generateMarker(marker_id)) {
                successful++;
            }
        }
        
        std::cout << "\nGeneration complete: " << successful << "/" << count 
                  << " markers generated successfully." << std::endl;
        
        return successful;
    }
    
    /**
     * @brief Generate specific marker IDs
     * @param marker_ids Vector of specific marker IDs to generate
     * @return Number of successfully generated markers
     */
    int generateSpecificMarkers(const std::vector<int>& marker_ids) {
        int successful = 0;
        
        std::cout << "\nGenerating " << marker_ids.size() << " specific ArUco markers..." << std::endl;
        
        for (int marker_id : marker_ids) {
            if (generateMarker(marker_id)) {
                successful++;
            }
        }
        
        std::cout << "\nGeneration complete: " << successful << "/" << marker_ids.size() 
                  << " markers generated successfully." << std::endl;
        
        return successful;
    }
    
    /**
     * @brief Print usage information
     */
    static void printUsage(const std::string& program_name) {
        std::cout << "Usage: " << program_name << " [options]" << std::endl;
        std::cout << "Options:" << std::endl;
        std::cout << "  --count N        Generate N markers starting from ID 0 (default: 10)" << std::endl;
        std::cout << "  --start N        Start from marker ID N (default: 0)" << std::endl;
        std::cout << "  --size N         Marker size in pixels (default: 200)" << std::endl;
        std::cout << "  --output DIR     Output directory (default: ../textures)" << std::endl;
        std::cout << "  --id N           Generate specific marker ID N (can be repeated)" << std::endl;
        std::cout << "  --help           Show this help message" << std::endl;
        std::cout << std::endl;
        std::cout << "Examples:" << std::endl;
        std::cout << "  " << program_name << "                    # Generate markers 0-9" << std::endl;
        std::cout << "  " << program_name << " --count 20         # Generate markers 0-19" << std::endl;
        std::cout << "  " << program_name << " --start 10 --count 5  # Generate markers 10-14" << std::endl;
        std::cout << "  " << program_name << " --id 5 --id 12 --id 25  # Generate specific markers" << std::endl;
    }
};

int main(int argc, char* argv[]) {
    // Default parameters
    int marker_size = 200;
    std::string output_dir = "../textures";
    int start_id = 0;
    int count = 10;
    std::vector<int> specific_ids;
    bool use_specific_ids = false;
    
    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--help" || arg == "-h") {
            ArucoMarkerGenerator::printUsage(argv[0]);
            return 0;
        } else if (arg == "--count" && i + 1 < argc) {
            count = std::stoi(argv[++i]);
        } else if (arg == "--start" && i + 1 < argc) {
            start_id = std::stoi(argv[++i]);
        } else if (arg == "--size" && i + 1 < argc) {
            marker_size = std::stoi(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--id" && i + 1 < argc) {
            specific_ids.push_back(std::stoi(argv[++i]));
            use_specific_ids = true;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            ArucoMarkerGenerator::printUsage(argv[0]);
            return 1;
        }
    }
    
    try {
        // Create generator
        ArucoMarkerGenerator generator(marker_size, output_dir);
        
        // Generate markers
        int successful;
        if (use_specific_ids) {
            successful = generator.generateSpecificMarkers(specific_ids);
        } else {
            successful = generator.generateMarkers(start_id, count);
        }
        
        if (successful > 0) {
            std::cout << "\nArUco marker generation completed successfully!" << std::endl;
            std::cout << "Generated markers can be found in: " << output_dir << std::endl;
            return 0;
        } else {
            std::cerr << "\nFailed to generate any markers!" << std::endl;
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
