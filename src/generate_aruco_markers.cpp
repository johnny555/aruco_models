/**
 * @file generate_aruco_markers.cpp
 * @brief C++ ArUco marker generator with multiple dictionary support
 * 
 * This program generates ArUco markers using various dictionaries available in OpenCV.
 * Supports all major ArUco dictionary types for comprehensive fiducial infrastructure.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

namespace fs = std::filesystem;

class ArucoMarkerGenerator {
private:
    int marker_size_;
    std::string output_dir_;
    
    // Dictionary mapping for all supported ArUco dictionaries
    static const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionary_map_;
    static const std::map<cv::aruco::PREDEFINED_DICTIONARY_NAME, int> dictionary_sizes_;
    
public:
    /**
     * @brief Constructor
     * @param marker_size Size of the marker in pixels (default: 200)
     * @param output_dir Output directory for generated markers
     */
    ArucoMarkerGenerator(int marker_size = 200, const std::string& output_dir = "../textures")
        : marker_size_(marker_size), output_dir_(output_dir) {
        
        // Create output directory if it doesn't exist
        fs::create_directories(output_dir_);
        
        std::cout << "ArUco Marker Generator initialized:" << std::endl;
        std::cout << "  Marker size: " << marker_size_ << "x" << marker_size_ << " pixels" << std::endl;
        std::cout << "  Output directory: " << output_dir_ << std::endl;
    }
    
    /**
     * @brief Get available dictionary names
     */
    static std::vector<std::string> getAvailableDictionaries() {
        std::vector<std::string> dicts;
        for (const auto& pair : dictionary_map_) {
            dicts.push_back(pair.first);
        }
        std::sort(dicts.begin(), dicts.end());
        return dicts;
    }
    
    /**
     * @brief Get dictionary size (number of available markers)
     */
    static int getDictionarySize(const std::string& dict_name) {
        auto it = dictionary_map_.find(dict_name);
        if (it != dictionary_map_.end()) {
            auto size_it = dictionary_sizes_.find(it->second);
            if (size_it != dictionary_sizes_.end()) {
                return size_it->second;
            }
        }
        return 0;
    }
    
    /**
     * @brief Print information about a dictionary
     */
    static void printDictionaryInfo(const std::string& dict_name) {
        auto it = dictionary_map_.find(dict_name);
        if (it == dictionary_map_.end()) {
            std::cout << "Dictionary '" << dict_name << "' not found." << std::endl;
            return;
        }
        
        int size = getDictionarySize(dict_name);
        std::cout << "Dictionary: " << dict_name << std::endl;
        std::cout << "  Available markers: 0 to " << (size - 1) << " (" << size << " total)" << std::endl;
        
        // Extract grid size from name
        if (dict_name.find("4X4") != std::string::npos) {
            std::cout << "  Grid size: 4x4" << std::endl;
        } else if (dict_name.find("5X5") != std::string::npos) {
            std::cout << "  Grid size: 5x5" << std::endl;
        } else if (dict_name.find("6X6") != std::string::npos) {
            std::cout << "  Grid size: 6x6" << std::endl;
        } else if (dict_name.find("7X7") != std::string::npos) {
            std::cout << "  Grid size: 7x7" << std::endl;
        }
    }
    
    /**
     * @brief List all available dictionaries
     */
    static void listDictionaries() {
        std::cout << "Available ArUco Dictionaries:" << std::endl;
        auto dicts = getAvailableDictionaries();
        for (const auto& dict : dicts) {
            int size = getDictionarySize(dict);
            std::cout << "  " << dict << " (0-" << (size-1) << ", " << size << " markers)" << std::endl;
        }
    }
    
    /**
     * @brief Generate a single ArUco marker
     * @param dict_name Dictionary name (e.g., "DICT_4X4_50")
     * @param marker_id ID of the marker
     * @return true if successful, false otherwise
     */
    bool generateMarker(const std::string& dict_name, int marker_id) {
        // Get dictionary
        auto dict_it = dictionary_map_.find(dict_name);
        if (dict_it == dictionary_map_.end()) {
            std::cerr << "Error: Unknown dictionary '" << dict_name << "'" << std::endl;
            return false;
        }
        
        // Check marker ID range
        int max_id = getDictionarySize(dict_name) - 1;
        if (marker_id < 0 || marker_id > max_id) {
            std::cerr << "Error: Marker ID " << marker_id << " is out of range for " 
                      << dict_name << " (0-" << max_id << ")" << std::endl;
            return false;
        }
        
        try {
            // Get the dictionary
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dict_it->second);
            
            // Generate the marker
            cv::Mat marker_img;
            cv::aruco::drawMarker(dictionary, marker_id, marker_size_, marker_img, 1);
            
            // Create filename with dictionary name and proper zero-padding
            std::string dict_lower = dict_name;
            std::transform(dict_lower.begin(), dict_lower.end(), dict_lower.begin(), ::tolower);
            
            std::string filename = output_dir_ + "/aruco_" + dict_lower + "_" + 
                                 std::string(4 - std::to_string(marker_id).length(), '0') + 
                                 std::to_string(marker_id) + ".png";
            
            // Save the marker
            bool success = cv::imwrite(filename, marker_img);
            
            if (success) {
                std::cout << "Generated ArUco marker " << dict_name << ":" << marker_id 
                          << " -> " << filename << std::endl;
            } else {
                std::cerr << "Error: Failed to save marker " << marker_id << " to " << filename << std::endl;
            }
            
            return success;
            
        } catch (const cv::Exception& e) {
            std::cerr << "OpenCV error generating marker " << dict_name << ":" << marker_id 
                      << ": " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Error generating marker " << dict_name << ":" << marker_id 
                      << ": " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Generate multiple ArUco markers
     * @param dict_name Dictionary name
     * @param start_id Starting marker ID
     * @param count Number of markers to generate
     * @return Number of successfully generated markers
     */
    int generateMarkers(const std::string& dict_name, int start_id = 0, int count = 10) {
        int successful = 0;
        int max_id = getDictionarySize(dict_name) - 1;
        
        std::cout << "\nGenerating " << count << " ArUco markers from " << dict_name 
                  << " (IDs " << start_id << " to " << (start_id + count - 1) << ")..." << std::endl;
        
        for (int i = 0; i < count; ++i) {
            int marker_id = start_id + i;
            if (marker_id > max_id) {
                std::cerr << "Warning: Marker ID " << marker_id << " exceeds dictionary size (" 
                          << max_id << "), stopping generation." << std::endl;
                break;
            }
            
            if (generateMarker(dict_name, marker_id)) {
                successful++;
            }
        }
        
        std::cout << "\nGeneration complete: " << successful << "/" << count 
                  << " markers generated successfully." << std::endl;
        
        return successful;
    }
    
    /**
     * @brief Generate specific marker IDs
     * @param dict_name Dictionary name
     * @param marker_ids Vector of specific marker IDs to generate
     * @return Number of successfully generated markers
     */
    int generateSpecificMarkers(const std::string& dict_name, const std::vector<int>& marker_ids) {
        int successful = 0;
        
        std::cout << "\nGenerating " << marker_ids.size() << " specific ArUco markers from " 
                  << dict_name << "..." << std::endl;
        
        for (int marker_id : marker_ids) {
            if (generateMarker(dict_name, marker_id)) {
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
        std::cout << "  --dict NAME      ArUco dictionary name (default: DICT_4X4_50)" << std::endl;
        std::cout << "  --count N        Generate N markers starting from ID 0 (default: 10)" << std::endl;
        std::cout << "  --start N        Start from marker ID N (default: 0)" << std::endl;
        std::cout << "  --size N         Marker size in pixels (default: 200)" << std::endl;
        std::cout << "  --output DIR     Output directory (default: ../textures)" << std::endl;
        std::cout << "  --id N           Generate specific marker ID N (can be repeated)" << std::endl;
        std::cout << "  --list-dicts     List all available dictionaries" << std::endl;
        std::cout << "  --info DICT      Show information about a specific dictionary" << std::endl;
        std::cout << "  --help           Show this help message" << std::endl;
        std::cout << std::endl;
        std::cout << "Examples:" << std::endl;
        std::cout << "  " << program_name << "                           # Generate 10 DICT_4X4_50 markers" << std::endl;
        std::cout << "  " << program_name << " --dict DICT_6X6_250       # Use 6x6 dictionary" << std::endl;
        std::cout << "  " << program_name << " --count 50                # Generate 50 markers" << std::endl;
        std::cout << "  " << program_name << " --start 100 --count 20    # Generate markers 100-119" << std::endl;
        std::cout << "  " << program_name << " --id 5 --id 12 --id 25    # Generate specific markers" << std::endl;
        std::cout << "  " << program_name << " --list-dicts              # Show all dictionaries" << std::endl;
        std::cout << "  " << program_name << " --info DICT_4X4_50        # Show dictionary info" << std::endl;
    }
};

// Static member definitions
const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> 
ArucoMarkerGenerator::dictionary_map_ = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000}
};

const std::map<cv::aruco::PREDEFINED_DICTIONARY_NAME, int> 
ArucoMarkerGenerator::dictionary_sizes_ = {
    {cv::aruco::DICT_4X4_50, 50},
    {cv::aruco::DICT_4X4_100, 100},
    {cv::aruco::DICT_4X4_250, 250},
    {cv::aruco::DICT_4X4_1000, 1000},
    {cv::aruco::DICT_5X5_50, 50},
    {cv::aruco::DICT_5X5_100, 100},
    {cv::aruco::DICT_5X5_250, 250},
    {cv::aruco::DICT_5X5_1000, 1000},
    {cv::aruco::DICT_6X6_50, 50},
    {cv::aruco::DICT_6X6_100, 100},
    {cv::aruco::DICT_6X6_250, 250},
    {cv::aruco::DICT_6X6_1000, 1000},
    {cv::aruco::DICT_7X7_50, 50},
    {cv::aruco::DICT_7X7_100, 100},
    {cv::aruco::DICT_7X7_250, 250},
    {cv::aruco::DICT_7X7_1000, 1000}
};

int main(int argc, char* argv[]) {
    // Default parameters
    int marker_size = 200;
    std::string output_dir = "../textures";
    std::string dict_name = "DICT_4X4_50";
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
        } else if (arg == "--list-dicts") {
            ArucoMarkerGenerator::listDictionaries();
            return 0;
        } else if (arg == "--info" && i + 1 < argc) {
            ArucoMarkerGenerator::printDictionaryInfo(argv[++i]);
            return 0;
        } else if (arg == "--dict" && i + 1 < argc) {
            dict_name = argv[++i];
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
        // Validate dictionary
        if (ArucoMarkerGenerator::getDictionarySize(dict_name) == 0) {
            std::cerr << "Error: Unknown dictionary '" << dict_name << "'" << std::endl;
            std::cout << "\nAvailable dictionaries:" << std::endl;
            ArucoMarkerGenerator::listDictionaries();
            return 1;
        }
        
        // Create generator
        ArucoMarkerGenerator generator(marker_size, output_dir);
        
        // Print dictionary info
        std::cout << "\nUsing dictionary: " << dict_name << std::endl;
        ArucoMarkerGenerator::printDictionaryInfo(dict_name);
        
        // Generate markers
        int successful;
        if (use_specific_ids) {
            successful = generator.generateSpecificMarkers(dict_name, specific_ids);
        } else {
            successful = generator.generateMarkers(dict_name, start_id, count);
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
