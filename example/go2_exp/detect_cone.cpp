#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <curl/curl.h>
#include "base64.hpp"
#include <nlohmann/json.hpp> // JSON parsing library
using json = nlohmann::json;

size_t write_callback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}


std::vector<unsigned char> read_file_binary(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    return std::vector<unsigned char>((std::istreambuf_iterator<char>(file)), {});
}

int main() {
    // const std::string image_path = "/home/chris/cone.png";
    const std::string image_path = "./cone.jpg";
    
    const std::string api_url = "https://serverless.roboflow.com/safety-cones-vfrj2/4?api_key=hQCOUahSPREdfnw1ze9L";

    // Read and encode the image
    std::vector<unsigned char> image_data = read_file_binary(image_path);
    std::string base64_str = base64::encode_into<std::string>(image_data.begin(), image_data.end());

    // Remove any accidental newlines (some encoders insert them)
    base64_str.erase(std::remove(base64_str.begin(), base64_str.end(), '\n'), base64_str.end());
    base64_str.erase(std::remove(base64_str.begin(), base64_str.end(), '\r'), base64_str.end());

    // Setup cURL
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Failed to initialize CURL\n";
        return 1;
    }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/x-www-form-urlencoded");

    curl_easy_setopt(curl, CURLOPT_URL, api_url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, base64_str.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, base64_str.size());
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L); // Optional: for debugging

    std::string response_string;
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);


    CURLcode res = curl_easy_perform(curl);
    std::cout << "Response JSON:\n" << response_string << std::endl;

    // 解析 response_string 为 JSON 对象
    json response_json = json::parse(response_string);

    // 提取图像尺寸
    int img_width = response_json["image"]["width"];
    int img_height = response_json["image"]["height"];

    // 提取第一个预测框
    if (!response_json["predictions"].empty()) {
        const auto& pred = response_json["predictions"][0];
        std::cout << "pred type: " << pred.type_name() << std::endl;

        float x_center = pred["x"];
        float y_center = pred["y"];
        float box_width = pred["width"];
        float box_height = pred["height"];
        float confidence = pred["confidence"];
        std::string cls = pred["class"];

        std::cout << "Class: " << cls << "\n";
        std::cout << "Confidence: " << confidence << "\n";
        std::cout << "Bounding box: center(" << x_center << ", " << y_center 
                << "), size(" << box_width << " x " << box_height << ")\n";
    }


    if (res != CURLE_OK)
        std::cerr << "cURL error: " << curl_easy_strerror(res) << "\n";

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return 0;
}
