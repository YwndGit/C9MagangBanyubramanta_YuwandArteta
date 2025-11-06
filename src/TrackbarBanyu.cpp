#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int hue_min1 = 0, hue_max1 = 10;      
int hue_min2 = 170, hue_max2 = 179;   
int sat_min = 50, sat_max = 255;      
int val_min = 50, val_max = 255;      


void onTrackbar(int, void*) {

}

// ========== MAIN FUNCTION ==========
int main(int argc, char** argv)
{

    string video_path;
  
    if (argc > 1) {
        // Ada argument, pakai argument sebagai path
        video_path = argv[1];
    } else {
        // Ga ada argument, minta input manual
        cout << "Enter video file path: ";
        getline(cin, video_path);
    }
    
    // Buka video file dengan VideoCapture
    VideoCapture cap(video_path);
    
    // Check apakah video berhasil dibuka
    if (!cap.isOpened()) {
        // Video ga bisa dibuka (file not found, format not supported, dll)
        cerr << "Error: Could not open video file: " << video_path << endl;
        return -1;  // Exit program dengan error code
    }
    
    
    int frame_width = (int)cap.get(CAP_PROP_FRAME_WIDTH);      
    int frame_height = (int)cap.get(CAP_PROP_FRAME_HEIGHT);    
    double fps = cap.get(CAP_PROP_FPS);                       
    int total_frames = (int)cap.get(CAP_PROP_FRAME_COUNT);     
    
    // Print info video ke terminal
    cout << "\n=== Video Loaded Successfully ===" << endl;
    cout << "File: " << video_path << endl;
    cout << "Resolution: " << frame_width << "x" << frame_height << endl;
    cout << "FPS: " << fps << endl;
    cout << "Total frames: " << total_frames << endl;
    cout << "Duration: " << (total_frames / fps) << " seconds" << endl;
    
    // Print keyboard controls
    cout << "\n=== Keyboard Controls ===" << endl;
    cout << "Space      - Pause/Resume video" << endl;
    cout << "R          - Restart video from beginning" << endl;
    cout << "S          - Save/Print current HSV values" << endl;
    cout << "Q or ESC   - Quit program" << endl;
    

    // Bikin 3 windows untuk display
    namedWindow("Original", WINDOW_NORMAL);          // Window untuk video original
    namedWindow("Masked", WINDOW_NORMAL);            // Window untuk hasil masking
    namedWindow("HSV Adjustments", WINDOW_NORMAL);   // Window untuk trackbars
    
    // Resize windows untuk display yang bagus
    // (Opsional, user bisa resize manual juga karena WINDOW_NORMAL)
    resizeWindow("Original", 640, 480);
    resizeWindow("Masked", 640, 480);
    resizeWindow("HSV Adjustments", 400, 350);  // Cukup tinggi untuk 8 trackbars
    
    
    createTrackbar("Hue Min 1", "HSV Adjustments", &hue_min1, 179, onTrackbar);
    
    // Trackbar 2: Hue Max 1
    createTrackbar("Hue Max 1", "HSV Adjustments", &hue_max1, 179, onTrackbar);
    
    // Trackbar 3: Hue Min 2
    createTrackbar("Hue Min 2", "HSV Adjustments", &hue_min2, 179, onTrackbar);
    
    // Trackbar 4: Hue Max 2
    createTrackbar("Hue Max 2", "HSV Adjustments", &hue_max2, 179, onTrackbar);
    
    // Trackbar 5: Sat Min
    // Saturation range 0-255 (8-bit)
    createTrackbar("Sat Min", "HSV Adjustments", &sat_min, 255, onTrackbar);
    
    // Trackbar 6: Sat Max
    createTrackbar("Sat Max", "HSV Adjustments", &sat_max, 255, onTrackbar);
    
    // Trackbar 7: Val Min
    // Value (brightness) range 0-255 (8-bit)
    createTrackbar("Val Min", "HSV Adjustments", &val_min, 255, onTrackbar);
    
    // Trackbar 8: Val Max
    createTrackbar("Val Max", "HSV Adjustments", &val_max, 255, onTrackbar);
    
    // TOTAL: 8 trackbars!
    

    // Declare Mat objects untuk processing
    Mat frame;          
    Mat hsv;           
    Mat mask1;         
    Mat mask2;         
    Mat mask_combined;   
    Mat result;          
    
 
    bool paused = false;                       
    int delay = max(1, (int)(1000.0 / fps));    
    

    while (true) {
        // Loop infinite sampai user tekan 'Q' atau ESC
        
        if (!paused) {
            // Kalau video NOT paused, process frame baru
            
            // ===== READ FRAME =====
            cap >> frame;  // Read frame baru dari video
            
            
            // Check apakah frame kosong (video ended)
            if (frame.empty()) {
                cout << "Video ended. Restarting from beginning..." << endl;
                
                // Restart video dari frame 0
                cap.set(CAP_PROP_POS_FRAMES, 0);
                
                // Continue loop (skip processing untuk frame ini)
                continue;
            }
            
            
            // ===== CONVERT COLOR SPACE =====
            cvtColor(frame, hsv, COLOR_BGR2HSV);
            
            
            Scalar lower1(hue_min1, sat_min, val_min);
            Scalar upper1(hue_max1, sat_max, val_max);
            inRange(hsv, lower1, upper1, mask1);
            
        
            Scalar lower2(hue_min2, sat_min, val_min);
            Scalar upper2(hue_max2, sat_max, val_max);
            inRange(hsv, lower2, upper2, mask2);
            
            
            // ===== COMBINE MASKS =====
            bitwise_or(mask1, mask2, mask_combined);

            Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
            

            morphologyEx(mask_combined, mask_combined, MORPH_OPEN, kernel);
            

            morphologyEx(mask_combined, mask_combined, MORPH_CLOSE, kernel);

            result = Mat::zeros(frame.size(), frame.type());

            frame.copyTo(result, mask_combined);
            
            // ===== DISPLAY FRAMES =====
            // Tampilkan frame original
            imshow("Original", frame);
            
            // Tampilkan frame saat masking
            imshow("Masked", mask_combined);
            
             // Tampilkan frame setelah masking diterapkan
            imshow("Result", result);
        }
        

        char key = (char)waitKey(paused ? 0 : delay);
 

        if (key == 'q' || key == 'Q' || key == 27) {
            // 'Q' atau ESC (ASCII 27) → Quit program
            cout << "\nExiting program..." << endl;
            break;  // Exit while loop
        }
        else if (key == ' ') {
            // SPACE → Toggle pause
            paused = !paused;  // Flip boolean (true → false, false → true)
            cout << (paused ? "Video paused" : "Video resumed") << endl;
        }
        else if (key == 'r' || key == 'R') {
            // 'R' → Restart video
            cap.set(CAP_PROP_POS_FRAMES, 0);  // Set frame position ke 0 (awal video)
            paused = false;  // Unpause
            cout << "Video restarted" << endl;
        }
        else if (key == 's' || key == 'S') {
            // 'S' → Save/Print current HSV values
            cout << "\n========== Current HSV Values ==========" << endl;
            cout << "Hue Range 1: [" << hue_min1 << ", " << hue_max1 << "]" << endl;
            cout << "Hue Range 2: [" << hue_min2 << ", " << hue_max2 << "]" << endl;
            cout << "Saturation:  [" << sat_min << ", " << sat_max << "]" << endl;
            cout << "Value:       [" << val_min << ", " << val_max << "]" << endl;
            cout << "========================================\n" << endl;
            
        }
    }
    

    cap.release();
    destroyAllWindows();
    
    cout << "Program terminated successfully." << endl;
    
    return 0;  // Exit program dengan success code
}