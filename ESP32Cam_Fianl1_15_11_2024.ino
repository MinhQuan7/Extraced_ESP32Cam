#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Base64.h"
#include "esp_camera.h"
#include "SD_MMC.h"  // Thêm thư viện SD card
#include "time.h"    // Thêm thư viện thời gian để đặt tên file
#include <vector>
#include <algorithm>

struct tm timeinfo;
// Biến để kiểm soát việc gửi ảnh
volatile bool isExtracting = false;
volatile unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_DELAY = 300;


//========================================
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;  // GMT+7 (Vietnam time: 7*3600)
const int daylightOffset_sec = 0;
//======================================== CAMERA_MODEL_AI_THINKER GPIO.
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
//========================================

// LED Flash PIN (GPIO 4)
#define FLASH_LED_PIN 4

//======================================== Enter your WiFi ssid and password.
const char* ssid = "eoh.io";
const char* password = "Eoh@2020";
//========================================

//======================================== Replace with your "Deployment ID" and Folder Name.
String myDeploymentID = "AKfycbyFIAeK-p3mZwWubDVvnW8-rltmF5I_2kGaDWuKtbiCbAU9uAlInAwIGd3TBYp4Lx2jCQ";
String myMainFolderName = "ESP32_CAM";
//========================================

//======================================== Variables for Timer/Millis.
unsigned long previousMillis = 0;
const int Interval = 20000;  //--> Capture and Send a photo every 20 seconds.
//========================================

// Variable to set capture photo with LED Flash.
// Set to "false", then the Flash LED will not light up when capturing a photo.
// Set to "true", then the Flash LED lights up when capturing a photo.
bool LED_Flash_ON = true;

// Initialize WiFiClientSecure.
WiFiClientSecure client;

//________________________________________________________________________________ Test_Con()
// This subroutine is to test the connection to "script.google.com".
void Test_Con() {
  const char* host = "script.google.com";
  while (1) {
    Serial.println("-----------");
    Serial.println("Connection Test...");
    Serial.println("Connect to " + String(host));

    client.setInsecure();

    if (client.connect(host, 443)) {
      Serial.println("Connection successful.");
      Serial.println("-----------");
      client.stop();
      break;
    } else {
      Serial.println("Connected to " + String(host) + " failed.");
      Serial.println("Wait a moment for reconnecting.");
      Serial.println("-----------");
      client.stop();
    }

    delay(1000);
  }
}


//=====================Hàm khởi tạo thời gian =========================
void initTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 5) {
    Serial.println("Failed to obtain time, retrying...");
    delay(1000);
    retry++;
  }

  if (retry >= 5) {
    Serial.println("Could not get time from NTP. Check your internet connection.");
  } else {
    Serial.println("Time synchronized successfully");
  }
}

//________________________________________________________________________________
//=======================SD Card=========================================
String getFileName() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return String("photo_") + String(millis()) + ".jpg";
  }
  char timeStringBuff[50];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y%m%d_%H%M%S", &timeinfo);
  return String("/photo_") + String(timeStringBuff) + ".jpg";
}

//=======================init SDCard Function==============================

esp_err_t res = ESP_OK;
// Function to initialize SD Card
static esp_err_t init_sdcard() {
  // Khởi tạo với one-bit mode
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Failed to mount SD card VFAT filesystem.");
    Serial.println("Please check if:");
    Serial.println("1. SD Card is properly inserted");
    Serial.println("2. SD Card pins are properly connected");
    return ESP_FAIL;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return ESP_FAIL;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // Thử tạo một file test để kiểm tra quyền ghi
  File testFile = SD_MMC.open("/test.txt", FILE_WRITE);
  if (!testFile) {
    Serial.println("Failed to create test file - Check SD card write permissions");
    return ESP_FAIL;
  }
  testFile.close();
  SD_MMC.remove("/test.txt");

  return ESP_OK;
}

//==========================end initSDCard===========================



//================ Function to save photo to SD Card===========
bool savePhotoToSD(camera_fb_t* fb) {
  String path = getFileName();
  Serial.printf("Picture file name: %s\n", path.c_str());

  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
    // Thử tạo thư mục nếu chưa tồn tại
    if (!SD_MMC.mkdir("/photos")) {
      Serial.println("Failed to create photos directory");
      return false;
    }
    // Thử mở file lại
    file = SD_MMC.open(path.c_str(), FILE_WRITE);
    if (!file) {
      return false;
    }
  }

  size_t written = file.write(fb->buf, fb->len);
  file.close();

  if (written != fb->len) {
    Serial.println("Failed to write complete file");
    return false;
  }

  Serial.printf("Saved file: %s, size: %u bytes\n", path.c_str(), fb->len);
  return true;
}
//====================End Save photo to SDCard=========================

//________________________________________________________________________________ SendCapturedPhotos()
// Subroutine for capturing and sending photos to Google Drive.
void SendCapturedPhotos() {
  if (isExtracting) return;
  const char* host = "script.google.com";
  Serial.println();
  Serial.println("-----------");
  Serial.println("Connect to " + String(host));

  client.setInsecure();

  //---------------------------------------- The Flash LED blinks once to indicate connection start.
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(100);
  digitalWrite(FLASH_LED_PIN, LOW);
  delay(100);
  //----------------------------------------

  //---------------------------------------- The process of connecting, capturing and sending photos to Google Drive.
  if (client.connect(host, 443)) {
    Serial.println("Connection successful.");

    if (LED_Flash_ON == true) {
      digitalWrite(FLASH_LED_PIN, HIGH);
      delay(100);
    }

    //.............................. Taking a photo.
    Serial.println();
    Serial.println("Taking a photo...");

    for (int i = 0; i <= 3; i++) {
      camera_fb_t* fb = NULL;
      fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        Serial.println("Restarting the ESP32 CAM.");
        delay(1000);
        ESP.restart();
        return;
      }
      esp_camera_fb_return(fb);
      delay(200);
    }

    camera_fb_t* fb = NULL;
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      Serial.println("Restarting the ESP32 CAM.");
      delay(1000);
      ESP.restart();
      return;
    }

    if (LED_Flash_ON == true) digitalWrite(FLASH_LED_PIN, LOW);

    Serial.println("Taking a photo was successful.");

    //===================================
    // Save to SD Card
    if (savePhotoToSD(fb)) {
      Serial.println("Saved photo to SD Card successfully");
    } else {
      Serial.println("Failed to save photo to SD Card");
    }

    //.............................. Sending image to Google Drive.
    Serial.println();
    Serial.println("Sending image to Google Drive.");
    Serial.println("Size: " + String(fb->len) + "byte");

    String url = "/macros/s/" + myDeploymentID + "/exec?folder=" + myMainFolderName;

    client.println("POST " + url + " HTTP/1.1");
    client.println("Host: " + String(host));
    client.println("Transfer-Encoding: chunked");
    client.println();

    int fbLen = fb->len;
    char* input = (char*)fb->buf;
    int chunkSize = 3 * 1000;  //--> must be multiple of 3.
    int chunkBase64Size = base64_enc_len(chunkSize);
    char output[chunkBase64Size + 1];

    Serial.println();
    int chunk = 0;
    for (int i = 0; i < fbLen; i += chunkSize) {
      int l = base64_encode(output, input, min(fbLen - i, chunkSize));
      client.print(l, HEX);
      client.print("\r\n");
      client.print(output);
      client.print("\r\n");
      delay(100);
      input += chunkSize;
      Serial.print(".");
      chunk++;
      if (chunk % 50 == 0) {
        Serial.println();
      }
    }
    client.print("0\r\n");
    client.print("\r\n");

    esp_camera_fb_return(fb);
    //..............................

    //.............................. Waiting for response.
    Serial.println("Waiting for response.");
    long int StartTime = millis();
    while (!client.available()) {
      Serial.print(".");
      delay(100);
      if ((StartTime + 10 * 1000) < millis()) {
        Serial.println();
        Serial.println("No response.");
        break;
      }
    }
    Serial.println();
    while (client.available()) {
      Serial.print(char(client.read()));
    }
    //..............................

    //.............................. Flash LED blinks once as an indicator of successfully sending photos to Google Drive.
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(500);
    //..............................
  } else {
    Serial.println("Connected to " + String(host) + " failed.");

    //.............................. Flash LED blinks twice as a failed connection indicator.
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(500);
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(500);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(500);
    //..............................
  }
  //----------------------------------------
  Serial.println("\n");
  Serial.println("**********************************");
  client.stop();
}

//__________________________________________________________________________________________________
const size_t BUFFER_SIZE = 1024;
const size_t BASE64_BUFFER_SIZE = (BUFFER_SIZE * 4 / 3) + 4;
//=============================Extract Video When press button=======================
// Struct để lưu thông tin file
#define BUTTON_PIN 12

struct FileInfo {
  String name;
  time_t modified;
};

// Struct để lưu thông tin file với các thành phần cơ bản
struct FileEntry {
    String path;
    unsigned long timestamp;
    
    FileEntry(String p, unsigned long t) : path(p), timestamp(t) {}
    
    // Operator so sánh cho việc sắp xếp
    bool operator < (const FileEntry& other) const {
        return timestamp > other.timestamp;  // Sắp xếp giảm dần (mới nhất trước)
    }
};


// Function để lấy 4 file ảnh mới nhất
std::vector<String> getLatestPhotos() {
    std::vector<FileEntry> fileList;
    std::vector<String> result;
    
    File root = SD_MMC.open("/");
    if(!root){
        Serial.println("Failed to open root directory");
        return result;
    }
    if(!root.isDirectory()){
        Serial.println("Root is not a directory");
        return result;
    }

    // Quét tất cả các file trong thư mục gốc
    File file = root.openNextFile();
    while(file) {
        if(!file.isDirectory()) {
            String fileName = String(file.name());
            if(fileName.endsWith(".jpg")) {
                // Lấy thời gian sửa đổi file
                struct stat st;
                String fullPath = "/" + fileName;
                if(stat(("/sdcard" + fullPath).c_str(), &st) == 0) {
                    fileList.push_back(FileEntry(fullPath, st.st_mtime));
                }
            }
        }
        file = root.openNextFile();
    }
    root.close();

    // Sắp xếp file theo thời gian
    std::sort(fileList.begin(), fileList.end());

    // Lấy 4 file mới nhất
    size_t count = min(size_t(4), fileList.size());
    for(size_t i = 0; i < count; i++) {
        result.push_back(fileList[i].path);
        Serial.println("Selected file: " + fileList[i].path);
    }

    return result;
}

// Thêm includes cần thiết
#include "esp_task_wdt.h"

// Cấu hình cho watchdog timer
const int WDT_TIMEOUT = 30; // seconds
esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,    // Bitmask of all cores
    .trigger_panic = true
};

// Cập nhật hàm SendCapturedPhotos() với watchdog timer
void SendLatestPhotos() {
  if (isExtracting) return;
  isExtracting = true;
  
  const char* host = "script.google.com";
  Serial.println("\n-----------");
  Serial.println("Starting photo extraction process");

  // Get latest photos before button press
  std::vector<String> beforePhotos = getLatestPhotos();
  // Get latest photos after button press (current photos)
  std::vector<String> afterPhotos = getLatestPhotos();
  
  // Combine both vectors into allPhotos
  std::vector<String> allPhotos;
  allPhotos.insert(allPhotos.end(), beforePhotos.begin(), beforePhotos.end());
  allPhotos.insert(allPhotos.end(), afterPhotos.begin(), afterPhotos.end());

  if (allPhotos.empty()) {
    Serial.println("No photos found on SD card");
    isExtracting = false;
    return;
  }

  // Debug print
  Serial.println("Files to be uploaded:");
  Serial.println("Before button press:");
  for (size_t i = 0; i < beforePhotos.size(); i++) {
    Serial.println(beforePhotos[i]);
  }
  Serial.println("After button press:");
  for (size_t i = 0; i < afterPhotos.size(); i++) {
    Serial.println(afterPhotos[i]);
  }

  // Create folder name with timestamp
  char folderName[50];
  if(!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    sprintf(folderName, "Extracted_%lu", millis());
  } else {
    strftime(folderName, sizeof(folderName), "Extracted_%Y%m%d_%H%M%S", &timeinfo);
  }
  
  String extractFolder = String(folderName);
  Serial.println("Creating folder: " + extractFolder);

  int successCount = 0;
  
  for (const String& photoPath : allPhotos) {
    // Reset client for each new connection
    client.stop();
    delay(100);
    client.setInsecure();
    
    Serial.println("\nProcessing: " + photoPath);
    
    // Verify file exists and is readable
    if (!SD_MMC.exists(photoPath)) {
      Serial.println("File does not exist: " + photoPath);
      continue;
    }

    File photoFile = SD_MMC.open(photoPath, FILE_READ);
    if (!photoFile) {
      Serial.println("Failed to open file: " + photoPath);
      continue;
    }

    if (photoFile.size() == 0) {
      Serial.println("File is empty: " + photoPath);
      photoFile.close();
      continue;
    }

    Serial.printf("File size: %d bytes\n", photoFile.size());

    // Connection with retry mechanism
    bool connected = false;
    for(int i = 0; i < 3 && !connected; i++) {
      Serial.printf("Connecting to %s (Attempt %d)\n", host, i + 1);
      
      if (client.connect(host, 443)) {
        connected = true;
        Serial.println("Connected!");
        break;
      }
      delay(100);
    }

    if (!connected) {
      Serial.println("Connection failed!");
      photoFile.close();
      continue;
    }

    // Extract filename from path
    String filename = photoPath;
    if (filename.startsWith("/")) {
      filename = filename.substring(1);
    }
    
    // Prepare HTTP request
    String url = "/macros/s/" + myDeploymentID + "/exec?folder=" + extractFolder + "&filename=" + filename;
    
    Serial.println("Sending request: " + url);
    
    // Send HTTP headers
    client.println("POST " + url + " HTTP/1.1");
    client.println("Host: " + String(host));
    client.println("Transfer-Encoding: chunked");
    client.println();

    // Process file in chunks
    const size_t bufferSize = 3000;
    uint8_t *buffer = (uint8_t *)malloc(bufferSize);
    if(!buffer) {
      Serial.println("Memory allocation failed!");
      photoFile.close();
      continue;
    }

    size_t totalRead = 0;
    size_t fileSize = photoFile.size();

    while (totalRead < fileSize) {
      size_t toRead = min(bufferSize, fileSize - totalRead);
      size_t bytesRead = photoFile.read(buffer, toRead);
      if (bytesRead == 0) break;

      char *base64Buffer = (char *)malloc(base64_enc_len(bytesRead) + 1);
      if (!base64Buffer) {
        Serial.println("Base64 buffer allocation failed!");
        break;
      }

      size_t base64Len = base64_encode(base64Buffer, (char *)buffer, bytesRead);
      base64Buffer[base64Len] = 0; // Null terminate the string

      // Send chunk size
      client.printf("%X\r\n", base64Len);
      
      // Send chunk data
      client.print(base64Buffer);
      client.print("\r\n");

      free(base64Buffer);
      totalRead += bytesRead;

      // Print progress
      Serial.printf("Progress: %d%%\r", (totalRead * 100) / fileSize);
      delay(10);
    }

    free(buffer);
    photoFile.close();

    // Send final chunk
    client.print("0\r\n\r\n");

    // Wait for and verify response
    String response = "";
    unsigned long timeout = millis();
    while (millis() - timeout < 10000) { // 10 second timeout
      if (client.available()) {
        char c = client.read();
        response += c;
        if (response.indexOf("\r\n\r\n") != -1) break;
      }
      delay(10);
    }

    if (response.indexOf("200 OK") != -1 || response.indexOf("302 Found") != -1) {
      Serial.println("\nUpload successful!");
      successCount++;
    } else {
      Serial.println("\nUpload failed! Response:");
      Serial.println(response);
    }

    delay(100); // Delay between files
  }

  client.stop();
  isExtracting = false;
  
  Serial.printf("\nExtraction complete. Successfully uploaded %d/%d files\n", 
                successCount, allPhotos.size());

  // Indicate completion with LED
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(50);
  digitalWrite(FLASH_LED_PIN, LOW);
}
//==============================END EXTRACT VIDEOS WHEN PRESS BUTTON=============================

//=============================Hàm xử lý ngắt=====================
// Khai báo biến volatile để đảm bảo đồng bộ giữa ISR và main loop
volatile bool buttonInterruptFlag = false;
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200; // Thời gian debounce (ms)

// Hàm xử lý interrupt
void IRAM_ATTR buttonISR() {
    unsigned long currentTime = millis();
    if ((currentTime - lastDebounceTime) > debounceDelay) {
        buttonInterruptFlag = true;
        lastDebounceTime = currentTime;
    }
}


//________________________________________________________________________________ VOID SETUP()
void setup() {
  // Disable brownout detector.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println();
  delay(1000);
  pinMode(FLASH_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  // Setting the ESP32 WiFi to station mode.
  Serial.println();
  Serial.println("Setting the ESP32 WiFi to station mode.");
  WiFi.mode(WIFI_STA);

  //---------------------------------------- The process of connecting ESP32 CAM with WiFi Hotspot / WiFi Router.
  Serial.println();
  Serial.print("Connecting to : ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  Serial.println("============================");
  Serial.println("Initializing time...");
  initTime();
  // SD camera init
  Serial.println("Mounting the SD card ...");
  esp_err_t card_err = init_sdcard();
  if (card_err != ESP_OK) {
    Serial.printf("SD Card init failed with error 0x%x", card_err);
    return;
  }

  Serial.println("Initializing SD Card...");
  if (init_sdcard() != ESP_OK) {
    Serial.println("SD Card initialization failed! - - - - -- Failed  - - - Failed");
    // ESP.restart();
  }
  // The process timeout of connecting ESP32 CAM with WiFi Hotspot / WiFi Router is 20 seconds.
  // If within 20 seconds the ESP32 CAM has not been successfully connected to WiFi, the ESP32 CAM will restart.
  // I made this condition because on my ESP32-CAM, there are times when it seems like it can't connect to WiFi, so it needs to be restarted to be able to connect to WiFi.
  int connecting_process_timed_out = 20;  //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(250);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(250);
    if (connecting_process_timed_out > 0) connecting_process_timed_out--;
    if (connecting_process_timed_out == 0) {
      Serial.println();
      Serial.print("Failed to connect to ");
      Serial.println(ssid);
      Serial.println("Restarting the ESP32 CAM.");
      delay(1000);
      ESP.restart();
    }
  }

  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.println();
  Serial.print("Successfully connected to ");
  Serial.println(ssid);
  //Serial.print("ESP32-CAM IP Address: ");
  //Serial.println(WiFi.localIP());
  //----------------------------------------

  //---------------------------------------- Set the camera ESP32 CAM.
  Serial.println();
  Serial.println("Set the camera ESP32 CAM...");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 8;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial.println();
    Serial.println("Restarting the ESP32 CAM.");
    delay(1000);
    ESP.restart();
  }

  sensor_t* s = esp_camera_sensor_get();

  // Selectable camera resolution details :
  // -UXGA   = 1600 x 1200 pixels
  // -SXGA   = 1280 x 1024 pixels
  // -XGA    = 1024 x 768  pixels
  // -SVGA   = 800 x 600   pixels
  // -VGA    = 640 x 480   pixels
  // -CIF    = 352 x 288   pixels
  // -QVGA   = 320 x 240   pixels
  // -HQVGA  = 240 x 160   pixels
  // -QQVGA  = 160 x 120   pixels
  s->set_framesize(s, FRAMESIZE_SXGA);  //--> UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA

  Serial.println("Setting the camera successfully.");
  Serial.println();

  delay(1000);

  Test_Con();

  Serial.println();
  Serial.println("ESP32-CAM captures and sends photos to the server every 20 seconds.");
  Serial.println();
  delay(2000);
}
//________________________________________________________________________________

//=============================VOID LOOP========================
void loop() {
    static bool buttonPressed = false;
    
    // Xử lý nút nhấn với debounce
    if (buttonInterruptFlag) {
        if (!isExtracting) { // Kiểm tra để tránh trùng lặp
            Serial.println("\nButton interrupt detected - Starting extract and upload...");
            SendLatestPhotos();
        }
        buttonInterruptFlag = false;
    }
    
    // Chức năng chụp và upload tự động vẫn hoạt động
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= Interval) {
        previousMillis = currentMillis;
        SendCapturedPhotos();
    }
}