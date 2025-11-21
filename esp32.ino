#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SENSOR_PIN 23
#define SERVO_PIN 27

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Servo barrierServo;

// Biến để theo dõi trạng thái cảm biến
bool lastSensorState = true;
bool objectDetected = true;
unsigned long lastDetectionTime = 0;
unsigned long detectionCount = 0;

// Biến lưu trữ biển số nhận được từ ESP32-CAM
String receivedLicensePlate = "";

// Biến điều khiển servo
bool shouldOpenBarrier = false;
bool barrierIsOpen = false;
unsigned long barrierOpenTime = 0;
const unsigned long BARRIER_OPEN_DURATION = 5000; // 3 giây

// Thông tin WiFi
const char* ssid = "HOANG_GIA_78";
const char* password = "hd456789";

// MAC của ESP nhận (ESP32-CAM)
uint8_t receiverMAC[] = {0xD4, 0xE9, 0xF4, 0xA2, 0xEF, 0x58};

// Struct dữ liệu ESP-NOW (đồng bộ với ESP32-CAM)
typedef struct struct_message {
    bool objectDetected;
    char licensePlate[32];
} struct_message;

struct_message dataToSend;
struct_message incomingData;

void updateDisplay() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    
    // Hiển thị trạng thái cảm biến
    display.print("Trang thai: ");
    display.println(objectDetected ? "CO VAT CAN" : "KHONG CO");
    
    // Hiển thị số lần phát hiện
    display.print("Dem: ");
    display.println(detectionCount);
    
    // Hiển thị trạng thái barrier
    display.print("Barrier: ");
    display.println(barrierIsOpen ? "MO" : "DONG");
    
    // Hiển thị biển số xe (nếu có)
    display.println("---------------");
    display.setTextSize(1);
    display.println("BIEN SO XE:");
    
    if (receivedLicensePlate.length() > 0) {
        // Hiển thị biển số với kích thước lớn hơn
        display.setTextSize(2);
        display.setCursor(0, 40);
        display.println(receivedLicensePlate);
    } else {
        display.setTextSize(1);
        display.setCursor(0, 40);
        display.println("Dang cho...");
    }
    
    display.display();
}

void openBarrier() {
    barrierServo.write(90); // Mở barrier (90 độ)
    barrierIsOpen = true;
    barrierOpenTime = millis();
    Serial.println("BARRIER: MO");
}

void closeBarrier() {
    barrierServo.write(0); // Đóng barrier (0 độ)
    barrierIsOpen = false;
    Serial.println("BARRIER: DONG");
}

void sendData() {
    dataToSend.objectDetected = objectDetected;
    // Xóa nội dung biển số cũ trong data gửi đi
    memset(dataToSend.licensePlate, 0, sizeof(dataToSend.licensePlate));
    
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *) &dataToSend, sizeof(dataToSend));
    
    if (result == ESP_OK) {
        Serial.println("Gui thanh cong");
    } else {
        Serial.println("Loi gui");
    }
}

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.print("Trang thai gui: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Thanh cong" : "That bai");
}

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    // Copy dữ liệu nhận được vào struct toàn cục
    memcpy(&incomingData, data, min(len, (int)sizeof(incomingData)));
    
    Serial.print("Nhan du lieu tu: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", info->src_addr[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    
    // Kiểm tra nếu có objectDetected và licensePlate
    if (incomingData.objectDetected && strlen(incomingData.licensePlate) > 0) {
        // Chuyển mảng char sang String và lưu vào biến toàn cục
        receivedLicensePlate = String(incomingData.licensePlate);
        
        Serial.print("Nhan duoc bien so: ");
        Serial.println(receivedLicensePlate);
        
        // Kích hoạt mở barrier
        shouldOpenBarrier = true;
        
        // Cập nhật thời gian nhận biển số
        lastDetectionTime = millis();
        
        // Cập nhật màn hình ngay lập tức để hiển thị biển số mới
        updateDisplay();
    } else {
        Serial.println("Khong co bien so trong du lieu nhan duoc");
    }
}

void connectWiFi() {
    Serial.println("Dang ket noi WiFi...");
    Serial.print("SSID: ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, password);
    
    // Chờ kết nối trong 5 giây, nếu không được thì tiếp tục cho ESP-NOW
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 5000) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nDa ket noi WiFi!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nKhong the ket noi WiFi, nhung ESP-NOW van hoat dong");
        Serial.println("ESP-NOW se su dung channel mac dinh");
    }
}

void setup() {
    Serial.begin(115200);
    
    // Khởi tạo OLED trước
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("Loi khoi tao SSD1306"));
        for(;;);
    }
    
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    
    // Hiển thị thông báo khởi động
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Dang khoi dong...");
    display.display();
    
    Serial.println("He thong bat dau");
    
    // Khởi tạo Servo
    barrierServo.attach(SERVO_PIN);
    closeBarrier(); // Đảm bảo barrier đóng khi khởi động
    Serial.println("Servo da khoi tao");
    
    // Đặt chế độ WiFi và kết nối
    WiFi.mode(WIFI_STA);
    connectWiFi();

    // Khởi tạo chân cảm biến
    pinMode(SENSOR_PIN, INPUT);
    
    Serial.println("Dang giam sat cam bien...");

    // Khởi tạo ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Loi khoi tao ESP-NOW!");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Thêm peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Loi them peer");
        return;
    }

    // Đọc giá trị cảm biến ban đầu
    lastSensorState = digitalRead(SENSOR_PIN);
    objectDetected = !lastSensorState; // Cảm biến tích cực mức LOW
    
    // Khởi tạo biến licensePlate trong struct gửi đi
    memset(dataToSend.licensePlate, 0, sizeof(dataToSend.licensePlate));
    
    // Cập nhật màn hình lần đầu
    updateDisplay();
    
    Serial.println("He thong san sang nhan tin hieu cam bien VA dieu khien barrier");
}

void loop() {
    bool currentSensorState = digitalRead(SENSOR_PIN);
    
    // Phát hiện thay đổi trạng thái cảm biến
    if (currentSensorState != lastSensorState) {
        lastSensorState = currentSensorState;
        objectDetected = !currentSensorState; // Đảo ngược vì cảm biến tích cực mức LOW
        
        // Cập nhật thời gian và số lần phát hiện
        if (objectDetected) {
            detectionCount++;
            lastDetectionTime = millis();
        }
        
        // Gửi dữ liệu qua ESP-NOW (chỉ gửi trạng thái cảm biến)
        sendData();
        
        // Cập nhật màn hình
        updateDisplay();
        
        Serial.print("Phat hien vat can: ");
        Serial.println(objectDetected ? "CO" : "KHONG");
    }
    
    // Xử lý mở barrier khi nhận được biển số
    if (shouldOpenBarrier && !barrierIsOpen) {
        openBarrier();
        shouldOpenBarrier = false;
        updateDisplay(); // Cập nhật hiển thị trạng thái barrier
    }
    
    // Xử lý đóng barrier sau 3 giây
    if (barrierIsOpen && (millis() - barrierOpenTime > BARRIER_OPEN_DURATION)) {
        closeBarrier();
        updateDisplay(); // Cập nhật hiển thị trạng thái barrier
    }
    
    // Cập nhật màn hình định kỳ để hiển thị thời gian
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 1000) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }
    
    delay(100); // Tránh đọc cảm biến quá nhanh
}