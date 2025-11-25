#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <PubSubClient.h>

// Chân kết nối cảm biến siêu âm
#define TRIG_PIN 5
#define ECHO_PIN 18
#define SERVO_PIN 27

Servo barrierServo;

// Biến cho cảm biến siêu âm
unsigned long lastSensorTime = 0;
const unsigned long SENSOR_INTERVAL = 100; // Đọc cảm biến mỗi 100ms
bool objectDetected = false;
bool lastObjectState = false;
float currentDistance = 0.0;

// Biến điều khiển servo
bool barrierIsOpen = false;
unsigned long barrierOpenTime = 0;
const unsigned long BARRIER_OPEN_DURATION = 5000; // 5 giây

// Thông tin WiFi
const char* ssid = "HOANG_GIA_78";
const char* password = "hd456789";

// Thông tin MQTT Server
const char* mqttServer = "tmsherk.id.vn"; // hoặc "13.215.140.112"
const int mqttPort = 1883;
const char* mqttTopic = "entry/gate/control";

// MAC của ESP nhận (ESP32-CAM)
uint8_t receiverMAC[] = {0xD4, 0xE9, 0xF4, 0xA2, 0xEF, 0x58};

// Struct dữ liệu ESP-NOW (đồng bộ với ESP32-CAM)
typedef struct struct_message {
    bool objectDetected;
    char licensePlate[32];
} struct_message;

struct_message dataToSend;
struct_message incomingData;

// Khai báo MQTT Client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Hàm đọc khoảng cách từ cảm biến siêu âm
float readDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.034 / 2; // Tính khoảng cách theo cm
    
    return distance;
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

// Hàm xử lý tin nhắn MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Nhan tin nhan MQTT: [");
    Serial.print(topic);
    Serial.print("] ");
    
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println(message);
    
    // Xử lý lệnh điều khiển barrier
    if (message == "OPEN_GATE") {
        Serial.println("LENH MO CUA");
        openBarrier();
    } else if (message == "CLOSE_GATE") {
        Serial.println("LENH DONG CUA");
        closeBarrier();
    }
}

// Hàm kết nối lại MQTT
void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Dang ket noi MQTT...");
        
        // Tạo client ID ngẫu nhiên
        String clientId = "ESP32-Barrier-";
        clientId += String(random(0xffff), HEX);
        
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("Da ket noi MQTT!");
            
            // Subscribe vào topic điều khiển
            mqttClient.subscribe(mqttTopic);
            Serial.print("Da subscribe vao topic: ");
            Serial.println(mqttTopic);
            
        } else {
            Serial.print("Loi ket noi MQTT, code: ");
            Serial.print(mqttClient.state());
            Serial.println(". Thu lai sau 5s...");
            delay(5000);
        }
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
    
    // CHỈ IN THÔNG TIN, KHÔNG MỞ BARRIER
    // Kiểm tra nếu có objectDetected và licensePlate
    if (incomingData.objectDetected && strlen(incomingData.licensePlate) > 0) {
        // Chuyển mảng char sang String
        String licensePlate = String(incomingData.licensePlate);
        
        Serial.print("Nhan duoc bien so: ");
        Serial.println(licensePlate);
        
        Serial.println("BIEN SO XE:");
        Serial.println(licensePlate);
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
    
    Serial.println("He thong bat dau");
    
    // Khởi tạo chân cảm biến siêu âm
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    // Khởi tạo Servo
    barrierServo.attach(SERVO_PIN);
    closeBarrier(); // Đảm bảo barrier đóng khi khởi động
    Serial.println("Servo da khoi tao");
    
    // Đặt chế độ WiFi và kết nối
    WiFi.mode(WIFI_STA);
    connectWiFi();
    
    // Cấu hình MQTT
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);
    
    Serial.println("Dang giam sat cam bien sieu am...");

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

    // Khởi tạo biến licensePlate trong struct gửi đi
    memset(dataToSend.licensePlate, 0, sizeof(dataToSend.licensePlate));
    
    Serial.println("He thong san sang nhan tin hieu cam bien VA dieu khien barrier qua MQTT");
}

void loop() {
    // Duy trì kết nối MQTT
    if (!mqttClient.connected()) {
        reconnectMQTT();
    }
    mqttClient.loop();
    
    // Đọc cảm biến siêu âm theo chu kỳ
    if (millis() - lastSensorTime >= SENSOR_INTERVAL) {
        lastSensorTime = millis();
        
        currentDistance = readDistance();
        
        // Kiểm tra ngưỡng phát hiện (3.5cm)
        bool newObjectState = (currentDistance < 3.5 && currentDistance > 0);
        
        // Chỉ gửi dữ liệu khi có thay đổi trạng thái
        if (newObjectState != lastObjectState) {
            objectDetected = newObjectState;
            lastObjectState = newObjectState;
            
            // Gửi dữ liệu qua ESP-NOW (GIỮ NGUYÊN)
            sendData();
            
            Serial.print("Khoang cach: ");
            Serial.print(currentDistance);
            Serial.print("cm - Phat hien vat can: ");
            Serial.println(objectDetected ? "CO" : "KHONG");
        }
    }
    
    // Xử lý đóng barrier sau 5 giây (nếu được mở bằng MQTT)
    if (barrierIsOpen && (millis() - barrierOpenTime > BARRIER_OPEN_DURATION)) {
        closeBarrier();
    }
    
    delay(10); // Tránh đọc cảm biến quá nhanh
}