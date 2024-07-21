#include <M5Unified.h>

#define LIDARSerial Serial1

// M5UnifiedのCanvasオブジェクトを初期化。
M5Canvas canvas(&M5.Display);

struct Point_t {
    int16_t x, y;  // 座標点を保持するための構造体
};
double v_Scale = 50.0;  // 描画時のスケーリング係数
enum State_t {
    STATE_FIND_HEADER,
    STATE_READ_PAYLOAD,
    STATE_PROCESS_PACKET
};  // LiDARデータの処理状態

// LiDARからのデータパケットの構造を定義
struct __attribute__((packed)) LidarPacket_t {
    uint8_t header[4];
    uint16_t rotation_speed;
    uint16_t angle_begin;
    uint16_t distance_0;
    uint8_t reserved_0;
    uint16_t distance_1;
    uint8_t reserved_1;
    uint16_t distance_2;
    uint8_t reserved_2;
    uint16_t distance_3;
    uint8_t reserved_3;
    uint16_t distance_4;
    uint8_t reserved_4;
    uint16_t distance_5;
    uint8_t reserved_5;
    uint16_t distance_6;
    uint8_t reserved_6;
    uint16_t distance_7;
    uint8_t reserved_7;
    uint16_t distance_8;
    uint8_t reserved_8;
    uint16_t distance_9;
    uint8_t reserved_9;
    uint16_t distance_10;
    uint8_t reserved_10;
    uint16_t distance_11;
    uint8_t reserved_11;
    uint16_t distance_12;
    uint8_t reserved_12;
    uint16_t distance_13;
    uint8_t reserved_13;
    uint16_t distance_14;
    uint8_t reserved_14;
    uint16_t distance_15;
    uint8_t reserved_15;
    uint16_t angle_end;
    uint16_t crc;
};

const uint8_t header[] = {0x55, 0xaa, 0x23, 0x10};
static Point_t pointCloud[360];
float rotation_speed;
long time_duration;
#define BUFFER_SIZE 1280     // バッファサイズ、必要に応じて調整
#define READ_CHUNK_SIZE 256  // 一度に読み取るバイト数
inline uint16_t convertDegree(uint16_t input) { return (input - 40960) / 64; }
inline uint16_t convertSpeed(uint16_t input) { return input / 64; }

void remapDegrees(uint16_t minAngle, uint16_t maxAngle, uint16_t* map) {
    int16_t delta = (maxAngle - minAngle + 360) % 360;
    if(map == nullptr || delta < 0) return;
    float step = delta / 15.0f;
    for(int32_t cnt = 0; cnt < 16; cnt++) {
        map[cnt] = (minAngle + static_cast<uint16_t>(step * cnt)) % 360;
    }
}

void plotDistanceMap(const uint16_t* degrees, const uint16_t* distances) {
    for(int32_t i = 0; i < 16; i++) {
        uint16_t degree = degrees[i];
        int16_t distance = distances[i];
        if((distance < 10000) && (distance > 0)) {
            float angle = PI * degree / 180.0f;
            int16_t x = cos(angle) * distance;
            int16_t y = sin(angle) * distance;
            pointCloud[degree] = {x, y};
        } else {
            pointCloud[degree] = {0, 0};
        }
    }
}

/* task1のループ関数 */
void task1(void* arg) {
    while(1) {
        static State_t state = STATE_FIND_HEADER;
        static uint8_t buffer[BUFFER_SIZE];
        static uint32_t bufferIndex = 0;
        static uint32_t packetStart = 0;
        static LidarPacket_t packet;

        unsigned long start_time = millis();

        // シリアルバッファからデータを読み取る
        while(LIDARSerial.available() && bufferIndex < BUFFER_SIZE) {
            size_t bytesToRead =
                min(static_cast<size_t>(BUFFER_SIZE - bufferIndex),
                    min(static_cast<size_t>(LIDARSerial.available()),
                        static_cast<size_t>(READ_CHUNK_SIZE)));
            size_t bytesRead =
                LIDARSerial.readBytes(buffer + bufferIndex, bytesToRead);
            bufferIndex += bytesRead;
        }

        // バッファ内のデータを処理
        while(packetStart + sizeof(LidarPacket_t) <= bufferIndex) {
            switch(state) {
                case STATE_FIND_HEADER:
                    // ヘッダーを探す
                    for(; packetStart + sizeof(header) <= bufferIndex;
                        packetStart++) {
                        if(memcmp(buffer + packetStart, header,
                                  sizeof(header)) == 0) {
                            state = STATE_READ_PAYLOAD;
                            break;
                        }
                    }
                    break;

                case STATE_READ_PAYLOAD:
                    // ペイロードを読み取る
                    if(packetStart + sizeof(LidarPacket_t) <= bufferIndex) {
                        memcpy(&packet, buffer + packetStart,
                               sizeof(LidarPacket_t));
                        state = STATE_PROCESS_PACKET;
                    }
                    break;

                case STATE_PROCESS_PACKET:
                    // パケットを処理
                    uint16_t degree_begin = convertDegree(packet.angle_begin);
                    uint16_t degree_end = convertDegree(packet.angle_end);
                    if(degree_begin < 360 && degree_end < 360) {
                        //                       Serial.printf("%3drpm %5d -
                        //                       %5d\n",
                        //                                     convertSpeed(packet.rotation_speed),
                        //                                     degree_begin,
                        //                                     degree_end);
                        rotation_speed = packet.rotation_speed;
                        uint16_t map[16];
                        uint16_t distances[16] = {packet.distance_0 & 0x3FFF,
                                                  packet.distance_1 & 0x3FFF,
                                                  packet.distance_2 & 0x3FFF,
                                                  packet.distance_3 & 0x3FFF,
                                                  packet.distance_4 & 0x3FFF,
                                                  packet.distance_5 & 0x3FFF,
                                                  packet.distance_6 & 0x3FFF,
                                                  packet.distance_7 & 0x3FFF,
                                                  packet.distance_8 & 0x3FFF,
                                                  packet.distance_9 & 0x3FFF,
                                                  packet.distance_10 & 0x3FFF,
                                                  packet.distance_11 & 0x3FFF,
                                                  packet.distance_12 & 0x3FFF,
                                                  packet.distance_13 & 0x3FFF,
                                                  packet.distance_14 & 0x3FFF,
                                                  packet.distance_15 & 0x3FFF};
                        remapDegrees(degree_begin, degree_end, map);
                        plotDistanceMap(map, distances);
                    }

                    packetStart += sizeof(LidarPacket_t);
                    state = STATE_FIND_HEADER;
                    break;
            }
        }
        // 測定したい処理
        unsigned long end_time = millis();
        time_duration = end_time - start_time;
        // 処理済みのデータをバッファから削除
        if(packetStart > 0) {
            memmove(buffer, buffer + packetStart, bufferIndex - packetStart);
            bufferIndex -= packetStart;
            packetStart = 0;
        }

        vTaskDelay(10);
    }
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    canvas.createSprite(320, 240);

    LIDARSerial.begin(230400, SERIAL_8N1, 32, 33);

    Serial.begin(115200);
    Serial.println(" Serial Start");
    Serial.println("LIDARSerial Start");

    xTaskCreatePinnedToCore(task1, "Task1", 4096, NULL, 2, NULL, 1);
}

void loop() {
    // 画面の描画処理を開始
    M5.Display.startWrite();
    canvas.fillRect(0, 0, 320, 240, BLACK);

    static bool view_flg = 1;
    for(int i = 0; i < 359; i++) {
        if((view_flg == 1) && (i <= 358)) {
            int x1 = pointCloud[i].x / v_Scale + 160;
            int y1 = pointCloud[i].y / v_Scale + 120;
            int x2 = pointCloud[i + 1].x / v_Scale + 160;
            int y2 = pointCloud[i + 1].y / v_Scale + 120;
            canvas.fillTriangle(160, 120, x1, y1, x2, y2, YELLOW);
        }

        else {
            int x1 = pointCloud[i].x / v_Scale + 160;
            int y1 = pointCloud[i].y / v_Scale + 120;
            canvas.drawPixel(x1, y1, ORANGE);
        }
    }
    canvas.setCursor(0, 0);
    canvas.printf("Speed : %d rpm  \n", convertSpeed(rotation_speed));
    canvas.printf("scale : %f  \n", v_Scale);
    canvas.printf("duration : %f  \n", time_duration / 1000.0);

    // Canvasに描画された内容をスクリーンに転送
    canvas.pushSprite(0, 0);

    // 画面の描画処理を終了
    M5.Display.endWrite();

    M5.update();
    if(M5.BtnA.isPressed()) {
        Serial.printf("BtnA isPressed");

        v_Scale += 2;
        if(v_Scale > 100) v_Scale = 100;
    }
    if(M5.BtnB.wasPressed()) {
        Serial.printf("BtnB wasPressed");
        view_flg = !view_flg;
    }
    if(M5.BtnC.isPressed()) {
        Serial.printf("BtnC isPressed");
        v_Scale -= 2;
        if(v_Scale < 2) v_Scale = 2;
    }

    M5.delay(10);
}
