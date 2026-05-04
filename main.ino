#include <WiFi.h>
#include <WebServer.h>

// ================== 引脚定义 ==================
#define PWMA  8
#define AIN1 10
#define AIN2 11
#define PWMB 16
#define BIN1 18
#define BIN2 19
#define STBY 15

#define TRIG_PIN  4
#define ECHO_PIN  6

// 编码器引脚
#define LEFT_A   5
#define LEFT_B   7
#define RIGHT_A  9
#define RIGHT_B  36

#define SAFE_DISTANCE  30.0

// ================== 核心参数 ==================
const int BASE_SPEED = 40;
const int PIVOT_SPEED = 40; 

// ★ 优化 1：缩小转向脉冲数，解决转角过大
const int TARGET_PULSES_90DEG = 250; 
#define TURN_LEFT 0
#define TURN_RIGHT 1

// PID 参数
float Kp = 0.9;
float Ki = 0.003;     
float Kd = 1.4;       

float errorSum = 0;
float lastError = 0;

volatile long leftCount = 0;
volatile long rightCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;

// ================== 系统全局变量 ==================
const char* ssid = "SmartCar-S3";
const char* password = "12345678";

WebServer server(80);
enum Mode { AUTO_AVOID, MANUAL };

// ★ 优化 2：开机默认进入自动避障模式，无需连接手机即可跑动
Mode currentMode = AUTO_AVOID; 

unsigned long manualEndTime = 0;
float currentDistance = 999.0;

TaskHandle_t TaskWeb;
TaskHandle_t TaskMotor;

// ================== 网页HTML/JS代码 (Raw Literal) ==================
const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no'>
  <title>智能小车驾驶舱</title>
  <style>
    body { font-family: Arial; text-align: center; background: #2c3e50; color: white; margin: 0; padding: 10px; overflow: hidden; }
    .card { background: #34495e; padding: 15px; margin: 10px auto; border-radius: 10px; max-width: 400px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
    button { background: #3498db; color: white; border: none; padding: 12px 20px; border-radius: 5px; margin: 5px; font-size: 16px; }
    button:active { background: #2980b9; }
    input { width: 60px; text-align: center; font-size: 16px; }
    .joystick-zone { position: relative; width: 200px; height: 200px; background: rgba(255,255,255,0.1); border-radius: 50%; margin: 20px auto; touch-action: none; }
    #stick { position: absolute; width: 60px; height: 60px; background: #e74c3c; border-radius: 50%; top: 70px; left: 70px; box-shadow: 0 4px 10px rgba(0,0,0,0.4); pointer-events: none; }
  </style>
</head>
<body>
  <h2>ESP32-S3智能小车控制中心</h2>
  <div class='card'>
    模式：<strong id='modeStr' style='color:#f1c40f;'>加载中...</strong><br>
    雷达：<span style='font-size:28px;color:#2ecc71;font-weight:bold;' id='dist'>-- cm</span>
  </div>
  
  <div class='card'>
    <button onclick="switchMode('auto')">自动避障</button>
    <button onclick="switchMode('manual')" style="background:#e67e22;">手动摇杆</button>
  </div>

  <div class='card'>
    <h3 style="margin-top:0;">方向摇杆</h3>
    <div class='joystick-zone' id='zone'>
      <div id='stick'></div>
    </div>
  </div>

  <div class='card'>
    <h3 style="margin-top:0;">PID 现场调参</h3>
    Kp: <input id='kp' value='0.9'> 
    Ki: <input id='ki' value='0.003'> 
    Kd: <input id='kd' value='1.4'><br><br>
    <button onclick="updatePID()" style="background:#9b59b6;">推送到小车</button>
  </div>

  <script>
    setInterval(() => { 
      fetch('/data').then(r=>r.json()).then(d => { 
        document.getElementById('dist').innerText = d.distance.toFixed(1) + ' cm'; 
        document.getElementById('modeStr').innerText = d.mode === 0 ? "自动避障" : "手动摇杆";
        if(!window.pidLoaded){
            document.getElementById('kp').value = d.kp;
            document.getElementById('ki').value = d.ki;
            document.getElementById('kd').value = d.kd;
            window.pidLoaded = true;
        }
      }).catch(e=>{}); 
    }, 500);

    function switchMode(m) { fetch('/mode/' + m); }
    function updatePID() { 
      let p = document.getElementById('kp').value, i = document.getElementById('ki').value, d = document.getElementById('kd').value; 
      fetch(`/setpid?p=${p}&i=${i}&d=${d}`).then(() => alert('PID参数已更新生效！')); 
    }

    let zone = document.getElementById('zone'); 
    let stick = document.getElementById('stick'); 
    let isMoving = false;

    zone.addEventListener('touchstart', start, {passive: false});
    window.addEventListener('touchend', end, {passive: false});
    zone.addEventListener('touchmove', move, {passive: false});

    function start(e) { isMoving = true; }
    function end(e) { 
      isMoving = false; 
      stick.style.left = '70px'; stick.style.top = '70px'; 
      sendJoy(0, 0); 
    }
    function move(e) {
      if (!isMoving) return; 
      e.preventDefault(); 
      let rect = zone.getBoundingClientRect(); 
      let touch = e.touches[0];
      let x = touch.clientX - rect.left - 100; 
      let y = touch.clientY - rect.top - 100;
      
      let dist = Math.sqrt(x*x + y*y); 
      if (dist > 80) { x *= 80/dist; y *= 80/dist; } 
      
      stick.style.left = (x + 70) + 'px'; 
      stick.style.top = (y + 70) + 'px';
      
      sendJoy(Math.round(x * 1.25), Math.round(-y * 1.25));
    }

    let lastSend = 0;
    function sendJoy(x, y) {
      let now = Date.now(); 
      if (now - lastSend < 80 && (x !== 0 || y !== 0)) return; 
      lastSend = now;
      fetch(`/joy?x=${x}&y=${y}`).catch(() => { console.log('网络波动'); });
    }
  </script>
</body>
</html>
)rawliteral";

// ================== 底层硬件驱动 ==================
void IRAM_ATTR leftISR() {
  if (digitalRead(LEFT_B)) leftCount++; else leftCount--;
}

void IRAM_ATTR rightISR() {
  if (digitalRead(RIGHT_B)) rightCount++; else rightCount--;
}

float getRawDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 40000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2.0;
}

float getMedianDistance() {
  float dists[5];
  for (int i = 0; i < 5; i++) {
    dists[i] = getRawDistance();
    vTaskDelay(3 / portTICK_PERIOD_MS); 
  }
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (dists[i] > dists[j]) { float temp = dists[i]; dists[i] = dists[j]; dists[j] = temp; }
    }
  }
  return dists[2]; 
}

void motorControl(int left, int right) {
  digitalWrite(AIN1, left > 0 ? HIGH : LOW);
  digitalWrite(AIN2, left < 0 ? HIGH : LOW);
  analogWrite(PWMA, abs(left));

  digitalWrite(BIN1, right > 0 ? HIGH : LOW);
  digitalWrite(BIN2, right < 0 ? HIGH : LOW);
  analogWrite(PWMB, abs(right));
}

// ================== 精确转向函数 ==================
void turn90Degrees(int direction) {
  motorControl(0, 0); 
  vTaskDelay(200 / portTICK_PERIOD_MS);

  long startLeft = leftCount;
  long startRight = rightCount;

  while (abs(leftCount - startLeft) < TARGET_PULSES_90DEG && abs(rightCount - startRight) < TARGET_PULSES_90DEG) {
    if (direction == TURN_RIGHT) {
      motorControl(PIVOT_SPEED, -PIVOT_SPEED);
    } else {
      motorControl(-PIVOT_SPEED, PIVOT_SPEED);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }

  motorControl(0, 0); 
  vTaskDelay(300 / portTICK_PERIOD_MS); 

  lastLeftCount = leftCount;
  lastRightCount = rightCount;
  errorSum = 0; 
  lastError = 0;
}

// ================== PID 控制算法 ==================
void goStraightWithPID() {
  long leftSpeed = leftCount - lastLeftCount;
  long rightSpeed = rightCount - lastRightCount;
  lastLeftCount = leftCount;
  lastRightCount = rightCount;

  float error = rightSpeed - leftSpeed;
  errorSum += error * 0.02; 
  if(errorSum > 100) errorSum = 100;
  if(errorSum < -100) errorSum = -100;

  float derivative = error - lastError;
  int correction = (int)(Kp * error + Ki * errorSum + Kd * derivative);

  int leftOut = BASE_SPEED + correction;
  int rightOut = BASE_SPEED - correction + 6;   

  leftOut = constrain(leftOut, 56, 75);
  rightOut = constrain(rightOut, 56, 75);

  motorControl(leftOut, rightOut);
  lastError = error;
}

// ================== 双核任务定义 ==================

// Core 0: WiFi 和 Web服务器
void TaskWebcode( void * pvParameters ) {
  for(;;) {
    server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// Core 1: 传感器采样、PID与运动控制
void TaskMotorcode( void * pvParameters ) {
  for(;;) {
    currentDistance = getMedianDistance(); 

    if (currentMode == MANUAL) {
      if (manualEndTime > 0 && millis() > manualEndTime) {
        motorControl(0, 0);
        manualEndTime = 0;
      }
    } else {
      if (currentDistance < SAFE_DISTANCE && currentDistance > 2) {
        motorControl(0, 0);
        vTaskDelay(300 / portTICK_PERIOD_MS); 

        int attempts = 0;
        while (attempts < 4) { 
          turn90Degrees(TURN_RIGHT); 
          
          currentDistance = getMedianDistance(); 
          if (currentDistance > SAFE_DISTANCE) {
            break; 
          }
          attempts++;
        }
      } else {
        goStraightWithPID();
      }
    }
    vTaskDelay(30 / portTICK_PERIOD_MS); 
  }
}

// ================== Setup 初始化 ==================
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(LEFT_A, INPUT_PULLUP); pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP); pinMode(RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightISR, RISING);

  // ★ 优化 3：设置信道为 6 避开拥堵，禁用休眠
  WiFi.softAP(ssid, password, 6); 
  WiFi.setSleep(false); 
  // 可选：如果网络依然极差，取消下方代码注释以增强发射功率
  // WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  Serial.println("系统启动...");
  Serial.println(WiFi.softAPIP().toString());

  server.on("/", []() { server.send(200, "text/html", index_html); });

  server.on("/data", []() { 
    String json = "{\"distance\":" + String(currentDistance, 1) + 
                  ", \"mode\":" + String(currentMode) + 
                  ", \"kp\":" + String(Kp, 3) + 
                  ", \"ki\":" + String(Ki, 3) + 
                  ", \"kd\":" + String(Kd, 3) + "}";
    server.send(200, "application/json", json); 
  });

  server.on("/joy", []() {
    if (currentMode != MANUAL) currentMode = MANUAL;
    int x = server.arg("x").toInt(); 
    int y = server.arg("y").toInt(); 
    
    if (x == 0 && y == 0) {
      motorControl(0, 0); 
    } else {
      int leftOut = y + x;
      int rightOut = y - x;
      leftOut = map(leftOut, -200, 200, -90, 90);
      rightOut = map(rightOut, -200, 200, -90, 90);
      motorControl(leftOut, rightOut);
    }
    
    manualEndTime = millis() + 1500; 
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  });

  server.on("/mode/auto", []() { currentMode = AUTO_AVOID; motorControl(0,0); server.send(200, "text/plain", "OK"); });
  server.on("/mode/manual", []() { currentMode = MANUAL; motorControl(0,0); server.send(200, "text/plain", "OK"); });
  server.on("/setpid", []() {
    if(server.hasArg("p")) Kp = server.arg("p").toFloat();
    if(server.hasArg("i")) Ki = server.arg("i").toFloat();
    if(server.hasArg("d")) Kd = server.arg("d").toFloat();
    server.send(200, "text/plain", "OK");
  });

  server.begin();

  xTaskCreatePinnedToCore(TaskWebcode, "WebTask", 8192, NULL, 1, &TaskWeb, 0);    
  xTaskCreatePinnedToCore(TaskMotorcode, "MotorTask", 4096, NULL, 2, &TaskMotor, 1);
}

void loop() {
  vTaskDelete(NULL); 
}