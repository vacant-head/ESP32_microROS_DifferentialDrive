#include <Arduino.h>
#include <PS4Controller.h>
#include <micro_ros_platformio.h>
#include <uxr/client/transport.h> // Add this line to include the transport header
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define ESP_BT_MAC "a0:a3:b3:ab:dd:02"



/*----------制御周期など----------*/
#define CONTROL_PERIOD 1000                             // us
#define CONTROL_FREQ (1.0f / (CONTROL_PERIOD * 1.0e-6)) // Hz
int control_count = 0;
int interval = 0;
int preinterval = 0;

// フィードバックゲイン
#define STEER_GAIN_P 1.0f
#define STEER_GAIN_I 0.0
#define STEER_GAIN_D 0.0

/*----------ピン設定----------*/
// モーターピン･チャンネル
#define MOTOR_R_IN1 32
#define MOTOR_R_IN2 25
#define MOTOR_L_IN1 26
#define MOTOR_L_IN2 27
// 磁気エンコーダピン アナログ出力
#define ENCO_R_PIN 34
#define ENCO_L_PIN 35
// ゲイン調整用ポテンショ
#define POT3 36
// ローターリーエンコーダ
#define TOP_ENCODER_A 18
#define TOP_ENCODER_B 19
#define BOTTOM_ENCODER_A 16
#define BOTTOM_ENCODER_B 17
// 状態表示LED
#define STATUS_LED1 14
#define STATUS_LED2 4
#define CH_LED1 5
#define CH_LED2 6
// スイッチ
#define SW_PIN 13
/*------------------------*/

/*----------メカの定数----------*/
#define WHEEL_DIAM 50.f // mm
#define TREAD 153.f     // mm
#define MAX_SPEED 50.f  // mm/s
/*-----------------------------*/

// ledcWrite()
#define CH_1 1
#define CH_2 2
#define CH_3 3
#define CH_4 4
#define LEDC_FREQ 20000   // Hz
#define LEDC_RESOLUTION 8 // bit

// タイヤの回転量
float theta_r = 0.0;
float theta_l = 0.0;
float theta_r_total = 0.0;
float theta_l_total = 0.0;
// タイヤの角速度
float omega_r = 0.0;
float omega_l = 0.0;

// ps4コントローラーの指令値
float ps4_y;
float ps4_yaw;

// PS4の司令値
float cmd_v_y;
float cmd_v_yaw;

//タイヤの速度目標値
float vr_ref = 0.0;
float vl_ref = 0.0;

//エンコーダ
int enco_r = 0;
int pre_enco_r = 0;
int enco_l = 0;
int pre_enco_l = 0;

// ロボットの位置
float x_robo = 0.0;
float y_robo = 0.0;
float yaw_robo = 0.0;
float pre_x_robo = 0.0;
float pre_y_robo = 0.0;
float pre_yaw_robo = 0.0;

// モーターデューティ
float motor_r_duty_out = 0.0;
float motor_l_duty_out = 0.0;

struct POSITION
{
  float x;   // mm
  float y;   // mm
  float yaw; // rad
};
POSITION robotPos;
POSITION worldPos;

// micro-ROSのセットアップ
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;// エグゼキュータの作成
rclc_support_t support;
rcl_allocator_t allocator;// アロケータの作成
rcl_node_t node;// ノードの作成
rcl_timer_t timer;// タイマーの作成

// 速度指令値を受信するコールバック関数
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear_x = msg->linear.x;
  float linear_y = msg->linear.y;
  float angular_z = msg->angular.z;

  // 受信した速度指令値を処理する
  // 例えば、受信した指令値に基づいてモーターの速度を設定する
  linear_x = cmd_v_y;
}

// タイマーのコールバック関数
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void) last_call_time;
  // タイマーのコールバック関数内でエグゼキュータをスピンする
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

void getPS4(float *y, float *yaw)
{
  // PS4の値を正規化
  if (PS4.LStickX() >= 10 || PS4.LStickX() <= -10)
  {
    *y = (float)PS4.LStickY() / 128.f;
  }
  else
  {
    *y = 0;
  }
  if (PS4.LStickY() >= 10 || PS4.LStickY() <= -10)
  {
    *yaw = (float)PS4.RStickX() / 128.f;
  }
  else
  {
    *yaw = 0;
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  set_microros_serial_transports(Serial);
  delay(2000);


    PS4.begin("a0:a3:b3:ab:dd:02");
  // if ()
  // {
  //   Serial.println("PS4 Controller Connected");
  // }
  // else
  // {
  //   Serial.println("Could not connect to PS4 Controller");
  // }

  // micro-ROSの初期化
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // ノードの作成
  rclc_node_init_default(&node, "esp32_node", "", &support);

  // サブスクライバの作成
  rclc_subscription_init_default(&subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"cmd_vel");
  const unsigned int timer_timeout = 100;
  rclc_timer_init_default(&timer,&support,RCL_MS_TO_NS(timer_timeout),timer_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

  ledcAttachPin(MOTOR_R_IN1, CH_1);
  ledcAttachPin(MOTOR_R_IN2, CH_2);
  ledcAttachPin(MOTOR_L_IN1, CH_3);
  ledcAttachPin(MOTOR_L_IN2, CH_4);
  ledcSetup(CH_1, LEDC_FREQ, LEDC_RESOLUTION);
  ledcSetup(CH_2, LEDC_FREQ, LEDC_RESOLUTION);
  ledcSetup(CH_3, LEDC_FREQ, LEDC_RESOLUTION);
  ledcSetup(CH_4, LEDC_FREQ, LEDC_RESOLUTION);

  // エンコーダ
  pinMode(ENCO_R_PIN, INPUT);
  pinMode(ENCO_L_PIN, INPUT);
  // ゲイン調整用ポテンショ
  pinMode(POT3, INPUT);

  pre_enco_r = analogRead(ENCO_R_PIN);
  pre_enco_l = analogRead(ENCO_L_PIN);

  // 状態表示LED
  // ledcAttachPin(STATUS_LED1, CH_LED1);
  // ledcSetup(CH_LED1, LEDC_FREQ, LEDC_RESOLUTION);
  // ledcAttachPin(STATUS_LED2, CH_LED2);
  // ledcSetup(CH_LED2, LEDC_FREQ, LEDC_RESOLUTION);

  // スイッチ
  // pinMode(SW_PIN, INPUT);
  robotPos.x = 0.0;
  robotPos.y = 0.0;
  robotPos.yaw = 0.0;
  worldPos.x = 0.0;
  worldPos.y = 0.0;
  worldPos.yaw = -M_PI / 2.0;
}

void loop()
{
  // エグゼキュータをスピンして、受信メッセージを処理する
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  // PS4
  if (PS4.isConnected())
  {
    getPS4(&ps4_y, &ps4_yaw);
  }
  else
  {
    ps4_y = 0.f;
    ps4_yaw = 0.f;
  }
  // ロボットの速度司令値
  cmd_v_y = ps4_y * MAX_SPEED;
  cmd_v_yaw = ps4_yaw * M_PI * TREAD / MAX_SPEED;
  //速度指令値-->車輪の目標値に変換
  vr_ref = (2*cmd_v_y+cmd_v_yaw*TREAD)/(WHEEL_DIAM);
  vl_ref = (2*cmd_v_y-cmd_v_yaw*TREAD)/(WHEEL_DIAM);

  // エンコーダ生の値
  enco_r = analogRead(ENCO_R_PIN);
  enco_l = analogRead(ENCO_L_PIN);
  // エンコーダ生の値で右と左の回転量を計算，アブソ-->インクリ
  int rot_r = enco_r - pre_enco_r;
  int rot_l = enco_l - pre_enco_l;
  if (rot_r < -2000)
  {
    rot_r = 4096 - pre_enco_r + enco_r;
  }
  else if (rot_r > 2000)
  {
    rot_r = -(pre_enco_r + 4096 - enco_r);
  }
  if (rot_l < -2000)
  {
    rot_l = 4096 - pre_enco_l + enco_l;
  }
  else if (theta_l > 2000)
  {
    rot_l = -(pre_enco_l + 4096 - enco_l);
  }
  pre_enco_r = enco_r;
  pre_enco_l = enco_l;

  // 車輪の移動量
  float mv_r = -(WHEEL_DIAM * 0.5 * (float)rot_r / 4096.f * 2.f * M_PI); // mm
  float mv_l = WHEEL_DIAM * 0.5 * (float)rot_l / 4096.f * 2.f * M_PI;    // mm

  //車輪の角速度
  omega_r = (float)rot_r / 4096.f * 2.f * M_PI / (CONTROL_PERIOD * 1.0e-6);
  omega_l = (float)rot_l / 4096.f * 2.f * M_PI / (CONTROL_PERIOD * 1.0e-6);

  // ロボットの移動量と回転量
  float disp_robo = (mv_r + mv_l) / 2.0; // rad
  float angle_robo = (mv_r - mv_l) / (2.0 * TREAD * 0.5);

  // ロボット座標系
  robotPos.yaw += angle_robo;
  if (robotPos.yaw > (2 * M_PI))
  {
    robotPos.yaw -= 2 * M_PI;
  }
  robotPos.x += disp_robo * cos(robotPos.yaw);
  robotPos.y += disp_robo * sin(robotPos.yaw);
  // ワールド座標系
  worldPos.yaw += robotPos.yaw;
  if (worldPos.yaw > (2 * M_PI))
  {
    worldPos.yaw -= 2 * M_PI;
  }
  worldPos.x += robotPos.x * cos(worldPos.yaw);
  worldPos.y += robotPos.y * sin(worldPos.yaw);




  float err_v_r = vr_ref - omega_r;
  float err_v_l = vl_ref - omega_l;

  motor_r_duty_out = -STEER_GAIN_P * err_v_r;
  motor_l_duty_out = STEER_GAIN_P * err_v_l;


  // 自己位置
  // float delta_x_robo = (lr + ll) / 2.0;
  // float delta_yaw_robo = (lr - ll) / (2.0 * TREAD*0.5);
  // float rotate_radius = 0.0;
  // if (abs(delta_yaw_robo) > 0.001)
  // {
  //   rotate_radius = delta_x_robo / delta_yaw_robo;
  // }
  // else
  // {
  //   rotate_radius = 0.0;
  // }

  // yaw_robo = pre_yaw_robo + delta_yaw_robo;
  // x_robo = pre_x_robo + 2 * rotate_radius * sin(delta_yaw_robo * 0.5) * cos(pre_yaw_robo + delta_yaw_robo * 0.5);
  // y_robo = pre_y_robo + 2 * rotate_radius * sin(delta_yaw_robo * 0.5) * sin(pre_yaw_robo + delta_yaw_robo * 0.5);

  // pre_yaw_robo = yaw_robo;
  // pre_x_robo = x_robo;
  // pre_y_robo = y_robo;

  // motor_r_duty_out = 255.0 * command_speed * 0.25 * 4.0 + steer_calcPID;
  // motor_l_duty_out = 255.0 * command_speed * 0.25 * 4.0 - steer_calcPID;


  if (control_count % 50)
  {
    // Serial.printf("%f,%f,%f\r\n", steer_gainP, steer_gainD, steer_gainD); // ゲイン
    Serial.printf("%f,%f\r\n", motor_r_duty_out, motor_l_duty_out); // デューティ
    // Serial.printf("%f\r\n", err_steer); // 偏差
    // Serial.printf("%f,%f\r\n", command_X, command_Y); // 指令値ベクトル
    // Serial.printf("%f,%f\r\n", command_steer*57.29577951, command_speed); // ステア指令値と速度指令値
    // Serial.println(steer_angle); // ステア角
    // Serial.printf("%d\r\n", angle_raw);
    // Serial.printf("%d\r\n", angle);
    // Serial.printf("%f,%f\n", theta_r_total, theta_l_total);
    // Serial.printf("%d:%d\n", enco_r, enco_l);
    // Serial.printf(">r:%f\n", theta_r_total);
    // Serial.printf(">l:%f\n", theta_l_total);
    // Serial.printf("x:%f,y:%f,yaw:%f\n", x_robo, y_robo, yaw_robo);
    // Serial.printf("x:%f,y:%f,yaw:%f\n", worldPos.x, worldPos.y, worldPos.yaw);
    // Serial.printf("x:%f,y:%f,yaw:%f\n", robotPos.x, robotPos.y, robotPos.yaw);
  }

  if (motor_r_duty_out > 255)
  {
    motor_r_duty_out = 255;
  }
  if (motor_r_duty_out < -255)
  {
    motor_r_duty_out = -255;
  }
  if (motor_l_duty_out > 255)
  {
    motor_l_duty_out = 255;
  }
  if (motor_l_duty_out < -255)
  {
    motor_l_duty_out = -255;
  }

  if (motor_r_duty_out >= 0.0)
  {
    ledcWrite(CH_1, abs((int)motor_r_duty_out));
    ledcWrite(CH_2, 0);
  }
  else
  {
    ledcWrite(CH_1, 0);
    ledcWrite(CH_2, abs((int)motor_r_duty_out));
  }
  if (motor_l_duty_out >= 0.0)
  {
    ledcWrite(CH_3, abs((int)motor_l_duty_out));
    ledcWrite(CH_4, 0);
  }
  else
  {
    ledcWrite(CH_3, 0);
    ledcWrite(CH_4, abs((int)motor_l_duty_out));
  }

  /*制御周期安定化*/
  control_count++;
  interval = micros() - preinterval;
  while (interval < CONTROL_PERIOD)
  {
    interval = micros() - preinterval;
  }
  preinterval = micros();
}