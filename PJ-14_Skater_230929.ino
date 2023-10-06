/*ライブラリの読み込み*/
#include <Ps3Controller.h> //PS3コントローラを使うためのライブラリ
#include "I2Cdev.h" //ジャイロをI2C接続するためのライブラリ
#include "MPU6050.h" //ジャイロを使うためのライブラリ
#include "esp32-hal-ledc.h" //ESP32でPWM波形をつくるためのライブラリ
#include <Time.h> //時間関数を使うためのライブラリ

MPU6050 accelgyro; //ジャイロで姿勢を取得するためのクラス

/*PWM波形をつくるための準備*/
const int L_F_pin = 25;//27
const int L_B_pin = 26;//14
const int R_F_pin = 27;//25
const int R_B_pin = 14;//26

const int CHANNEL_0 = 0;
const int CHANNEL_1 = 1;
const int CHANNEL_2 = 2;
const int CHANNEL_3 = 3;

const int LEDC_TIMER_BIT = 10; //PWM波形の分解能（２の１０乗＝１０２４）
const int LEDC_BASE_FREQ = 500; //PWM波形の周波数　つまり周期は : 2ms
const int VALUE_MAX = 255;

int OuterLoopCount = 10; //インナーループ何回に対しアウターループ１回実行するかを決める定数　小さすぎると送信機からの介入が頻繁になり機体が振動しやすく、大きすぎると送信機からの指令に対して反応が緩慢になる

float pid_limit = 150; //20211102 changed 120/ 20210922 default:100 //フィードバック値が過大になりすぎないようにするための制限
float pwm_limit = 400; //375 20211102 changed 300/ 20210922 default:255 //同上

float L_F_duty = 0;
float L_B_duty = 0;
float R_F_duty = 0;
float R_B_duty = 0;

float roll_trim = -4.00; //ロール（左右）方向の偏りを矯正
float pitch_trim = -1.00; //ピッチ（前後）方向の偏りを矯正
float yaw_trim = 5.00; //ヨー（自転）方向の偏りを矯正

int initialize_number = 200; //ジャイロ校正時のサンプリング数
float yaw_offset = -0.40; //ジャイロになぜか残る偏差の矯正用
float throttle_offset = 100; //スロットルの中立値調整用 50

/* IIR Lowpass filter (Jetter8 6/27)　および姿勢制御に必要なパラメータ*/
typedef struct
{
  float a1, a2, b0, b1, b2;
} IIR_Coeff;

//sensor filter
//7hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.922286512869545,  -0.92519529534950118, 0.00072719561998898304, 0.0014543912399779661, 0.00072719561998898304};

//15hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.8337326589246479,  -0.84653197479202391, 0.003199828966843966, 0.0063996579336879321, 0.003199828966843966};

//30hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.66920314293119312,  -0.71663387350415764, 0.011857682643241156, 0.023715365286482312, 0.011857682643241156};

//60hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.3489677452527946 ,  -0.51398189421967566, 0.041253537241720303, 0.082507074483440607, 0.041253537241720303};

//100hz, 800hz
IIR_Coeff gyro_fil_coeff = {0.94280904158206336,  -0.33333333333333343, 0.09763107293781749 , 0.19526214587563498 , 0.09763107293781749 };

float gyro_x_pre0_X, gyro_x_pre0_Y, gyro_x_pre0_Z, gyro_x_pre1_X, gyro_x_pre1_Y, gyro_x_pre1_Z;
float gyro_y_pre0_X, gyro_y_pre0_Y, gyro_y_pre0_Z, gyro_y_pre1_X, gyro_y_pre1_Y, gyro_y_pre1_Z;

int16_t ax, ay, az, gx, gy, gz;

float elapsedTime, time1, timePrev;

float Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Gyro_angle_x, Gyro_angle_y, Gyro_angle_z;
float Gyro_raw_error_x, Gyro_raw_error_y, Gyro_raw_error_z;

int acc_error = 0;
float rad_to_deg = 180 / 3.141592654;
float Acc_rawX, Acc_rawY, Acc_rawZ;
float Acc_angle_x, Acc_angle_y, Acc_angle_z;
float Acc_angle_error_x, Acc_angle_error_y, Acc_angle_error_z;

float Total_angle_x, Total_angle_y, Total_angle_z;

float angle_x[10], angle_y[10], angle_z[10]; //ジャイロ値のノイズ除去用に１０回移動平均するための数値格納用配列

int i; //Serial.printの頻度を決める定数

int min0 = 512; // (512/1024)*2ms = 1 ms (-90deg)
int max0 = 1024; // (1024/1024)*2ms = 2 ms (+90deg)

int input_YAW = 128;      //PS3コントローラからの入力値格納用
int input_PITCH = 128;    //同上
int input_ROLL = 128;     //同上
float input_THROTTLE = 0; //同上

//*for Thtottle_hold 20220912　スロットル値保持機能用
float hold_THROTTLE = 0;
float throttle_coef = 1;
float throttle_inclimental = 0;
//*/for Thtottle_hold 20220912

float comp = 0.6; //20230131 default=1.0 for Maneuverability UP Throttle Compensation　前後左右移動時に揚力の垂直成分が減る際のスロットル量補完用

int arming = 0; //バッテリーをつないだ途端にモータが回らないようにするために、Armを手動でおこなうためのフラグ
int POWER = 0;

int pid_change_gauge = 0; //PS3コントローラのボタンでゲイン等を調整できるようにするためのワンショットキーフラグ

int gain_shift = 1;

float PID_switch = 1.00; //230624ものサポ講座用

/*ここからPIDゲイン値定義*/
//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_rate_error, roll_rate_previous_error;
float roll_pid_p = 0;
float roll_pid_i = 0;
float roll_pid_d = 0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp = 15.0; //12.0
double roll_ki = 0.0;
double roll_kd = 0.00; //0.08
double roll_rate_kp = 0.025; //0.020
double roll_rate_ki = 0.0; //0.00035
double roll_rate_kd = 0.000015; //0.0000175
float roll_desired_angle = 0.0;
float roll_desired_angle_rate = 0.0;

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error, pitch_rate_error, pitch_rate_previous_error;
float pitch_pid_p = 0;
float pitch_pid_i = 0;
float pitch_pid_d = 0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp = 15.0; //12.0
double pitch_ki = 0.0;
double pitch_kd = 0.0; //0.15
double pitch_rate_kp = 0.025; //0.018
double pitch_rate_ki = 0.0; //0.00035; //default:0.0 0.0002だと影響少なすぎ（0とほぼ一緒）、0.0005だと影響大きすぎ（20210906）
double pitch_rate_kd = 0.000025; //0.000015
float pitch_desired_angle = 0.0;
float pitch_desired_angle_rate = 0.0;

//////////////////////////////PID FOR YAW//////////////////////////
float yaw_PID, yaw_error, yaw_previous_error, yaw_rate_error, yaw_rate_previous_error;
float yaw_pid_p = 0;
float yaw_pid_i = 0;
float yaw_pid_d = 0;
///////////////////////////////YAW PID CONSTANTS///////////////////
double yaw_kp = 0.7; //0.5
double yaw_ki = 0.0;
double yaw_kd = 0.0;
double yaw_rate_kp = 0.5; //0.5
double yaw_rate_ki = 0.0;
double yaw_rate_kd = 0.003;
float yaw_desired_angle = 0.0;
float yaw_desired_angle_rate = 0.0;

/*インナーループ用定数（１サンプリングタイム前のパラメータを覚えておくため*/
// rate deriv noise filter for Kd controll (7/5 Jetter8)
float D_filter_coef = 0.025;
float roll_rate_deriv_raw, roll_rate_deriv, roll_rate_deriv_pre;
float pitch_rate_deriv_raw, pitch_rate_deriv, pitch_rate_deriv_pre;
float yaw_rate_deriv_raw, yaw_rate_deriv, yaw_rate_deriv_pre;

/*PS3コントローラの接続*/
void onConnect(){
    Serial.println("Connected.");
}

void setup()
{
  /*PWM出力ピンの準備*/
  pinMode( L_F_pin, OUTPUT );
  pinMode( L_B_pin, OUTPUT );
  pinMode( R_F_pin, OUTPUT );
  pinMode( R_B_pin, OUTPUT );

  ledcSetup(CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(CHANNEL_1, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(CHANNEL_2, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(CHANNEL_3, LEDC_BASE_FREQ, LEDC_TIMER_BIT);

  ledcAttachPin(L_F_pin, CHANNEL_0);
  ledcAttachPin(L_B_pin, CHANNEL_1);
  ledcAttachPin(R_F_pin, CHANNEL_2);
  ledcAttachPin(R_B_pin, CHANNEL_3);

/*I２C接続開始、シリアル通信開始*/
  Wire.begin();
  Serial.begin(115200);

//PS3コントローラ接続（固有のBluetooth MAC Addressを使用）
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("E0:5A:1B:A0:48:4E"); //for 4300KV motor
  Serial.println("Ready.");

  accelgyro.initialize();
  time1 = millis();

  i = 1;

  initialize(); //ジャイロ初期化（校正）・・・今の姿勢を中立値として覚える

/*ESC起動用*/
  ledcWrite(CHANNEL_0, min0);
  ledcWrite(CHANNEL_1, min0);
  ledcWrite(CHANNEL_2, min0);
  ledcWrite(CHANNEL_3, min0);
  
  delay(2000);

  time1 = millis(); /*正確なサンプリングタイムをあとで計算するため、現在の時刻を覚えておく*/
}

void initialize() //ジャイロを校正するための関数定義
{
  //ここから校正（initialize_numberサンプルを使う）
  for (int g = 0; g < initialize_number; g++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Gyr_rawX = gx;
    Gyr_rawY = gy; //-gy for PJ-09F
    Gyr_rawZ = gz; //-gy for PJ-09F
    Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX / 32.8);
    Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY / 32.8);
    Gyro_raw_error_z = Gyro_raw_error_z + (Gyr_rawZ / 131.2); //20210827 131.2⇒32.8
    if (g == initialize_number - 1) {
      Gyro_raw_error_x = Gyro_raw_error_x / initialize_number;
      Gyro_raw_error_y = Gyro_raw_error_y / initialize_number;
      Gyro_raw_error_z = Gyro_raw_error_z / initialize_number;
    }
  }
  Total_angle_z = 0;
  yaw_desired_angle = 0 - yaw_trim; //20211102

  for (int a = 0; a < initialize_number; a++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Acc_rawX = ax;
    Acc_rawY = ay; //-ay for PJ-09F
    Acc_rawZ = az; //-az for PJ-09F
    Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));
    Acc_angle_error_y = Acc_angle_error_y + ((atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));
    if (a == initialize_number - 1) {
      Acc_angle_error_x = Acc_angle_error_x / initialize_number;
      Acc_angle_error_y = Acc_angle_error_y / initialize_number;
    }
  }

  gyro_x_pre0_X = 0.0;
  gyro_x_pre0_Y = 0.0;
  gyro_x_pre0_Z = 0.0;
  gyro_x_pre1_X = 0.0;
  gyro_x_pre1_Y = 0.0;
  gyro_x_pre1_Z = 0.0;
  roll_rate_deriv_pre = 0.0;
  pitch_rate_deriv_pre = 0.0;
  yaw_rate_deriv_pre = 0.0;

}

/*ここからメインループ*/
void loop()
{
//PS3コントローラからの値読み込み
    input_YAW = 128 - Ps3.data.analog.stick.lx; //yaw
    input_THROTTLE = 128 - Ps3.data.analog.stick.ly; //throttle
    input_ROLL = 128 - Ps3.data.analog.stick.rx; //roll
    input_PITCH = 128 - Ps3.data.analog.stick.ry; //pitch

    input_THROTTLE = 0.000042 * pow(input_THROTTLE, 3) - 0.0162 * pow(input_THROTTLE, 2) + 2.3859 * input_THROTTLE - 0.0001;  //20211018 for throttle curve
    
//スティック中央値付近を不感帯に（ヒステリシス対策）
    if(input_YAW > 123 and input_YAW < 133) //20211020 default:116-140, 20211013 default:123-133
    {
      input_YAW = 128;
    }

    if (Ps3.event.button_down.start) { //Arming
      arming = 1;
      initialize(); //IMU校正
    }
    if (Ps3.event.button_down.select) { //Disarming
      arming = 0;
      initialize(); //IMU校正
    }

    ///////////////////////////Gain Tuning////////////////////////////////20210928 Gain Controll from PS3 Controller
      if (Ps3.event.button_down.up and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.up){
        if(gain_shift == 1){
          roll_kd = roll_kd + 0.01;
        }
        if(gain_shift == 2){
          pitch_kd = pitch_kd + 0.01;
        }
        if(gain_shift == 3){
          yaw_kd = yaw_kd + 0.01;
        }
        pid_change_gauge = 0;
      }
      if (Ps3.event.button_down.down and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.down){
        if(gain_shift == 1){
          roll_kd = roll_kd - 0.01;
        }
        if(gain_shift == 2){
          pitch_kd = pitch_kd - 0.01;
        }
        if(gain_shift == 3){
          yaw_kd = yaw_kd - 0.01;
        }
        pid_change_gauge = 0;
      }

      if (Ps3.event.button_down.left and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.left){
        if(gain_shift == 1){
          roll_kp = roll_kp + 1.00;
        }
        if(gain_shift == 2){
          pitch_kp = pitch_kp + 1.00;
        }
        if(gain_shift == 3){
          yaw_kp = yaw_kp + 0.1;
        }
        pid_change_gauge = 0;
      }
      if (Ps3.event.button_down.right and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.right){
        if(gain_shift == 1){
          roll_kp = roll_kp - 1.00;
        }
        if(gain_shift == 2){
          pitch_kp = pitch_kp - 1.00;
        }
        if(gain_shift == 3){
          yaw_kp = yaw_kp - 0.1;
        }
        pid_change_gauge = 0;
      }

     if (Ps3.event.button_down.triangle and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.triangle){
        if(gain_shift == 1){
          roll_rate_kd = roll_rate_kd + 0.000001;
        }
        if(gain_shift == 2){
          pitch_rate_kd = pitch_rate_kd + 0.000001;
        }
        if(gain_shift == 3){
          yaw_rate_kd = yaw_rate_kd + 0.001;
        }
        pid_change_gauge = 0;
      }
      if (Ps3.event.button_down.cross and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.cross){
        if(gain_shift == 1){
          roll_rate_kd = roll_rate_kd - 0.000001;
        }
        if(gain_shift == 2){
          pitch_rate_kd = pitch_rate_kd - 0.000001;
        }
        if(gain_shift == 3){
          yaw_rate_kd = yaw_rate_kd - 0.001;
        }
        pid_change_gauge = 0;
      }

      if (Ps3.event.button_down.square and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.square){
        if(gain_shift == 1){
          roll_rate_kp = roll_rate_kp + 0.001;
        }
        if(gain_shift == 2){
          pitch_rate_kp = pitch_rate_kp + 0.001;
        }
        if(gain_shift == 3){
          yaw_rate_kp = yaw_rate_kp + 0.1;
        }
        pid_change_gauge = 0;
      }
      if (Ps3.event.button_down.circle and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.circle){
        if(gain_shift == 1){
          roll_rate_kp = roll_rate_kp - 0.001;
        }
        if(gain_shift == 2){
          pitch_rate_kp = pitch_rate_kp - 0.001;
        }
        if(gain_shift == 3){
          yaw_rate_kp = yaw_rate_kp - 0.1;
        }
        pid_change_gauge = 0;
      }

/*
      if (Ps3.event.button_down.l1 and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.l1){
        PID_switch = 1.00;
        //hold_THROTTLE = input_THROTTLE; //for ThrottleHold 20220912
        //throttle_coef = 0; //for ThrottleHold 20220912
        //throttle_inclimental = 0.001; //for ThrottleHold 20220912
        pid_change_gauge = 0;
      }
      if (Ps3.event.button_down.l2 and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.l2){
        PID_switch = 0.00;
        //hold_THROTTLE = 0; //for ThrottleHold 20220912
        //throttle_coef = 1; //for ThrottleHold 20220912
        //throttle_inclimental = 0; //for ThrottleHold 20220912
        pid_change_gauge = 0;
      }
*/
      
/*
      if (Ps3.event.button_down.l1 and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.l1){
        hold_THROTTLE = input_THROTTLE; //for ThrottleHold 20220912
        throttle_coef = 0; //for ThrottleHold 20220912
        throttle_inclimental = 0.001; //for ThrottleHold 20220912
        pid_change_gauge = 0;
      }
      if (Ps3.event.button_down.l2 and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.l2){
        hold_THROTTLE = 0; //for ThrottleHold 20220912
        throttle_coef = 1; //for ThrottleHold 20220912
        throttle_inclimental = 0; //for ThrottleHold 20220912
        pid_change_gauge = 0;
      }
*/


      if (Ps3.event.button_down.l1 and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.l1){
        gain_shift = gain_shift - 1;
        if(gain_shift < 1){
          gain_shift = 1;
        }
        pid_change_gauge = 0;
      }
      if (Ps3.event.button_down.l2 and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.l2){
        gain_shift = gain_shift + 1;
        if(gain_shift > 3){
          gain_shift = 3;
        }
        pid_change_gauge = 0;
      }


      if (Ps3.event.button_down.r1 and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.r1){
        throttle_offset = throttle_offset + 10;
        pid_change_gauge = 0;
      }
      if (Ps3.event.button_down.r2 and pid_change_gauge == 0){
        pid_change_gauge = 1;
      }
      if (pid_change_gauge == 1 and Ps3.event.button_up.r2){
        throttle_offset = throttle_offset - 10;
        if(throttle_offset < 0){
          throttle_offset = 0;
        }
        pid_change_gauge = 0;
      }   

    /////////////////////////////I M U/////////////////////////////////////
    timePrev = time1;  // the previous time is stored before the actual time read
    time1 = millis();  // actual time read
    elapsedTime = (time1 - timePrev) / 1000; //sampling time length

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //IMUデータ更新

    // IIR Filtering to gyro signals (Jetter8 6/27)　無限インパルス応答フィルタ演算
    Gyr_rawX = gyro_fil_coeff.b0 * gx
               + gyro_fil_coeff.b1 * gyro_x_pre0_X + gyro_fil_coeff.b2 * gyro_x_pre1_X
               + gyro_fil_coeff.a1 * gyro_y_pre0_X + gyro_fil_coeff.a2 * gyro_y_pre1_X;
    Gyr_rawY = gyro_fil_coeff.b0 * gy //-gy for PJ-09F
               + gyro_fil_coeff.b1 * gyro_x_pre0_Y + gyro_fil_coeff.b2 * gyro_x_pre1_Y
               + gyro_fil_coeff.a1 * gyro_y_pre0_Y + gyro_fil_coeff.a2 * gyro_y_pre1_Y;
    Gyr_rawZ = gyro_fil_coeff.b0 * gz //-gz for PJ-09F
               + gyro_fil_coeff.b1 * gyro_x_pre0_Z + gyro_fil_coeff.b2 * gyro_x_pre1_Z
               + gyro_fil_coeff.a1 * gyro_y_pre0_Z + gyro_fil_coeff.a2 * gyro_y_pre1_Z;

    // Shift IIR filter state
    gyro_x_pre1_X = gyro_x_pre0_X;
    gyro_x_pre1_Y = gyro_x_pre0_Y;
    gyro_x_pre1_Z = gyro_x_pre0_Z;
    gyro_y_pre1_X = gyro_y_pre0_X;
    gyro_y_pre1_Y = gyro_y_pre0_Y;
    gyro_y_pre1_Z = gyro_y_pre0_Z;

    gyro_x_pre0_X = gx;
    gyro_x_pre0_Y = gy; //-gy for PJ-09F
    gyro_x_pre0_Z = gz; //-gz for PJ-09F
    gyro_y_pre0_X = Gyr_rawX;
    gyro_y_pre0_Y = Gyr_rawY;
    gyro_y_pre0_Z = Gyr_rawZ;

    Acc_rawX = ax;
    Acc_rawY = ay; //-ay for PJ-09F
    Acc_rawZ = az; //-az for PJ-09F

    Gyr_rawX = (Gyr_rawX / 32.8) - Gyro_raw_error_x;
    Gyr_rawY = (Gyr_rawY / 32.8) - Gyro_raw_error_y;
    Gyr_rawZ = (Gyr_rawZ / 131.2) - Gyro_raw_error_z; //20210827 131.2⇒32.8

    Gyro_angle_x = Gyr_rawX * elapsedTime;
    Gyro_angle_y = Gyr_rawY * elapsedTime;
    Gyro_angle_z = Gyr_rawZ * elapsedTime;

    Acc_angle_x = (atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_x;
    Acc_angle_y = (atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_y;

    Total_angle_x = 0.98 * (Total_angle_x + Gyro_angle_x) + 0.02 * Acc_angle_x;
    Total_angle_y = 0.98 * (Total_angle_y + Gyro_angle_y) + 0.02 * Acc_angle_y;// - 0.105; //angle offset calibration
    Total_angle_z = Total_angle_z + Gyro_angle_z;

//Mean Filter 20230201
    for(int i=1; i<9; i++){
      angle_x[i] = angle_x[i-1];
    }
    angle_x[0] = Total_angle_x;
    Total_angle_x = 0;
    for(int i=0; i<9; i++){
      Total_angle_x = Total_angle_x + angle_x[0];
    }
    Total_angle_x = Total_angle_x / 10;

    for(int i=1; i<9; i++){
      angle_y[i] = angle_y[i-1];
    }
    angle_y[0] = Total_angle_y;
    Total_angle_y = 0;
    for(int i=0; i<9; i++){
      Total_angle_y = Total_angle_y + angle_y[0];
    }
    Total_angle_y = Total_angle_y / 10;

    for(int i=1; i<9; i++){
      angle_z[i] = angle_z[i-1];
    }
    angle_z[0] = Total_angle_z;
    Total_angle_z = 0;
    for(int i=0; i<9; i++){
      Total_angle_z = Total_angle_z + angle_z[0];
    }
    Total_angle_z = Total_angle_z / 10;
//Mean Filter 20230201

    /*///////////////////////////P I D///////////////////////////////////*/

    // Outer Loop (6/20 Jetter8)
    if (i % OuterLoopCount == 1) {//OuterLoopCountに1回、外部指令を角度制御に反映する

      //roll_desired_angle = - (input_ROLL - 128) * 0.20 - roll_trim; //20211021 for high resolution control
      roll_desired_angle = -0.00000971 * pow(input_ROLL, 3) + 0.00371 * pow(input_ROLL, 2) - 0.550 * input_ROLL + 30.008 - roll_trim;
      //pitch_desired_angle = - (input_PITCH - 128) * 0.20 - pitch_trim; //20211021 for high resolution control
      pitch_desired_angle = -0.00000971 * pow(input_PITCH, 3) + 0.00371 * pow(input_PITCH, 2) - 0.550 * input_PITCH + 30.008 - pitch_trim;
      
      if(input_YAW <= 123){
        yaw_desired_angle = map(input_YAW, 0, 123, -179, 0); //20211019 changed default:255
      }
      if(input_YAW >= 133){
        yaw_desired_angle = map(input_YAW, 133, 255, 0, 179); //20211019 changed default:255
      }
      if(input_YAW == 128){
        yaw_desired_angle = 0;
      }

      roll_error = roll_desired_angle - Total_angle_y;
      pitch_error = pitch_desired_angle - Total_angle_x;
      yaw_error = yaw_desired_angle - Total_angle_z;

      roll_pid_p = roll_kp * roll_error;
      pitch_pid_p = pitch_kp * pitch_error;
      yaw_pid_p = yaw_kp * yaw_error;

      if (-3 < roll_error < 3) {
        roll_pid_i = roll_pid_i + (roll_ki * roll_error);
      }
      if (-3 < pitch_error < 3) {
        pitch_pid_i = pitch_pid_i + (pitch_ki * pitch_error);
      }

      if (elapsedTime > 0.0015) { // countermeasure for elapsedTime probrem (Jetter8 6/27)
        roll_pid_d = roll_kd * ((roll_error - roll_previous_error) / elapsedTime);
        pitch_pid_d = pitch_kd * ((pitch_error - pitch_previous_error) / elapsedTime);
        yaw_pid_d = yaw_kd * ((yaw_error - yaw_previous_error) / elapsedTime);
      }

      roll_desired_angle_rate = roll_pid_p + roll_pid_i + roll_pid_d;
      pitch_desired_angle_rate = pitch_pid_p + pitch_pid_i + pitch_pid_d;
      yaw_desired_angle_rate = yaw_pid_p + yaw_pid_i + yaw_pid_d;
    }

    // Inner Loop (6/20 Jetter8)　角速度制御
    roll_rate_error = Gyr_rawY - roll_desired_angle_rate;
    pitch_rate_error = Gyr_rawX - pitch_desired_angle_rate;
    yaw_rate_error = Gyr_rawZ - yaw_desired_angle_rate;

    roll_pid_p = roll_rate_kp * roll_rate_error;
    pitch_pid_p = pitch_rate_kp * pitch_rate_error;
    yaw_pid_p = yaw_rate_kp * yaw_rate_error;

    if (-3 < roll_rate_error < 3)
    {
      roll_pid_i = roll_pid_i + (roll_rate_ki * roll_rate_error);
    }
    if (-3 < pitch_rate_error < 3)
    {
      pitch_pid_i = pitch_pid_i + (pitch_rate_ki * pitch_rate_error);
    }

    if (elapsedTime > 0.0015) { // countermeasure for elapsedTime probrem (Jetter8 6/27)
      roll_rate_deriv_raw = (roll_rate_error - roll_rate_previous_error) / elapsedTime;
      roll_rate_deriv = roll_rate_deriv_pre + (roll_rate_deriv_raw - roll_rate_deriv_pre) * D_filter_coef;
      roll_pid_d = roll_rate_kd * roll_rate_deriv;
      roll_rate_deriv_pre = roll_rate_deriv;
      pitch_rate_deriv_raw = (pitch_rate_error - pitch_rate_previous_error) / elapsedTime;
      pitch_rate_deriv = pitch_rate_deriv_pre + (pitch_rate_deriv_raw - pitch_rate_deriv_pre) * D_filter_coef;
      pitch_pid_d = pitch_rate_kd * pitch_rate_deriv;
      pitch_rate_deriv_pre = pitch_rate_deriv;
      yaw_rate_deriv_raw = (yaw_rate_error - yaw_rate_previous_error) / elapsedTime;
      yaw_rate_deriv = yaw_rate_deriv_pre + (yaw_rate_deriv_raw - yaw_rate_deriv_pre) * D_filter_coef;
      yaw_pid_d = yaw_rate_kd * yaw_rate_deriv;
      yaw_rate_deriv_pre = yaw_rate_deriv;
    }

//for Integer Gain Tuning, locate the Integer Limit (20210906)
    if(input_THROTTLE <= 135){
      roll_pid_i = 0;
      pitch_pid_i = 0;
    }
//for Integer Gain Tuning, locate the Integer Limit (20210906)

    roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
    pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
    yaw_PID = yaw_pid_p + yaw_pid_i + yaw_pid_d;

//PID値が過大にならないための制限
    if (roll_PID < -pid_limit) { //20210922 default:255
      roll_PID = -pid_limit; //default:-100
    }
    if (roll_PID > pid_limit) { //20210922 default:255
      roll_PID = pid_limit;  //default:100
    }
    if (pitch_PID < -pid_limit) { //20210922 default:255
      pitch_PID = -pid_limit; //default:-100
    }
    if (pitch_PID > pid_limit) { //20210922 default:255
      pitch_PID = pid_limit; //default:100
    }
    if (yaw_PID < -pid_limit) { //20210922 default:255
      yaw_PID = -pid_limit; //default:-100
    }
    if (yaw_PID > pid_limit) { //20210922 default:255
      yaw_PID = pid_limit; //default:100
    }

/*ここで４個のブラシレスモータに出力するPWM値を決める*/
//For Hovering at Throttole Neutral Position 20210919
    hold_THROTTLE = hold_THROTTLE + throttle_inclimental * (input_THROTTLE - 128);
    pwm_R_F  = 1 + ( input_THROTTLE * throttle_coef + hold_THROTTLE + throttle_offset ) / ( cos(3.1415 / 180 * roll_desired_angle * comp) * cos(3.1415 / 180 * pitch_desired_angle * comp) ) + (roll_PID - pitch_PID + yaw_PID) * 2 * PID_switch; //3だと振動発生 この場合、きちんと合わせるなら2.4のほうが良い？（あとのDuty演算で1.25倍しているので、3/1.25=2.4だから）//for ThrottleHold 20220912
    pwm_R_B  = 1 + ( input_THROTTLE * throttle_coef + hold_THROTTLE + throttle_offset ) / ( cos(3.1415 / 180 * roll_desired_angle * comp) * cos(3.1415 / 180 * pitch_desired_angle * comp) ) + (roll_PID + pitch_PID - yaw_PID) * 2 * PID_switch; //3）//for ThrottleHold 20220912
    pwm_L_B  = 1 + ( input_THROTTLE * throttle_coef + hold_THROTTLE + throttle_offset ) / ( cos(3.1415 / 180 * roll_desired_angle * comp) * cos(3.1415 / 180 * pitch_desired_angle * comp) ) - (roll_PID - pitch_PID - yaw_PID) * 2 * PID_switch; //3）//for ThrottleHold 20220912
    pwm_L_F  = 1 + ( input_THROTTLE * throttle_coef + hold_THROTTLE + throttle_offset ) / ( cos(3.1415 / 180 * roll_desired_angle * comp) * cos(3.1415 / 180 * pitch_desired_angle * comp) ) - (roll_PID + pitch_PID + yaw_PID) * 2 * PID_switch; //3）//for ThrottleHold 20220912
//For Hovering at Throttole Neutral Position 20210919

    //Right front
    if (pwm_R_F < 0)
    {
      pwm_R_F = 0;
    }
    if (pwm_R_F > pwm_limit) //20210922 default = 255
    {
      pwm_R_F = pwm_limit; //20210922 default = 255
    }

    //Left front
    if (pwm_L_F < 0)
    {
      pwm_L_F = 0;
    }
    if (pwm_L_F > pwm_limit) //20210922 default = 255
    {
      pwm_L_F = pwm_limit; //20210922 default = 255
    }

    //Right back
    if (pwm_R_B < 0)
    {
      pwm_R_B = 0;
    }
    if (pwm_R_B > pwm_limit) //20210922 default = 255
    {
      pwm_R_B = pwm_limit; //20210922 default = 255
    }

    //Left back
    if (pwm_L_B < 0)
    {
      pwm_L_B = 0;
    }
    if (pwm_L_B > pwm_limit) //20210922 default = 255
    {
      pwm_L_B = pwm_limit; //20210922 default = 255
    }

    roll_previous_error = roll_error; //Remember to store the previous error.
    pitch_previous_error = pitch_error; //Remember to store the previous error.
    yaw_previous_error = yaw_error;

/*PWM値をDUTYに変換*/
//For Hovering at Throttole Neutral Position 20210919
    L_F_duty = 512 + pwm_L_F * 1.25;
    L_B_duty = 512 + pwm_L_B * 1.25;
    R_F_duty = 512 + pwm_R_F * 1.25;
    R_B_duty = 512 + pwm_R_B * 1.25;
//For Hovering at Throttole Neutral Position 20210919

    if (L_F_duty <= 512) { //Dutyが512未満にならないように
      L_F_duty = 512;
    }
    if (L_B_duty <= 512) {
      L_B_duty = 512;
    }
    if (R_F_duty <= 512) {
      R_F_duty = 512;
    }
    if (R_B_duty <= 512) {
      R_B_duty = 512;
    }

    //ここから、Armingしないとモータが回らないようにするためのプログラム（12/24追加）
    if (arming == 0) {
      L_F_duty = 512;
      L_B_duty = 512;
      R_F_duty = 512;
      R_B_duty = 512;
    }
    //ここまで、Armingしないとモータが回らないようにするためのプログラム（12/24追加）

    ledcWrite( CHANNEL_0, L_F_duty ); //モータの回転数を決定
    ledcWrite( CHANNEL_1, L_B_duty );
    ledcWrite( CHANNEL_2, R_F_duty );
    ledcWrite( CHANNEL_3, R_B_duty );

    i = i + 1;
    
    if (i % 5 == 1) { //25サイクルに1回 シリアルモニタに出力

      String str = String(Total_angle_x) + 'q' +String(Total_angle_y) + 'w' +String(Total_angle_z) + 'e' +String(pitch_desired_angle) + 'r' +String(roll_desired_angle) + 't' +String(yaw_desired_angle) + 'y' +String(Gyr_rawX) + 'u' +String(Gyr_rawY) + 'i' +String(Gyr_rawZ) + 'o' +String(pitch_desired_angle_rate) + 'p' +String(roll_desired_angle_rate)+ '@' +String(yaw_desired_angle_rate) + '\0';
//      Serial.println(str);
      
      Serial.print(Total_angle_x);   Serial.print("\t");
      Serial.print(Total_angle_y);   Serial.print("\t");
      Serial.print(Total_angle_z);   Serial.print("\t");

      Serial.print(L_F_duty);   Serial.print("\t");
      Serial.print(L_B_duty);   Serial.print("\t");
      Serial.print(R_F_duty);   Serial.print("\t");
      Serial.print(R_B_duty);   Serial.print("\t");
      
      Serial.print("\n");
    }
}
