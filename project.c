#define MBED_CONF_MBED_TRACE_ENABLE 1
#define MBED_TRACE_MAX_LEVEL TRACE_LEVEL_INFO
#include "mbed.h"
#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP  "cs431"
#include "stm32f413h_discovery_lcd.h"

typedef void (*func_t)(void);

// motor control pins
PwmOut in1(p21);
PwmOut in2(p22);
PwmOut in3(p23);
PwmOut in4(p24);
// end motor control pins

// simulation variables, do not modify
Ticker lcd_ticker;
// end simulation variables

// ultrasonic sensor emulator class, do not modify
class Ultrasonic
{
  protected:
    AnalogIn* ain;
    Timeout* delayer;
    func_t echo_isr;
  public:
    static const int SPEED_OF_SOUND = 3350;
    Ultrasonic(PinName pin, func_t echo_cb)
    {
      this->ain = new AnalogIn(pin);
      this->delayer = new Timeout();
      this->echo_isr = echo_cb;
    }
    ~Ultrasonic()
    {
      delete ain;
      delete delayer;
    }
    void trigger()
    {
      if(this->echo_isr != NULL)
      {
        const float delay = (20.0+this->ain->read()*400.0)/SPEED_OF_SOUND;
        tr_debug("Ultrasonic: emulating %.0fus delay\n", delay*1e6);
        this->delayer->attach_us(this->echo_isr, delay*1e6);
      }
    }
};

// lcd drawing task that emulates the robot, do not modify
#define MOTOR_ACTIVATION_POWER 0.01
#define ROBOT_HEAD_TO_TAIL_PX  8
#define ROBOT_SPEED_MULTIPLIER 2
#define ROBOT_TURN_MULTIPLIER  0.1
#define MAP MOTOR_ACTIVATION_POWER
void lcd_draw_task()
{
  
  // simulation variables, do not modify
  static float tail_x=BSP_LCD_GetXSize()/2;
  static float tail_y=BSP_LCD_GetYSize()/2;
  static float head_x=BSP_LCD_GetXSize()/2;
  static float head_y=BSP_LCD_GetYSize()/2;
  static float dir = 0;
  static float mleft = 0.0f;
  static float mright = 0.0f;
  
  // clear LCD
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_FillCircle(round(tail_x), round(tail_y), 3);
  BSP_LCD_FillCircle(round(head_x), round(head_y), 3);
  
  // error checking
  if(in1 >= MAP && in2 >= MAP)
  {
    tr_err("Invalid power feed to motors: +in1 & +in2");
    return;
  }
  if(in3 >= MAP && in4 >= MAP)
  {
    tr_err("Invalid power feed to motors: +in3 & +in4");
    return;
  }
  
  // combining inputs into direction
  if(in1 >= MAP)
  {
    mleft = in1;
  }
  else if(in2 >= MAP)
  {
    mleft = -in2;
  }
  else
  {
    mleft = 0.0;
  }

  if(in3 >= MAP)
  {
    mright = in3;
  }
  else if(in4 >= MAP)
  {
    mright = -in4;
  }
  else
  {
    mright = 0.0;
  }
  
  const float finalpow = mleft + mright;
  const float powdiff = mleft - mright;
  // step
  tail_x += sin(dir)*finalpow*2;
  tail_y += cos(dir)*finalpow*2;     
  head_x = tail_x + sin(dir)*ROBOT_HEAD_TO_TAIL_PX;
  head_y = tail_y + cos(dir)*ROBOT_HEAD_TO_TAIL_PX;
  // turn
  dir += powdiff*ROBOT_TURN_MULTIPLIER;
  tr_debug("mright, mleft = [%.2f, %.2f]; finalpow, powdiff = [%.2f, %.2f]; dir = %.2f", mright, mleft, finalpow, powdiff, dir*180/3.14);

  // drawing
  BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_FillCircle(round(tail_x), round(tail_y), 3);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_FillCircle(round(head_x), round(head_y), 3);
}


// splitted wait function which allows for lcd drawing simulation and ultrasonic sensor emulation to work
// do not use bare wait_ms and instead use this function wherever you may need wait.
// Hint: you should only need this function in your main loop to avoid browser crash. 
// You should not need any busy waits like this!!
void splitted_wait_ms(int delay_ms)
{
  static Timer internalTimer;
  internalTimer.start();
  internalTimer.reset();
  while(internalTimer.read_ms()<delay_ms)
  {
    wait_ms(1);
  }
}

void robot_emulator_init()
{
  mbed_trace_init();     // initialize the trace library
  BSP_LCD_Init();
  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  lcd_ticker.attach(lcd_draw_task, 0.2);
  printf("Speed of sound is %d m/s due to JS engine limitations\n", Ultrasonic::SPEED_OF_SOUND);  
}

// USER GLOBAL CODE SPACE BEGIN
#define ULTRASONIC_SENSITIVITY 30.0

void ultrasonic_beep();
// your codes should go in here
bool serial_flag = false;
bool sensor_flag = false;
bool ultra_flag = false;
bool timer_running = false;

Serial pc(USBTX, USBRX); // tx, rx

InterruptIn bluetooth(p20);

InterruptIn noise_sensorF(p5);
InterruptIn noise_sensorB(p6);
InterruptIn noise_sensorL(p7);
InterruptIn noise_sensorR(p8);

char c[128];
int cindex = 0;
char sensor_direction;
Timer timer;
Ticker ticker;
Ultrasonic u(p19, ultrasonic_beep);

void serial_rx_isr()
{
    c[cindex] = pc.getc();
    cindex++;
    if(bluetooth.read() == 1)
    {
        serial_flag = true;
    }
}

void serial()
{
    serial_flag = false;
    switch (c[0]) 
    {
        case 'v':
            if(!ultra_flag)
            {
                in1 = 0.5;
                in2 = 0;
                in3 = 0.5;
                in4 = 0;
                splitted_wait_ms(1000);
                in1 = 0;
                in2 = 0;
                in3 = 0;
                in4 = 0;
            }
            break;
        case 'w':
            if(!ultra_flag)
            {
                in1 = 1;
                in2 = 0;
                in3 = 1;
                in4 = 0;
                splitted_wait_ms(1000);
                in1 = 0;
                in2 = 0;
                in3 = 0;
                in4 = 0;
            }
            break;
        case 's':
            in1 = 0;
            in2 = 0.5;
            in3 = 0;
            in4 = 0.5;
            splitted_wait_ms(1000);
            in1 = 0;
            in2 = 0;
            in3 = 0;
            in4 = 0;
            break;
        case 'a':
            if(!ultra_flag)
            {
                in1 = 1;
                in2 = 0;
                in3 = 0;
                in4 = 0;
                splitted_wait_ms(1000);
                in1 = 0;
                in2 = 0;
                in3 = 0;
                in4 = 0;
            }
            break;  
        case 'd':
            if(!ultra_flag)
            {
                in1 = 0;
                in2 = 0;
                in3 = 1;
                in4 = 0;
                splitted_wait_ms(1000);
                in1 = 0;
                in2 = 0;
                in3 = 0;
                in4 = 0;
            }
            break;
        default:
            break;
    }
    cindex = 0;
}


void bluetooth_isr()
{
    cindex = 0;
}


void sensor_forward_isr()
{
    if(bluetooth.read() == 0)
    {
        sensor_direction = 'F';
        sensor_flag = true;
    }
}

void sensor_backward_isr()
{
    if(bluetooth.read() == 0)
    {
        sensor_direction = 'B';
        sensor_flag = true;
    }
}

void sensor_left_isr()
{
    if(bluetooth.read() == 0)
    {
        sensor_direction = 'L';
        sensor_flag = true;
    }
}

void sensor_right_isr()
{
    if(bluetooth.read() == 0)
    {
        sensor_direction = 'R';
        sensor_flag = true;
    }
}

void noise_sensor_control()
{
    sensor_flag = false;
    
    if(sensor_direction == 'F' && !ultra_flag)
    {
        in1 = 1;
        in2 = 0;
        in3 = 1;
        in4 = 0;
        splitted_wait_ms(1000);
        in1 = 0;
        in2 = 0;
        in3 = 0;
        in4 = 0;
    }
    else if(sensor_direction == 'B')
    {
        in1 = 0;
        in2 = 1;
        in3 = 0;
        in4 = 1;
        splitted_wait_ms(1000);
        in1 = 0;
        in2 = 0;
        in3 = 0;
        in4 = 0;
    }
    else if(sensor_direction == 'L' && !ultra_flag)
    {
        in1 = 1;
        in2 = 0;
        in3 = 0;
        in4 = 0;
        splitted_wait_ms(3000);
        in1 = 0;
        in2 = 0;
        in3 = 0;
        in4 = 0;
    }
    else if(sensor_direction == 'R' && !ultra_flag)
    {
        in1 = 0;
        in2 = 0;
        in3 = 1;
        in4 = 0;
        splitted_wait_ms(3000);
        in1 = 0;
        in2 = 0;
        in3 = 0;
        in4 = 0;
    }
    else
    {
        pc.printf("Funny things happen\n");
    }
}

void ultrasonic_control()
{
    timer.start();
    u.trigger();
}

void ultrasonic_beep()
{
    timer.stop();
    //pc.printf("%f meters to the wall \n", timer.read()*(Ultrasonic::SPEED_OF_SOUND/2));
    if(timer.read()*(Ultrasonic::SPEED_OF_SOUND/2) <= ULTRASONIC_SENSITIVITY)
    {
        ultra_flag = true;
    }
    else
    {
        ultra_flag = false;
    }
    timer.reset();
}

// USER GLOBAL CODE SPACE END
int main() 
{
  // DO NOT REMOVE THIS CALL FROM MAIN!
  robot_emulator_init();
  // USER MAIN CODE SPACE BEGIN
  pc.attach(serial_rx_isr);
  /*
  the furthest distance from wall at 5V potentiometer is ~216.
  So the ticker will check every 0.2 seconds.
  216 * 2 / 3350 = ~0.13. But in order to not miss results periodic check can not be faster than 0.2 seconds.
  */
  ticker.attach(&ultrasonic_control, 0.2);
  
  bluetooth.rise(&bluetooth_isr);
  
  noise_sensorF.rise(&sensor_forward_isr);
  noise_sensorB.rise(&sensor_backward_isr);
  noise_sensorL.rise(&sensor_left_isr);
  noise_sensorR.rise(&sensor_right_isr);
  
  while (1) 
  {
    if(serial_flag)
    {
        serial();
    }
    if(sensor_flag)
    {
        noise_sensor_control();
    }
    
    splitted_wait_ms(10);
  }
  // USER MAIN CODE SPACE END
}