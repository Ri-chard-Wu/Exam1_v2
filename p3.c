/*
#include "mbed.h"

using namespace std::chrono;


Ticker flipper;

DigitalOut led1(LED1);

DigitalOut led2(LED2);


void flip()

{

   led2 = !led2;

}


int main()

{

   led2 = 1;
   flipper.attach(&flip, 2s); // the address of the function to be attached (flip) and the interval (2 seconds)

   // spin in a main loop. flipper will interrupt it to call flip

   while (1)

   {

      led1 = !led1;

      ThisThread::sleep_for(200ms);

   }

}*/



// exam p3

#include "mbed.h"

#include "stm32l475e_iot01_accelero.h"


InterruptIn btnRecord(BUTTON1);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;


int16_t pDataXYZ[3] = {0};
int16_t X[10] = {0};
int16_t Y[10] = {0};
int16_t Z[10] = {0};
int i_s = 0;
int idR[32] = {0};
int indexR = 0;

void compute_avg(){
    int x = 0,y=0,z=0;
    for(int i =0;i<10;i++){
        x+=X[i];
        y+=Y[i];
        z+=Z[i];
    }
    x  = x/10;
    y  = y/10;
    z  = z/10;
    printf("%d, %d, %d\n", x, y, z);
    
}

void record(void) {
   BSP_ACCELERO_AccGetXYZ(pDataXYZ);
   X[i_s++] = pDataXYZ[0];
   Y[i_s++] = pDataXYZ[1];
   Z[i_s++] = pDataXYZ[2];
   if(i_s == 10){
       i_s = 0;
       queue.call(&compute_avg);
   }
   //printf("%d, %d, %d\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);

}


void startRecord(void) {
   printf("---start---\n");
   idR[indexR++] = queue.call_every(1ms, record);
   indexR = indexR % 32;

}


void stopRecord(void) {
   printf("---stop---\n");
   for (auto &i : idR)
      queue.cancel(i);

}


int main() {

   printf("Start accelerometer init\n");
   BSP_ACCELERO_Init();
   t.start(callback(&queue, &EventQueue::dispatch_forever));
   btnRecord.fall(queue.event(startRecord));
   btnRecord.rise(queue.event(stopRecord));

}





// exam p2
/*
#include "EventQueue.h"
#include "ThisThread.h"
#include "mbed.h"
#include "uLCD_4DGL.h"

using namespace std::chrono;
const double amplitude = 1.0f;
const float V_ref = 3.;
const double scaling = 65535. / V_ref;
bool GenWave = false;

AnalogIn Ain(A0);     // Sampling
AnalogOut aout(PA_4); // Signal output

DigitalOut led2(LED2);
Thread t1, t2, t3;

Timer debounce;
EventQueue queue(32 * EVENTS_EVENT_SIZE);

int i = 0 ;
float sample;
int frequency = 10; 
int N = 100; 
float dt = 1000000. / (2* frequency * N);




void f(int n);
void Wave_Gen();



int main() {
  printf("begin\n");
  t1.start(&Wave_Gen);
  t3.start(callback(&queue, &EventQueue::dispatch_forever));
  while (true) {
    ThisThread::sleep_for(500ms);
  }
}


void Wave_Gen() {

  while (1) {

      f(i++);
      wait_us((int)dt);
      

  }
}

float t = 0;
void f(int n){ // N at least > 300.
    
    n = n%N;
    if(n == 0){
        queue.call(printf, "One period completed\r\n");
    }
    t = (n/100.)*6.0;
    if(t < 3.0) sample = (uint16_t) (scaling * 3.0 * tanh(t));
    else if(t < 6.0) sample = (uint16_t)(scaling * (3.0 * (1 - tanh(3 - t)))); 
    aout.write_u16(sample);
   

}*/


//exam p1
/*
//#include "EventQueue.h"
//#include "ThisThread.h"
#include "mbed.h"
#include "uLCD_4DGL.h"
using namespace std::chrono;


InterruptIn btn(BUTTON1);
Ticker flipper;
int c = 0;
int state = 1;
bool en_counting = false;

int minute = 0;
int sec = 0;
int one_10th_sec = 0;
uLCD_4DGL uLCD(D1, D0, D2);



void flip(){
    if(en_counting){
        c++;
    }
     
}

void change_state(){
    printf("current state = %d", state);
    if(state == 1){
        en_counting = true;
        flipper.attach(&flip, 100ms);
 
        state = 2;
    }else if(state == 2){
        en_counting = false;
        state = 3;
    }else if(state == 3){
        c = 0;
        state = 1;

    }
}


void print_to_uLCD() 
{ 
        //c measured in 0.1s
        //1min = 600 0.1s
        //1s = 10 0.1s
        minute = c%600;
        sec = (c - minute * 600) % 10;
        one_10th_sec = c - minute*600 - sec*10;
        uLCD.cls();
        uLCD.printf("\n%d:%d:%d\n", minute, sec, one_10th_sec); 
}



int main() {
  printf("begin\n");
  btn.rise(&change_state);
  while (true) {
      
      ThisThread::sleep_for(200ms);
      
  }
}
*/





// exam p2
/*
#include "EventQueue.h"
#include "ThisThread.h"
#include "mbed.h"
#include "uLCD_4DGL.h"

using namespace std::chrono;
const double amplitude = 1.0f;
const float V_ref = 3.;
const double scaling = 65535. / V_ref;
bool GenWave = false;

AnalogIn Ain(A0);     // Sampling
AnalogOut aout(PA_4); // Signal output

InterruptIn btn_A(PB_2), btn_B(PA_15);
DigitalOut led2(LED2);
Thread t1, t2, t3;

uLCD_4DGL uLCD(D1, D0, D2);
Timer debounce;
EventQueue queue(32 * EVENTS_EVENT_SIZE);

int i_Gen = 0, i_Sam = 0, i_transfer = 0;

// f()
int frequency = 10; //int frequency = 100;
int N = 100; //int N = 30;
float dt = 1000000. / (2* frequency * N);
//uint32_t wait_time = 1000/(frequency * N); //float dt = 1000000. / (2* frequency * N);
float r0 = 0.1; // 10%
float r1 = 0.7; // 70%
float slope0 = V_ref / (N * r0);
float slope1 = V_ref / (N * (r1 - 1)); // smaller than 0.
const int capacity = 256;
float ADCdata[capacity];
int sample = 0;



void Update_uLCD_Wave_Generating();
void Update_uLCD_Transferring();
void Update_uLCD_btn_B_ISR();
void Update_uLCD_btn_A_ISR();
void Wave_Sam();
void f(int n);
void Wave_Gen();
void send();
void btn_A_ISR();
void btn_B_ISR();

int main() {
  debounce.start();
  btn_A.rise(&btn_A_ISR);
  btn_B.rise(&btn_B_ISR);

  t1.start(&Wave_Gen);
  t2.start(&Wave_Sam);
  t3.start(callback(&queue, &EventQueue::dispatch_forever));
  while (true) {
    ThisThread::sleep_for(500ms);
  }
}


void Wave_Gen() {

  while (1) {
    if (GenWave) {
      f(i_Gen++);
      wait_us((int)dt);
      //ThisThread::sleep_for(1000ms/(frequency* N));
    } else {
      i_Gen = 0;
    }
  }
}


void f(int n){ // N at least > 300.
    
    n = n%N;

    if(n < r0*N) sample = (uint16_t) (scaling * slope0 * n);
    else if(n > r1*N) sample = (uint16_t)(scaling * (V_ref + slope1 * (n - r1*N))); 
    aout.write_u16(sample);
   
 

}




*/







// HW 2
/*
#include "EventQueue.h"
#include "ThisThread.h"
#include "mbed.h"
#include "uLCD_4DGL.h"

using namespace std::chrono;
const double amplitude = 1.0f;
const float V_ref = 3.;
const double scaling = 65535. / V_ref;
bool GenWave = false;

AnalogIn Ain(A0);     // Sampling
AnalogOut aout(PA_4); // Signal output

InterruptIn btn_A(PB_2), btn_B(PA_15);
DigitalOut led2(LED2);
Thread t1, t2, t3;

uLCD_4DGL uLCD(D1, D0, D2);
Timer debounce;
EventQueue queue(32 * EVENTS_EVENT_SIZE);

int i_Gen = 0, i_Sam = 0, i_transfer = 0;

// f()
int frequency = 10; //int frequency = 100;
int N = 100; //int N = 30;
float dt = 1000000. / (2* frequency * N);
//uint32_t wait_time = 1000/(frequency * N); //float dt = 1000000. / (2* frequency * N);
float r0 = 0.1; // 10%
float r1 = 0.7; // 70%
float slope0 = V_ref / (N * r0);
float slope1 = V_ref / (N * (r1 - 1)); // smaller than 0.
const int capacity = 256;
float ADCdata[capacity];
int sample = 0;



void Update_uLCD_Wave_Generating();
void Update_uLCD_Transferring();
void Update_uLCD_btn_B_ISR();
void Update_uLCD_btn_A_ISR();
void Wave_Sam();
void f(int n);
void Wave_Gen();
void send();
void btn_A_ISR();
void btn_B_ISR();

int main() {
  debounce.start();
  btn_A.rise(&btn_A_ISR);
  btn_B.rise(&btn_B_ISR);

  t1.start(&Wave_Gen);
  t2.start(&Wave_Sam);
  t3.start(callback(&queue, &EventQueue::dispatch_forever));
  while (true) {
    ThisThread::sleep_for(500ms);
  }
}


void Wave_Gen() {

  while (1) {
    if (GenWave) {
      f(i_Gen++);
      wait_us((int)dt);
      //ThisThread::sleep_for(1000ms/(frequency* N));
    } else {
      i_Gen = 0;
    }
  }
}


void f(int n){ // N at least > 300.
    
    n = n%N;

    if(n < r0*N) sample = (uint16_t) (scaling * slope0 * n);
    else if(n > r1*N) sample = (uint16_t)(scaling * (V_ref + slope1 * (n - r1*N))); 
    aout.write_u16(sample);
   
 

}

void Wave_Sam() {
  while (1) {
    if (GenWave) {
      if (i_Sam < capacity) {
        ADCdata[i_Sam++] = Ain;
        ThisThread::sleep_for(1ms);
      }

    } else {
      i_Sam = 0;
    }
  }
}



void btn_A_ISR() {
  if (duration_cast<milliseconds>(debounce.elapsed_time()).count() > 1000) {
    queue.call(&Update_uLCD_btn_A_ISR);
    GenWave = true;
    queue.call(&Update_uLCD_Wave_Generating);

    debounce.reset();
  }
}

void btn_B_ISR() {
  if (duration_cast<milliseconds>(debounce.elapsed_time()).count() > 1000) {
    queue.call(&Update_uLCD_btn_B_ISR);
    GenWave = false;
    i_transfer = i_Sam;

    queue.call(&Update_uLCD_Transferring);
    queue.call(&send);

    debounce.reset();
  }
}

void send() {
  for (int i = 0; i < i_transfer; i++) {
    printf("%f\r\n", ADCdata[i]);
    ThisThread::sleep_for(100ms);
  }
}


void Update_uLCD_btn_A_ISR() { uLCD.printf("\nInterrupt A Detected\n"); }

void Update_uLCD_btn_B_ISR() { uLCD.printf("\nInterrupt B Detected\n"); }

void Update_uLCD_Transferring() { uLCD.printf("\nTransferring...\n"); }

void Update_uLCD_Wave_Generating() { uLCD.printf("\nGenerating Wave...\n"); }





*/





























/*
#include "mbed.h"


const double pi = 3.141592653589793238462;

const double amplitude = 0.5f;

const double offset = 65535. / 2;


// The sinewave is created on this pin

// Adjust analog output pin name to your board spec.

AnalogOut aout(PA_4);
const float V_ref = 3.;
const double scaling = 65535. / V_ref;


int frequency = 10;
int N = 100;
float dt = 1000000. / (frequency * N);
float r0 = 0.1; // 10%
float r1 = 0.7; // 70%
float slope0 = V_ref / (N * r0);
float slope1 = V_ref / (N * (r1 - 1)); // smaller than 0.
uint16_t sample=0;
int n = 0;


void f(int n){ // N at least > 300.
    
    n = n%N;

    if(n < r0*N) sample = (uint16_t) (scaling * slope0 * n);
    else if(n > r1*N) sample = (uint16_t)(scaling * (V_ref + slope1 * (n - r1*N))); 
    aout.write_u16(sample);
    wait_us((int)dt);
 

}


int main()

{


   while (1) {


    //f(10, 100);
    f(n++);

 

   }

}



*/

































//mbed 8 ex.1 data collection
/*
#include "mbed.h"
#include "stm32l475e_iot01_accelero.h"


InterruptIn btnRecord(BUTTON1);
EventQueue queue(32 * EVENTS_EVENT_SIZE);

Thread t;

int16_t pDataXYZ[3] = {0};
int idR[32] = {0};
int indexR = 0;

void record(void) {
   BSP_ACCELERO_AccGetXYZ(pDataXYZ);
   printf("%d, %d, %d\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
}


void startRecord(void) {
   printf("---start---\n");
   idR[indexR++] = queue.call_every(1ms, record);
   indexR = indexR % 32;
}


void stopRecord(void) {
   printf("---stop---\n");
   for (auto &i : idR)
      queue.cancel(i);
}

int main() {
   printf("Start accelerometer init\n");
   BSP_ACCELERO_Init();
   t.start(callback(&queue, &EventQueue::dispatch_forever));
   btnRecord.fall(queue.event(startRecord));
   btnRecord.rise(queue.event(stopRecord));
}
*/



















//mbed 8 ex.2 model deployment


/*
#include "accelerometer_handler.h"
#include "config_tflite.h"
#include "magic_wand_model_data.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

// Return the result of the last prediction
int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config_tflite.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

int main(int argc, char* argv[]) {

  // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {

    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return -1;
  }


  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config_tflite.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return -1;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return -1;
  }


  error_reporter->Report("Set up successful...\n");

  while (true) {

    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    // Produce an output
    if (gesture_index < label_num) {
      error_reporter->Report(config_tflite.output_message[gesture_index]);
    }
  }
}
*/























/*
#include "mbed.h"

I2C m_i2c(D14, D15);
char m_addr = 0x90;
int main()
{
   while (1)
   {
      const char tempRegAddr = 0x00;

      m_i2c.write(m_addr, &tempRegAddr, 1);
      //Set pointer to the temperature register

      char reg[2] = {0, 0};
      m_i2c.read(m_addr, reg, 2); //Read

      unsigned short res = (reg[0] << 4) | (reg[1] >> 4);
      float temp =  (float) ((float)res * 0.0625);
      printf("Temp code=(%d, %d)\r\n", reg[0], reg[1]);
      printf("Temp = %f.\r\n", temp);
      ThisThread::sleep_for(1s);
   }
}
*/

/*#include "mbed.h"

#include "TextLCD.h"


// Host PC Communication channels

static BufferedSerial pc(USBTX, USBRX); // tx, rx


// I2C Communication

I2C i2c_lcd(D14, D15); // SDA, SCL


//TextLCD_SPI lcd(&spi_lcd, p8, TextLCD::LCD40x4);   // SPI bus, 74595 expander,
CS pin, LCD Type

TextLCD_I2C lcd(&i2c_lcd, 0x4E, TextLCD::LCD16x2);   // I2C bus, PCF8574
Slaveaddress, LCD Type

                                                     //TextLCD_I2C lcd(&i2c_lcd,
0x42, TextLCD::LCD16x2, TextLCD::WS0010);

                                                     // I2C bus, PCF8574
Slaveaddress, LCD Type, Device Type

                                                     //TextLCD_SPI_N
lcd(&spi_lcd, p8, p9);

                                                     // SPI bus, CS pin, RS pin,
LCDType=LCD16x2, BL=NC, LCDTCtrl=ST7032_3V3

//TextLCD_I2C_N lcd(&i2c_lcd, ST7032_SA, TextLCD::LCD16x2, NC,
TextLCD::ST7032_3V3);

// I2C bus, Slaveaddress, LCD Type, BL=NC, LCDTCtrl=ST7032_3V3


FileHandle *mbed::mbed_override_console(int fd)

{

   return &pc;

}


int main()

{


   printf("LCD Test. Columns=%d, Rows=%d\n\r", lcd.columns(), lcd.rows());


   for (int row = 0; row < lcd.rows(); row++)

   {

      int col = 0;

      printf("MemAddr(Col=%d, Row=%d)=0x%02X\n\r", col, row, lcd.getAddress(col,
row));

      //      lcd.putc('-');

      lcd.putc('0' + row);


   for (col = 1; col < lcd.columns() - 1; col++)

   {

      lcd.putc('*');

   }


      printf("MemAddr(Col=%d, Row=%d)=0x%02X\n\r", col, row, lcd.getAddress(col,
row));

      lcd.putc('+');

   }


   // Show cursor as blinking character

   lcd.setCursor(TextLCD::CurOff_BlkOn);


   // Set and show user defined characters. A maximum of 8 UDCs are supported by
the HD44780.

   // They are defined by a 5x7 bitpattern.

   lcd.setUDC(0, (char *)udc_0); // Show |>

   lcd.putc(0);                  //lcd.putc(0);

   lcd.setUDC(1, (char *)udc_1); // Show <|

   lcd.putc(1);                  //lcd.putc(1);

}*/
