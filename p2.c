
// exam p2

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
    t = (n/100.)*6.0;
    if(t < 3.0) sample = (uint16_t) (scaling * 3.0 * tanh(t));
    else if(t < 6.0) sample = (uint16_t)(scaling * (3.0 * (1 - tanh(3 - t)))); 
    aout.write_u16(sample);
   

}
