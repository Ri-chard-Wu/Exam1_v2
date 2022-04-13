





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
  printf("begin");
  btn.rise(&change_state);
  while (true) {
      ThisThread::sleep_for(200ms);
      
  }
}



