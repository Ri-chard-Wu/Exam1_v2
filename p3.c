
#include "mbed.h"

#include "stm32l475e_iot01_accelero.h"


InterruptIn btnRecord(BUTTON1);
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;


int16_t pDataXYZ[3] = {0};

int16_t x0 = 0,y0 = 0,z0 = 0;
int16_t x = 0,y=0,z=0;
int16_t dx = 0,dy=0,dz=0;
int16_t X[10] = {0};
int16_t Y[10] = {0};
int16_t Z[10] = {0};
int i_s = 0;
int idR[32] = {0};
int indexR = 0;

void compute_avg(){
    
    for(int i =0;i<10;i++){
        x+=X[i];
        y+=Y[i];
        z+=Z[i];
    }
    x  = x/10;
    y  = y/10;
    z  = z/10;

    int16_t dot = x*x0 + y*y0 + z*z0;
    int16_t mag_product = sqrt(x*x+y*y+z*z) * sqrt(x0*x0+y0*y0+z0*z0);
    float angle = acos((float)dot/mag_product);
    printf("%f\n", angle);
    
}

void record(void) {
   BSP_ACCELERO_AccGetXYZ(pDataXYZ);
   X[i_s] = pDataXYZ[0];
   Y[i_s] = pDataXYZ[1];
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


