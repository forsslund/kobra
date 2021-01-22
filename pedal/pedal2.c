#include <stdio.h>
#include <fcntl.h>
//#include <linux/hiddev.h>

struct hiddev_event {
  unsigned hid;
  signed int value;
};

int main() {
  int fd = open("/dev/infinity-in-usb-2", O_NONBLOCK);
  //ioctl(fd,HIDIOCINITREPORT,0);	
  //ioctl(fd,0x05,0);	

  printf("Open: %d. Size of struct: %d. Test 2\n", fd, sizeof(struct hiddev_event));

  int state[] = { 0, 0, 0 }; 
  int i;

  struct hiddev_event event;

  while (1) {
    for (i = 0; i < 3; i++) {
      while (1) {
        int rcount = read(fd, &event, sizeof(struct hiddev_event));

        /*
        if(rcount > 0){
          printf("Read: %d bytes: ", rcount);
          printf("event.hid: %d; event.value: %d \n", event.hid, event.value);
        }*/

        // New protocol 2021
        if (rcount > 0 && event.hid == 589899){
          //printf("new protocol message. i=%d\n",i);
          if(i==0){
            state[0] = (event.value & 1);
            state[1] = (event.value & 2)>>1;
            state[2] = (event.value & 4)>>2;
            break;
          }
          if(i==1){
            i=100; // restart count
            break; // only 2 messages, the last being always 0
          }
        }
        // Old protocol
        else if (rcount > 0 && (event.hid & 0xFF) == (i + 1)) {
            /* TODO: Assert that the whole hiddev_event has been read. */
            state[i] = event.value;
            break;
        }         
        if (rcount <= 0) {
          usleep(5*1000); /* 5 ms delay only if nothing was read. */
        }
      }
    }

    printf("Pedal state: %d, %d, %d\n", state[0], state[1], state[2]);
  }
}
