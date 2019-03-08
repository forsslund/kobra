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

  int state[] = { 0, 0, 0 }; 
  int i;

  struct hiddev_event event;

  while (1) {
    for (i = 0; i < 3; i++) {
      while (1) {
        int rcount = read(fd, &event, sizeof(struct hiddev_event));

        if (rcount > 0 && (event.hid & 0xFF) == (i + 1)) {
            /* TODO: Assert that the whole hiddev_event has been read. */
            state[i] = event.value;
            break;
        } else if (rcount <= 0) {
          usleep(5*1000); /* 5 ms delay only if nothing was read. */
        }
      }
    }

    printf("Pedal state: %d, %d, %d\n", state[0], state[1], state[2]);
  }
}
