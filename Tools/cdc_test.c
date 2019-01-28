#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s </dev/tty*>\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  int fd = open(argv[1], O_RDWR);
  if (fd < 0) {
    perror(argv[1]);
    exit(EXIT_FAILURE);
  }
#if 0
  for (size_t i = 0; i < 256; ++i) {
    uint8_t buf[256];
    for (size_t j = 0; j < i; ++j)
      buf[j] = i;
    write(fd, buf, i);
  }
#endif
//#if 0
  uint8_t data = 0;
  for (;;) {
    //printf("out $%02x\n", data);
    //fflush(stdout);
    write(fd, &data, 1);
    data++;
  }
//#endif
  return 0;
}
