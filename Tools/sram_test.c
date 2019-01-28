#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

void check(int fd, const char* command) {
  uint8_t buf[1];
  int read_size = read(fd, &buf, 1);
  if (read_size != 1 || buf[0] != '+') {
    char s[128];
    snprintf(s, 128, "Failed on %s; size=%d, result=%c",
        command, read_size, buf[0]);
    perror(s);
    exit(EXIT_FAILURE);
  }
}

void command(int fd, const char* command, size_t len, const char* comment) {
  write(fd, command, len);
  check(fd, comment);
  printf("%s: done\n", comment);
}

int main(int argc, char** argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s </dev/tty*>\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  int fd = open(argv[1], O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror(argv[1]);
    exit(EXIT_FAILURE);
  }
  struct termios ts;
  memset(&ts, 0, sizeof(ts));
  cfmakeraw(&ts);
  tcsetattr(fd, TCSANOW, &ts);

  command(fd, "@L", 2, "LED ON");
  command(fd, "@O", 2, "Lock");
  command(fd, "@a", 2, "Address reset");

  int burst = 256;
  uint8_t buf[burst * 2 + 3];
//#if 0
  buf[0] = '@';
  buf[1] = 'W';
  buf[2] = burst - 1;
  printf("memory writing (burst %d)", burst);
  fflush(stdout);
  for (int i = 0; i < 0x10000; i += burst) {
    for (int j = 0; j < burst; ++j) {
      uint8_t x = ((i + j) >> 8) ^ ((i + j) & 0xff);
      buf[3 + j * 2] = x;
      buf[4 + j * 2] = x;
    }
    write(fd, buf, burst * 2 + 3);
    check(fd, "write");
    if (!(i & 0xff)) {
      printf(".");
      fflush(stdout);
    }
  }
  puts("done");

  command(fd, "@a", 2, "Address reset");
//#endif

//#if 0
  printf("memory reading");
  fflush(stdout);
  for (int i = 0; i < 0x10000; i += 16) {
    buf[0] = '@';
    buf[1] = 'R';
    buf[2] = 16 - 1;
    write(fd, buf, 3);
    int read_size = read(fd, buf, 34);
    if (read_size != 34 || buf[0] != '!' || buf[33] != '+') {
      char s[128];
      snprintf(s, 128, "Failed on read; size=%d, data=$%04x, ..., result=%c,%c",
          read_size, (buf[1] << 8) | buf[2], buf[0], buf[33]);
      perror(s);
      exit(EXIT_FAILURE);
    }
    for (int j = 0; j < 16; ++j) {
      uint8_t x = ((i + j) >> 8) ^ ((i + j) & 0xff);
      uint16_t data = (buf[1 + j * 2] << 8) | buf[2 + j * 2];
      uint16_t expected = (x << 8) | x;
      if (expected != data) {
        printf("error at $%04x: expected $%04x, actual $%04x\n",
            i + j, expected, data);
      }
    }
    if (!(i & 0xff)) {
      printf(".");
      fflush(stdout);
    }
  }
  printf("done\n");
//#endif

  command(fd, "@C", 2, "Unlock");
  command(fd, "@l", 2, "LED OFF");
  return 0;
}
