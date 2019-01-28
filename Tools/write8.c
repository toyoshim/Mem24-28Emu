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
  if (argc != 3) {
    fprintf(stderr, "Usage: %s </dev/tty*> <binary>\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  int bin = open(argv[2], O_RDONLY);
  if (bin < 0) {
    perror(argv[2]);
    exit(EXIT_FAILURE);
  }
  struct stat st;
  if (fstat(bin, &st)) {
    perror("fstat");
    exit(EXIT_FAILURE);
  }
  size_t bin_size = st.st_size;
  uint8_t bin_data[bin_size];
  if (bin_size != read(bin, bin_data, bin_size)) {
    perror("file read");
    exit(EXIT_FAILURE);
  }
  fprintf(stderr, "filename: %s\n", argv[2]);
  fprintf(stderr, "filesize: %ld\n", bin_size);
  if (bin_size & 0xff) {
    fprintf(stderr, "filesize should be 256 * N\n");
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
  buf[0] = '@';
  buf[1] = 'W';
  buf[2] = burst - 1;
  printf("memory writing (burst %d)", burst);
  fflush(stdout);
  for (int i = 0; i < bin_size; i+= burst) {
    for (int j = 0; j < burst; ++j) {
      // store to both upper and lower memory.
      buf[3 + j * 2] = bin_data[i + j];
      buf[4 + j * 2] = bin_data[i + j];
    }
    write(fd, buf, burst * 2 + 3);
    check(fd, "write");
    if (!(i & 0xff)) {
      printf(".");
      fflush(stdout);
    }
  }
  puts("done");

  command(fd, "@C", 2, "Unlock");
  command(fd, "@l", 2, "LED OFF");
  return 0;
}
