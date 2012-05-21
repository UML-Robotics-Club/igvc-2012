#ifndef CROAK_HH
#define CROAK_HH

#include <stdio.h>
#include <stdlib.h>

#define croak(s) croak_real(s, __FILE__, __LINE__)
inline void croak_real(const char* msg, const char* file, int line) {
    fprintf(stderr, "Fatal error: %s at %s:%d\n", msg, file, line);
    exit(11);
}

#define croaki(s,i) croaki_real(s, i, __FILE__, __LINE__)
inline void croaki_real(const char* msg, int i, const char* file, int line) {
    fprintf(stderr, "Fatal error: %s (%d) at %s:%d\n", msg, i, file, line);
    exit(11);
}

#define check_syscall(name, rv) \
    if(rv < 0) { perror(name); croak("syscall failed"); }

#endif
