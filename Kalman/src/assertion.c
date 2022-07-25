#include "assertion.h"

#ifdef KATANA_TRACKING_ASSERT

#include "print.h"

void func_assert(int x, const char* file, int line){
    if(x==0){
        mcu_print("Assertion failed at %s:%d\r\n", file, line);
        
        while(1){
            os_TaskSleep(1000);
        }
    } else {
        mcu_print("Assertion passed %s:%d\r\n", file, line);
    }
}

#endif

#ifdef __GNUC__
#include <stdio.h>
void __assert (const char *msg, const char *file, int line){
    fprintf(stderr, "Error: \"%s\" at file \"%s\", line: %d", msg, file, line);
}
#endif
