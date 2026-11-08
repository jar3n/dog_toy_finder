/**

Dog Toy Finder Pico 2 W firmware Main file

*/


#include <pico/time.h>
#include <stdio.h>
#include "pico/stdio.h"

#include "config/wifi_config.h"



int main(void){
    stdio_init_all();
    sleep_ms(1500);
    while (true) {
        printf("Hello World from the PICO\n");
        sleep_ms(100);
    }
    
}