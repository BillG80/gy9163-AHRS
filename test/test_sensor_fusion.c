#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "core/fusion/sensor_fusion.h"

// Declare the sensor_fusion_main function from sensor_fusion.c
extern int sensor_fusion_main(int argc, char *argv[]);

int main(int argc, char *argv[]) {
    printf("Starting Sensor Fusion Test\n");
    
    // Default to Madgwick if no algorithm specified
    if (argc < 2) {
        printf("No algorithm specified, using Madgwick\n");
        char *args[] = {argv[0], "MADGWICK"};
        return sensor_fusion_main(2, args);
    }
    
    return sensor_fusion_main(argc, argv);
}