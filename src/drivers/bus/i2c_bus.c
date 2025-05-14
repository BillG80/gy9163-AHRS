#include "drivers/bus/i2c_bus.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// Define a local buffer limit as BCM2835_I2C_BUFFER_LENGTH seems unavailable
// Check bcm2835.h or documentation for the actual limit if possible.
// Using a conservative value for now.
#define I2C_BUS_BUFFER_LIMIT 16

// Define the actual structure (implementation detail)
struct i2c_bus_device {
    int fd; // File descriptor for /dev/i2c-X
    uint8_t bus_num;
    uint32_t clock_speed; // For API compatibility, not actively set
};

// --- Initialization ---
// --- Initialization ---
// --- Initialization ---
status_t i2c_bus_init(const i2c_config_t* config, i2c_bus_handle_t* handle_ptr) {
    if (!config || !handle_ptr) return ERROR_NULL_POINTER;

    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%d", config->bus_num);

    int fd = open(dev_path, O_RDWR);
    if (fd < 0) {
        perror("ERROR: Failed to open I2C device");
        return ERROR_HARDWARE;
    }

    struct i2c_bus_device* dev = malloc(sizeof(struct i2c_bus_device));
    if (!dev) {
        close(fd);
        return ERROR_NO_MEM;
    }
    dev->fd = fd;
    dev->bus_num = config->bus_num;
    dev->clock_speed = config->speed;

    *handle_ptr = dev;
    printf("INFO: I2C Bus Initialized (bus %d, fd %d)\n", dev->bus_num, dev->fd);
    return STATUS_OK;
}

// --- Close ---
status_t i2c_bus_close(i2c_bus_handle_t handle) {
    if (!handle) return STATUS_OK;
    struct i2c_bus_device* dev = (struct i2c_bus_device*)handle;
    close(dev->fd);
    free(dev);
    printf("INFO: I2C Bus Closed\n");
    return STATUS_OK;
}

// --- Write Bytes ---
status_t i2c_bus_write(i2c_bus_handle_t handle, uint8_t i2c_addr, const uint8_t* data, size_t len) {
    if (!handle || !data || len == 0) return ERROR_NULL_POINTER;
    struct i2c_bus_device* dev = (struct i2c_bus_device*)handle;

    if (ioctl(dev->fd, I2C_SLAVE, i2c_addr) < 0) {
        perror("ERROR: ioctl I2C_SLAVE failed");
        return ERROR_HARDWARE;
    }
    ssize_t written = write(dev->fd, data, len);
    if (written != (ssize_t)len) {
        perror("ERROR: write failed");
        return ERROR_HARDWARE;
    }
    return STATUS_OK;
}

// --- Read Bytes ---
status_t i2c_bus_read(i2c_bus_handle_t handle, uint8_t i2c_addr, uint8_t* data, size_t len) {
    if (!handle || !data || len == 0) return ERROR_NULL_POINTER;
    struct i2c_bus_device* dev = (struct i2c_bus_device*)handle;

    if (ioctl(dev->fd, I2C_SLAVE, i2c_addr) < 0) {
        perror("ERROR: ioctl I2C_SLAVE failed");
        return ERROR_HARDWARE;
    }
    ssize_t read_bytes = read(dev->fd, data, len);
    if (read_bytes != (ssize_t)len) {
        perror("ERROR: read failed");
        return ERROR_HARDWARE;
    }
    return STATUS_OK;
}

// --- Write followed by Read (Combined Transaction) ---
// Ensure this function exists and matches the header declaration
#include <linux/i2c.h>
#include <sys/ioctl.h>

// ...

status_t i2c_bus_write_read(i2c_bus_handle_t handle, uint8_t i2c_addr,
                            const uint8_t* write_data, size_t write_len,
                            uint8_t* read_data, size_t read_len)
{
    if (!handle || !write_data || !read_data) {
        return ERROR_NULL_POINTER;
    }
    if (write_len == 0 || read_len == 0) {
        return ERROR_INVALID_PARAM;
    }
    struct i2c_bus_device* dev = (struct i2c_bus_device*)handle;

    struct i2c_msg msgs[2];
    msgs[0].addr  = i2c_addr;
    msgs[0].flags = 0; // Write
    msgs[0].len   = write_len;
    msgs[0].buf   = (uint8_t*)write_data;

    msgs[1].addr  = i2c_addr;
    msgs[1].flags = I2C_M_RD; // Read
    msgs[1].len   = read_len;
    msgs[1].buf   = read_data;

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs  = msgs;
    ioctl_data.nmsgs = 2;

    if (ioctl(dev->fd, I2C_RDWR, &ioctl_data) < 0) {
        perror("ERROR: I2C_RDWR ioctl failed");
        return ERROR_HARDWARE;
    }
    return STATUS_OK;
}

// Paste or write the full implementation (definition) here:
status_t i2c_bus_cleanup(i2c_bus_handle_t handle) {
    if (!handle) {
        return STATUS_OK; // Nothing to clean up
    }
    struct i2c_bus_device* dev = (struct i2c_bus_device*)handle;
    if (dev->fd >= 0) {
        close(dev->fd);
    }
    free(dev);
    printf("INFO: I2C Bus Cleaned Up\n");
    return STATUS_OK;
}