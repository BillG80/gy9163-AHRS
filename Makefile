# ==== Toolchain and Flags ====
CROSS_COMPILE ?=
CC = $(CROSS_COMPILE)gcc

INCLUDES = \
    -Iinclude \
    -Iinclude/display \
    -Iinclude/core/navigation \
    -Iinclude/drivers/ssd1306 \
    -Iinclude/logging \
    -Isrc \
    -Ifonts

CFLAGS = -Wall -O2 $(INCLUDES) -lm -lpthread

LDFLAGS = -lbcm2835 -lm -lpthread

# ==== Source Files ====
SRC = \
    src/main.c \
    src/display/screens.c \
    src/display/icon_animation.c \
    src/display/display_manager.c \
    src/core/fusion/sensor_fusion.c \
    src/core/fusion/calibration.c \
    src/core/navigation/elevator_nav.c \
    src/core/navigation/vertical_motion.c \
    src/core/ahrs/ekf_ahrs.c \
    src/core/ahrs/madgwick_ahrs.c \
    src/core/ahrs/mahony_ahrs.c \
    src/drivers/icm20948/icm20948.c \
    src/drivers/bus/spi_bus.c \
    src/drivers/bus/i2c_bus.c \
    src/drivers/bcm_manager.c \
    src/drivers/gpio/gpio.c \
    src/drivers/ms5611/ms5611.c \
    src/drivers/ssd1306/ssd1306.c \
    src/drivers/ssd1306/ssd1306_graphics.c \
    src/fonts/SansCondensed.c \
    src/drivers/led_indicator.c \
    src/core/fusion/svd3.c \
    src/display/calib_pose_icons.c \
    src/logging/file_logger.c \
    src/logging/calibration_data_log.c

OBJ = $(SRC:.c=.o)

# ==== Target ====
TARGET = elevator_app

# ==== Build Rules ====
.PHONY: all clean test_bus run install-service

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

# Pattern rule for object files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# ==== Test Bus Build ====
TEST_BUS_SRC = test/test_bus.c \
               src/drivers/bus/spi_bus.c \
               src/drivers/bus/i2c_bus.c \
               src/drivers/bcm_manager.c \
               src/drivers/gpio/gpio.c

TEST_BUS_OBJ = $(TEST_BUS_SRC:.c=.o)
TEST_BUS_TARGET = test_bus

test_bus: $(TEST_BUS_OBJ)
	$(CC) $(TEST_BUS_OBJ) -o $(TEST_BUS_TARGET) $(LDFLAGS)

TEST_ICON_SRC = test/test_icon.c \
               src/drivers/bus/i2c_bus.c \
               src/drivers/bcm_manager.c \
               src/drivers/gpio/gpio.c \
               src/drivers/ssd1306/ssd1306.c \
               src/drivers/ssd1306/ssd1306_graphics.c \
               src/display/icon_animation.c \
               src/fonts/SansCondensed.c

TEST_ICON_OBJ = $(TEST_ICON_SRC:.c=.o)
TEST_ICON_TARGET = test_icon

test_icon: $(TEST_ICON_OBJ)
	$(CC) $(CFLAGS) $(TEST_ICON_OBJ) -o $(TEST_ICON_TARGET) $(LDFLAGS)

# Define individual object files for test_display
TEST_DISPLAY_OBJS = test/test_display.o \
                   src/drivers/bus/i2c_bus.o \
                   src/drivers/bcm_manager.o \
                   src/drivers/gpio/gpio.o \
                   src/drivers/ssd1306/ssd1306.o \
                   src/drivers/ssd1306/ssd1306_graphics.o \
                   src/display/icon_animation.o \
                   src/display/screens.o \
                   src/fonts/SansCondensed.o

# Direct build rule for test_display
test_display: $(TEST_DISPLAY_OBJS)
	$(CC) $(CFLAGS) $^ -o test_display $(LDFLAGS)

# Individual compilation rules
test/test_display.o: test/test_display.c
	$(CC) $(CFLAGS) -c $< -o $@

src/display/screens.o: src/display/screens.c include/display/screens.h
	$(CC) $(CFLAGS) -c $< -o $@

# Define individual object files for test_arrow
TEST_ARROW_OBJS = test/test_arrow.o \
                 src/drivers/bus/i2c_bus.o \
                 src/drivers/bcm_manager.o \
                 src/drivers/gpio/gpio.o \
                 src/drivers/ssd1306/ssd1306.o \
                 src/drivers/ssd1306/ssd1306_graphics.o \
                 src/display/icon_animation.o \
                 src/fonts/SansCondensed.o

# Direct build rule for test_arrow
test_arrow: $(TEST_ARROW_OBJS)
	$(CC) $(CFLAGS) $^ -o test_arrow $(LDFLAGS)

# Individual compilation rules
test/test_arrow.o: test/test_arrow.c
	$(CC) $(CFLAGS) -c $< -o $@

# Define individual object files for test_sensor_fusion
TEST_FUSION_OBJS = test/test_sensor_fusion.o \
                  src/core/fusion/sensor_fusion.o \
                  src/core/fusion/calibration.o \
                  src/core/navigation/elevator_nav.o \
                  src/core/ahrs/ekf_ahrs.o \
                  src/core/ahrs/madgwick_ahrs.o \
                  src/core/ahrs/mahony_ahrs.o \
                  src/drivers/icm20948/icm20948.o \
                  src/drivers/bus/spi_bus.o \
                  src/drivers/bus/i2c_bus.o \
                  src/drivers/bcm_manager.o \
                  src/drivers/gpio/gpio.o \
                  src/drivers/ms5611/ms5611.o \
                  src/logging/file_logger.o

# Force debug build for sensor fusion test so DPRINTF is defined
/* test_sensor_fusion: CFLAGS += -DENABLE_DEBUG -g */
test_sensor_fusion: $(TEST_FUSION_OBJS)
	$(CC) $(CFLAGS) $^ -o test_sensor_fusion $(LDFLAGS)

# Individual compilation rules
test/test_sensor_fusion.o: test/test_sensor_fusion.c
	$(CC) $(CFLAGS) -c $< -o $@

# led_service moved to its own target
LED_SERVICE_SRC = \
    src/drivers/led_service.c \
    src/drivers/led_indicator.c \
    src/drivers/bus/i2c_bus.c \
    src/drivers/bcm_manager.c \
    src/drivers/gpio/gpio.c

LED_SERVICE_OBJ = $(LED_SERVICE_SRC:.c=.o)

led_service: $(LED_SERVICE_OBJ)
	$(CC) $(CFLAGS) $(LED_SERVICE_OBJ) -o $@ $(LDFLAGS)

# Direct run rule
run: all
	sudo ./$(TARGET)

# ==== Install Service ====
install-service: all
	install -m0755 $(TARGET) /usr/local/bin/$(TARGET)
	install -m0755 led_service /usr/local/bin/led_service
	install -m0644 elevator_app.service /etc/systemd/system/elevator_app.service
	install -m0644 elevator_led.service /etc/systemd/system/elevator_led.service
	systemctl daemon-reload
	systemctl enable elevator_app elevator_led

# ==== Clean ====
clean:
	rm -f $(OBJ) $(TARGET) $(TEST_BUS_OBJ) $(TEST_BUS_TARGET) $(TEST_ICON_OBJ) $(TEST_ICON_TARGET) $(TEST_DISPLAY_OBJS) test_display $(TEST_ARROW_OBJS) test_arrow $(TEST_FUSION_OBJS) test_sensor_fusion $(LED_SERVICE_OBJ) led_service