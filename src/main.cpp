#include <mbed.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

typedef struct rplidar_response_device_info_t {
    uint8_t   model;
    uint16_t  firmware_version;
    uint8_t   hardware_version;
    uint8_t   serialnum[16];
} __attribute__((packed)) rplidar_response_device_info_t;

typedef struct rplidar_response_device_health_t {
    uint8_t   status;
    uint16_t  error_code;
} __attribute__((packed)) rplidar_response_device_health_t;

typedef struct rplidar_response_measurement_node_t {
    uint8_t    quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    uint16_t   angle;        // check_bit:1;angle_q6:15;
    uint16_t   distance;
} __attribute__((packed)) rplidar_response_measurement_node_t;

typedef uint8_t measurement_buffer_t[5];

void lidarRead(void * buffer, size_t length);

const uint8_t comResetCore[] = {0xA5, 0x40};
const uint8_t comStopScan[] = {0xA5, 0x25};
const uint8_t comStartScan[] = {0xA5, 0x20};
const uint8_t rspStartScan[] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
const uint8_t comGetInfo[] = {0xA5, 0x50};
const uint8_t rspGetInfo[] = {0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04};
const uint8_t comGetHealth[] = {0xA5, 0x52};
const uint8_t rspGetHealth[] = {0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06};

UARTSerial lidar(D1, D0, 115200);
Serial pc(USBTX, USBRX, 115200);
uint8_t lidBuffer[32];
rplidar_response_device_health_t health;
rplidar_response_device_info_t info;
measurement_buffer_t measurement[8192];

int main() {
  DigitalOut red(LED1);
  DigitalOut green(LED2);
  DigitalOut motoctl(D3);
  int32_t i;
  ssize_t result;

  // Lights out
  red = 1;
  green = 1;

  // Say hello
  pc.printf("Lidar Client\n");

  // Get health
  result = lidar.write(comGetHealth, sizeof(comGetHealth));
  assert(result == sizeof(comGetHealth));
  lidarRead(lidBuffer, sizeof(rspGetHealth));
  if (memcmp(lidBuffer, rspGetHealth, sizeof(rspGetHealth)) != 0) { 
    pc.printf("Health: Bad response\n");
    assert(false);
  }
  else {
    lidarRead(&health, sizeof(rplidar_response_device_health_t));
  }
  if (health.status == 0) {
    pc.printf("Health status: Good\n");
  }
  else {
    pc.printf("Health status: Warning/Error %d\n", health.error_code);
  }

  // Get info
  result = lidar.write(comGetInfo, sizeof(comGetInfo));
  assert(result == sizeof(comGetInfo));
  lidarRead(lidBuffer, sizeof(rspGetInfo));
  if (memcmp(lidBuffer, rspGetInfo, sizeof(rspGetInfo)) != 0) { 
    pc.printf("Info: Bad response\n");
    assert(false);
  }
  else {
    lidarRead(&info, sizeof(rplidar_response_device_info_t));
    pc.printf("Model: %d\n", info.model);
    pc.printf("Firmware version: %d.%d\n", info.firmware_version >> 8, info.firmware_version & 0x00FF);
    pc.printf("Hardware version: %d\n", info.hardware_version);
    pc.printf("Serial Number: ");
    for (i = 15; i >= 0; i--) {
      pc.printf("%02x", info.serialnum[i]);
    }
    pc.printf("\n\n");
  }

  while (true) {

    // Start the motor
    motoctl = 1;

    // Make sure the receive buffer is empty before starting a new scan
    while (lidar.readable()) {
      lidarRead(lidBuffer, 1);
    }

    red = 0;

    // Start a scan
    result = lidar.write(comStartScan, sizeof(comStartScan));
    assert(result == sizeof(comStartScan));
    lidarRead(lidBuffer, sizeof(rspStartScan));
    if (memcmp(lidBuffer, rspStartScan, sizeof(rspStartScan)) != 0) {
      pc.printf("Start scan: Bad response\n");
      assert(false);
    }

    green = 1;

    /* Demonstrate that, once a scan starts, it can keep going without
     * further intervention.
     * There's only about 70us between readings, so we don't have time for much
     * processing here/
     */ 
    //i = 0;
    //while (true) {
      //lidarRead(measurement[0], sizeof(measurement_buffer_t));
      //if (measurement[0][1] & 0x01) {
	//i += 1;
      //}
      //if (i == 500) {
	//green = 1 - green;
	//i = 0;
      //}
    //}

    // Capture some readings
    for (i = 0; i < 8192; i++) {
      lidarRead(measurement[i], sizeof(measurement_buffer_t));
    }

    red = 1;
    green = 0;

    // Stop the scan
    result = lidar.write(comStopScan, sizeof(comStopScan));
    assert(result == sizeof(comStopScan));

    // Stop the motor
    motoctl = 0;

    // Print the readings
    for (i = 0; i < 8192; i++) {
      rplidar_response_measurement_node_t *m = (rplidar_response_measurement_node_t *)&(measurement[i]);
      pc.printf("%4ld: %02X %02X %02X %02X %02X\n", i, measurement[i][0],measurement[i][1],measurement[i][2],measurement[i][3],measurement[i][4]);

      pc.printf("Quality : %d\n", m->quality >> 2);
      pc.printf("Angle   : %5.1f\n", (m->angle >> 1) / 64.0);
      pc.printf("Distance: %5.1f\n\n", m->distance  / 4.0);
    }
  }
}

void lidarRead(void *buffer, size_t length) {
  ssize_t result;
  size_t nRemaining;
  uint8_t * pbuf = (uint8_t *)buffer;

  nRemaining = length;
  while (nRemaining > 0) {
    result = lidar.read(pbuf, nRemaining);
    assert(result >= 0);
    nRemaining -= result;
    pbuf += result;
  }
}
