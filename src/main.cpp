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

void lidarFlushBuffers(void);
void lidarWrite(const uint8_t buffer[], size_t length);
void lidarRead(uint8_t buffer[], size_t length);
int lidarReadScanResponse(void);
int lidarReadResponse(const uint8_t *response, uint32_t length);
void timeoutAbort(void);
int lidarReadByte(void);
void serialCbHandler(int events);

const uint8_t comResetCore[] = {0xA5, 0x40};
const uint8_t comStopScan[] = {0xA5, 0x25};
const uint8_t comStartScan[] = {0xA5, 0x20};
const uint8_t rspStartScan[] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
const uint8_t comGetInfo[] = {0xA5, 0x50};
const uint8_t rspGetInfo[] = {0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04};
const uint8_t comGetHealth[] = {0xA5, 0x52};
const uint8_t rspGetHealth[] = {0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06};

RawSerial lidar(D1, D0, 115200);
Serial pc(USBTX, USBRX, 115200);
uint8_t lidBuffer[32];
rplidar_response_device_health_t health;
rplidar_response_device_info_t info;
measurement_buffer_t measurement[8192];
Timeout timeout;
volatile bool timedout;
volatile bool byteReceived;
event_callback_t serialCb = serialCbHandler;
uint8_t rxByte;

int main() {
  DigitalOut red(LED1);
  DigitalOut green(LED2);
  DigitalOut motoctl(D3);
  int32_t i;


  // Lights out
  red = 1;
  green = 1;

  // Say hello
  pc.printf("Lidar Client\n");


  // Get health
  do {
    lidarWrite(comGetHealth, sizeof(comGetHealth));
  } while (! lidarReadResponse(rspGetHealth, sizeof(rspGetHealth)));
  lidarRead((uint8_t *)&health, sizeof(rplidar_response_device_health_t));
  if (health.status == 0) {
    pc.printf("Health status: Good\n");
  }
  else {
    pc.printf("Health status: Warning/Error %d\n", health.error_code);
  }

  // Get info
  do {
    lidarWrite(comGetInfo, sizeof(comGetInfo));
  } while (! lidarReadResponse(rspGetInfo, sizeof(rspGetInfo)));
  lidarRead((uint8_t *)&info, sizeof(rplidar_response_device_info_t));
  pc.printf("Model: %d\n", info.model);
  pc.printf("Firmware version: %d.%d\n", info.firmware_version >> 8, info.firmware_version & 0x00FF);
  pc.printf("Hardware version: %d\n", info.hardware_version);
  pc.printf("Serial Number: ");
  for (i = 15; i >= 0; i--) {
    pc.printf("%02x", info.serialnum[i]);
  }
  pc.printf("\n\n");

  while (true) {


    // Start the motor
    motoctl = 1;

    red = 0;

    // Start a scan
    do {
      lidarWrite(comStartScan, sizeof(comStartScan));
    }
    while (!lidarReadResponse(rspStartScan, sizeof(rspStartScan)));

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
    //result = lidar.write(comStopScan, sizeof(comStopScan));
    //assert(result == sizeof(comStopScan));
    lidarWrite(comStopScan, sizeof(comStopScan));

    // Stop the motor
    motoctl = 0;
    //wait(2);
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

void lidarFlushBuffers(void) {
  int c;
  // Flush TX buffer
  //lidar.sync();
  // Flush RX buffer 
  while (lidar.readable()) {
    c = lidarReadByte();
  }
}

void lidarWrite(const uint8_t buffer[], size_t length) {
  uint32_t i;
  int c;

  for (i = 0; i < length; i += 1) {
    c = lidar.putc(buffer[i]);
    assert(c == buffer[i]);
  }
}

void lidarRead(uint8_t buffer[], size_t length) {
  uint32_t i;

  for (i = 0; i < length; i += 1) {
    buffer[i] = (uint8_t)lidarReadByte();
  }
}

int lidarReadResponse(const uint8_t *response, uint32_t length) {
  int i = 0;
  int c;
  int result = 0;

  timedout = false;
  timeout.attach(&timeoutAbort, 0.01);

  while (i < length && !timedout) {
    c = lidarReadByte();
    if (c == response[i]) {
      i += 1;
    }
    else if (c == response[0]) {
      i = 1;
    }
    else {
      i = 0;
    }
  }
  if (!timedout) {
    timeout.detach();
    result = 1;
  }
  return result;
}

void timeoutAbort(void) {
  timedout = true;
  lidar.abort_read();
}

int lidarReadByte(void) {
  int result;

  byteReceived = false;
  result = lidar.read(&rxByte, 1, serialCb, SERIAL_EVENT_RX_COMPLETE, '\xFF');
  while (! byteReceived) {
  }
  //assert(result == 1);
  return rxByte;
}

void serialCbHandler(int events) {
  byteReceived = true;
}
    
