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

typedef struct rplidar_ans_header_t {
    uint8_t  syncByte1; // must be RPLIDAR_ANS_SYNC_BYTE1
    uint8_t  syncByte2; // must be RPLIDAR_ANS_SYNC_BYTE2
    uint32_t size_q30_subtype; // see uint32_t size:30; uint32_t subType:2;
    uint8_t  type;
} __attribute__((packed)) rplidar_ans_header_t;

typedef struct rplidar_response_measurement_node_t {
    uint8_t    quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    uint16_t   angle;        // check_bit:1;angle_q6:15;
    uint16_t   distance;
} __attribute__((packed)) rplidar_response_measurement_node_t;

typedef uint8_t measurement_buffer_t[5];

//void txChar(char c);
void lidarRead(void * buffer, size_t length);

const uint8_t comStartScan[] = {0xA5, 0x20};
const uint8_t rspStartScan[] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
const uint8_t comGetInfo[] = {0xA5, 0x50};
const uint8_t rspGetInfo[] = {0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04};
const uint8_t comGetHealth[] = {0xA5, 0x52};
const uint8_t rspGetHealth[] = {0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06};

UARTSerial lidar(D1, D0, 115200);
Serial pc(USBTX, USBRX, 115200);
char lidBuffer[128];
rplidar_response_device_health_t health;
rplidar_response_device_info_t info;
measurement_buffer_t measurement[8192];

int main() {
  DigitalOut red(LED1);
  DigitalOut green(LED2);
  DigitalOut motoctl(D2);
  int32_t i;
  ssize_t result;

  pc.printf("Lidar Client\n");

  //motoctl = 1;
  while (true) {
    //for (i = 0; i < sizeof(comGetHealth); i++) {
      //txChar(comGetHealth[i]);
    //}
    result = lidar.write(comGetHealth, sizeof(comGetHealth));
    red = 0;
    green = 1;
    //for (i = 0; i < sizeof(rspGetHealth); i++) {
      //lidBuffer[i] = lidar.getc();
    //}
    result = lidar.read(lidBuffer, sizeof(rspGetHealth));
    if (memcmp(lidBuffer, rspGetHealth, sizeof(rspGetHealth)) != 0) { 
      pc.printf("Health: Bad response\n");
      assert(false);
    }
    else {
      //for (i = 0; i < sizeof(rplidar_response_device_health_t); i++) {
	//((char *)&health)[i] = lidar.getc();
      //}
      lidarRead(&health, sizeof(rplidar_response_device_health_t));
    }
    if (health.status == 0) {
      pc.printf("Health status: Good\n");
    }
    else {
      pc.printf("Health status: Warning/Error %d\n", health.error_code);
    }
    //for (i = 0; i < sizeof(comGetInfo); i++) {
      //txChar(comGetInfo[i]);
    //}
    result = lidar.write(comGetInfo, sizeof(comGetInfo));
    red = 0;
    green = 1;
    //for (i = 0; i < sizeof(rspGetInfo); i++) {
      //lidBuffer[i] = lidar.getc();
    //}
    lidarRead(lidBuffer, sizeof(rspGetInfo));
    if (memcmp(lidBuffer, rspGetInfo, sizeof(rspGetInfo)) != 0) { 
      pc.printf("Info: Bad response\n");
      assert(false);
    }
    else {
      //for (i = 0; i < sizeof(rplidar_response_device_info_t); i++) {
	//((char *)&info)[i] = lidar.getc();
      //}
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
    red = 1;
    green = 0;
    motoctl = 1;
    //for (i = 0; i < sizeof(comStartScan); i++) {
      //txChar(comStartScan[i]);
    //}
    result = lidar.write(comStartScan, sizeof(comStartScan));
    //for (i = 0; i < sizeof(rspStartScan); i++) {
      //lidBuffer[i] = lidar.getc();
    //}
    lidarRead(lidBuffer, sizeof(rspStartScan));
    if (memcmp(lidBuffer, rspStartScan, sizeof(rspStartScan)) != 0) {
      pc.printf("Start scan: Bad response\n");
      assert(false);
    }
    for (i = 0; i < 8192; i++) {
      lidarRead(measurement[i], 5);
    }
    for (i = 0; i < 8192; i++) {
      rplidar_response_measurement_node_t *m = (rplidar_response_measurement_node_t *)&(measurement[i]);
      pc.printf("%02X %02X %02X %02X %02X\n", measurement[i][0],measurement[i][1],measurement[i][2],measurement[i][3],measurement[i][4]);

      pc.printf("Quality : %d\n", m->quality >> 2);
      pc.printf("Angle   : %05.3f\n", (m->angle >> 1) / 64.0);
      pc.printf("Distance: %05.3f\n\n", m->distance  / 4.0);
    }
  }
}

//void txChar(char c) {
  //if (lidar.putc(c) != c) {
    //assert(false);
  //}
//}

void lidarRead(void * buffer, size_t length) {
  ssize_t result;
  size_t nRemaining;
  uint8_t * pbuf = (uint8_t *)buffer;

  nRemaining = length;
  while (nRemaining > 0) {
    result = lidar.read(pbuf, nRemaining);
    nRemaining -= result;
    pbuf += result;
  }
}
