// select the aruco database to use (matches the arucos on the test frame)
#define ARUCO_DB ARUCO_DB_4X4_1000
#define ARUCO_DB_SIZE 100

// include the ArucoLite header
#include <ArucoLite.h>

// include a constant array with the test frame
#include "test_frame.h"

// declare an ArucoLite object, with the resolution of the test image
ArucoLite<324, 324, 16, false> Aruco;

void setup(void)
{
}

void loop(void)
{
  // copy the test frame to the ArucoLite object frame. On a real application
  // with a camera, ideally the camera would be setup to DMA the image data
  // directly into the frame, thus avoiding the memcpy completely
  memcpy(Aruco.frame[0], test_frame, sizeof(test_frame));

  // call process and measure the processing time
  unsigned long time = micros();
  Aruco.process();
  time = micros() - time;

  // print the high level information
  Serial.print("found ");
  Serial.print(Aruco.arucos_found);
  Serial.print(" arucos in ");
  Serial.print(time);
  Serial.println(" us:");

  // print information on each aruco found
  for (int i = 0; i < Aruco.arucos_found; i++) {
    Serial.print("   aruco ");
    Serial.print(i);
    Serial.print(", id ");
    Serial.print(Aruco.result[i].aruco_idx);
    Serial.print(": ");
    for (int j = 0; j < 4; j++) {
      Serial.print("(");
      Serial.print(Aruco.result[i].pt[j].x);
      Serial.print(",");
      Serial.print(Aruco.result[i].pt[j].y);
      Serial.print(") ");
    }
    Serial.println("");
  }

  // just sleep for a second to reduce scrolling
  delay(1000);
}
