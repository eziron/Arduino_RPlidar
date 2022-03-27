#include <RPlidar.h>

RPlidar lidar;

uint16_t lidar_angulo_q6 = 0;
uint16_t lidar_distancia_q2 = 0;
uint32_t lidar_sample_time = 0;

void setup() {
  Serial.begin(1500000);

  lidar.begin(10);
  delay(500);

  //lidar.star_extended_scan();
  //lidar.star_legacy_scan();
  lidar.star_normal_scan();
}
void loop() {
  lidar.read_lidar_scan();

  if(lidar.get_sample(&lidar_angulo_q6,&lidar_distancia_q2, &lidar_sample_time)){
    Serial.print((float)lidar_angulo_q6/64,2);
    Serial.print("° - ");
    Serial.print((float)lidar_distancia_q2/4,2);
    Serial.println(" mm");
  }
}










