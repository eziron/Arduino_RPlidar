#include <Servo.h>
#include <RPlidar.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

RPlidar lidar;
Servo lidar_servo;

unsigned int servo_us = 500;
bool servo_dir = false;
uint32_t servo_time = 0;

uint16_t lidar_angulo1_q6 = 0;
uint16_t lidar_angulo2_q6 = 0;
uint16_t lidar_distancia_q2 = 0;
uint32_t lidar_sample_time = 0;
uint8_t tx_buffer[20];

uint16_t analog_val = 0;
uint32_t analog_time_ref = 0; 
uint16_t sample_pot_q6[8000];
uint32_t sample_time_pot[8000];
uint16_t n_write_pot = 0;
uint16_t n_read_pot = 0;

uint32_t send_time_ref = 0;

struct repeating_timer timer_pot;

void setup() {
  pinMode(16, OUTPUT);
  pinMode(25, OUTPUT);
  digitalWrite(25,0);
  analogReadResolution(12);

  Serial.begin(1500000);

  lidar_servo.attach(11,500,2500);
  lidar_servo.write(0);

  uart_init(uart1, 115200);
  gpio_set_function(8, GPIO_FUNC_UART);
  gpio_set_function(9, GPIO_FUNC_UART);


  uart_init(uart0, 1500000);
  gpio_set_function(13, GPIO_FUNC_UART);
  gpio_set_function(12, GPIO_FUNC_UART);
  
  add_repeating_timer_us(60, leer_pot, NULL, &timer_pot);

  lidar.begin(10);
  delay(10);
  lidar.star_extended_scan(100);
  //lidar.star_legacy_scan(100);
  //lidar.star_normal_scan(100);
  send_time_ref = micros();
}
void loop() {
  if(micros() - send_time_ref >= 166){
    if(lidar.get_sample(&lidar_angulo1_q6,&lidar_distancia_q2, &lidar_sample_time)){
      send_time_ref = micros();
      buscar_por_tiempo(lidar_sample_time);
      send_lidar_data();
    }
  }
}

void setup1() {
  lidar_servo.attach(3, 500, 2500);
  lidar_servo.write(0);
  servo_time = micros();
}

void loop1() {
  lidar.read_lidar_scan();
  control_servo();
}










