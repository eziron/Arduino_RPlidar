# Arduino_RPlidar

### Librería para controlar un sensor RPlidar de SLAMTEC desde una arduino

Por defecto la librería está configurada para funcionar con el Serial2 de arduino

Esto de define en el archivo RPlidar.h

https://github.com/eziron/Arduino_RPlidar/blob/master/RPlidar.h#L19
```C++
#define lidar_port Serial2
```
en el caso de necesitar otro puesto, simplemente cambie la definición al Serial que necesite

idealmente se recomienda trabajar con una arduino con al menos 2 puertos seriales por hardware


## Ejemplo básico
```C++
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
```
## Funciones principales
```C++
void begin(uint motor_pin, unsigned long lidar_baudrate);
//motor_pin: pin donde esta conectado CTRL_MOTO del lidar, necesariamente debe tener soporte para PWM
//lidar_baudrate: velocidad del serial, por defecto es 115200

==============================================

//Inicio de los diferentes modos funcionamiento
bool star_normal_scan(uint pwm_val);
bool star_force_scan(uint pwm_val);
bool star_legacy_scan(uint pwm_val);
bool star_extended_scan(uint pwm_val);
//pwm_val: valor del PWM del motor, va de 0 a 255
//retorna True si el modo se activó correctamente

==============================================

void read_lidar_scan();
//lee y decodifica las muestras que envia el lidar

//mientras están llegando los paquetes de datos,
//esta función simplemente va a tomar los que hayan disponibles y va a continuar

//una vez que llegan todos los paquetes de datos necesarios para decodificar una muestra
//este los procesa y los agrega a la pila de muestras automáticamente

//en los modos legacy y extended, la decodificación de los datos puede demorar algo de tiempo

==============================================

//obtén una muestra del lidar desde la pila de muestras
bool get_sample(uint16_t *out_angle_q6,uint16_t *out_distance_q2, uint32_t *out_time);
//out_angle_q6: ángulo de la muestra
//out_distance_q2; distancia de la muestra
//out_time: tiempo en us de cuando fue tomada la muestra por el lidar
//retorna false si no hay muestras disponibles para leer

==============================================

uint16_t get_sample_available()
//retorna el número de muestras disponibles para leer en la cola de muestras
```

## funciones extra
```C++
void set_motor_speed(uint val);
bool wait_lidar_response(uint32_t time_out);
bool wait_lidar_response(uint32_t time_out,uint8_t buffer_target);
void stop_scan();
void reset_lidar();

//lectura de una cadena de bytes que envie el lidar
bool read_lidar(uint8_t *out_rx_buffer,uint8_t rx_lim, bool synq_byte_en, uint32_t time_out);
//out_rx_buffer: buffer de salida de los bytes
//rx_lim: número de bytes que se espera recibir desde el lidar
//synq_byte_en: habilitar los bytes de sincronisacion
//time_out: tiempo máximo que se puede tardar la respuesta en ms

//retorna True cuando la cadena de bytes es leída correctamente

//si time_out = 0, entonces la función se comporta como no bloqueante,
//es decir, va a ir guardando los bytes en un buffer 
//y continuando con el código hasta que la cadena de bytes se complete

//con time_out>0 también va a guardar los bytes en el buffer interno, 
//pero en el caso de no haber datos disponibles para leer en el serial, 
//va a esperar el tiempo indicado en time_out hasta que lleguen datos
```
