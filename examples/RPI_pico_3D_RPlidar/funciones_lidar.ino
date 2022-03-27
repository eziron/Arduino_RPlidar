void send_lidar_data() {
  tx_buffer[0] = 254;
  tx_buffer[1] = 252;

  tx_buffer[2] = 127;
  tx_buffer[3] = 'H';
  tx_buffer[4] = 3;

  tx_buffer[5] = lidar_angulo1_q6 >> 8;
  tx_buffer[6] = lidar_angulo1_q6;

  tx_buffer[7] = lidar_angulo2_q6 >> 8;
  tx_buffer[8] = lidar_angulo2_q6;

  tx_buffer[9] = lidar_distancia_q2 >> 8;
  tx_buffer[10] = lidar_distancia_q2;

  Serial.write(tx_buffer, 11);
  //uart_write_blocking(uart0,tx_buffer, 11);
}

void control_servo() {
  if (micros() - servo_time >= 2500) {
    servo_time = micros();

    if (servo_us >= 2500 || servo_us <= 500) {
      servo_dir = !servo_dir;
    }

    if (servo_dir) {
      servo_us++;
    }
    else {
      servo_us--;
    }

    lidar_servo.writeMicroseconds(servo_us);
  }
}

bool leer_pot(struct repeating_timer *t){
    analog_time_ref = micros();
    analog_val = round(map(constrain(analogRead(A0),677,3582),677,3582,0,11520));
    sample_pot_q6[n_write_pot] = analog_val;
    sample_time_pot[n_write_pot] = analog_time_ref;

    n_write_pot = constrain_circ(n_write_pot+1,0,8000);
    if(n_write_pot == n_read_pot){
      n_read_pot = constrain_circ(n_read_pot+1,0,8000);
    }
    return true;
}

bool buscar_por_tiempo(uint32_t time_ref){
  if(calc_cola_dist(n_write_pot,n_read_pot,8000) > 0){
    if(sample_time_pot[n_read_pot] == time_ref){
      lidar_angulo2_q6 = sample_pot_q6[n_read_pot];
      return true;
    }
    else{
      if(sample_time_pot[n_read_pot] < time_ref){
        while (sample_time_pot[n_read_pot] < time_ref){
          if(calc_cola_dist(n_write_pot,n_read_pot,8000) == 0){
            return false;
          }
          n_read_pot = constrain_circ(n_read_pot+1,0,8000);
        }
        n_read_pot = constrain_circ(n_read_pot-1,0,8000);

        lidar_angulo2_q6 = sample_pot_q6[n_read_pot];
        return true;
      }
      else{
        return false;
      }
    }
  }
  else{
    return false;
  }
}

uint16_t constrain_circ(uint16_t val, uint16_t min_val, uint16_t max_val){
    if(val >= max_val){
        return min_val;
    }
    else if (val <= min_val){
        return max_val-1;
    }
    else{
        return val;
    }
}

uint16_t calc_cola_dist(uint16_t n_write,uint16_t n_read,uint16_t n_max){
    if(n_write == n_read){
        return 0;
    }
    else{
        if(n_write > n_read){
            return n_write-n_read;
        }
        //n_read > n_write
        else{
            return n_max-n_read+n_write;
        }
    }
}