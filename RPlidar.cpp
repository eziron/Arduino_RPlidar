#include "Arduino.h"
#include "RPlidar.h"




RPlidar::RPlidar(){}

uint16_t RPlidar::constrain_circ(uint16_t val, uint16_t min_val, uint16_t max_val){
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

uint16_t RPlidar::calc_cola_dist(uint16_t n_write,uint16_t n_read,uint16_t n_max){
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

void RPlidar::reset_buffers(){
    first_sample = false;
    first_sample_time = false;

    n_write_sample = 0;
    n_read_sample = 0;

    rx_count = 0;
    rx_c = 0;

    pass_rx_buffer_time1 = 0;
    pass_rx_buffer_time2 = 0;

    for(int i = 0; i < serial_buffer_len; i++){
        prev_rx_buffer[i] = 0;
        pass_rx_buffer[i] = 0;
        comm_rx_buffer[i] = 0;
        deco_rx_buffer[i] = 0;
    }
    for(int j = 0; j < max_lidar_sample; j++){
        sample_buffer[j][0] = 0;
        sample_buffer[j][1] = 0;
        sample_buffer_timeus[j] = 0;
    }
}

uint16_t RPlidar::get_sample_available(){
    return calc_cola_dist(n_write_sample,n_read_sample,max_lidar_sample);
}

bool RPlidar::add_sample(uint16_t in_angle_q6,uint16_t in_distance_q2,uint32_t in_time_samp){
    if (in_distance_q2 >= min_distance && 
        in_distance_q2 <= max_distance && 
        in_angle_q6 >= 0 &&
        in_angle_q6 <= 23040) 
        {
        sample_buffer[n_write_sample][0] = in_angle_q6;
        sample_buffer[n_write_sample][1] = in_distance_q2;
        sample_buffer_timeus[n_write_sample] = in_time_samp;
        if(get_sample_available() >= max_lidar_sample-1){
            n_read_sample = constrain_circ(n_read_sample+1,0,max_lidar_sample);
        }
        n_write_sample = constrain_circ(n_write_sample+1,0,max_lidar_sample);
        return true;
    }
    return false;
}

bool RPlidar::get_sample(uint16_t *out_angle_q6,uint16_t *out_distance_q2, uint32_t *out_time){
    if(get_sample_available()>0){
        *out_angle_q6    = sample_buffer[n_read_sample][0];
        *out_distance_q2 = sample_buffer[n_read_sample][1];
        *out_time = sample_buffer_timeus[n_read_sample];
        n_read_sample = constrain_circ(n_read_sample+1,0,max_lidar_sample);
        return true;
    }
    else{
        return false;
    }
}

bool RPlidar::wait_lidar_response(uint32_t time_out,uint8_t buffer_target){
    lidar_time_ref = millis();
    while (lidar_port.available() < buffer_target && millis() - lidar_time_ref < time_out)
    {
        delay(1);
    }
    return lidar_port.available() >= buffer_target;
}

void RPlidar::clear_port(){
    while (lidar_port.available()>0){
        rx_c = lidar_port.read();
    }
    
}

bool RPlidar::wait_lidar_response(uint32_t time_out){
    lidar_time_ref = millis();
    while (lidar_port.available() == 0 && millis() - lidar_time_ref < time_out)
    {
        delay(1);
    }
    return lidar_port.available() > 0;
}

bool RPlidar::read_lidar(uint8_t *out_rx_buffer,uint8_t rx_lim, bool synq_byte_en, uint32_t time_out){
    if(time_out > 0){
        wait_lidar_response(time_out,rx_lim);
    }
    while (lidar_port.available() > 0) {
        rx_c = lidar_port.read();
        if (synq_byte_en) {
            if (
                (rx_count == 0 && (rx_c >> 4) == lidar_synq_byte1) ||
                (rx_count == 1 && (rx_c >> 4) == lidar_synq_byte2) ||
                (rx_count > 1)
            ) {
                prev_rx_buffer[rx_count] = rx_c;
                rx_count++;
            }
            else {
                rx_count = 0;
                return false;
            }
        }
        else {
            prev_rx_buffer[rx_count] = rx_c;
            rx_count++;
        }

        if (rx_count == rx_lim) {
            for(int i = 0;i<rx_lim;i++){
                out_rx_buffer[i] = prev_rx_buffer[i];
            }
            rx_count = 0;
            return true;
        }
    }
    return false;
}

bool RPlidar::read_lidar(uint8_t *out_rx_buffer,uint8_t rx_lim, bool synq_byte_en){
    return read_lidar(out_rx_buffer,rx_lim,synq_byte_en,0);
}

bool RPlidar::read_lidar(uint8_t *out_rx_buffer,uint8_t rx_lim){
    return read_lidar(out_rx_buffer,rx_lim,false,0);
}

void RPlidar::read_lidar_scan(){
    if(scan_mode == scan_mode_normal){
        if(read_lidar(deco_rx_buffer,normal_scan_len)){
            deco_rx_buffer_time = micros();
            decode_normal_scan();
        }
    }
    else if (scan_mode == scan_mode_legacy){
        if(read_lidar(deco_rx_buffer,legacy_scan_len,true)){
            deco_rx_buffer_time = micros();
            decode_legacy_scan();
        }
    }
    else if (scan_mode == scan_mode_extended){
        if(read_lidar(deco_rx_buffer,extended_scan_len,true)){
            deco_rx_buffer_time = micros();
            decode_extended_scan();
        }
    }
} 

void RPlidar::stop_scan(){
    set_motor_speed(0);
    lidar_port.write(stop_scan_bytes, 2);
    scan_mode = 0;
    first_sample = false;
    reset_buffers();
}

void RPlidar::reset_lidar(){
    set_motor_speed(0);
    lidar_port.write(reset_lidar_bytes, 2);
    scan_mode = 0;
    first_sample = false;
    reset_buffers();
}

bool RPlidar::star_normal_scan(uint pwm_val){
    clear_port();
    lidar_port.write(star_normal_scan_bytes, 2);

    if(read_lidar(comm_rx_buffer,7, true, command_time_out)){
        if (comm_rx_buffer[2] == 0x05 && comm_rx_buffer[5] == 0x40 && comm_rx_buffer[6] == 0x81) {
            reset_buffers();
            scan_mode = scan_mode_normal;
            set_motor_speed(pwm_val);
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

bool RPlidar::star_force_scan(uint pwm_val){
    clear_port();
    lidar_port.write(star_force_scan_bytes, 2);

    if(read_lidar(comm_rx_buffer,7, true, command_time_out)){
        if (comm_rx_buffer[2] == 0x05 && comm_rx_buffer[5] == 0x40 && comm_rx_buffer[6] == 0x81) {
            reset_buffers();
            scan_mode = scan_mode_normal;
            set_motor_speed(pwm_val);
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

bool RPlidar::star_legacy_scan(uint pwm_val){
    clear_port();
    lidar_port.write(star_legacy_scan_bytes, 9);

    if(read_lidar(comm_rx_buffer,7, true, command_time_out)){
        if (comm_rx_buffer[2] == 0x54 && comm_rx_buffer[5] == 0x40 && comm_rx_buffer[6] == 0x82) {
            reset_buffers();
            first_sample = false;
            scan_mode = scan_mode_legacy;
            set_motor_speed(pwm_val);
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

bool RPlidar::star_extended_scan(uint pwm_val){
    clear_port();
    lidar_port.write(star_extended_scan_bytes, 9);

    if(read_lidar(comm_rx_buffer,7, true, command_time_out)){
        if (comm_rx_buffer[2] == 0x84 && comm_rx_buffer[5] == 0x40 && comm_rx_buffer[6] == 0x84) {
            reset_buffers();
            first_sample = false;
            scan_mode = scan_mode_extended;
            set_motor_speed(pwm_val);
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

void RPlidar::decode_normal_scan(){
    if(first_sample_time){
        angle_q6[0] = (deco_rx_buffer[1] >> 1) | (deco_rx_buffer[2] << 7);
        dist_q2[0] = deco_rx_buffer[3] | (deco_rx_buffer[4] << 8);

        add_sample(angle_q6[0],dist_q2[0],pass_rx_buffer_time1);
    }
    else{
        first_sample_time = true;
    }
    pass_rx_buffer_time1 = deco_rx_buffer_time;
}

void RPlidar::decode_legacy_scan(){
    if(first_sample && first_sample_time){
        currentStartAngle_q8 = (int32_t)(((deco_rx_buffer[3] & 0x7F) << 8) | (deco_rx_buffer[2]))<<2;
        prevStartAngle_q8 = (int32_t)(((pass_rx_buffer[3] & 0x7F) << 8) | (pass_rx_buffer[2]))<<2;

        diff_time_buffer = pass_rx_buffer_time1 - pass_rx_buffer_time2;
        time_buffer_inc = diff_time_buffer/32;
        est_time_sample = pass_rx_buffer_time1;

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        angleInc_q16 = (diffAngle_q8 << 3);
        currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for(uint8_t i = 0;i<16;i++){
            n_base = (i*5)+4;

            dist_q2[0] = (pass_rx_buffer[n_base]>>2) | (pass_rx_buffer[n_base+1]<<6); 
            dist_q2[1] = (pass_rx_buffer[n_base+2]>>2) | (pass_rx_buffer[n_base+3]<<6);

            angle_offset1_q3 = ((pass_rx_buffer[n_base] & 0x03)<<4) | (pass_rx_buffer[n_base+4] & 0x0f);
            angle_offset2_q3 = ((pass_rx_buffer[n_base+2] & 0x03)<<4) | (pass_rx_buffer[n_base+4] >> 4);
        
            angle_q6[0] = ((currentAngle_raw_q16 - (angle_offset1_q3 << 13)) >> 10);
            currentAngle_raw_q16 += angleInc_q16;

            angle_q6[1] = ((currentAngle_raw_q16 - (angle_offset2_q3 << 13)) >> 10);
            currentAngle_raw_q16 += angleInc_q16;

            for (int cpos = 0; cpos < 2; ++cpos) {
                
                time_sample[cpos] = est_time_sample;
                est_time_sample += time_buffer_inc;

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
                if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

                add_sample(angle_q6[cpos],dist_q2[cpos],time_sample[cpos]);
            }
        }
    }
    else{
        if(first_sample){
            first_sample_time = true;
        }
        else{
            first_sample = true;
        }
    }
    for(int i = 0; i<serial_buffer_len;i++){
        pass_rx_buffer[i] = deco_rx_buffer[i];
    }
    pass_rx_buffer_time2 = pass_rx_buffer_time1;
    pass_rx_buffer_time1 = deco_rx_buffer_time;
}

uint32_t RPlidar::_varbitscale_decode(uint32_t scaled, uint32_t & scaleLevel){
    for (size_t i = 0; i < 5; ++i) {
        int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
        if (remain >= 0) {
            scaleLevel = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << scaleLevel);
        }
    }
    return 0;
}

void RPlidar::decode_extended_scan(){
    if(first_sample && first_sample_time){
        currentStartAngle_q8 = (int32_t)(((deco_rx_buffer[3] & 0x7F) << 8) | (deco_rx_buffer[2]))<<2;
        prevStartAngle_q8 = (int32_t)(((pass_rx_buffer[3] & 0x7F) << 8) | (pass_rx_buffer[2]))<<2;

        diff_time_buffer = pass_rx_buffer_time1 - pass_rx_buffer_time2;
        time_buffer_inc = diff_time_buffer/96;
        est_time_sample = pass_rx_buffer_time1;

        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8);
        if (prevStartAngle_q8 > currentStartAngle_q8) {
            diffAngle_q8 += (360 << 8);
        }

        angleInc_q16 = (diffAngle_q8 << 3)/3;
        currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for(uint8_t i = 0;i<32;i++){
            n_base = (i*4)+4;

            //combined_x3 = (pass_rx_buffer[n_base] << 24) | (pass_rx_buffer[n_base+1] << 16) | (pass_rx_buffer[n_base+2] << 8) | pass_rx_buffer[n_base+3];
            combined_x3 = (pass_rx_buffer[n_base]) | (pass_rx_buffer[n_base+1] << 8) | (pass_rx_buffer[n_base+2] << 16) | (pass_rx_buffer[n_base+3]<<24);

            dist_major = (combined_x3 & 0xFFF);
            dist_predict1 = (((int)(combined_x3 << 10)) >> 22);
            dist_predict2 = (((int)combined_x3) >> 22);


            uint32_t scalelvl1,scalelvl2;

            if (i == 31) {
                dist_major2 = ((deco_rx_buffer[6] & 0xf) << 8) | deco_rx_buffer[7];
            }
            else {
                uint8_t n_base_sig = ((i+1)*4)+4;
                dist_major2 = ((pass_rx_buffer[n_base_sig+1] & 0xf) << 8) | (pass_rx_buffer[n_base_sig]);
            }

            dist_major = _varbitscale_decode(dist_major, scalelvl1);
            dist_major2 = _varbitscale_decode(dist_major2, scalelvl2);

            dist_base1 = dist_major;
            dist_base2 = dist_major2;

            if ((!dist_major) && dist_major2) {
                dist_base1 = dist_major2;
                scalelvl1 = scalelvl2;
            }

            dist_q2[0] = (dist_major << 2);
            if ((dist_predict1 == 0xFFFFFE00) || (dist_predict1 == 0x1FF)) {
                dist_q2[1] = 0;
            }
            else {
                dist_predict1 = (dist_predict1 << scalelvl1);
                dist_q2[1] = (dist_predict1 + dist_base1) << 2;

            }

            if ((dist_predict2 == 0xFFFFFE00) || (dist_predict2 == 0x1FF)) {
                dist_q2[2] = 0;
            }
            else {
                dist_predict2 = (dist_predict2 << scalelvl2);
                dist_q2[2] = (dist_predict2 + dist_base2) << 2;
            }

            for (uint8_t cpos = 0; cpos < 3; ++cpos) {

                offsetAngleMean_q16 = (int)(7.5 * 3.1415926535 * (1 << 16) / 180.0);
                //offsetAngleMean_q16 = 8578;

                if (dist_q2[cpos] >= 200){
                    k2 = int(98361 / dist_q2[cpos]);

                    offsetAngleMean_q16 = (int)(8 * 3.1415926535 * (1 << 16) / 180) - (k2 << 6) - (k2 * k2 * k2) / 98304;
                    //offsetAngleMean_q16 = 9150 - (k2 << 6) - (k2 * k2 * k2) / 98304;
                }

                angle_q6[cpos] = ((currentAngle_raw_q16 - int(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10);
                //angle_q6[cpos] = ((currentAngle_raw_q16 - int(offsetAngleMean_q16 * 57)) >> 10);
                currentAngle_raw_q16 += angleInc_q16;

                time_sample[cpos] = est_time_sample;
                est_time_sample += time_buffer_inc;

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360 << 6);
                if (angle_q6[cpos] >= (360 << 6)) angle_q6[cpos] -= (360 << 6);

                add_sample(angle_q6[cpos],dist_q2[cpos],time_sample[cpos]);
            }
        }
    }
    else{
        if(first_sample){
            first_sample_time = true;
        }
        else{
            first_sample = true;
        }
    }
    for(int i = 0; i<serial_buffer_len;i++){
        pass_rx_buffer[i] = deco_rx_buffer[i];
    }
    pass_rx_buffer_time2 = pass_rx_buffer_time1;
    pass_rx_buffer_time1 = deco_rx_buffer_time;
}

void RPlidar::set_motor_speed(uint val){
    if(val == 0){
        digitalWrite(_motor_pin,0);
    }
    else if(val >= 254){
        digitalWrite(_motor_pin,1);
    }
    else {
        analogWrite(_motor_pin, val);
    }
    
}

void RPlidar::begin(uint motor_pin, unsigned long lidar_baudrate){
    _motor_pin = motor_pin;
    pinMode(_motor_pin,OUTPUT);
    digitalWrite(_motor_pin,0);
    lidar_port.begin(lidar_baudrate);
    delay(100);
    stop_scan();
}
