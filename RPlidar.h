#include "Arduino.h"

#define lidar_synq_byte1 0xA
#define lidar_synq_byte2 0x5
#define command_time_out 2000
#define scan_time_out 50

#define scan_mode_normal 1
#define scan_mode_legacy 2
#define scan_mode_extended 3

#define normal_scan_len 5
#define legacy_scan_len 84
#define extended_scan_len 132
#define serial_buffer_len 132


#define max_lidar_sample 8000
#define lidar_port Serial2


const uint8_t stop_scan_bytes[] = {0xA5, 0x25};
const uint8_t reset_lidar_bytes[] = {0xA5, 0x40};

const uint8_t star_normal_scan_bytes[] = {0xA5, 0x20};
const uint8_t star_force_scan_bytes[] = {0xA5, 0x21};
const uint8_t star_legacy_scan_bytes[] = {0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22};
const uint8_t star_extended_scan_bytes[] = {0xA5, 0x82, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x20};

const uint32_t VBS_SCALED_BASE[] = {3328,1792,1280,512,0};
const uint32_t VBS_SCALED_LVL[] = {4,3,2,1,0};
const uint32_t VBS_TARGET_BASE[] = {16384,4096,2048,512,0};



class RPlidar{
    private:
        uint32_t lidar_time_ref;
        uint8_t prev_rx_buffer[serial_buffer_len];
        uint8_t comm_rx_buffer[serial_buffer_len];

        uint8_t pass_rx_buffer[serial_buffer_len];
        uint32_t pass_rx_buffer_time1 = 0;
        uint32_t pass_rx_buffer_time2 = 0;
        
        uint8_t deco_rx_buffer[serial_buffer_len];
        uint32_t deco_rx_buffer_time = 0;

        uint32_t diff_time_buffer = 0;
        uint32_t time_buffer_inc = 0;
        uint32_t est_time_sample = 0;
        int32_t time_sample[3];

        uint16_t rx_count = 0;
        uint8_t rx_c = 0;

        uint16_t sample_buffer[max_lidar_sample][2];
        uint32_t sample_buffer_timeus[max_lidar_sample];
        uint16_t n_write_sample = 0;
        uint16_t n_read_sample = 0;

        uint16_t min_distance = 1200;
        uint16_t max_distance = 48000;
        uint8_t scan_mode = 0;
        bool first_sample = false;
        bool first_sample_time = false;

        //variables para decodificacion de legary y extended
        uint8_t n_base = 0;
        int diffAngle_q8;
        int currentStartAngle_q8;
        int prevStartAngle_q8;
        int angleInc_q16;
        int currentAngle_raw_q16;
        int dist_q2[3];
        int angle_q6[3];

        //variables para decodificacion de legacy
        int32_t angle_offset1_q3;
        int32_t angle_offset2_q3;

        //variables para decodificacion de extended
        uint32_t combined_x3;
        int dist_major;
        int dist_predict1;
        int dist_predict2;

        int dist_major2;
        uint32_t _varbitscale_decode(uint32_t scaled, uint32_t & scaleLevel);
        //uint32_t scalelvl1;
        //uint32_t scalelvl2;

        int dist_base1;
        int dist_base2;

        int offsetAngleMean_q16;
        int k1;
        int k2;

        uint _motor_pin;

        uint16_t constrain_circ(uint16_t val, uint16_t min_val, uint16_t max_val);
        
        void decode_normal_scan();
        void decode_legacy_scan();
        void decode_extended_scan();

        uint16_t calc_cola_dist(uint16_t n_write,uint16_t n_read,uint16_t n_max);
        bool add_sample(uint16_t in_angle_q6,uint16_t in_distance_q2, uint32_t in_time_samp);
        void reset_buffers();
        void clear_port();
    public:
        RPlidar();
        void begin(uint motor_pin, unsigned long lidar_baudrate=115200);
        bool wait_lidar_response(uint32_t time_out);
        bool wait_lidar_response(uint32_t time_out,uint8_t buffer_target);

        bool read_lidar(uint8_t *out_rx_buffer,uint8_t rx_lim, bool synq_byte_en, uint32_t time_out);
        bool read_lidar(uint8_t *out_rx_buffer,uint8_t rx_lim, bool synq_byte_en);
        bool read_lidar(uint8_t *out_rx_buffer,uint8_t rx_lim);
        void read_lidar_scan();

        void stop_scan();
        void reset_lidar();

        bool star_normal_scan(uint pwm_val=255);
        bool star_force_scan(uint pwm_val=255);
        bool star_legacy_scan(uint pwm_val=255);
        bool star_extended_scan(uint pwm_val=255);

        bool get_sample(uint16_t *out_angle_q6,uint16_t *out_distance_q2, uint32_t *out_time);
        uint16_t get_sample_available();
        void set_motor_speed(uint val);
};
