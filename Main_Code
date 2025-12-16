# Behavior-detection-system.C
/*
 * TPK128_System_V13_TimeCalibrated.c
 *
 * [V13 최종 수정 사항]
 * 1. 타이머 속도 정밀 보정 (45초 -> 30초로 단축)
 * - 기존 설정이 45초 걸렸으므로, OCR0 값을 180에서 120으로 줄여 속도를 1.5배 높임
 * - 이제 30,000 카운트가 실제 시간 약 30초와 일치하게 됩니다.
 * * [소리 패턴]
 * - 입장 감지: "띠" (1번)
 * - 내부 활동: "띠띠" (2번)
 * - 30초 경고: "띠띠띠띠" (4번)
 *
 * [기타 유지]
 * - 인터럽트 스케줄러 (센서 충돌 방지)
 * - 감지 거리 10cm
 */

#define F_CPU 14745600L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "tpk_lcd8.h"

/*======================== 사용자 설정 ========================*/
// 감지 거리 (cm)
#define DIST_THRESHOLD_ENTRY   20   
#define DIST_THRESHOLD_CEILING 20   
#define DIST_THRESHOLD_LEFT    10   
#define DIST_THRESHOLD_RIGHT   10   
#define DIST_THRESHOLD_FRONT   10   

// 시간 설정 (30초)
#define TIMEOUT_WARNING_MS     30000UL  
#define BEEP_PIN               PE4
/*============================================================*/

// 핀 매핑
#define TRIG1_PIN   PF0  
#define ECHO1_PIN   PF1
#define TRIG2_PIN   PD2  
#define ECHO2_PIN   PD3
#define TRIG3_PIN   PF6  
#define ECHO3_PIN   PF7
#define TRIG4_PIN   PF2  
#define ECHO4_PIN   PF3
#define TRIG5_PIN   PF4  
#define ECHO5_PIN   PF5

// 전역 변수
volatile uint32_t ms_ticks = 0;
volatile uint32_t system_timer_ms = 0;
volatile uint32_t motion_idle_ms = 0;
volatile bool     system_active = false;

// 인터럽트 스케줄러용 변수
volatile uint8_t  sens_sched_flag = 0; 

/* ================= 초음파 함수 (Port F) ================= */
static inline void trig_pulse_F(uint8_t pin){
    PORTF &= ~(1<<pin); _delay_us(2);
    PORTF |=  (1<<pin); _delay_us(10);
    PORTF &= ~(1<<pin);
}

static uint32_t measure_echo_us_F(uint8_t echo_pin){
    const double tick_us = (8.0 * 1000000.0) / (double)F_CPU; 
    uint32_t timeout = 40000UL;
    
    while ((PINF & (1<<echo_pin)) && --timeout);
    if (!timeout) return 0;
    
    timeout = 40000UL;
    while (!(PINF & (1<<echo_pin)) && --timeout);
    if (!timeout) return 0;
    
    TCCR1A = 0; TCCR1B = (1<<CS11); TCNT1 = 0; 
    timeout = 40000UL;
    
    while ((PINF & (1<<echo_pin)) && --timeout);
    
    uint16_t ticks = TCNT1; 
    TCCR1B = 0;
    
    if (!timeout) return 0;
    return (uint32_t)(ticks * tick_us);
}

static uint16_t measure_distance_F(uint8_t trig, uint8_t echo){
    trig_pulse_F(trig);
    _delay_us(50); 
    uint32_t us = measure_echo_us_F(echo);
    if (us == 0 || us > 35000UL) return 999;
    return (uint16_t)(us / 58UL);
}

/* ================= 초음파 함수 (Port D) ================= */
static inline void trig_pulse_D(uint8_t pin){
    PORTD &= ~(1<<pin); _delay_us(2);
    PORTD |=  (1<<pin); _delay_us(10);
    PORTD &= ~(1<<pin);
}

static uint32_t measure_echo_us_D(uint8_t echo_pin){
    const double tick_us = (8.0 * 1000000.0) / (double)F_CPU; 
    uint32_t timeout = 40000UL;
    
    while ((PIND & (1<<echo_pin)) && --timeout);
    if (!timeout) return 0;
    
    timeout = 40000UL;
    while (!(PIND & (1<<echo_pin)) && --timeout);
    if (!timeout) return 0;
    
    TCCR1A = 0; TCCR1B = (1<<CS11); TCNT1 = 0;
    timeout = 40000UL;
    
    while ((PIND & (1<<echo_pin)) && --timeout);
    
    uint16_t ticks = TCNT1; 
    TCCR1B = 0;
    
    if (!timeout) return 0;
    return (uint32_t)(ticks * tick_us);
}

static uint16_t measure_distance_D(uint8_t trig, uint8_t echo){
    trig_pulse_D(trig);
    _delay_us(50);
    uint32_t us = measure_echo_us_D(echo);
    if (us == 0 || us > 35000UL) return 999;
    return (uint16_t)(us / 58UL);
}

/* ================= 부저 함수 모음 ================= */

// 1. 입장 감지: [띠] (1번)
void play_entry_beep(void){
    DDRE |= (1<<BEEP_PIN);
    for(int i=0; i<200; i++){ 
        PORTE |= (1<<BEEP_PIN); _delay_us(500); 
        PORTE &= ~(1<<BEEP_PIN); _delay_us(500); 
    }
}

// 2. 내부 활동 감지: [띠띠] (2번)
void play_motion_beep(void){
    DDRE |= (1<<BEEP_PIN);
    for(int k=0; k<2; k++) { // 2번 반복
        for(int i=0; i<80; i++){ 
            PORTE |= (1<<BEEP_PIN); _delay_us(500); 
            PORTE &= ~(1<<BEEP_PIN); _delay_us(500); 
        }
        _delay_ms(100); 
    }
}

// 3. 30초 경과 경고: [띠띠띠띠] (4번)
void play_warning_beep(void){
    DDRE |= (1<<BEEP_PIN);
    for(int k=0; k<4; k++) { // 4번 반복
        for(int i=0; i<60; i++){ 
            PORTE |= (1<<BEEP_PIN); _delay_us(400); 
            PORTE &= ~(1<<BEEP_PIN); _delay_us(400); 
        }
        _delay_ms(80); 
    }
}

/* ---- Timer0 ISR (시스템 시간 & 센서 스케줄링) ---- */
ISR(TIMER0_COMP_vect){
    ms_ticks++;
    
    if (system_active) { 
        system_timer_ms++; 
        motion_idle_ms++;
    }

    // 센서 스케줄러
    static uint8_t sensor_timer = 0;
    static uint8_t sensor_seq = 0;

    sensor_timer++;
    // 타이머 속도에 맞춰 스케줄링 간격도 조정 (약 60ms 주기 유지)
    if(sensor_timer >= 60) { 
        sensor_timer = 0;
        
        if(sensor_seq == 0)      sensor_sched_flag = 3; 
        else if(sensor_seq == 1) sensor_sched_flag = 4; 
        else if(sensor_seq == 2) sensor_sched_flag = 5; 
        
        sensor_seq++;
        if(sensor_seq > 2) sensor_seq = 0;
    }
}

/* ---- 초기화 함수 (타이머 재보정됨) ---- */
static void io_init(void){
    // JTAG 비활성화
    MCUCSR |= (1 << JTD);
    MCUCSR |= (1 << JTD); 

    // Port 설정
    DDRF |=  (1<<TRIG1_PIN)|(1<<TRIG3_PIN)|(1<<TRIG4_PIN)|(1<<TRIG5_PIN);
    DDRF &= ~((1<<ECHO1_PIN)|(1<<ECHO3_PIN)|(1<<ECHO4_PIN)|(1<<ECHO5_PIN));
    PORTF = 0x00;
    
    DDRD |=  (1<<TRIG2_PIN);
    DDRD &= ~(1<<ECHO2_PIN);
    PORTD &= ~(1<<ECHO2_PIN);
    
    DDRE |= (1<<BEEP_PIN);
    PORTE &= ~(1<<BEEP_PIN);

    // [타이머 속도 재설정]
    // 기존 설정(OCR=180)에서 45초 걸렸으므로, 30초로 맞추기 위해 속도를 1.5배 높임
    // New OCR = 180 * (30/45) = 120
    // Prescaler 1024는 유지 (안정성 위해)
    
    TCCR0 = (1<<WGM01) | (1<<CS02) | (1<<CS00); // CTC Mode, Prescaler 1024
    OCR0 = 120; // 이 값으로 1ms tick에 근접하게 보정
    TIMSK |= (1<<OCIE0);
}

/*============================ 메인 ============================*/
int main(void){
    cli();
    io_init();
    lcd_init();
    sei();

    char ent_str[17];
    char motion_str[17];

    bool us1_last_state = false;
    bool inside_last_state = false;

    uint16_t d1 = 999, d2 = 999, d3 = 999, d4 = 999, d5 = 999;

    while(1){
        // 1. 입구 센서 (US1)
        d1 = measure_distance_F(TRIG1_PIN, ECHO1_PIN); 
        
        // 2. 내부 센서 스케줄링
        if(system_active) {
            _delay_ms(20); 
            d2 = measure_distance_D(TRIG2_PIN, ECHO2_PIN);

            if (sensor_sched_flag == 3) {
                d3 = measure_distance_F(TRIG3_PIN, ECHO3_PIN);
                sensor_sched_flag = 0; 
            }
            else if (sensor_sched_flag == 4) {
                d4 = measure_distance_F(TRIG4_PIN, ECHO4_PIN);
                sensor_sched_flag = 0;
            }
            else if (sensor_sched_flag == 5) {
                d5 = measure_distance_F(TRIG5_PIN, ECHO5_PIN);
                sensor_sched_flag = 0;
            }
        } else {
            d2=999; d3=999; d4=999; d5=999;
        }

        // 3. 감지 판별
        bool det_entry = (d1 < DIST_THRESHOLD_ENTRY && d1 != 999);
        
        bool det_ceiling = (d2 < DIST_THRESHOLD_CEILING && d2 != 999);
        bool det_left    = (d3 < DIST_THRESHOLD_LEFT && d3 != 999);  
        bool det_right   = (d4 < DIST_THRESHOLD_RIGHT && d4 != 999); 
        bool det_front   = (d5 < DIST_THRESHOLD_FRONT && d5 != 999); 
        
        bool det_inside = det_ceiling || det_left || det_right || det_front;

        // 4. 로직 처리
        
        // [입장 감지] -> "띠" (1번)
        if(det_entry && !us1_last_state){
            play_entry_beep();  
            if(system_active){
                system_active = false;  
            } else {
                system_active = true;   
                system_timer_ms = 0;
                motion_idle_ms = 0;
                d3=999; d4=999; d5=999;
            }
        }
        us1_last_state = det_entry;

        // [내부 활동 감지] -> "띠띠" (2번)
        if(system_active && det_inside){
            cli();
            motion_idle_ms = 0;  // 타이머 리셋
            sei();
            
            if(!inside_last_state) {
                play_motion_beep();  // 2번 울림
            }
        }
        inside_last_state = det_inside;

        // [30초 경과 경고] -> "띠띠띠띠" (4번)
        uint32_t idle_ms;
        cli(); idle_ms = motion_idle_ms; sei();
        
        if(system_active && idle_ms >= TIMEOUT_WARNING_MS){
            play_warning_beep();  // 4번 울림
            _delay_ms(200);       // 반복 간격
        }

        // 5. LCD 표시
        uint32_t sys_ms;
        bool running;
        cli(); sys_ms = system_timer_ms; running = system_active; sei();

        char warning_mark = (idle_ms >= TIMEOUT_WARNING_MS && running) ? '!' : ' ';

        sprintf(ent_str, "T1:%lu.%01lus %s%c",
            sys_ms / 1000, (sys_ms % 1000) / 100, 
            running ? "ON " : "OFF",
            warning_mark);
        lcd_display_position(1, 1);
        lcd_string(ent_str);

        sprintf(motion_str, "T2:%lu.%01lus      ",
            idle_ms / 1000, (idle_ms % 1000) / 100);
        lcd_display_position(2, 1);
        lcd_string(motion_str);
    }
}
