#include <stdint.h>
#include "stm32l053xx.h"

/* ===== LCD pins ===== */
#define LCD_RS_PORT GPIOA
#define LCD_RS_PIN  0u
#define LCD_E_PORT  GPIOA
#define LCD_E_PIN   1u
#define LCD_D4_PORT GPIOA
#define LCD_D4_PIN  8u
#define LCD_D5_PORT GPIOA
#define LCD_D5_PIN  10u
#define LCD_D6_PORT GPIOA
#define LCD_D6_PIN  5u
#define LCD_D7_PORT GPIOA
#define LCD_D7_PIN  6u

/* ===== Semáforo LEDs ===== */
#define LED_ROJO_PORT  GPIOC
#define LED_ROJO_PIN   0u
#define LED_AMAR_PORT  GPIOC
#define LED_AMAR_PIN   1u
#define LED_VERDE_PORT GPIOC
#define LED_VERDE_PIN  2u

/* ===== Buzzer PA9 (activo) ===== */
#define BUZ_PORT GPIOA
#define BUZ_PIN  9u
#define BUZ_ON()  (BUZ_PORT->BSRR = (1u<<BUZ_PIN))
#define BUZ_OFF() (BUZ_PORT->BSRR = (1u<<(BUZ_PIN+16)))

/* ===== Botones inicio ===== */
#define BTN_TREN_PORT  GPIOA  /* PA4 */
#define BTN_TREN_PIN   4u
#define BTN_PEAT_PORT  GPIOA  /* PA12 */
#define BTN_PEAT_PIN   12u

/* ===== 7-seg: PB0..PB7, dígitos PC5/6/8/9 ===== */
#define SEG_PORT  GPIOB
#define DIG_PORT  GPIOC
#define DIG0_PIN  5u
#define DIG1_PIN  6u
#define DIG2_PIN  8u
#define DIG3_PIN  9u

/* ===== USART2 (PA2/PA3 AF4) ===== */
#define UART_PORT   GPIOA
#define UART_TX_PIN 2u
#define UART_RX_PIN 3u
#define USART_BRR_9600_16MHZ 0x0683u

/* ===== Keypad (PB8..PB11 filas, PB12..PB15 cols) =====
   PB12: A B C D
   PB13: 3 6 9 #
   PB14: 2 5 8 0
   PB15: 1 4 7 * */
#define KP_PORT   GPIOB
#define COL0_PIN  12u
#define COL1_PIN  13u
#define COL2_PIN  14u
#define COL3_PIN  15u

/* ===== Stepper (28BYJ-48) PC3/PC4/PC7/PC11 ===== */
#define STP_PORT GPIOC
#define STP_IN1  3u
#define STP_IN2  4u
#define STP_IN3  7u
#define STP_IN4  11u

/* ===== Helpers ===== */
#define GPIO_SET(p,n)  ((p)->BSRR = (1u<<(n)))
#define GPIO_CLR(p,n)  ((p)->BSRR = (1u<<((n)+16u)))

/* ==== Prototipos ==== */
static void Clocks_Init(void);
static void GPIO_Init(void);
static void EXTI_Init(void);
static void TIM21_Init_1kHz(void);
static void TIM22_Init_2kHz(void);
static void USART2_Init(void);
static void USART2_SendString(const char* s);
static void mantto_show_menu(void);

/* ====================== LCD asíncrona ====================== */
typedef enum {LCDI_PWRWAIT=0,LCDI_FN0,LCDI_FN1,LCDI_FN2,LCDI_SET4,LCDI_FUNCSET,LCDI_DISPON,LCDI_CLEAR,LCDI_ENTRY,LCDI_READY} lcd_init_st_t;
static volatile lcd_init_st_t lcd_init_st = LCDI_PWRWAIT;
static volatile uint16_t lcd_wait_ms=0;
#define LCDQ_SIZE 128
static volatile uint8_t lcdq_data[LCDQ_SIZE], lcdq_isdata[LCDQ_SIZE];
static volatile uint8_t lcdq_head=0, lcdq_tail=0, lcd_sending=0, lcd_send_isdata=0, lcd_send_byte=0, lcd_phase=0;

static void lcd_gpio_put_nibble(uint8_t n){
  GPIO_CLR(LCD_D4_PORT,LCD_D4_PIN); GPIO_CLR(LCD_D5_PORT,LCD_D5_PIN);
  GPIO_CLR(LCD_D6_PORT,LCD_D6_PIN); GPIO_CLR(LCD_D7_PORT,LCD_D7_PIN);
  if(n&1) GPIO_SET(LCD_D4_PORT,LCD_D4_PIN);
  if(n&2) GPIO_SET(LCD_D5_PORT,LCD_D5_PIN);
  if(n&4) GPIO_SET(LCD_D6_PORT,LCD_D6_PIN);
  if(n&8) GPIO_SET(LCD_D7_PORT,LCD_D7_PIN);
}
static void lcd_enqueue_cmd(uint8_t c){ uint8_t n=(lcdq_head+1)&(LCDQ_SIZE-1); if(n!=lcdq_tail){ lcdq_data[lcdq_head]=c; lcdq_isdata[lcdq_head]=0; lcdq_head=n; } }
static void lcd_enqueue_data(uint8_t d){ uint8_t n=(lcdq_head+1)&(LCDQ_SIZE-1); if(n!=lcdq_tail){ lcdq_data[lcdq_head]=d; lcdq_isdata[lcdq_head]=1; lcdq_head=n; } }
static void lcd_enqueue_text_fixed16(const char* s){ for(uint8_t i=0;i<16;i++) lcd_enqueue_data( (s&&s[i])?(uint8_t)s[i]:' ' ); }
static void lcd_clear_home(void){ lcd_enqueue_cmd(0x01); lcd_enqueue_cmd(0x80); }
static void lcd_set_cursor_nb(uint8_t r,uint8_t c){ lcd_enqueue_cmd(0x80 | ((r?0x40:0x00) + (c&0x0F))); }
static void lcd_show_two(const char* l1,const char* l2){ lcd_clear_home(); lcd_enqueue_text_fixed16(l1); lcd_set_cursor_nb(1,0); lcd_enqueue_text_fixed16(l2); }
static void lcd_start_init(void){
  lcd_init_st=LCDI_PWRWAIT; lcd_wait_ms=20u; lcd_sending=0; lcd_phase=0;
  GPIO_CLR(LCD_RS_PORT,LCD_RS_PIN); GPIO_CLR(LCD_E_PORT,LCD_E_PIN); lcd_gpio_put_nibble(0);
}
static void lcd_service_1ms(void){
  if(lcd_wait_ms){ lcd_wait_ms--; return; }
  if(lcd_init_st!=LCDI_READY){
    switch(lcd_init_st){
      case LCDI_PWRWAIT: GPIO_CLR(LCD_RS_PORT,LCD_RS_PIN); lcd_gpio_put_nibble(0x3); GPIO_SET(LCD_E_PORT,LCD_E_PIN); lcd_wait_ms=1; lcd_init_st=LCDI_FN0; break;
      case LCDI_FN0:     GPIO_CLR(LCD_E_PORT,LCD_E_PIN); lcd_wait_ms=5; lcd_init_st=LCDI_FN1; break;
      case LCDI_FN1:     lcd_gpio_put_nibble(0x3); GPIO_SET(LCD_E_PORT,LCD_E_PIN); lcd_wait_ms=1; lcd_init_st=LCDI_FN2; break;
      case LCDI_FN2:     GPIO_CLR(LCD_E_PORT,LCD_E_PIN); lcd_wait_ms=5; lcd_init_st=LCDI_SET4; break;
      case LCDI_SET4:    lcd_gpio_put_nibble(0x2); GPIO_SET(LCD_E_PORT,LCD_E_PIN); lcd_wait_ms=1; GPIO_CLR(LCD_E_PORT,LCD_E_PIN); lcd_wait_ms=5; lcd_init_st=LCDI_FUNCSET; break;
      case LCDI_FUNCSET: lcd_enqueue_cmd(0x28); lcd_init_st=LCDI_DISPON; break;
      case LCDI_DISPON:  lcd_enqueue_cmd(0x0C); lcd_init_st=LCDI_CLEAR; break;
      case LCDI_CLEAR:   lcd_enqueue_cmd(0x01); lcd_wait_ms=2; lcd_init_st=LCDI_ENTRY; break;
      case LCDI_ENTRY:   lcd_enqueue_cmd(0x06); lcd_init_st=LCDI_READY; break;
      default: break;
    }
  }
  if(lcd_sending){
    switch(lcd_phase){
      case 0: if(lcd_send_isdata) GPIO_SET(LCD_RS_PORT,LCD_RS_PIN); else GPIO_CLR(LCD_RS_PORT,LCD_RS_PIN);
              lcd_gpio_put_nibble((lcd_send_byte>>4)&0xF); GPIO_SET(LCD_E_PORT,LCD_E_PIN); lcd_wait_ms=1; lcd_phase=1; break;
      case 1: GPIO_CLR(LCD_E_PORT,LCD_E_PIN); lcd_wait_ms=1; lcd_phase=2; break;
      case 2: lcd_gpio_put_nibble(lcd_send_byte&0xF); GPIO_SET(LCD_E_PORT,LCD_E_PIN); lcd_wait_ms=1; lcd_phase=3; break;
      case 3: GPIO_CLR(LCD_E_PORT,LCD_E_PIN); lcd_wait_ms= (!lcd_send_isdata && ((lcd_send_byte==0x01)||(lcd_send_byte==0x02)))?2:1; lcd_sending=0; lcd_phase=0; break;
    } return;
  }
  if(lcdq_tail!=lcdq_head){ lcd_send_byte=lcdq_data[lcdq_tail]; lcd_send_isdata=lcdq_isdata[lcdq_tail]; lcdq_tail=(lcdq_tail+1)&(LCDQ_SIZE-1); lcd_sending=1; lcd_phase=0; }
}

/* ====================== 7 segmentos ====================== */
static const uint8_t seg_lut[10]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
static volatile uint8_t seg_digits[4]={0,0,0,0}, mux_idx=0;
static inline void seg_set_pattern(uint8_t p){ SEG_PORT->BSRR=(0xFFu<<16); for(uint8_t i=0;i<8;i++) if(p&(1<<i)) SEG_PORT->BSRR=(1u<<i); }
static inline void digits_all_off(void){ GPIO_CLR(DIG_PORT,DIG0_PIN); GPIO_CLR(DIG_PORT,DIG1_PIN); GPIO_CLR(DIG_PORT,DIG2_PIN); GPIO_CLR(DIG_PORT,DIG3_PIN); }
static inline void seg_show_mmss(uint16_t s){ uint8_t mm=s/60u, ss=s%60u; seg_digits[3]=mm/10u; seg_digits[2]=mm%10u; seg_digits[1]=ss/10u; seg_digits[0]=ss%10u; }

/* ====================== UART (TX cola) ====================== */
#define UTXQ_SIZE 128
static volatile uint8_t utxq[UTXQ_SIZE]; static volatile uint8_t utx_head=0, utx_tail=0;
static inline void _usart2_tx_kick(void){ if((USART2->CR1 & USART_CR1_TXEIE)==0){ if(utx_tail!=utx_head) USART2->CR1|=USART_CR1_TXEIE; } }
static void USART2_SendString(const char* s){ while(*s){ uint8_t n=(utx_head+1)&(UTXQ_SIZE-1); if(n==utx_tail) break; utxq[utx_head]=(uint8_t)*s++; utx_head=n; } _usart2_tx_kick(); }
static void UART_SendLabelTime(const char* label,uint16_t secs){
  USART2_SendString(label); USART2_SendString(": ");
  char t[6]; uint8_t mm=secs/60u, ss=secs%60u;
  t[0]='0'+mm/10; t[1]='0'+mm%10; t[2]=':'; t[3]='0'+ss/10; t[4]='0'+ss%10; t[5]='\0';
  USART2_SendString(t); USART2_SendString("\r\n");
}

/* ====================== Buzzer pattern ====================== */
typedef enum {BUZ_MODE_OFF=0, BUZ_MODE_STEADY, BUZ_MODE_BEEP} buz_mode_t;
static volatile buz_mode_t g_buz_mode = BUZ_MODE_OFF;
static volatile uint16_t g_buz_timer_ms = 0;
static volatile uint8_t  g_buz_phase = 0; /* 0=ON phase, 1=OFF phase */
#define BEEP_ON_MS   200u
#define BEEP_OFF_MS  200u
static inline void buz_set_mode(buz_mode_t m){
  g_buz_mode = m; g_buz_timer_ms = 0; g_buz_phase = 0;
  if(m==BUZ_MODE_OFF){ BUZ_OFF(); }
  else if(m==BUZ_MODE_STEADY){ BUZ_ON(); }
  else { BUZ_ON(); } /* start beep with ON */
}
static inline void buz_service_1ms(void){
  if(g_buz_mode==BUZ_MODE_OFF){ BUZ_OFF(); return; }
  if(g_buz_mode==BUZ_MODE_STEADY){ BUZ_ON(); return; }
  /* BEEP pattern */
  uint16_t limit = g_buz_phase ? BEEP_OFF_MS : BEEP_ON_MS;
  if(++g_buz_timer_ms >= limit){
    g_buz_timer_ms = 0;
    g_buz_phase ^= 1u;
    if(g_buz_phase) BUZ_OFF(); else BUZ_ON();
  }
}

/* ====================== FSM y mensajes ====================== */
typedef enum {MODE_TREN=0, MODE_PEATON} sem_mode_t;
typedef enum {
  S_VERDE=0, S_AMARILLO, S_ROJO,
  S_CONFIG_TREN,
  S_CONFIG_PEATON,
  S_CONFIG_PEATON_BUZ,
  S_CONFIG_PEATON_BAR,
  S_MSG,
  S_MANTTO
} sem_state_t;

typedef enum {LCD_MSG_NONE=0, LCD_MSG_LIBRE, LCD_MSG_PRECAUCION, LCD_MSG_TREN, LCD_MSG_PEATON, LCD_MSG_MANTTO} lcd_msg_t;

static volatile sem_state_t g_state=S_VERDE;
static volatile sem_state_t g_prev_state=S_VERDE;
static volatile sem_mode_t g_mode=MODE_TREN;
static volatile uint16_t g_blink_div_ms=0, g_div_1s_ms=0, g_msg_timer_ms=0;
static volatile uint32_t g_tmr_yellow_ms=0, g_tmr_red_ms=0;
static volatile lcd_msg_t g_lcd_msg=LCD_MSG_NONE;

/* tiempos configurables */
static volatile uint16_t g_tren_segundos=30, g_peaton_segundos=20;
/* flags de PEATÓN configurables en menú */
static volatile uint8_t g_peaton_buzzer_en=0;
static volatile uint8_t g_peaton_barrier_en=0;

/* buffer edición MM:SS */
static uint8_t buf[4]={0,0,3,0}, buf_len=2;
static inline void buf_clear(void){ buf_len=0; for(int i=0;i<4;i++) buf[i]=0; }
static inline void buf_push_digit(uint8_t d){ if(buf_len<4) buf[buf_len++]=d; else { buf[0]=buf[1]; buf[1]=buf[2]; buf[2]=buf[3]; buf[3]=d; buf_len=4; } }
static inline void buf_to_mmss(uint8_t* mm,uint8_t* ss){ uint8_t t[4]={0,0,0,0}; for(int i=0;i<buf_len && i<4;i++) t[4-buf_len+i]=buf[i]; *mm=(uint8_t)(t[0]*10+t[1]); *ss=(uint8_t)(t[2]*10+t[3]); }
static void lcd_show_config_value(const char* titulo){
  char l2[17]; for(int i=0;i<16;i++) l2[i]=' ';
  uint8_t t[4]={0,0,0,0}; for(int i=0;i<buf_len && i<4;i++) t[4-buf_len+i]=buf[i];
  l2[0]='0'+t[0]; l2[1]='0'+t[1]; l2[2]=':'; l2[3]='0'+t[2]; l2[4]='0'+t[3]; l2[16]='\0';
  lcd_set_cursor_nb(0,0); lcd_enqueue_text_fixed16(titulo);
  lcd_set_cursor_nb(1,0); lcd_enqueue_text_fixed16(l2);
}
static void lcd_show_peaton_buzzer_menu(void){
  lcd_show_two("Peaton: Buzzer","0:Off   1:On   #OK");
}
static void lcd_show_peaton_bar_menu(void){
  lcd_show_two("Peaton: Barrera","0:No    1:Si   #OK");
}

/* ===== Mantto menu ===== */
static void mantto_show_menu(void){
  lcd_show_two("MANTTO: Barrera","0:Abajo 1:Arr   C:Salir");
}

/* ====================== Stepper ====================== */
#define STEPS_PER_REV 2048u
#define STEPS_90 (STEPS_PER_REV/4u)
static const uint8_t seq8[8]={0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001};
static volatile int32_t stp_target_steps=0; static volatile uint8_t stp_idx=0; static volatile uint16_t stp_div=0;
static inline void stp_apply(uint8_t p){
  if(p&1) GPIO_SET(STP_PORT,STP_IN1); else GPIO_CLR(STP_PORT,STP_IN1);
  if(p&2) GPIO_SET(STP_PORT,STP_IN2); else GPIO_CLR(STP_PORT,STP_IN2);
  if(p&4) GPIO_SET(STP_PORT,STP_IN3); else GPIO_CLR(STP_PORT,STP_IN3);
  if(p&8) GPIO_SET(STP_PORT,STP_IN4); else GPIO_CLR(STP_PORT,STP_IN4);
}
static void stp_service_1ms(void){
  if(!stp_target_steps) return;
  if(++stp_div<3) return; stp_div=0;
  if(stp_target_steps>0){ stp_idx=(uint8_t)((stp_idx+1)&7); stp_target_steps--; }
  else { stp_idx=(uint8_t)((stp_idx+7)&7); stp_target_steps++; }
  stp_apply(seq8[stp_idx]);
}
static inline void stp_right_90(void){ stp_target_steps += STEPS_90; }   /* subir talanquera */
static inline void stp_back_home_90(void){ stp_target_steps -= STEPS_90; }/* bajar talanquera */

/* ====================== Keypad (IRQ+TIM21) ====================== */
#define DEBOUNCE_MS 20u
#define RELEASE_MS  20u
typedef enum {K_IDLE=0,K_DEB_PRESS,K_HELD} kstate_t;
static volatile kstate_t k_state=K_IDLE; static volatile int8_t k_row=-1,k_col=-1; static volatile uint16_t k_cnt=0; static volatile uint8_t g_scan_col=0;
static inline uint8_t read_rows_mask(void){ return (uint8_t)((KP_PORT->IDR>>8)&0x0F); }
static inline void cols_all_hi(void){ KP_PORT->BSRR=(1u<<COL0_PIN)|(1u<<COL1_PIN)|(1u<<COL2_PIN)|(1u<<COL3_PIN); }
static inline void col_drive_low(uint8_t c){
  cols_all_hi();
  if(c==0) KP_PORT->BSRR=(1u<<(COL0_PIN+16));
  else if(c==1) KP_PORT->BSRR=(1u<<(COL1_PIN+16));
  else if(c==2) KP_PORT->BSRR=(1u<<(COL2_PIN+16));
  else          KP_PORT->BSRR=(1u<<(COL3_PIN+16));
}
static inline char map_key_from_rowcol(uint8_t r,uint8_t c){
  static const char c0[4]={'A','B','C','D'}, c1[4]={'3','6','9','#'}, c2[4]={'2','5','8','0'}, c3[4]={'1','4','7','*'};
  return (c==0)?c0[r]:(c==1)?c1[r]:(c==2)?c2[r]:c3[r];
}

/* ====================== Botones flags ====================== */
static volatile uint8_t g_btn_tren=0, g_btn_peaton=0;

/* ====================== FSM transitions ====================== */
static void LCD_Request(lcd_msg_t m){ g_lcd_msg=m; }
static void UART_Announce(sem_state_t st){
  if(st==S_VERDE) USART2_SendString("VERDE\r\n");
  else if(st==S_AMARILLO) USART2_SendString("AMARILLO\r\n");
  else if(st==S_ROJO) USART2_SendString("ROJO\r\n");
  else if(st==S_MANTTO) USART2_SendString("MANTENIMIENTO\r\n");
}
static void Semaforo_Set(sem_state_t st){
  g_prev_state = g_state;
  g_state = st;

  switch(st){
    case S_VERDE:
      GPIO_SET(LED_VERDE_PORT,LED_VERDE_PIN);
      GPIO_CLR(LED_AMAR_PORT,LED_AMAR_PIN);
      GPIO_CLR(LED_ROJO_PORT,LED_ROJO_PIN);
      buz_set_mode(BUZ_MODE_OFF);
      if(g_mode==MODE_TREN){ stp_right_90(); }
      else if(g_mode==MODE_PEATON && g_peaton_barrier_en){ stp_right_90(); }
      seg_digits[0]=seg_digits[1]=seg_digits[2]=seg_digits[3]=0; digits_all_off();
      LCD_Request(LCD_MSG_LIBRE); UART_Announce(S_VERDE);
    break;

    case S_AMARILLO:
      GPIO_CLR(LED_VERDE_PORT,LED_VERDE_PIN);
      GPIO_CLR(LED_ROJO_PORT,LED_ROJO_PIN);
      GPIO_SET(LED_AMAR_PORT,LED_AMAR_PIN);
      g_tmr_yellow_ms=3000u; g_blink_div_ms=0u;

      if(g_mode==MODE_TREN){
        buz_set_mode(BUZ_MODE_STEADY);
        stp_back_home_90();
        LCD_Request(LCD_MSG_PRECAUCION);
      } else {
        buz_set_mode(g_peaton_buzzer_en ? BUZ_MODE_STEADY : BUZ_MODE_OFF);
        if(g_peaton_barrier_en) stp_back_home_90();
        LCD_Request(LCD_MSG_PRECAUCION);
      }
      UART_Announce(S_AMARILLO);
    break;

    case S_ROJO: {
      GPIO_CLR(LED_VERDE_PORT,LED_VERDE_PIN);
      GPIO_CLR(LED_AMAR_PORT,LED_AMAR_PIN);
      GPIO_SET(LED_ROJO_PORT,LED_ROJO_PIN);
      g_div_1s_ms=0;
      uint16_t secs=(g_mode==MODE_TREN)?g_tren_segundos:g_peaton_segundos;
      g_tmr_red_ms=(uint32_t)secs*1000u; seg_show_mmss(secs);

      if(g_mode==MODE_TREN){
        buz_set_mode(BUZ_MODE_BEEP);            /* pi-silencio-pi... */
        LCD_Request(LCD_MSG_TREN);
      } else {
        buz_set_mode(g_peaton_buzzer_en ? BUZ_MODE_STEADY : BUZ_MODE_OFF);
        LCD_Request(LCD_MSG_PEATON);
      }
      UART_Announce(S_ROJO);
    } break;

    case S_CONFIG_TREN: {
      buz_set_mode(BUZ_MODE_OFF);
      uint16_t s=g_tren_segundos; buf[0]=(s/60)/10; buf[1]=(s/60)%10; buf[2]=(s%60)/10; buf[3]=(s%60)%10; buf_len=4;
      lcd_show_two("Tiempo Tren (MM:SS)","               ");
      lcd_show_config_value("Tiempo Tren (MM:SS)");
      USART2_SendString("MENU: Tren\r\n");
    } break;

    case S_CONFIG_PEATON: {
      buz_set_mode(BUZ_MODE_OFF);
      uint16_t s=g_peaton_segundos; buf[0]=(s/60)/10; buf[1]=(s/60)%10; buf[2]=(s%60)/10; buf[3]=(s%60)%10; buf_len=4;
      lcd_show_two("Peaton: Tiempo","MM:SS  (#=OK)");
      lcd_show_config_value("Peaton: Tiempo");
      USART2_SendString("MENU: Peaton (1/3) Tiempo\r\n");
    } break;

    case S_CONFIG_PEATON_BUZ:
      buz_set_mode(BUZ_MODE_OFF); lcd_show_peaton_buzzer_menu();
      USART2_SendString("MENU: Peaton (2/3) Buzzer 0/1\r\n");
    break;

    case S_CONFIG_PEATON_BAR:
      buz_set_mode(BUZ_MODE_OFF); lcd_show_peaton_bar_menu();
      USART2_SendString("MENU: Peaton (3/3) Barrera 0/1\r\n");
    break;

    case S_MSG:
      buz_set_mode(BUZ_MODE_OFF);
    break;

    case S_MANTTO:
      GPIO_CLR(LED_VERDE_PORT,LED_VERDE_PIN);
      GPIO_CLR(LED_ROJO_PORT,LED_ROJO_PIN);
      GPIO_CLR(LED_AMAR_PORT,LED_AMAR_PIN);
      buz_set_mode(BUZ_MODE_OFF);
      g_blink_div_ms = 0u;
      if(g_prev_state != S_VERDE){ stp_right_90(); } /* levantar por seguridad */
      seg_digits[0]=seg_digits[1]=seg_digits[2]=seg_digits[3]=0;
      digits_all_off();
      g_tmr_yellow_ms = 0u;
      g_tmr_red_ms    = 0u;
      LCD_Request(LCD_MSG_MANTTO);
      UART_Announce(S_MANTTO);
    break;
  }
}

/* ====================== Init HW ====================== */
static void Clocks_Init(void){
  RCC->CR |= (1u<<0); RCC->CFGR |= (1u<<0);
  RCC->IOPENR |= (1u<<0)|(1u<<1)|(1u<<2);
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN|RCC_APB2ENR_TIM21EN|RCC_APB2ENR_TIM22EN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
}
static void GPIO_Init(void){
  /* LCD */
  GPIOA->MODER &= ~((3u<<(LCD_RS_PIN*2))|(3u<<(LCD_E_PIN*2))|(3u<<(LCD_D4_PIN*2))|(3u<<(LCD_D5_PIN*2))|(3u<<(LCD_D6_PIN*2))|(3u<<(LCD_D7_PIN*2)));
  GPIOA->MODER |=  ((1u<<(LCD_RS_PIN*2))|(1u<<(LCD_E_PIN*2))|(1u<<(LCD_D4_PIN*2))|(1u<<(LCD_D5_PIN*2))|(1u<<(LCD_D6_PIN*2))|(1u<<(LCD_D7_PIN*2)));
  /* LEDs */
  GPIOC->MODER &= ~((3u<<(LED_ROJO_PIN*2))|(3u<<(LED_AMAR_PIN*2))|(3u<<(LED_VERDE_PIN*2)));
  GPIOC->MODER |=  ((1u<<(LED_ROJO_PIN*2))|(1u<<(LED_AMAR_PIN*2))|(1u<<(LED_VERDE_PIN*2)));
  /* Buzzer */
  GPIOA->MODER &= ~(3u<<(BUZ_PIN*2)); GPIOA->MODER |= (1u<<(BUZ_PIN*2)); BUZ_OFF();
  /* Botones PA4/PA12 */
  GPIOA->MODER &= ~(3u<<(BTN_TREN_PIN*2));
  GPIOA->MODER &= ~(3u<<(BTN_PEAT_PIN*2));
  /* 7-seg PB0..PB7 */
  for(uint8_t i=0;i<8;i++){ SEG_PORT->MODER &= ~(3u<<(i*2)); SEG_PORT->MODER |= (1u<<(i*2)); }
  /* dígitos PC5/6/8/9 */
  uint8_t dp[4]={DIG0_PIN,DIG1_PIN,DIG2_PIN,DIG3_PIN};
  for(int i=0;i<4;i++){ DIG_PORT->MODER &= ~(3u<<(dp[i]*2)); DIG_PORT->MODER |= (1u<<(dp[i]*2)); }
  SEG_PORT->BSRR=(0xFFu<<16); digits_all_off();
  /* USART2 AF4 */
  UART_PORT->MODER &= ~(3u<<(UART_TX_PIN*2)); UART_PORT->MODER |= (2u<<(UART_TX_PIN*2));
  UART_PORT->MODER &= ~(3u<<(UART_RX_PIN*2)); UART_PORT->MODER |= (2u<<(UART_RX_PIN*2));
  UART_PORT->AFR[0] &= ~(0xFu<<(UART_TX_PIN*4)); UART_PORT->AFR[0] |= (4u<<(UART_TX_PIN*4));
  UART_PORT->AFR[0] &= ~(0xFu<<(UART_RX_PIN*4)); UART_PORT->AFR[0] |= (4u<<(UART_RX_PIN*4));
  /* Keypad filas PB8..PB11 + pull-up; cols PB12..PB15 salida=1 */
  for(uint8_t p=8;p<=11;p++){ KP_PORT->MODER &= ~(3u<<(p*2)); KP_PORT->PUPDR &= ~(3u<<(p*2)); KP_PORT->PUPDR |= (1u<<(p*2)); }
  for(uint8_t p=12;p<=15;p++){ KP_PORT->MODER &= ~(3u<<(p*2)); KP_PORT->MODER |= (1u<<(p*2)); }
  cols_all_hi();
  /* Stepper PC3/4/7/11 */
  uint8_t sp[4]={STP_IN1,STP_IN2,STP_IN3,STP_IN4};
  for(int i=0;i<4;i++){ STP_PORT->MODER &= ~(3u<<(sp[i]*2)); STP_PORT->MODER |= (1u<<(sp[i]*2)); }
  stp_apply(0);
}
static void EXTI_Init(void){
  /* PA4 y PA12 */
  SYSCFG->EXTICR[1] &= ~(0xFu<<0); SYSCFG->EXTICR[1] |= (0u<<0); EXTI->IMR|=(1u<<4); EXTI->FTSR|=(1u<<4);
  SYSCFG->EXTICR[3] &= ~(0xFu<<0); SYSCFG->EXTICR[3] |= (0u<<0); EXTI->IMR|=(1u<<12); EXTI->FTSR|=(1u<<12);
  /* Keypad PB8..PB11 */
  SYSCFG->EXTICR[2] &= ~((0xFu<<0)|(0xFu<<4)|(0xFu<<8)|(0xFu<<12));
  SYSCFG->EXTICR[2] |=  ((1u<<0)|(1u<<4)|(1u<<8)|(1u<<12));
  EXTI->IMR  |= (1u<<8)|(1u<<9)|(1u<<10)|(1u<<11);
  EXTI->FTSR |= (1u<<8)|(1u<<9)|(1u<<10)|(1u<<11);
  EXTI->RTSR |= (1u<<8)|(1u<<9)|(1u<<10)|(1u<<11);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}
static void TIM21_Init_1kHz(void){ TIM21->PSC=159u; TIM21->ARR=99u; TIM21->EGR=TIM_EGR_UG; TIM21->DIER|=TIM_DIER_UIE; TIM21->CR1|=TIM_CR1_CEN; NVIC_EnableIRQ(TIM21_IRQn); }
static void TIM22_Init_2kHz(void){ TIM22->PSC=79u;  TIM22->ARR=99u; TIM22->EGR=TIM_EGR_UG; TIM22->DIER|=TIM_DIER_UIE; TIM22->CR1|=TIM_CR1_CEN; NVIC_EnableIRQ(TIM22_IRQn); }
static void USART2_Init(void){
  USART2->CR1=0; USART2->BRR=USART_BRR_9600_16MHZ;
  USART2->CR1|=USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE|USART_CR1_UE;
  NVIC_EnableIRQ(USART2_IRQn);
}

/* ====================== ISRs ====================== */
void EXTI4_15_IRQHandler(void){
  if(EXTI->PR & (1u<<4)){ if(((BTN_TREN_PORT->IDR>>BTN_TREN_PIN)&1u)==0u) g_btn_tren=1; EXTI->PR=(1u<<4); }
  if(EXTI->PR & (1u<<12)){ if(((BTN_PEAT_PORT->IDR>>BTN_PEAT_PIN)&1u)==0u) g_btn_peaton=1; EXTI->PR=(1u<<12); }

  uint32_t pend = EXTI->PR & ((1u<<8)|(1u<<9)|(1u<<10)|(1u<<11));
  if(pend){
    uint8_t rows=read_rows_mask(); int8_t hit=-1; uint8_t zeros=0;
    for(uint8_t r=0;r<4;r++) if(((rows>>r)&1u)==0){ zeros++; hit=r; }
    if(zeros==1 && k_state==K_IDLE){ k_row=hit; k_col=(int8_t)g_scan_col; k_cnt=DEBOUNCE_MS; k_state=K_DEB_PRESS; }
    EXTI->PR = pend;
  }
}
void TIM21_IRQHandler(void){
  if(TIM21->SR & TIM_SR_UIF){
    TIM21->SR=0;

    stp_service_1ms();
    lcd_service_1ms();
    buz_service_1ms();

    g_scan_col=(uint8_t)((g_scan_col+1u)&0x03u); col_drive_low(g_scan_col);

    switch(k_state){
      case K_IDLE: break;
      case K_DEB_PRESS:
        if(k_cnt) k_cnt--;
        if(k_cnt==0){
          uint8_t rows=read_rows_mask();
          if(((rows>>k_row)&1u)==0){
            char k=map_key_from_rowcol((uint8_t)k_row,(uint8_t)k_col);

            /* --- teclas globales --- */
            if(k=='C'){
              if(g_state==S_MANTTO) Semaforo_Set(S_VERDE);
              else                   Semaforo_Set(S_MANTTO);
            } else if(k=='D'){
              /* Cancelar cualquier ciclo activo (AMARILLO o ROJO) y volver a VERDE */
              if(g_state==S_AMARILLO || g_state==S_ROJO){
                g_tmr_yellow_ms = 0u;
                g_tmr_red_ms    = 0u;
                Semaforo_Set(S_VERDE);
                USART2_SendString("CANCEL: D -> VERDE\r\n");
              }
            } else
            /* ===== Lógica por estado ===== */
            if(g_state==S_VERDE){
              if(k=='A') Semaforo_Set(S_CONFIG_TREN);
              else if(k=='B') Semaforo_Set(S_CONFIG_PEATON);
            }
            else if(g_state==S_CONFIG_TREN){
              if(k>='0' && k<='9'){ buf_push_digit((uint8_t)(k-'0')); lcd_show_config_value("Tiempo Tren (MM:SS)"); }
              else if(k=='*'){ buf_clear(); lcd_show_config_value("Tiempo Tren (MM:SS)"); }
              else if(k=='#'){
                uint8_t mm,ss; buf_to_mmss(&mm,&ss);
                if(ss<60){
                  g_tren_segundos=(uint16_t)mm*60u+(uint16_t)ss;
                  UART_SendLabelTime("TREN tiempo", g_tren_segundos);
                  lcd_show_two("Tiempo guardado","Volviendo a VERDE"); g_msg_timer_ms=1000u; g_state=S_MSG;
                } else lcd_show_two("Segundos < 60","Corrige con *");
              }
            }
            else if(g_state==S_CONFIG_PEATON){
              if(k>='0' && k<='9'){ buf_push_digit((uint8_t)(k-'0')); lcd_show_config_value("Peaton: Tiempo"); }
              else if(k=='*'){ buf_clear(); lcd_show_config_value("Peaton: Tiempo"); }
              else if(k=='#'){
                uint8_t mm,ss; buf_to_mmss(&mm,&ss);
                if(ss<60){
                  g_peaton_segundos=(uint16_t)mm*60u+(uint16_t)ss;
                  UART_SendLabelTime("PEATON tiempo", g_peaton_segundos);
                  Semaforo_Set(S_CONFIG_PEATON_BUZ);
                } else lcd_show_two("Segundos < 60","Corrige con *");
              }
            }
            else if(g_state==S_CONFIG_PEATON_BUZ){
              if(k=='0'){ g_peaton_buzzer_en=0; lcd_show_peaton_buzzer_menu(); USART2_SendString("PEATON buzzer=OFF\r\n"); }
              else if(k=='1'){ g_peaton_buzzer_en=1; lcd_show_peaton_buzzer_menu(); USART2_SendString("PEATON buzzer=ON\r\n"); }
              else if(k=='#'){ Semaforo_Set(S_CONFIG_PEATON_BAR); }
            }
            else if(g_state==S_CONFIG_PEATON_BAR){
              if(k=='0'){ g_peaton_barrier_en=0; lcd_show_peaton_bar_menu(); USART2_SendString("PEATON barrera=NO\r\n"); }
              else if(k=='1'){ g_peaton_barrier_en=1; lcd_show_peaton_bar_menu(); USART2_SendString("PEATON barrera=SI\r\n"); }
              else if(k=='#'){
                lcd_show_two("Config Peaton OK","Volviendo a VERDE");
                USART2_SendString("MENU: Peaton guardado\r\n");
                g_msg_timer_ms=1000u; g_state=S_MSG;
              }
            }
            else if(g_state==S_MANTTO){
              /* En mantenimiento: controlar talanquera */
              if(k=='0'){ /* Abajo */
                stp_back_home_90();
                USART2_SendString("MANTTO: barrera=ABAJO\r\n");
                mantto_show_menu();
              } else if(k=='1'){ /* Arriba */
                stp_right_90();
                USART2_SendString("MANTTO: barrera=ARRIBA\r\n");
                mantto_show_menu();
              }
            }

            k_cnt=RELEASE_MS; k_state=K_HELD;
          }else k_state=K_IDLE;
        } break;
      case K_HELD: {
        uint8_t rows=read_rows_mask();
        if(rows==0x0F){ if(k_cnt) k_cnt--; if(!k_cnt) k_state=K_IDLE; }
        else k_cnt=RELEASE_MS;
      } break;
    }

    /* Botones físicos para iniciar ciclos (solo en VERDE) */
    if(g_btn_tren){ g_btn_tren=0; if(g_state==S_VERDE){ g_mode=MODE_TREN; Semaforo_Set(S_AMARILLO); } }
    if(g_btn_peaton){ g_btn_peaton=0; if(g_state==S_VERDE){ g_mode=MODE_PEATON; Semaforo_Set(S_AMARILLO); } }

    /* Timers del semáforo */
    if(g_state==S_AMARILLO){
      if(g_tmr_yellow_ms){
        g_tmr_yellow_ms--;
        if(++g_blink_div_ms>=250u){ g_blink_div_ms=0u; GPIO_SET(LED_AMAR_PORT,LED_AMAR_PIN); }
        if(g_tmr_yellow_ms==0u) Semaforo_Set(S_ROJO);
      }
    }else if(g_state==S_ROJO){
      if(g_tmr_red_ms){
        g_tmr_red_ms--;
        if(++g_div_1s_ms>=1000u){ g_div_1s_ms=0u; uint16_t s=(uint16_t)(g_tmr_red_ms/1000u); seg_show_mmss(s); }
        if(g_tmr_red_ms==0u) Semaforo_Set(S_VERDE);
      }
    }else if(g_state==S_MSG){
      if(g_msg_timer_ms){ g_msg_timer_ms--; if(!g_msg_timer_ms) Semaforo_Set(S_VERDE); }
    }else if(g_state==S_MANTTO){
      /* Parpadeo continuo del LED AMARILLO ~300ms */
      if(++g_blink_div_ms>=300u){
        g_blink_div_ms=0u;
        if( (GPIOC->ODR & (1u<<LED_AMAR_PIN)) ) GPIO_CLR(LED_AMAR_PORT,LED_AMAR_PIN);
        else                                   GPIO_SET(LED_AMAR_PORT,LED_AMAR_PIN);
      }
    }

    /* Mensajes LCD pendientes */
    if(lcd_init_st==LCDI_READY && g_lcd_msg!=LCD_MSG_NONE){
      lcd_clear_home();
      if(g_lcd_msg==LCD_MSG_LIBRE){
        lcd_enqueue_text_fixed16("Libre");
        lcd_set_cursor_nb(1,0);
        lcd_enqueue_text_fixed16("A: Tren   B: Peaton");
      }else if(g_lcd_msg==LCD_MSG_PRECAUCION){
        lcd_enqueue_text_fixed16("Precaucion");
        lcd_set_cursor_nb(1,0);
        lcd_enqueue_text_fixed16("Esperando a Rojo");
      }else if(g_lcd_msg==LCD_MSG_TREN){
        lcd_enqueue_text_fixed16("Tren pasando");
      }else if(g_lcd_msg==LCD_MSG_PEATON){
        lcd_enqueue_text_fixed16("Paso peatonal");
      }else if(g_lcd_msg==LCD_MSG_MANTTO){
        mantto_show_menu();
      }
      g_lcd_msg=LCD_MSG_NONE;
    }
  }
}
void TIM22_IRQHandler(void){
  if(TIM22->SR & TIM_SR_UIF){
    TIM22->SR=0;
    if(g_state!=S_ROJO){ digits_all_off(); return; }
    digits_all_off();
    uint8_t idx=mux_idx & 3u; uint8_t v=seg_digits[idx]; if(v>9) v=0;
    seg_set_pattern(seg_lut[v]);
    if(idx==0) GPIO_SET(DIG_PORT,DIG0_PIN);
    else if(idx==1) GPIO_SET(DIG_PORT,DIG1_PIN);
    else if(idx==2) GPIO_SET(DIG_PORT,DIG2_PIN);
    else            GPIO_SET(DIG_PORT,DIG3_PIN);
    mux_idx=(idx+1u)&3u;
  }
}
void USART2_IRQHandler(void){
  static volatile uint8_t utx_head_local; (void)utx_head_local;
  if(USART2->ISR & USART_ISR_TXE){
    if(utx_tail!=utx_head){ USART2->TDR=utxq[utx_tail]; utx_tail=(utx_tail+1)&(UTXQ_SIZE-1); }
    else USART2->CR1 &= ~USART_CR1_TXEIE;
  }
  if(USART2->ISR & USART_ISR_RXNE){ (void)USART2->RDR; }
}

/* ====================== MAIN ====================== */
int main(void){
  Clocks_Init(); GPIO_Init(); EXTI_Init();
  TIM21_Init_1kHz(); TIM22_Init_2kHz(); USART2_Init();
  lcd_start_init(); Semaforo_Set(S_VERDE);
  __enable_irq();
  while(1){ __NOP(); } /* todo por IRQ */
}







































































































