#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"     
#include "hardware/pwm.h"  
#include "pico/bootrom.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "pio_matrix.pio.h"



// Periféricos gerais

#define VRX 27
#define VRY 26  
#define BUZZER_A 21
#define BUZZER_B 10
#define LED_VERMELHO 13
#define LED_VERDE 11
#define LED_AZUL 12
#define BOTAO_A 5
#define BOTAO_B 6


// Display

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
ssd1306_t ssd;

// Variáveis globais

bool confirmacao = false;
bool caso_b = true;
bool mensagem_inicial = true;
bool refrigeracao = false;
uint slice_red;
uint slice_green;
uint slice_blue;
uint idx, modo1, modo2;
uint tempo_buzz;
static volatile uint32_t last_time = 0;

// Matriz de led's
PIO pio;
uint sm;
uint32_t VALOR_LED;
unsigned char R, G, B;
#define NUM_PIXELS 25
#define OUT_PIN 7

// Desenhos da matriz

double X[] = {
    0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0, 
    0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 
    0.1, 0.0, 0.0, 0.0, 0.1
};

double apaga[] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0
};

double V[] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 
    0.0, 0.1, 0.0, 0.0, 0.0
};


// Inicialização de dispositivos de saída
void inicializar_led(uint pino) {
    
    gpio_init(pino);
    gpio_set_dir(pino,GPIO_OUT);

}

// Inicialização de botões
void inicializar_botao(uint pino) {
    
    gpio_init(pino);
    gpio_set_dir(pino,GPIO_IN);
    gpio_pull_up(pino);

}

// Inicialização de dispositivos PWM
uint pwm_init_gpio(uint gpio, uint wrap) {
    
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, wrap);
    
    pwm_set_enabled(slice_num, true);  
    return slice_num;  

}

// Inicialização do Display SSD1306
void init_ssd1306() {
   
    i2c_init(I2C_PORT, 400 * 1000);

    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);


    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

}

// Layout usado nas falas
void molde(){
    ssd1306_rect(&ssd, 3, 3, 122, 60, true, !true); 
    ssd1306_line(&ssd, 3, 25, 123, 25, true); 
    ssd1306_line(&ssd, 3, 37, 123, 37, true);
    ssd1306_line(&ssd, 84, 37, 84, 60, true);
    ssd1306_rect(&ssd, 52, 90, 8, 8, true, !true); 
    ssd1306_rect(&ssd, 52, 102, 8, 8, true, !true);     
    ssd1306_rect(&ssd, 52, 114, 8, 8, true, !true); 
}

// Ativação do buzzer
void buzz(uint8_t BUZZER_PIN, uint16_t freq, uint16_t duration) {
    int period = 1000000 / freq;
    int pulse = period / 2;
    int cycles = freq * duration / 1000;
    for (int j = 0; j < cycles; j++) {
        gpio_put(BUZZER_PIN, 1);
        sleep_us(pulse);
        gpio_put(BUZZER_PIN, 0);
        sleep_us(pulse);
    }
}

uint32_t matrix_rgb(double b, double r, double g)
{
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

// Desenha na matriz de leds em verde
void desenho_pio_green(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){

    for (int16_t i = 0; i < NUM_PIXELS; i++) {
            valor_led = matrix_rgb(b = 0.0, r=0.0, desenho[24-i]);
            pio_sm_put_blocking(pio, sm, valor_led);
    }
}
// Desenha na matriz de leds em vermelho
void desenho_pio_red(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){

    for (int16_t i = 0; i < NUM_PIXELS; i++) {
            valor_led = matrix_rgb(b = 0.0, desenho[24-i], g = 0.0);
            pio_sm_put_blocking(pio, sm, valor_led);
    }
}

// Ativação do buzzer por um dado tempo 
void buzz_for_duration(uint8_t BUZZER_PIN, uint16_t freq, uint16_t duration, uint16_t total_time_ms) {
    uint16_t elapsed_time = 0;

    while (elapsed_time < total_time_ms) {
        buzz(BUZZER_PIN, freq, duration);
        elapsed_time += duration;
        sleep_ms(1); // Espera 1ms entre cada chamada de buzz
    }
}

// Função chamada nas interrupções
static void gpio_irq_handler(uint gpio, uint32_t events) {
    
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    if (current_time - last_time > 600000){
        last_time = current_time; 
        
        if (gpio == BOTAO_A) {
            if(idx == 1) {
                confirmacao = !confirmacao;
            }
            if(idx == 2) {
                refrigeracao = true;
            }
        } else if (gpio == BOTAO_B) {
            if(caso_b == true) {
                idx = 1;
                modo1 = 1;
            } else if(caso_b == false) {
                idx = 2;
                modo2 = 1;
            }
            mensagem_inicial = false;
            caso_b = !caso_b;
        }
    }
}

int main() {
    stdio_init_all();

    uint entrada = 1;
    uint pwm_wrap = 1365;
    uint raw_value;
    float referencia = 0.5928f;
    float coeficiente = 0.0021f;
    float temperatura_celsius;
    float voltage;
    char str_temp[5], str_light[5];

    // Setando matriz de led's
    double r = 0.0, b = 0.0 , g = 0.0;
    bool ok;
    ok = set_sys_clock_khz(128000, false);
    pio = pio0;

    uint offset = pio_add_program(pio, &FINAL_PROJECT_program);
    uint sm = pio_claim_unused_sm(pio, true);
    FINAL_PROJECT_program_init(pio, sm, offset, OUT_PIN);

    slice_red = pwm_init_gpio(LED_VERMELHO,pwm_wrap);
    slice_green = pwm_init_gpio(LED_VERDE,pwm_wrap);
    slice_blue = pwm_init_gpio(LED_AZUL,pwm_wrap);

    // Inicializando periféricos
    inicializar_botao(BOTAO_A);
    inicializar_botao(BOTAO_B);

    inicializar_led(BUZZER_A);

    // Inicializando ADC
    adc_init();
    adc_gpio_init(VRX);
    
    // Inicializando Display
    init_ssd1306();

    // Habilitando interrupções
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    

    while(true) {
        
        if(entrada == 1) {
            ssd1306_draw_full_image(&ssd, raio2);
            ssd1306_send_data(&ssd);
            sleep_ms(3000);
            entrada++;
        }
        // Alinhar
        if(mensagem_inicial){
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "APERTE B PARA", 12, 16);
            ssd1306_draw_string(&ssd, "ALTERNAR ENTRE", 8, 28);
            ssd1306_draw_string(&ssd, "OS MODOS", 32, 40);
            ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
            ssd1306_rect(&ssd, 3, 3, 122, 58, true, false);
            ssd1306_send_data(&ssd); 
        }
        
        switch (idx){

            case 1: // aferidor de luminosidade
                
                if(modo1 == 1){
                    ssd1306_fill(&ssd, false);
                    ssd1306_draw_string(&ssd, "MODO 1", 40, 16);
                    ssd1306_draw_string(&ssd, "SELECIONADO", 16, 28);
                    ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
                    ssd1306_rect(&ssd, 3, 3, 122, 58, true, false);
                    ssd1306_send_data(&ssd);
                    sleep_ms(1500);
                    modo1++;
                }
                adc_select_input(1);
                uint16_t vrx_value = adc_read();
                uint16_t conv_x;

                sprintf(str_light, "%d", vrx_value);
                
                //Alinhar
                ssd1306_fill(&ssd, false);
                ssd1306_draw_string(&ssd, "APERTE A PARA", 8, 6);
                ssd1306_draw_string(&ssd, "CONFIRMAR", 32, 16);
                ssd1306_draw_string(&ssd, "LIGHT", 28, 41);
                ssd1306_draw_string(&ssd, str_light, 30, 52);
                molde();
                ssd1306_send_data(&ssd);

                    if(confirmacao){
                        if (vrx_value >= 0 && vrx_value < 1365) {
                            ssd1306_fill(&ssd, false);
                            ssd1306_draw_string(&ssd, "AMBIENTE", 32, 16);
                            ssd1306_draw_string(&ssd, "COM POUCA LUZ", 8, 28);
                            ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
                            ssd1306_rect(&ssd, 3, 3, 122, 58, true, false);
                            ssd1306_send_data(&ssd);
                            desenho_pio_red(X, VALOR_LED, pio, sm, R, G, B);
                            buzz_for_duration(BUZZER_A, 1500, 300, 2500);
                        } else if (vrx_value >= 1365 && vrx_value < 2730) {
                            ssd1306_fill(&ssd, false);
                            ssd1306_draw_string(&ssd, "AMBIENTE BEM", 12, 16);
                            ssd1306_draw_string(&ssd, "ILUMINADO", 28, 28);
                            ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
                            ssd1306_rect(&ssd, 3, 3, 122, 58, true, false);
                            ssd1306_send_data(&ssd);
                            desenho_pio_green(V, VALOR_LED, pio, sm, R, G, B);
                            buzz_for_duration(BUZZER_A, 1000, 100, 100);
                            buzz_for_duration(BUZZER_A, 1500, 100, 100);
                            buzz_for_duration(BUZZER_A, 2000, 100, 100);
                        } else {
                            ssd1306_fill(&ssd, false);
                            ssd1306_draw_string(&ssd, "AMBIENTE", 32, 16);
                            ssd1306_draw_string(&ssd, "COM LUZ", 32, 28);
                            ssd1306_draw_string(&ssd, "EXCEDENTE", 32, 40);
                            ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
                            ssd1306_rect(&ssd, 3, 3, 122, 58, true, false);
                            ssd1306_send_data(&ssd);
                            desenho_pio_red(X, VALOR_LED, pio, sm, R, G, B);
                            buzz_for_duration(BUZZER_A, 1500, 300, 2500);
                        }
                        sleep_ms(3000);
                        desenho_pio_red(apaga, VALOR_LED, pio, sm, R, G, B);
                        confirmacao = false;
                    }

                    if (vrx_value >= 0 && vrx_value < 1365) {
                        conv_x = (vrx_value);  // Mapeia diretamente para 0-1365
                        pwm_set_gpio_level(LED_AZUL, conv_x);
                        pwm_set_gpio_level(LED_VERDE, 0);
                        pwm_set_gpio_level(LED_VERMELHO, 0);
                        ssd1306_fill(&ssd, false);
                        
                    } else if (vrx_value >= 1365 && vrx_value < 2730) {
                        conv_x = vrx_value - 1365;  // Mapeia para 0-1365
                        pwm_set_gpio_level(LED_VERDE, conv_x);
                        pwm_set_gpio_level(LED_AZUL, 0);
                        pwm_set_gpio_level(LED_VERMELHO, 0);

                    } else {
                        conv_x = vrx_value - 2730;  // Mapeia para 0-1365
                        pwm_set_gpio_level(LED_VERMELHO, conv_x);
                        pwm_set_gpio_level(LED_AZUL, 0);
                        pwm_set_gpio_level(LED_VERDE, 0);
                    }
                    break;
            
            case 2: // Regulador de temperatura
                
                if(modo2 == 1){
                    ssd1306_fill(&ssd, false);
                    ssd1306_draw_string(&ssd, "MODO 2", 40, 16);
                    ssd1306_draw_string(&ssd, "SELECIONADO", 16, 28);
                    ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
                    ssd1306_rect(&ssd, 3, 3, 122, 58, true, false);
                    ssd1306_send_data(&ssd);
                    sleep_ms(1500);
                    ssd1306_fill(&ssd, false);
                    modo2++;
                }


                adc_select_input(4);
                raw_value = adc_read();
                voltage = raw_value * 3.3f / (1 << 12);
                temperatura_celsius = 27.0f - (voltage - referencia) / coeficiente;
                uint16_t temp_conv;

                sprintf(str_temp, "%.1f", temperatura_celsius);

                
                ssd1306_fill(&ssd, false);
                ssd1306_draw_string(&ssd, "APERTE A PARA", 8, 6);
                ssd1306_draw_string(&ssd, "REFRIGERAR", 28, 16);
                ssd1306_draw_string(&ssd, "TEMP", 28, 41);
                ssd1306_draw_string(&ssd, str_temp, 30, 52);
                molde();
                ssd1306_send_data(&ssd);
                    if(temperatura_celsius > 15){
                        if(refrigeracao) { 
                            while(temperatura_celsius > 15){
                                temperatura_celsius -= 0.1f; // Reduz a temperatura em 0.1°C a cada iteração

                            // Mapear a temperatura para as cores dos LEDs
                            if (temperatura_celsius >= 0 && temperatura_celsius < 18) {
                                // Azul (frio)
                                temp_conv = (uint16_t)((temperatura_celsius / 18.0f) * pwm_wrap);
                                pwm_set_gpio_level(LED_AZUL, temp_conv);
                                pwm_set_gpio_level(LED_VERDE, 0);
                                pwm_set_gpio_level(LED_VERMELHO, 0);
                            } else if (temperatura_celsius >= 18 && temperatura_celsius < 25) {
                                // Verde (moderado)
                                temp_conv = (uint16_t)(((temperatura_celsius - 18.0f) / 7.0f) * pwm_wrap);
                                pwm_set_gpio_level(LED_VERDE, temp_conv);
                                pwm_set_gpio_level(LED_AZUL, 0);
                                pwm_set_gpio_level(LED_VERMELHO, 0);
                            } else {
                                // Vermelho (quente)
                                temp_conv = (uint16_t)(((temperatura_celsius - 25.0f) / 10.0f) * pwm_wrap);
                                pwm_set_gpio_level(LED_VERMELHO, temp_conv);
                                pwm_set_gpio_level(LED_AZUL, 0);
                                pwm_set_gpio_level(LED_VERDE, 0);
                            }
                            
                            sprintf(str_temp, "%.2f", temperatura_celsius);
                            ssd1306_draw_string(&ssd, str_temp, 22, 52);
                            ssd1306_send_data(&ssd);
                

                            
                            sleep_ms(50);
                            }
                            refrigeracao = false;
                            desenho_pio_green(V, VALOR_LED, pio, sm, R, G, B);
                            ssd1306_draw_string(&ssd, "REFRIGERADO", 18, 28);
                            ssd1306_send_data(&ssd);
                            buzz_for_duration(BUZZER_A, 1000, 100, 100);
                            buzz_for_duration(BUZZER_A, 1500, 100, 100);
                            buzz_for_duration(BUZZER_A, 2000, 100, 100);
                            sleep_ms(5000);
                            desenho_pio_green(apaga, VALOR_LED, pio, sm, R, G, B);
                        }
                    } else{
                        ssd1306_fill(&ssd, false);
                        ssd1306_draw_string(&ssd, "TEMPERATURA", 0, 16);
                        ssd1306_draw_string(&ssd, "IDEAL", 0, 28);
                        ssd1306_send_data(&ssd);
                        sleep_ms(3000);
                    }
                
                

                if (temperatura_celsius >= 0 && temperatura_celsius < 18) {
                    temp_conv = (uint16_t)((temperatura_celsius / 18.0f) * pwm_wrap);
                    pwm_set_gpio_level(LED_AZUL, temp_conv);
                    pwm_set_gpio_level(LED_VERDE, 0);
                    pwm_set_gpio_level(LED_VERMELHO, 0);
                } else if (temperatura_celsius >= 18 && temperatura_celsius < 25) {
                    temp_conv = (uint16_t)(((temperatura_celsius - 18.0f) / 7.0f) * pwm_wrap);
                    pwm_set_gpio_level(LED_VERDE, temp_conv);
                    pwm_set_gpio_level(LED_AZUL, 0);
                    pwm_set_gpio_level(LED_VERMELHO, 0);
                } else {
                    temp_conv = (uint16_t)(((temperatura_celsius - 25.0f) / 10.0f) * pwm_wrap);
                    pwm_set_gpio_level(LED_VERMELHO, temp_conv);
                    pwm_set_gpio_level(LED_AZUL, 0);
                    pwm_set_gpio_level(LED_VERDE, 0);
                }               
        }
    }
}
