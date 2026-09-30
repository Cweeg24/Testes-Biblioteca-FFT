#include <Arduino.h>
#include "esp32s3_dsp.h" 
#include "esp_heap_caps.h"

// --- Hardware e Tempo Discreto ---
#define PINO_SENSOR_CORRENTE 34    // Pino ADC para ler a tensão do SCT-013
#define FFT_SIZE             1024  // Tamanho do vetor da FFT
#define FFT_SAMPLES          1024  // Quantidade de leituras por ciclo
#define TAXA_AMOSTRAGEM      4000.0 // Frequência de amostragem (fs) em Hz

// Na teoria de sistemas de tempo discreto, o período de amostragem (Ts) é 1/fs.
// Para 4000 Hz, precisamos ler os sensores exatamente a cada 250 microssegundos.
const int PERIODO_AMOSTRAGEM_US = (1.0 / TAXA_AMOSTRAGEM) * 1000000;

// --- Instâncias DSP ---
ESP32S3_FFT fft_vibracao;
ESP32S3_FFT fft_corrente;
ESP32S3_LP_FILTER filtro_inversor_vib; 
ESP32S3_NOTCH_FILTER filtro_rede_corr; 

// --- Ponteiros de Memória (PSRAM) ---
float *vib_dado_sujo;     
float *vib_dado_limpo;    
float *vib_espetro_ia;  

float *corr_dado_sujo;    
float *corr_dado_limpo;   
float *corr_espetro_ia; 

void setup(void) {
  Serial.begin(115200);
  vTaskDelay(100); 
  
  pinMode(PINO_SENSOR_CORRENTE, INPUT);

  // Inicialização matemática dos filtros
  // O passa-baixa corta ruídos de chaveamento PWM acima de 1000Hz.
  // O Notch cria um entalhe profundo em 60Hz para isolar falhas mecânicas na corrente.
  filtro_inversor_vib.init(1000.0, TAXA_AMOSTRAGEM, 0.707);
  filtro_rede_corr.init(60.0, TAXA_AMOSTRAGEM, -40.0, 0.707);

  fft_table_t *tabela_vib = fft_vibracao.init(FFT_SIZE, FFT_SAMPLES, SPECTRAL_AVERAGE);
  fft_table_t *tabela_corr = fft_corrente.init(FFT_SIZE, FFT_SAMPLES, SPECTRAL_AVERAGE);

  // Alocação de memória externa para evitar estouro da RAM interna do ESP32-S3
  vib_dado_sujo  = (float *)heap_caps_malloc((tabela_vib->size_input_bufr * sizeof(float)), MALLOC_CAP_SPIRAM);
  vib_dado_limpo = (float *)heap_caps_malloc((tabela_vib->size_input_bufr * sizeof(float)), MALLOC_CAP_SPIRAM);
  vib_espetro_ia = (float *)heap_caps_malloc((FFT_SIZE * sizeof(float)), MALLOC_CAP_SPIRAM);

  corr_dado_sujo  = (float *)heap_caps_malloc((tabela_corr->size_input_bufr * sizeof(float)), MALLOC_CAP_SPIRAM);
  corr_dado_limpo = (float *)heap_caps_malloc((tabela_corr->size_input_bufr * sizeof(float)), MALLOC_CAP_SPIRAM);
  corr_espetro_ia = (float *)heap_caps_malloc((FFT_SIZE * sizeof(float)), MALLOC_CAP_SPIRAM);

  if (!vib_dado_sujo || !corr_dado_sujo) {
    while(true) { vTaskDelay(10); } // Trava de segurança
  }
}

void loop(void) {
  
  // Fase 1: Aquisição Sincronizada
  // Enchemos o buffer de 1024 posições respeitando rigorosamente o relógio de 4000 Hz.
  for(int i = 0; i < FFT_SAMPLES; i++) {
     
     // Marcamos no relógio interno do processador o exato momento em que a leitura começou
     unsigned long tempo_inicio_leitura = micros();

     // Leitura 1: Vibração
     // Aqui você chamará a função da biblioteca do seu acelerómetro (ex: ler o eixo Z via I2C)
     // vib_dado_sujo[i] = mpu.getAccelerationZ(); 
     vib_dado_sujo[i] = 0.0f; // Placeholder
     
     // Leitura 2: Corrente Elétrica
     // O conversor ADC lê um valor de 0 a 4095. Numa aplicação real industrial, 
     // você multiplicaria este valor pelo fator de conversão do seu circuito condicionador.
     corr_dado_sujo[i] = analogRead(PINO_SENSOR_CORRENTE); 

     // Sincronização: O ESP32 é muito rápido e faz a leitura em poucos microssegundos.
     // Este loop "while" prende o processador até que se completem exatamente os 250 microssegundos 
     // necessários para manter a taxa de 4000 Hz, garantindo a integridade do domínio da frequência.
     while (micros() - tempo_inicio_leitura < PERIODO_AMOSTRAGEM_US) {
        // Aguarda
     }
  }

  // Fase 2: Condicionamento de Sinal (Filtragem)
  // Passamos os vetores com o sinal bruto e ruidoso pelas funções de filtro.
  // O resultado limpo é guardado nos vetores '_dado_limpo'.
  filtro_inversor_vib.apply(vib_dado_sujo, vib_dado_limpo, FFT_SAMPLES);
  filtro_rede_corr.apply(corr_dado_sujo, corr_dado_limpo, FFT_SAMPLES);

  // Fase 3: Processamento DSP (Transformada de Fourier)
  // Converte a série temporal filtrada num espetro de frequências.
  // Cada posição do vetor final representa a amplitude de uma frequência específica.
  fft_vibracao.compute(vib_dado_limpo, vib_espetro_ia);
  fft_corrente.compute(corr_dado_limpo, corr_espetro_ia);
  
  // Pausa de 1 segundo antes de recolher o próximo lote de dados do motor
  vTaskDelay(pdMS_TO_TICKS(1000)); 
}