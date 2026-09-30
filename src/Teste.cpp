#include <Arduino.h>
#include "esp32s3_dsp.h" 
#include "esp_heap_caps.h" // Necessário para usar a memória externa (PSRAM)

// Configurações base do sistema
#define FFT_SIZE            1024   // Resolução do espectro (deve ser potência de 2)
#define FFT_SAMPLES         1024   // Amostras por ciclo
#define TAXA_AMOSTRAGEM     4000.0 // Frequência de leitura dos sensores (4 kHz)

// Instâncias da biblioteca matemática
ESP32S3_FFT fft_vibracao;
ESP32S3_FFT fft_corrente;

ESP32S3_LP_FILTER filtro_inversor_vib; // Passa-baixa para ruído do inversor
ESP32S3_NOTCH_FILTER filtro_rede_corr; // Notch para remover o zumbido de 60Hz da rede

// Buffers para Vibração
float *vib_dado_sujo;     
float *vib_dado_limpo;    
float *vib_espetro_ia;  

// Buffers para Corrente
float *corr_dado_sujo;    
float *corr_dado_limpo;   
float *corr_espetro_ia; 

void setup(void) {
  Serial.begin(115200);
  vTaskDelay(100); 

  // Inicializa os filtros (Frequência alvo, taxa de amostragem, atenuação, fator Q)
  filtro_inversor_vib.init(1000.0, TAXA_AMOSTRAGEM, 0.707);
  filtro_rede_corr.init(60.0, TAXA_AMOSTRAGEM, -40.0, 0.707);

  // Prepara a FFT usando média espectral (ideal para suavizar dados para a IA)
  fft_table_t *tabela_vib = fft_vibracao.init(FFT_SIZE, FFT_SAMPLES, SPECTRAL_AVERAGE);
  fft_table_t *tabela_corr = fft_corrente.init(FFT_SIZE, FFT_SAMPLES, SPECTRAL_AVERAGE);

  // Aloca os buffers na PSRAM para não travar a memória interna do ESP32-S3
  vib_dado_sujo  = (float *)heap_caps_malloc((tabela_vib->size_input_bufr * sizeof(float)), MALLOC_CAP_SPIRAM);
  vib_dado_limpo = (float *)heap_caps_malloc((tabela_vib->size_input_bufr * sizeof(float)), MALLOC_CAP_SPIRAM);
  vib_espetro_ia = (float *)heap_caps_malloc((FFT_SIZE * sizeof(float)), MALLOC_CAP_SPIRAM);

  corr_dado_sujo  = (float *)heap_caps_malloc((tabela_corr->size_input_bufr * sizeof(float)), MALLOC_CAP_SPIRAM);
  corr_dado_limpo = (float *)heap_caps_malloc((tabela_corr->size_input_bufr * sizeof(float)), MALLOC_CAP_SPIRAM);
  corr_espetro_ia = (float *)heap_caps_malloc((FFT_SIZE * sizeof(float)), MALLOC_CAP_SPIRAM);

  // Trava o sistema num loop infinito caso a placa falhe ao alocar a PSRAM
  if (!vib_dado_sujo || !corr_dado_sujo) {
    while(true) { vTaskDelay(10); } 
  }
}

void loop(void) {
  // 1. Leitura dos sensores
  for(int i = 0; i < FFT_SAMPLES; i++) {
     vib_dado_sujo[i] = 0.0f;  // Substituir pela leitura real do acelerômetro
     corr_dado_sujo[i] = 0.0f; // Substituir pela leitura real do sensor de corrente
  }

  // 2. Aplicação dos filtros para limpar os dados em tempo real
  filtro_inversor_vib.apply(vib_dado_sujo, vib_dado_limpo, FFT_SAMPLES);
  filtro_rede_corr.apply(corr_dado_sujo, corr_dado_limpo, FFT_SAMPLES);

  // 3. Execução da FFT para extrair as frequências do sinal limpo
  fft_vibracao.compute(vib_dado_limpo, vib_espetro_ia);
  fft_corrente.compute(corr_dado_limpo, corr_espetro_ia);
  
  // Os vetores 'vib_espetro_ia' e 'corr_espetro_ia' estão prontos para a IA
  
  vTaskDelay(pdMS_TO_TICKS(2000)); // Aguarda 2 segundos até o próximo ciclo
}