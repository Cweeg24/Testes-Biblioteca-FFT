/*
  *Bilioteca de Processamento Digital de Sinais (DSP) otimizada para o microcontrolador ESP32-S3.

Requer memória PSRAM devido ao tamanho dos buffers de áudio.
O ESP32-S3 possui um acelerador matemático embutido (DSP) feito especialmente para processar sinais. 

Converte um sinal do Domínio do Tempo  para o Domínio da Frequência.
Entregamos o som caótico captado pelo microfone, e nos devolve um raio-X completo, mostrando a freq exata de cada som isolado.
O acelerômetro no motor gera o mesmo tipo de onda que um microfone.
Usar filtros garantem que a Rede Neural receba apenas dados limpos

*/


#include <Arduino.h>
#include "esp32s3_fft.h"
#include "esp_heap_caps.h" //Biblioteca necessária para gerir a memória externa (PSRAM) do ESP32-S3

//É a "resolução do Raio-X". Define em quantas "gavetas" (bins) o sinal será dividido. 
// Obrigatório ser uma potência de 2 (256, 512, 1024). Quanto maior, mais nítido é o gráfico, mas exige mais memória.
#define FFT_SIZE            1024  

//Quantos pontos de áudio/vibração captamos antes de fazer a conta. Normalmente igual ao FFT_SIZE.
#define FFT_SAMPLES         1024  

//A "velocidade". Quantas vezes por segundo o sensor leu.
//16000 significa que o sensor tira 16.000 amostras do sinal por segundo (16 kHz).
#define AUDIO_SAMPLE_RATE   16000 

//Frequência de teste que vamos simular por software (1000 Hz).
#define SINEWAVE_FREQ       1000

//Instancia a biblioteca FFT (Cria o objeto 'fft' que fará todo o trabalho pesado)
ESP32S3_FFT fft;

void setup(void)
{
  Serial.begin(115200);                  
  vTaskDelay(50); 
  Serial.println("");

      // INICIALIZAÇÃO E ALOCAÇÃO DE MEMÓRIA
  
  // init(): A função mais importante. Calcula as fórmulas matemáticas preliminares e devolve 
  // um ponteiro (fft_table) que nos diz exatamente quanta memória precisamos reservar.
  // SPECTRAL_AVERAGE: Faz a média do sinal, entregando um resultado mais limpo e estável.
  fft_table_t *fft_table = fft.init(FFT_SIZE, FFT_SAMPLES, SPECTRAL_AVERAGE);


  /* 
    (SRAM vs PSRAM):
    As duas linhas abaixo criam os buffers na memória interna (SRAM) do ESP32. 
    Para tamanhos pequenos (ex: 256) funciona. Para tamanhos grandes (ex: 1024 ou mais), 
    a placa vai bloquear (crash) porque esgota a memória interna. 
  */
  float input_bufr[fft_table->size_input_bufr];
  float output_bufr[FFT_SAMPLES];

  /* 
    A forma correta seria utilizar a PSRAM (memória RAM externa do ESP32-S3) usando o MALLOC_CAP_SPIRAM.
      psramInit();                
      float *input_bufr = (float *)heap_caps_malloc((fft_table->size_input_bufr * sizeof(float)), MALLOC_CAP_SPIRAM ); 
      float *output_bufr = (float *)heap_caps_malloc(FFT_SAMPLES * sizeof(float), MALLOC_CAP_SPIRAM );  
  */


      // CRIAÇÃO DO SINAL FALSO (Simulação do Sensor)
  
  // Como não temos um sensor, este bloco gera uma onda perfeita de 1000 Hz e guarda-a no 'input_bufr'.
  float phase = 0.0f;
  const float phase_step = TWO_PI * SINEWAVE_FREQ / AUDIO_SAMPLE_RATE;   
  
  for (int j = 0; j < FFT_SAMPLES; j++) {
     input_bufr[j] = sinf(phase); // sinf = seno em formato float (muito mais rápido no ESP32-S3)
     phase += phase_step;
  }

  // EXECUÇÃO (Domínio do Tempo -> Domínio da Frequência)
  
  /* compute(): Lê a onda do 'input_bufr', faz os cálculos da Transformada Rápida de Fourier, 
  e cospe as "gavetas" de energia (frequências isoladas) no 'output_bufr'.*/
  fft.compute(input_bufr, output_bufr);

  /*
    COMO LER O RESULTADO:
   * Cada índice (gaveta) do output_bufr representa uma fatia de frequência.
   * Fórmula da Resolução: Sample Rate / FFT_SIZE.
   * Exemplo do nosso código: 16000 / 1024 = 15,625 Hz.
   * Isso significa que:
   * output_bufr[0] = energia nos 0 Hz
   * output_bufr[1] = energia nos 15,625 Hz
   * output_bufr[2] = energia nos 31,250 Hz ... e assim por diante.
   * 
   * TEOREMA DE NYQUIST:
   * Uma regra da física diz que só conseguimos ler frequências até METADE da nossa taxa de amostragem.
   * Se lemos a 16.000 Hz, o máximo que conseguimos "ver" é 8.000 Hz.
   * Por isso, a segunda metade do array é apenas um espelho matemático inútil (lixo).
   * Descartamos a segunda metade e lemos apenas de 0 até FFT_SIZE/2.
   */

  // APRESENTAÇÃO DOS DADOS
  
  // Imprime os valores no Monitor. Lemos apenas a primeira metade (i < FFT_SIZE/2) 
  // por causa do limite de Nyquist explicado acima.
  for(int i=0; i<FFT_SIZE/2; i++) {    
    if(i > 0 && i % 10 == 0) // Quebra a linha a cada 10 números para o ecrã não ficar confuso
      Serial.println("");
      
    int16_t sample = int(output_bufr[i]);
    Serial.printf("%d ", sample); // Imprime a energia daquela gaveta
  }
  Serial.println("");
  
  // end(): Liberta toda a memória interna que a biblioteca alocou (previne memory leaks).
  fft.end();
}

void loop(void) 
{

}

/*
#define QTD_AMOSTRAS 512
#define TAXA_AMOSTRAGEM 4000.0 // 4000 Hz 

// PASSO 1: INSTANCIAR
// Cria o objeto do filtro antes do setup() para que seja global.
ESP32S3_NOTCH_FILTER filtro_rede_eletrica;

// Buffers para guardar as medições
float leitura_bruta[QTD_AMOSTRAS];
float leitura_filtrada[QTD_AMOSTRAS];

void setup() {
    Serial.begin(115200);

    // Esta função deve ser chamada APENAS UMA VEZ no setup. 
    // Ela traduz as configurações em coeficientes matemáticos complexos.
    // Parâmetros: Frequência a remover (60Hz), Taxa do Sensor (4000Hz), Ganho (-40dB para atenuar fortemente), Fator Q (0.707).
    filtro_rede_eletrica.init(60.0, TAXA_AMOSTRAGEM, -40.0, 0.707);
}

void loop() {
    for(int i = 0; i < QTD_AMOSTRAS; i++) {
        leitura_bruta[i] = sinf(TWO_PI * 120.0 * i / TAXA_AMOSTRAGEM) +  // Vibração real do motor (120 Hz)
                           sinf(TWO_PI * 60.0 * i / TAXA_AMOSTRAGEM);    // Ruído induzido da tomada (60 Hz)
    }

    // Executa a limpeza matemática em tempo real.
    // Pega nos dados do primeiro array, filtra tudo de uma vez, e guarda no segundo array.
    filtro_rede_eletrica.apply(leitura_bruta, leitura_filtrada, QTD_AMOSTRAS);

    // Agora o array 'leitura_filtrada' está livre do zumbido de 60 Hz.
    // Daqui, o próximo passo seria enviar este array limpo para o 'fft.compute()'.
    
    delay(100);
}
*/