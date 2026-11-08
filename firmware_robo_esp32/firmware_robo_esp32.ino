/*
 * Robô móvel 2WD - ESP32 - Controle de POSIÇÃO (2ª Avaliação de Robótica)
 *
 * O que o firmware faz:
 *   - Malha aberta: velocidade (RPM ou PWM fixo) durante um tempo definido,
 *     registrando a odometria SEM usá-la para corrigir o movimento.
 *   - Malha fechada de posição com os controladores P, I, D, PI, PD e PID
 *     (selecionáveis), erro = referência - posição por odometria.
 *     A saída do controlador é a VELOCIDADE COMANDADA às rodas (RPM), que
 *     o PID de rotação de cada roda (malha interna) segue.
 *   - Ganhos separados para cada controlador, salvos na flash (NVS).
 *   - Estados do experimento, critério de término, métricas automáticas.
 *   - Registro dos dados na RAM (100 ms por amostra) e envio após o teste.
 *   - Odometria COM SINAL (ré desconta), necessária quando o robô passa do
 *     alvo e o controlador precisa voltar.
 *
 * Bibliotecas: QuickPID (Gerenciador de Bibliotecas) + Preferences (já vem no core).
 * Compatível com o core ESP32 2.x e 3.x.
 *
 * ============================ ORGANIZAÇÃO DO ARQUIVO =======================
 *   1. Bibliotecas
 *   2. Configuração (todas as constantes ajustáveis)
 *   3. Tipos e estado global (motores, PIDs, navegação, experimento, registro)
 *   4. Comunicação (BLE, fila de saída, Serial)
 *   5. Hardware (PWM, encoder, acionamento dos motores)
 *   6. Controle (PID de rotação, navegação por distância "G", direção manual)
 *   7. Experimento (malha aberta e malha fechada de posição)
 *   8. Registro (LOG enviado em partes)
 *   9. Armazenamento (flash NVS)
 *  10. Protocolo (linhas enviadas à IHM e textos de status)
 *  11. Comandos (parser e execução)
 *  12. setup() e loop()
 *
 * Cadeia de controle (a cada 100 ms, em loop()):
 *   encoders -> RPM/odometria -> navegação (G) -> experimento (posição)
 *   -> PID de RPM de cada roda -> PWM
 *
 * ======================= COMANDOS (Serial 115200 ou BLE) ===================
 * --- Experimento (palavras, maiúsculas ou minúsculas) ---
 *   MODO MA|P|I|D|PI|PD|PID   (ou 0..6)  seleciona o controle
 *   GANHO kp ki kd            ganhos do controlador selecionado
 *                             (erro em cm, saída em RPM)
 *   REF 0.80                  referência de posição em metros (0..1,5)
 *   VMAX 150                  saturação da velocidade comandada (RPM)
 *   TOL 10                    tolerância de chegada (mm)
 *   TMAX 15                   tempo máximo do experimento (s)
 *   ABERTA tipo valor tempo   malha aberta: tipo 0 = RPM, 1 = PWM fixo (%);
 *                             tempo em s (0 = automático: ref / velocidade, só tipo 0)
 *   RETO 1|0                  liga/desliga a correção "andar reto" na malha fechada
 *   INICIAR | PARAR | REINICIAR
 *   LOG                       envia os dados registrados do último experimento
 *   CFG                       envia as linhas de configuração (C e P)
 *   TEL 1|0                   telemetria também pela Serial (para IHM via USB)
 *   SALVAR | PADRAO           grava na flash / volta aos valores de fábrica
 *
 * --- Comandos da versão 1 (continuam valendo) ---
 *   A D 0..100 | A R rpm | A F hz | A S 1/-1/0 | A W mm | A P ppr | A K kp ki kd
 *   G m | V rpm | R rpm | K kp ki kd | H kp ki kd | L pct | X | M | Z | J f g
 *   I 0/1 | E 0/1 | C | ?
 *   (o prefixo A ou B escolhe a roda; sem prefixo vale para as duas)
 *
 * ======================= LINHAS ENVIADAS À IHM (CSV + \n) ==================
 *   T,...  telemetria das rodas (igual à v1)
 *   C,...  configuração das rodas (igual à v1)
 *   N,...  fim do trajeto do comando G (igual à v1)
 *   E,t,estado,modo,ref,pos,erroMm,u,cmdA,cmdB,rpmA,rpmB,amostras
 *          experimento: 100 ms durante o teste, 200 ms parado
 *   P,modo,ref,vmax,tol,tmax,maTipo,maRpm,maDuty,maTempo,reto,telSerial,
 *     kpMA,kiMA,kdMA,kpP,kiP,kdP, ... ,kpPID,kiPID,kdPID   (7 x 3 ganhos)
 *   R,modo,ref,posFinal,erroMm,sobreMm,sobrePct,tSubida,tAcomod,tTotal,estado
 *   LH,modo,ref,kp,ki,kd,n,unidadeU,estado,dtMs   cabeçalho do LOG
 *   LD,i,tS,pos,u,cmdA,cmdB,rpmA,rpmB             uma amostra
 *   LF,n                                          fim do LOG
 *
 *   estado: 0 PRONTO, 1 EXECUTANDO, 2 FINALIZANDO, 3 CONCLUIDO, 4 PARADO, 5 TEMPO ESGOTADO
 *   modo:   0 MA, 1 P, 2 I, 3 D, 4 PI, 5 PD, 6 PID
 *   unidadeU: 0 = RPM, 1 = % de PWM (malha aberta com PWM fixo)
 */
 
// =====================================================================
// 1. BIBLIOTECAS
// =====================================================================
#include <esp_arduino_version.h>
#include <QuickPID.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =====================================================================
// 2. CONFIGURAÇÃO (constantes ajustáveis)
// =====================================================================

// ---------------- PWM ----------------
const uint8_t  PWM_RES   = 10;       // resolução em bits (0..1023)
const uint32_t FREQ_MIN  = 10;       // Hz
const uint32_t FREQ_MAX  = 40000;    // Hz
const uint32_t FREQ_INIT = 1000;     // Hz

// ---------------- Encoders ----------------
const uint16_t PPR_INIT           = 20;      // furos do disco encoder
const float    DIAMETRO_INIT_MM   = 65.0f;   // roda comum de motor TT amarelo
const uint32_t DEBOUNCE_US        = 300;     // ignora pulsos mais próximos que isso
const uint32_t TIMEOUT_PARADO_US  = 1000000; // sem pulsos por 1 s => RPM = 0
const uint32_t INTERVALO_CALC_MS  = 100;     // período de cálculo da RPM e dos PIDs
const uint32_t INTERVALO_PRINT_MS = 500;     // período do monitor contínuo
const float    FILTRO_ALFA        = 0.5f;    // filtro exponencial da RPM
// O LM393 não detecta o sentido. O sinal da contagem segue o sentido
// comandado, mas só troca quando a roda quase parou (evita contar errado
// os pulsos da inércia logo após uma inversão).
const float    RPM_TROCA_SENTIDO  = 20.0f;
const float    RPM_MINIMA         = 6.0f;    // abaixo disso a roda é considerada parada

// ---------------- PID de RPM (malha interna) ----------------
const float KP_RPM_INIT = 0.3f;
const float KI_RPM_INIT = 1.5f;
const float KD_RPM_INIT = 0.0f;
const float DUTY_MIN = 10.0f;        // zona morta do motor (%)
const float DUTY_MAX_INIT = 50.0f;   // limite de segurança (%), ajustável com "L"

// ---------------- Navegação por distância (comando G, v1) ----------------
const float    RPM_CRUZEIRO_INIT  = 120.0f;
const float    RPM_APROX          = 50.0f;
const float    DIST_DESACEL_M     = 0.15f;
const float    ACEL_RPM_POR_CICLO = 15.0f;
const float    ANTECIPACAO_M      = 0.005f;
const uint32_t FREIO_MS           = 200;
const uint32_t TIMEOUT_TRAVADO_MS = 2000;

// ---------------- PID de andar reto ----------------
// Entrada = diferença de distância A-B (mm), saída = correção de RPM.
const float KP_RETO_INIT     = 1.0f;
const float KI_RETO_INIT     = 0.5f;
const float KD_RETO_INIT     = 0.0f;
const float CORRECAO_MAX_RPM = 30.0f;

// ---------------- Direção manual (J) e inversão (I) ----------------
const bool     RODA_A_ESQUERDA     = false;
const uint32_t TIMEOUT_DIRECAO_MS  = 500;   // sem "J" por esse tempo => para

// ---------------- Experimento de posição ----------------
const uint16_t LOG_MAX          = 600;   // 60 s a 100 ms
const uint32_t ASSENTAMENTO_MS  = 500;   // tempo parado dentro da tolerância para concluir
const uint32_t PARADA_MA_MS     = 300;   // malha aberta: rodas paradas por esse tempo = fim
const uint32_t FINALIZA_MAX_MS  = 2500;  // malha aberta: espera máxima pela parada
const float    FILTRO_D         = 0.5f;  // filtro da derivada (0 = sem filtro)
// Limites da pista: fora deles o experimento é interrompido (proteção, útil no I puro).
const float    POS_MAX_M        = 1.60f;
const float    POS_MIN_M        = -0.40f;

// ---------------- BLE (Nordic UART Service) ----------------
#define BLE_NOME          "Carrinho-ESP32"
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

const uint32_t INTERVALO_TELEMETRIA_MS = 200;
const uint32_t BLE_INTERVALO_PACOTE_MS = 6;
const size_t   BLE_PACOTE              = 20;     // bytes por notificação
const size_t   BLE_FILA_TAM            = 4096;   // fila circular de saída
const size_t   CMD_TAM                 = 64;     // tamanho máximo de um comando
const bool     PARAR_AO_DESCONECTAR    = true;

// =====================================================================
// 3. TIPOS E ESTADO GLOBAL
// =====================================================================

// ---------------- Motor (uma roda) ----------------
struct Motor {
  char     nome;
  uint8_t  pinPWM;   // ENA / ENB (PWMA / PWMB no TB6612)
  uint8_t  pinIN1;
  uint8_t  pinIN2;
  uint8_t  pinEnc;   // saída D0 do módulo LM393
  uint8_t  canal;    // canal LEDC (canais com timers diferentes!)
  uint32_t freq;
  float    dutyPct;
  int8_t   sentido;

  // Encoder / medições
  uint16_t ppr;
  float    diametroMm;
  volatile int32_t  pulsos;        // contagem COM SINAL (posição)
  volatile uint32_t pulsosAbs;     // contagem sem sinal (velocidade)
  volatile uint32_t ultimoPulsoUs;
  volatile int8_t   sentidoEnc;    // sinal usado pela ISR (+1 / -1)
  uint32_t pulsosAnt;
  uint32_t tAntUs;
  bool     temRef;
  float    rpm;                    // módulo
  float    velocidade;             // m/s com sinal
  float    distancia;              // m (odometria com sinal)

  // PID de RPM
  bool     pidAtivo;
  float    pidEntrada;
  float    pidSaida;               // 0..100
  float    pidSetpoint;            // RPM desejada (sempre >= 0)
  float    kp, ki, kd;
};

Motor motores[2] = {
  // nome PWM IN1 IN2 ENC canal freq       duty sentido ppr       diâmetro
  { 'A', 25, 26, 27, 35, 0,    FREQ_INIT, 0,   1,      PPR_INIT, DIAMETRO_INIT_MM },
  { 'B', 33, 32, 13, 34, 2,    FREQ_INIT, 0,   1,      PPR_INIT, DIAMETRO_INIT_MM }
};

portMUX_TYPE muxEnc = portMUX_INITIALIZER_UNLOCKED;   // protege os contadores da ISR

// ---------------- PIDs de rotação e de andar reto ----------------
QuickPID pidA(&motores[0].pidEntrada, &motores[0].pidSaida, &motores[0].pidSetpoint);
QuickPID pidB(&motores[1].pidEntrada, &motores[1].pidSaida, &motores[1].pidSetpoint);
QuickPID *pids[2] = { &pidA, &pidB };

float retoEntrada = 0, retoSaida = 0, retoSetpoint = 0;
float kpReto = KP_RETO_INIT, kiReto = KI_RETO_INIT, kdReto = KD_RETO_INIT;
QuickPID pidReto(&retoEntrada, &retoSaida, &retoSetpoint);

float dutyMax = DUTY_MAX_INIT;   // limite de duty em uso (comando L)

// ---------------- Navegação por distância (comando G) ----------------
enum EstadoNav { NAV_PARADO, NAV_ANDANDO, NAV_FREANDO };
struct Navegacao {
  EstadoNav estado = NAV_PARADO;
  float     alvoM = 0;
  int8_t    sentido = 1;
  int32_t   inicio[2] = {0, 0};
  float     rpmBase = 0;
  float     rpmCruzeiro = RPM_CRUZEIRO_INIT;
  float     ultimaMedia = 0;
  uint32_t  tProgresso = 0;
  uint32_t  tFreio = 0;
  bool      abortado = false;
} nav;

// ---------------- Direção manual e modos de exibição ----------------
bool monitorAtivo  = true;
bool invertido     = false;   // comando I: traseira vira a frente
bool ladosTrocados = false;   // comando E: esquerda e direita trocadas
bool dirigindo     = false;
uint32_t tUltimoJ  = 0;

// ---------------- Experimento: modos e estados ----------------
enum ModoCtrl { MODO_MA, MODO_P, MODO_I, MODO_D, MODO_PI, MODO_PD, MODO_PID, N_MODOS };
const char *NOMES_MODO[N_MODOS] = { "MA", "P", "I", "D", "PI", "PD", "PID" };
// bit0 = P, bit1 = I, bit2 = D
const uint8_t TERMOS[N_MODOS] = { 0, 1, 2, 4, 3, 5, 7 };

enum EstadoExp { EXP_PRONTO, EXP_EXECUTANDO, EXP_FINALIZANDO, EXP_CONCLUIDO, EXP_PARADO, EXP_TEMPO_ESGOTADO };
const char *NOMES_ESTADO[] = { "PRONTO", "EXECUTANDO", "FINALIZANDO", "CONCLUIDO", "PARADO", "TEMPO ESGOTADO" };

// Ganhos de fábrica (erro em cm -> velocidade em RPM). Ponto de partida
// para a sintonia; ajuste no robô e grave com SALVAR.
const float GANHOS_PADRAO[N_MODOS][3] = {
  { 0.0f, 0.0f, 0.0f },   // MA (não usa)
  { 4.0f, 0.0f, 0.0f },   // P
  { 0.0f, 1.0f, 0.0f },   // I (use VMAX ~60 para caber na pista)
  { 0.0f, 0.0f, 3.0f },   // D
  { 3.0f, 0.3f, 0.0f },   // PI
  { 4.0f, 0.0f, 1.5f },   // PD
  { 3.5f, 0.3f, 1.2f },   // PID
};

// ---------------- Experimento: parâmetros e execução ----------------
struct Experimento {
  // parâmetros (ajustados pelos comandos MODO, GANHO, REF, VMAX, TOL, TMAX, ABERTA, RETO)
  uint8_t   modo = MODO_PID;
  float     ganhos[N_MODOS][3];
  float     refM = 0.50f;
  float     vmaxRpm = 150.0f;
  float     tolMm = 10.0f;
  float     tmaxS = 15.0f;
  uint8_t   maTipo = 0;          // 0 = RPM, 1 = PWM fixo
  float     maRpm = 100.0f;
  float     maDuty = 30.0f;
  float     maTempoS = 0.0f;     // 0 = automático
  bool      reto = true;

  // execução
  EstadoExp estado = EXP_PRONTO;
  uint32_t  t0 = 0;
  uint32_t  tFimMA = 0;          // duração da malha aberta (ms)
  uint32_t  tFinalizar = 0;
  uint32_t  tCond = 0;           // início da condição de término
  bool      freando = false;
  float     eAnt = 0, integral = 0, dFilt = 0;
  float     u = 0, cmdA = 0, cmdB = 0;
  float     pos = 0, erro = 0;
} ex;

// ---------------- Registro do experimento (RAM) ----------------
struct Amostra { uint32_t t; float pos, u, cmdA, cmdB, rpmA, rpmB; };
Amostra logBuf[LOG_MAX];
uint16_t logN = 0;
uint8_t  logModo = 0;
float    logRef = 0, logKp = 0, logKi = 0, logKd = 0;
uint8_t  logUnidade = 0;

// Envio do LOG em partes (sem travar o loop)
bool    dumpAtivo = false, dumpBle = false, dumpSerial = false;
int32_t dumpIdx = -1;

bool telSerial = false;     // telemetria também pela Serial (IHM via USB)
Preferences prefs;

// ---------------- Declarações antecipadas ----------------
void cancelarExperimento();
void enviarParametros();

// =====================================================================
// 4. COMUNICAÇÃO (BLE, fila de saída, Serial)
// =====================================================================
BLEServer         *bleServidor = nullptr;
BLECharacteristic *bleTx       = nullptr;
volatile bool bleConectado     = false;
volatile bool bleDesconectou   = false;
QueueHandle_t filaComandos;      // comandos recebidos pelo BLE (ISR do BLE -> loop)
bool comandoViaBle = false;      // true enquanto processa um comando vindo do BLE

// Fila circular de bytes a enviar pelo BLE (enviada em pacotes de 20 bytes)
uint8_t  bleFila[BLE_FILA_TAM];
size_t   bleIni = 0, bleFim = 0;
uint32_t bleUltimoEnvio = 0;

size_t bleLivre() { return (bleIni + BLE_FILA_TAM - bleFim - 1) % BLE_FILA_TAM; }

void bleFilaByte(uint8_t c) {
  if (!bleConectado || bleLivre() == 0) return;
  bleFila[bleFim] = c;
  bleFim = (bleFim + 1) % BLE_FILA_TAM;
}

void bleFilaTexto(const char *txt) {
  size_t n = strlen(txt);
  if (!bleConectado || bleLivre() < n) return;
  for (size_t i = 0; i < n; i++) bleFilaByte((uint8_t)txt[i]);
}

// Envia um pacote da fila (chamada a cada volta do loop).
void bleEnviarPendentes() {
  if (!bleConectado) { bleIni = bleFim = 0; return; }
  if (bleIni == bleFim || millis() - bleUltimoEnvio < BLE_INTERVALO_PACOTE_MS) return;
  uint8_t pacote[BLE_PACOTE];
  size_t n = 0;
  while (n < BLE_PACOTE && bleIni != bleFim) {
    pacote[n++] = bleFila[bleIni];
    bleIni = (bleIni + 1) % BLE_FILA_TAM;
  }
  bleTx->setValue(pacote, n);
  bleTx->notify();
  bleUltimoEnvio = millis();
}

// Saída de texto para Serial e BLE ao mesmo tempo (usada nas mensagens e respostas).
class SaidaDupla : public Print {
public:
  size_t write(uint8_t c) override {
    Serial.write(c);
    bleFilaByte(c);
    return 1;
  }
  size_t write(const uint8_t *buf, size_t n) override {
    Serial.write(buf, n);
    for (size_t i = 0; i < n; i++) bleFilaByte(buf[i]);
    return n;
  }
};
SaidaDupla Saida;

// Linhas de dados (T, E): BLE sempre; Serial só com "TEL 1".
void enviarDados(const char *linha) {
  bleFilaTexto(linha);
  if (telSerial) Serial.print(linha);
}

class ServidorCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { bleConectado = true; }
  void onDisconnect(BLEServer *s) override {
    bleConectado = false;
    bleDesconectou = true;
  }
};

// Monta as linhas de comando recebidas pelo BLE e as coloca na fila.
class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    static char linha[CMD_TAM];
    static size_t n = 0;
    auto valor = c->getValue();
    const char *d = valor.c_str();
    for (size_t i = 0; i < valor.length(); i++) {
      char ch = d[i];
      if (ch == '\n' || ch == '\r') {
        if (n > 0) {
          linha[n] = '\0';
          xQueueSend(filaComandos, linha, 0);
          n = 0;
        }
      } else if (n < CMD_TAM - 1) {
        linha[n++] = ch;
      }
    }
  }
};

void bleIniciar() {
  filaComandos = xQueueCreate(8, CMD_TAM);
  BLEDevice::init(BLE_NOME);
  bleServidor = BLEDevice::createServer();
  bleServidor->setCallbacks(new ServidorCB());

  BLEService *svc = bleServidor->createService(NUS_SERVICE_UUID);
  bleTx = svc->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  BLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();

  BLEAdvertising *anuncio = BLEDevice::getAdvertising();
  anuncio->addServiceUUID(NUS_SERVICE_UUID);
  anuncio->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// =====================================================================
// 5. HARDWARE (PWM, encoder, acionamento dos motores)
// =====================================================================

// ---------------- PWM (camada que isola o core 2.x do 3.x) ----------------
void pwmIniciar(Motor &m) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(m.pinPWM, m.freq, PWM_RES, m.canal);
#else
  ledcSetup(m.canal, m.freq, PWM_RES);
  ledcAttachPin(m.pinPWM, m.canal);
#endif
}

void pwmEscrever(Motor &m, uint32_t valor) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(m.pinPWM, valor);
#else
  ledcWrite(m.canal, valor);
#endif
}

bool pwmMudarFrequencia(Motor &m, uint32_t freq) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcChangeFrequency(m.pinPWM, freq, PWM_RES) != 0;
#else
  return ledcChangeFrequency(m.canal, freq, PWM_RES) != 0;
#endif
}

// ---------------- Encoder: interrupção e leitura ----------------
void IRAM_ATTR isrEncoder(void *arg) {
  Motor *m = (Motor *)arg;
  uint32_t agora = micros();
  if (agora - m->ultimoPulsoUs < DEBOUNCE_US) return;
  portENTER_CRITICAL_ISR(&muxEnc);
  m->pulsos += m->sentidoEnc;
  m->pulsosAbs++;
  m->ultimoPulsoUs = agora;
  portEXIT_CRITICAL_ISR(&muxEnc);
}

int32_t lerPulsos(Motor &m) {
  portENTER_CRITICAL(&muxEnc);
  int32_t p = m.pulsos;
  portEXIT_CRITICAL(&muxEnc);
  return p;
}

float circunferencia(Motor &m) {
  return PI * m.diametroMm / 1000.0f;
}

// Atualiza RPM (filtrada), velocidade e odometria. Chamada a cada 100 ms.
void calcularVelocidade(Motor &m) {
  portENTER_CRITICAL(&muxEnc);
  uint32_t pa = m.pulsosAbs;
  int32_t  ps = m.pulsos;
  uint32_t t  = m.ultimoPulsoUs;
  portEXIT_CRITICAL(&muxEnc);

  uint32_t n = pa - m.pulsosAnt;

  if (n > 0) {
    if (m.temRef) {
      uint32_t dt = t - m.tAntUs;
      if (dt > 0) {
        float rpmInst = (n * 60000000.0f) / ((float)m.ppr * dt);
        m.rpm = (m.rpm == 0) ? rpmInst
                             : FILTRO_ALFA * rpmInst + (1.0f - FILTRO_ALFA) * m.rpm;
      }
    }
    m.temRef    = true;
    m.pulsosAnt = pa;
    m.tAntUs    = t;
  } else if (micros() - m.tAntUs > TIMEOUT_PARADO_US) {
    m.rpm    = 0;
    m.temRef = false;
  } else if (m.temRef) {
    // Sem pulso novo: a RPM não pode ser maior que "1 pulso desde o último".
    // Faz a leitura cair rápido quando a roda para (antes demorava 1 s).
    float rpmMax = 60000000.0f / ((float)m.ppr * (micros() - m.tAntUs));
    if (m.rpm > rpmMax) m.rpm = (rpmMax < RPM_MINIMA) ? 0 : rpmMax;
  }

  // Atualiza o sinal da contagem quando a roda já quase parou.
  if (m.sentido != 0 && m.sentido != m.sentidoEnc && m.rpm < RPM_TROCA_SENTIDO) {
    m.sentidoEnc = m.sentido;
  }

  m.velocidade = m.sentidoEnc * (m.rpm / 60.0f) * circunferencia(m);
  m.distancia  = ((float)ps / m.ppr) * circunferencia(m);
}

float rpmComSinal(Motor &m) { return m.sentidoEnc * m.rpm; }

void zerarOdometria() {
  portENTER_CRITICAL(&muxEnc);
  for (Motor &m : motores) m.pulsos = 0;
  portEXIT_CRITICAL(&muxEnc);
  for (Motor &m : motores) m.distancia = 0;
  nav.inicio[0] = nav.inicio[1] = 0;
}

// ---------------- Acionamento dos motores ----------------
void aplicarSentido(Motor &m) {
  digitalWrite(m.pinIN1, m.sentido == 1);
  digitalWrite(m.pinIN2, m.sentido == -1);
}

void aplicarDuty(Motor &m) {
  if (m.dutyPct < 0)   m.dutyPct = 0;
  if (m.dutyPct > dutyMax) m.dutyPct = dutyMax;
  uint32_t maximo = (1UL << PWM_RES) - 1;
  uint32_t valor  = (m.sentido == 0) ? 0 : (uint32_t)(m.dutyPct * maximo / 100.0f + 0.5f);
  pwmEscrever(m, valor);
}

// Freio ativo: IN1 = IN2 = HIGH (L298N / TB6612).
void aplicarFreio(Motor &m) {
  digitalWrite(m.pinIN1, HIGH);
  digitalWrite(m.pinIN2, HIGH);
  pwmEscrever(m, (1UL << PWM_RES) - 1);
}

// Para o motor e desliga o PID de rotação dele.
void soltarMotor(Motor &m) {
  m.pidAtivo = false;
  m.pidSetpoint = 0;
  m.dutyPct = 0;
  m.sentido = 0;
  aplicarSentido(m);
  aplicarDuty(m);
}

// =====================================================================
// 6. CONTROLE (PID de rotação, navegação "G", direção manual)
// =====================================================================

// ---------------- PID de rotação (malha interna) ----------------
void pidReiniciar(QuickPID &pid, float &saida) {
  saida = 0;
  pid.SetMode(QuickPID::Control::manual);
  pid.SetMode(QuickPID::Control::timer);
}

void controlarRPM(uint8_t i) {
  Motor &m = motores[i];
  if (!m.pidAtivo) return;

  if (m.pidSetpoint <= 0.0f || m.sentido == 0) {
    m.dutyPct = 0;
    aplicarDuty(m);
    pidReiniciar(*pids[i], m.pidSaida);
    return;
  }

  m.pidEntrada = m.rpm;
  pids[i]->Compute();
  m.dutyPct = DUTY_MIN + m.pidSaida * (dutyMax - DUTY_MIN) / 100.0f;
  aplicarDuty(m);
}

// Define a RPM desejada da roda (sinal = sentido). Liga o PID se estiver desligado.
void definirRpm(uint8_t i, float rpm) {
  Motor &m = motores[i];
  m.sentido = (rpm > 0) ? 1 : (rpm < 0 ? -1 : 0);
  aplicarSentido(m);
  if (!m.pidAtivo) {
    m.pidAtivo = true;
    pidReiniciar(*pids[i], m.pidSaida);
  }
  m.pidSetpoint = fabsf(rpm);
}

// ---------------- Navegação por distância (comando G) ----------------
void distanciasNav(float &dA, float &dB) {
  dA = nav.sentido * (float)(lerPulsos(motores[0]) - nav.inicio[0]) / motores[0].ppr * circunferencia(motores[0]);
  dB = nav.sentido * (float)(lerPulsos(motores[1]) - nav.inicio[1]) / motores[1].ppr * circunferencia(motores[1]);
}

void iniciarNavegacao(float distM) {
  cancelarExperimento();
  bool frenteUsuario = distM > 0;
  if (invertido) distM = -distM;
  dirigindo = false;
  nav.alvoM   = fabsf(distM);
  nav.sentido = (distM > 0) ? 1 : -1;

  for (uint8_t i = 0; i < 2; i++) {
    Motor &m = motores[i];
    m.sentido = nav.sentido;
    aplicarSentido(m);
    m.pidAtivo = true;
    m.pidSetpoint = 0;
    pidReiniciar(*pids[i], m.pidSaida);
    nav.inicio[i] = lerPulsos(m);
  }
  pidReiniciar(pidReto, retoSaida);

  nav.rpmBase     = min(RPM_APROX, nav.rpmCruzeiro);
  nav.ultimaMedia = 0;
  nav.tProgresso  = millis();
  nav.abortado    = false;
  nav.estado      = NAV_ANDANDO;
  Saida.printf("Navegacao: %.3f m %s a %.0f RPM\n",
                nav.alvoM, frenteUsuario ? "para frente" : "de re", nav.rpmCruzeiro);
}

void iniciarFreio(bool abortado) {
  for (Motor &m : motores) {
    m.pidAtivo = false;
    m.pidSetpoint = 0;
    m.dutyPct = 0;
    aplicarFreio(m);
  }
  nav.abortado = abortado;
  nav.tFreio   = millis();
  nav.estado   = NAV_FREANDO;
}

void cancelarNavegacao() {
  if (nav.estado != NAV_PARADO) {
    nav.estado = NAV_PARADO;
    Saida.println(F("Navegacao cancelada."));
  }
}

// Comandos manuais (D, R, S, J, G...) cancelam a navegação e o experimento.
void cancelarManual() {
  cancelarNavegacao();
  cancelarExperimento();
}

// Rampa de aceleração, desaceleração perto do alvo e correção de andar reto.
void atualizarNavegacao() {
  if (nav.estado != NAV_ANDANDO) return;

  float dA, dB;
  distanciasNav(dA, dB);
  float media    = (dA + dB) / 2.0f;
  float restante = nav.alvoM - media;

  if (media > nav.ultimaMedia + 0.001f) {
    nav.ultimaMedia = media;
    nav.tProgresso  = millis();
  } else if (millis() - nav.tProgresso > TIMEOUT_TRAVADO_MS) {
    Saida.println(F("ERRO: sem progresso (roda travada ou encoder sem sinal). Abortando."));
    iniciarFreio(true);
    return;
  }

  nav.rpmBase += ACEL_RPM_POR_CICLO;
  if (nav.rpmBase > nav.rpmCruzeiro) nav.rpmBase = nav.rpmCruzeiro;

  float rpmAprox = min(RPM_APROX, nav.rpmCruzeiro);
  float limite = rpmAprox + (nav.rpmCruzeiro - rpmAprox) * (restante / DIST_DESACEL_M);
  if (limite < rpmAprox) limite = rpmAprox;
  if (nav.rpmBase > limite) nav.rpmBase = limite;

  // Em coordenadas "da navegação" (sempre positivas): A andou mais -> reduz A.
  retoEntrada = (dA - dB) * 1000.0f;
  pidReto.Compute();

  motores[0].pidSetpoint = max(0.0f, nav.rpmBase + retoSaida);
  motores[1].pidSetpoint = max(0.0f, nav.rpmBase - retoSaida);
}

// Detecta a chegada (começa a frear) e conclui o trajeto depois do freio.
void verificarChegada() {
  if (nav.estado == NAV_ANDANDO) {
    float dA, dB;
    distanciasNav(dA, dB);
    if ((dA + dB) / 2.0f >= nav.alvoM - ANTECIPACAO_M) iniciarFreio(false);
  }

  if (nav.estado == NAV_FREANDO && millis() - nav.tFreio >= FREIO_MS) {
    for (Motor &m : motores) {
      m.sentido = 0;
      aplicarSentido(m);
      aplicarDuty(m);
    }
    nav.estado = NAV_PARADO;

    float dA, dB;
    distanciasNav(dA, dB);
    float media = (dA + dB) / 2.0f;
    Saida.println(nav.abortado ? F("------ NAVEGACAO ABORTADA ------")
                                : F("------ CHEGOU ------"));
    Saida.printf("Alvo: %.3f m | percorrido: %.3f m (A=%.3f B=%.3f)\n",
                  nav.alvoM, media, dA, dB);
    Saida.printf("Erro: %+.1f mm | diferenca A-B: %+.1f mm\n",
                  (media - nav.alvoM) * 1000.0f, (dA - dB) * 1000.0f);
    Saida.printf("N,%.3f,%.3f,%.3f,%.3f,%d\n", nav.alvoM, media, dA, dB, nav.abortado ? 1 : 0);
  }
}

// ---------------- Parada geral e direção manual (J) ----------------
void pararTudo() {
  cancelarNavegacao();
  cancelarExperimento();
  dirigindo = false;
  for (Motor &m : motores) soltarMotor(m);
  Saida.println(F("PARADO."));
}

// frente e giro em RPM; giro > 0 vira à direita.
void dirigir(float frente, float giro) {
  cancelarManual();
  if (invertido) frente = -frente;
  float esq = constrain(frente + giro, -300.0f, 300.0f);
  float dir = constrain(frente - giro, -300.0f, 300.0f);
  bool aEsquerda = RODA_A_ESQUERDA != ladosTrocados;
  definirRpm(0, aEsquerda ? esq : dir);
  definirRpm(1, aEsquerda ? dir : esq);
  dirigindo = (frente != 0 || giro != 0);
  tUltimoJ  = millis();
}

// =====================================================================
// 7. EXPERIMENTO (malha aberta e malha fechada de posição)
// =====================================================================
bool experimentoAtivo() {
  return ex.estado == EXP_EXECUTANDO || ex.estado == EXP_FINALIZANDO;
}

// Posição no referencial do usuário ("frente" respeita o comando I).
float posicaoAtual() {
  float dA = (float)lerPulsos(motores[0]) / motores[0].ppr * circunferencia(motores[0]);
  float dB = (float)lerPulsos(motores[1]) / motores[1].ppr * circunferencia(motores[1]);
  float sF = invertido ? -1.0f : 1.0f;
  return sF * (dA + dB) / 2.0f;
}

// Guarda uma amostra no registro (RAM) a cada 100 ms.
void registrarAmostra() {
  if (logN >= LOG_MAX) return;
  Amostra &a = logBuf[logN++];
  a.t    = millis() - ex.t0;
  a.pos  = ex.pos;
  a.u    = ex.u;
  a.cmdA = ex.cmdA;
  a.cmdB = ex.cmdB;
  a.rpmA = rpmComSinal(motores[0]);
  a.rpmB = rpmComSinal(motores[1]);
}

// Calcula as métricas a partir do registro e envia o resultado (texto + linha R).
void enviarResultado(EstadoExp final) {
  float ref = logRef;
  float posFinal = posicaoAtual();
  float maxPos = posFinal;
  float tSub10 = -1, tSub90 = -1, tAcom = 0;
  float tolM = ex.tolMm / 1000.0f;
  for (uint16_t i = 0; i < logN; i++) {
    float p = logBuf[i].pos, t = logBuf[i].t / 1000.0f;
    if (p > maxPos) maxPos = p;
    if (tSub10 < 0 && p >= 0.1f * ref) tSub10 = t;
    if (tSub90 < 0 && p >= 0.9f * ref) tSub90 = t;
    if (fabsf(ref - p) > tolM) tAcom = (i + 1 < logN) ? logBuf[i + 1].t / 1000.0f : -1;
  }
  if (fabsf(ref - posFinal) > tolM) tAcom = -1;
  float tSubida = (tSub10 >= 0 && tSub90 >= 0) ? tSub90 - tSub10 : -1;
  float sobreMm = max(0.0f, (maxPos - ref) * 1000.0f);
  float sobrePct = ref > 0 ? sobreMm / (ref * 10.0f) : 0;
  float tTotal = logN ? logBuf[logN - 1].t / 1000.0f : 0;
  float erroMm = (ref - posFinal) * 1000.0f;

  Saida.printf("------ %s | %s ------\n", NOMES_MODO[logModo], NOMES_ESTADO[final]);
  Saida.printf("Ref %.3f m | final %.3f m | erro %+.1f mm | sobressinal %.1f mm (%.1f%%)\n",
               ref, posFinal, erroMm, sobreMm, sobrePct);
  Saida.printf("Subida(10-90%%) %.2f s | acomodacao %.2f s | duracao %.2f s | %u amostras\n",
               tSubida, tAcom, tTotal, logN);
  Saida.printf("R,%u,%.3f,%.4f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%d\n",
               logModo, ref, posFinal, erroMm, sobreMm, sobrePct, tSubida, tAcom, tTotal, (int)final);
}

void encerrarExperimento(EstadoExp final) {
  for (Motor &m : motores) soltarMotor(m);
  bool estavaAtivo = experimentoAtivo();
  ex.estado = final;
  if (estavaAtivo) enviarResultado(final);
}

void cancelarExperimento() {
  if (experimentoAtivo()) encerrarExperimento(EXP_PARADO);
}

bool iniciarExperimento() {
  if (experimentoAtivo()) { Saida.println(F("Experimento ja em execucao.")); return false; }
  if (ex.modo == MODO_MA && ex.maTipo == 1 && ex.maTempoS <= 0) {
    Saida.println(F("Malha aberta com PWM fixo: defina o tempo (ABERTA 1 duty tempo).")); return false;
  }
  cancelarNavegacao();
  dirigindo = false;
  for (Motor &m : motores) soltarMotor(m);
  zerarOdometria();                    // cada teste começa na posição 0
  for (Motor &m : motores) m.sentidoEnc = invertido ? -1 : 1;

  logN = 0; logModo = ex.modo; logRef = ex.refM;
  logKp = ex.ganhos[ex.modo][0]; logKi = ex.ganhos[ex.modo][1]; logKd = ex.ganhos[ex.modo][2];
  logUnidade = (ex.modo == MODO_MA && ex.maTipo == 1) ? 1 : 0;

  ex.pos = 0; ex.erro = ex.refM;
  ex.eAnt = ex.refM * 100.0f;          // sem "chute" da derivada no início
  ex.integral = 0; ex.dFilt = 0; ex.u = 0; ex.cmdA = ex.cmdB = 0;
  ex.tCond = 0; ex.freando = false;
  pidReiniciar(pidReto, retoSaida);

  if (ex.modo == MODO_MA) {
    float tempo = ex.maTempoS;
    if (tempo <= 0) {                  // automático: t = distância / velocidade
      float v = ex.maRpm / 60.0f * (circunferencia(motores[0]) + circunferencia(motores[1])) / 2.0f;
      tempo = v > 0 ? ex.refM / v : 0;
    }
    ex.tFimMA = (uint32_t)(tempo * 1000.0f + 0.5f);
    Saida.printf("Malha aberta: %s %.0f%s por %.2f s (ref %.3f m)\n",
                 ex.maTipo ? "PWM" : "velocidade", ex.maTipo ? ex.maDuty : ex.maRpm,
                 ex.maTipo ? "%" : " RPM", tempo, ex.refM);
  } else {
    Saida.printf("Malha fechada %s: Kp=%.3f Ki=%.3f Kd=%.3f ref=%.3f m\n",
                 NOMES_MODO[ex.modo], logKp, logKi, logKd, ex.refM);
  }
  ex.t0 = millis();
  ex.estado = EXP_EXECUTANDO;
  registrarAmostra();                  // amostra t = 0
  return true;
}

// Rodas no referencial do usuário: v > 0 = para frente.
void comandarRodas(float rpmA, float rpmB) {
  float sF = invertido ? -1.0f : 1.0f;
  ex.cmdA = rpmA; ex.cmdB = rpmB;
  definirRpm(0, sF * rpmA);
  definirRpm(1, sF * rpmB);
}

// Malha externa. Chamada a cada 100 ms. Etapas:
//   1) limites da pista  2) finalizando (MA)  3) malha aberta
//   4) malha fechada: P/I/D + anti-windup + andar reto  5) critério de término
void atualizarExperimento() {
  if (!experimentoAtivo()) return;
  const float dt = INTERVALO_CALC_MS / 1000.0f;
  uint32_t agora = millis();
  uint32_t t = agora - ex.t0;
  ex.pos  = posicaoAtual();
  ex.erro = ex.refM - ex.pos;

  // ---------- 1) Proteção: limites da pista ----------
  if (ex.estado == EXP_EXECUTANDO && (ex.pos > POS_MAX_M || ex.pos < POS_MIN_M)) {
    Saida.printf("Limite de pista atingido (%.2f m). Experimento interrompido.\n", ex.pos);
    registrarAmostra();
    encerrarExperimento(EXP_PARADO);
    return;
  }

  // ---------- 2) Finalizando (malha aberta: freio e espera a parada) ----------
  if (ex.estado == EXP_FINALIZANDO) {
    if (ex.freando && agora - ex.tFinalizar >= FREIO_MS) {
      ex.freando = false;
      for (Motor &m : motores) soltarMotor(m);
    }
    bool parado = motores[0].rpm < 1 && motores[1].rpm < 1;
    if (parado) { if (!ex.tCond) ex.tCond = agora; } else ex.tCond = 0;
    registrarAmostra();
    if ((ex.tCond && agora - ex.tCond >= PARADA_MA_MS) || agora - ex.tFinalizar > FINALIZA_MAX_MS) {
      encerrarExperimento(EXP_CONCLUIDO);
    }
    return;
  }

  // ---------- 3) Malha aberta ----------
  if (ex.modo == MODO_MA) {
    if (t < ex.tFimMA) {
      if (ex.maTipo == 0) {
        ex.u = ex.maRpm;
        comandarRodas(ex.maRpm, ex.maRpm);       // sem correção pela odometria
      } else {
        ex.u = ex.maDuty;
        int8_t s = invertido ? -1 : 1;
        for (Motor &m : motores) {
          m.pidAtivo = false;
          m.sentido = s;
          aplicarSentido(m);
          m.dutyPct = ex.maDuty;
          aplicarDuty(m);
        }
        ex.cmdA = ex.cmdB = ex.maDuty;
      }
    } else {
      // Tempo esgotado: freia e passa a registrar a parada (inércia).
      ex.u = 0; ex.cmdA = ex.cmdB = 0;
      for (Motor &m : motores) { m.pidAtivo = false; aplicarFreio(m); }
      ex.freando = true;
      ex.tFinalizar = agora;
      ex.tCond = 0;
      ex.estado = EXP_FINALIZANDO;
    }
    registrarAmostra();
    return;
  }

  // ---------- 4) Malha fechada: P, I, D, PI, PD, PID ----------
  uint8_t termos = TERMOS[ex.modo];
  float kp = ex.ganhos[ex.modo][0], ki = ex.ganhos[ex.modo][1], kd = ex.ganhos[ex.modo][2];
  float eCm = ex.erro * 100.0f;
  bool dentro = fabsf(ex.erro * 1000.0f) <= ex.tolMm;

  // Derivada do erro, com filtro
  float de = (eCm - ex.eAnt) / dt;
  ex.dFilt = FILTRO_D * ex.dFilt + (1.0f - FILTRO_D) * de;
  ex.eAnt = eCm;

  float termoP = (termos & 1) ? kp * eCm : 0;
  float termoD = (termos & 4) ? kd * ex.dFilt : 0;
  float termoI = 0;
  if (termos & 2) {
    float novaInt = ex.integral;
    if (!dentro) novaInt += eCm * dt;                 // congela dentro da tolerância
    float uTeste = termoP + ki * novaInt + termoD;
    bool satura = fabsf(uTeste) > ex.vmaxRpm && (uTeste * eCm > 0);
    if (!satura) ex.integral = novaInt;               // anti-windup (integração condicional)
    if (ki > 0) {
      float lim = ex.vmaxRpm / ki;                    // |Ki*I| <= VMAX
      ex.integral = constrain(ex.integral, -lim, lim);
    }
    termoI = ki * ex.integral;
  }

  float u = termoP + termoI + termoD;
  if (dentro) u = 0;                                  // zona morta = tolerância
  u = constrain(u, -ex.vmaxRpm, ex.vmaxRpm);
  ex.u = u;

  // Correção para andar reto (opcional): só altera a diferença entre as rodas.
  float corr = 0;
  if (ex.reto && u != 0) {
    float dA = (float)lerPulsos(motores[0]) / motores[0].ppr * circunferencia(motores[0]);
    float dB = (float)lerPulsos(motores[1]) / motores[1].ppr * circunferencia(motores[1]);
    float sF = invertido ? -1.0f : 1.0f;
    float sU = (u > 0) ? 1.0f : -1.0f;
    retoEntrada = sU * sF * (dA - dB) * 1000.0f;      // >0: roda A adiantada no sentido do movimento
    pidReto.Compute();
    corr = sU * retoSaida;
  } else {
    pidReiniciar(pidReto, retoSaida);
  }
  float cA = u + corr, cB = u - corr;
  if (cA * u < 0) cA = 0;                             // a correção não inverte a roda
  if (cB * u < 0) cB = 0;
  comandarRodas(cA, cB);

  registrarAmostra();

  // ---------- 5) Término: dentro da tolerância e rodas paradas por ASSENTAMENTO_MS ----------
  bool parado = motores[0].rpm < 1 && motores[1].rpm < 1;
  if (dentro && parado) {
    if (!ex.tCond) ex.tCond = agora;
    else if (agora - ex.tCond >= ASSENTAMENTO_MS) { encerrarExperimento(EXP_CONCLUIDO); return; }
  } else {
    ex.tCond = 0;
  }
  if (t >= (uint32_t)(ex.tmaxS * 1000.0f) || logN >= LOG_MAX) encerrarExperimento(EXP_TEMPO_ESGOTADO);
}

void reiniciarExperimento() {
  cancelarExperimento();
  cancelarNavegacao();
  dirigindo = false;
  for (Motor &m : motores) soltarMotor(m);
  zerarOdometria();
  logN = 0;
  dumpAtivo = false;
  ex.estado = EXP_PRONTO;
  ex.pos = 0; ex.erro = ex.refM; ex.u = ex.cmdA = ex.cmdB = 0;
  ex.integral = 0; ex.dFilt = 0;
  Saida.println(F("Experimento reiniciado: odometria zerada, registro apagado."));
}

// =====================================================================
// 8. REGISTRO (LOG enviado em partes)
// =====================================================================
void iniciarDump(bool viaBle) {
  if (logN == 0) { Saida.println(F("LOG vazio: execute um experimento.")); return; }
  if (experimentoAtivo()) { Saida.println(F("Aguarde o fim do experimento.")); return; }
  dumpBle = viaBle;
  dumpSerial = !viaBle || telSerial;
  dumpIdx = -1;
  dumpAtivo = true;
}

// Envia até 3 linhas por volta do loop: LH (cabeçalho), LD (amostras) e LF (fim).
void processarDump() {
  if (!dumpAtivo) return;
  if (dumpBle && !bleConectado) { dumpAtivo = false; return; }
  char l[128];
  for (uint8_t k = 0; k < 3 && dumpAtivo; k++) {
    if (dumpIdx < 0) {
      snprintf(l, sizeof(l), "LH,%u,%.3f,%.3f,%.3f,%.3f,%u,%u,%d,%lu\n",
               logModo, logRef, logKp, logKi, logKd, logN, logUnidade, (int)ex.estado,
               (unsigned long)INTERVALO_CALC_MS);
    } else if (dumpIdx < (int32_t)logN) {
      Amostra &a = logBuf[dumpIdx];
      snprintf(l, sizeof(l), "LD,%ld,%.2f,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
               (long)dumpIdx, a.t / 1000.0f, a.pos, a.u, a.cmdA, a.cmdB, a.rpmA, a.rpmB);
    } else {
      snprintf(l, sizeof(l), "LF,%u\n", logN);
    }
    size_t n = strlen(l);
    if (dumpBle && bleLivre() < n + 160) return;            // deixa espaço para a telemetria
    if (dumpSerial && Serial.availableForWrite() < (int)n) return;
    if (dumpBle) bleFilaTexto(l);
    if (dumpSerial) Serial.print(l);
    if (dumpIdx >= (int32_t)logN) dumpAtivo = false;
    dumpIdx++;
  }
}

// =====================================================================
// 9. ARMAZENAMENTO (flash NVS, namespace "robo")
// =====================================================================
void aplicarGanhosRpm() {
  for (uint8_t i = 0; i < 2; i++) pids[i]->SetTunings(motores[i].kp, motores[i].ki, motores[i].kd);
  pidReto.SetTunings(kpReto, kiReto, kdReto);
}

// Valores de fábrica (não grava na flash; use SALVAR).
void carregarPadroes() {
  memcpy(ex.ganhos, GANHOS_PADRAO, sizeof(ex.ganhos));
  ex.modo = MODO_PID; ex.refM = 0.5f; ex.vmaxRpm = 150; ex.tolMm = 10; ex.tmaxS = 15;
  ex.maTipo = 0; ex.maRpm = 100; ex.maDuty = 30; ex.maTempoS = 0; ex.reto = true;
  for (Motor &m : motores) {
    m.kp = KP_RPM_INIT; m.ki = KI_RPM_INIT; m.kd = KD_RPM_INIT;
    m.diametroMm = DIAMETRO_INIT_MM; m.ppr = PPR_INIT;
  }
  kpReto = KP_RETO_INIT; kiReto = KI_RETO_INIT; kdReto = KD_RETO_INIT;
  dutyMax = DUTY_MAX_INIT;
  nav.rpmCruzeiro = RPM_CRUZEIRO_INIT;
}

void salvarFlash() {
  prefs.begin("robo", false);
  prefs.putBytes("ganhos", ex.ganhos, sizeof(ex.ganhos));
  prefs.putUChar("modo", ex.modo);
  prefs.putFloat("ref", ex.refM);
  prefs.putFloat("vmax", ex.vmaxRpm);
  prefs.putFloat("tol", ex.tolMm);
  prefs.putFloat("tmax", ex.tmaxS);
  prefs.putUChar("maTipo", ex.maTipo);
  prefs.putFloat("maRpm", ex.maRpm);
  prefs.putFloat("maDuty", ex.maDuty);
  prefs.putFloat("maTempo", ex.maTempoS);
  prefs.putBool("reto", ex.reto);
  float rpmG[2][3] = { { motores[0].kp, motores[0].ki, motores[0].kd },
                       { motores[1].kp, motores[1].ki, motores[1].kd } };
  prefs.putBytes("rpmG", rpmG, sizeof(rpmG));
  float retoG[3] = { kpReto, kiReto, kdReto };
  prefs.putBytes("retoG", retoG, sizeof(retoG));
  float calib[4] = { motores[0].diametroMm, motores[1].diametroMm,
                     (float)motores[0].ppr, (float)motores[1].ppr };
  prefs.putBytes("calib", calib, sizeof(calib));
  prefs.putFloat("dutyMax", dutyMax);
  prefs.putFloat("cruzeiro", nav.rpmCruzeiro);
  prefs.end();
  Saida.println(F("Parametros gravados na flash."));
}

// Começa pelos padrões e sobrescreve com o que estiver gravado.
void carregarFlash() {
  carregarPadroes();
  prefs.begin("robo", true);
  if (prefs.getBytesLength("ganhos") == sizeof(ex.ganhos)) {
    prefs.getBytes("ganhos", ex.ganhos, sizeof(ex.ganhos));
    ex.modo     = min((uint8_t)(N_MODOS - 1), prefs.getUChar("modo", ex.modo));
    ex.refM     = prefs.getFloat("ref", ex.refM);
    ex.vmaxRpm  = prefs.getFloat("vmax", ex.vmaxRpm);
    ex.tolMm    = prefs.getFloat("tol", ex.tolMm);
    ex.tmaxS    = prefs.getFloat("tmax", ex.tmaxS);
    ex.maTipo   = prefs.getUChar("maTipo", ex.maTipo);
    ex.maRpm    = prefs.getFloat("maRpm", ex.maRpm);
    ex.maDuty   = prefs.getFloat("maDuty", ex.maDuty);
    ex.maTempoS = prefs.getFloat("maTempo", ex.maTempoS);
    ex.reto     = prefs.getBool("reto", ex.reto);
    float rpmG[2][3];
    if (prefs.getBytes("rpmG", rpmG, sizeof(rpmG)) == sizeof(rpmG)) {
      for (uint8_t i = 0; i < 2; i++) { motores[i].kp = rpmG[i][0]; motores[i].ki = rpmG[i][1]; motores[i].kd = rpmG[i][2]; }
    }
    float retoG[3];
    if (prefs.getBytes("retoG", retoG, sizeof(retoG)) == sizeof(retoG)) {
      kpReto = retoG[0]; kiReto = retoG[1]; kdReto = retoG[2];
    }
    float calib[4];
    if (prefs.getBytes("calib", calib, sizeof(calib)) == sizeof(calib)) {
      motores[0].diametroMm = calib[0]; motores[1].diametroMm = calib[1];
      motores[0].ppr = (uint16_t)calib[2]; motores[1].ppr = (uint16_t)calib[3];
    }
    dutyMax = prefs.getFloat("dutyMax", dutyMax);
    nav.rpmCruzeiro = prefs.getFloat("cruzeiro", nav.rpmCruzeiro);
    Serial.println(F("Parametros carregados da flash."));
  }
  prefs.end();
}

// =====================================================================
// 10. PROTOCOLO (linhas para a IHM e textos de status)
// =====================================================================
const char *nomeSentido(int8_t s) {
  return s == 1 ? "frente" : (s == -1 ? "re" : "parado");
}

// ---------------- Textos para o monitor serial ----------------
void mostrarStatus() {
  Saida.println(F("--------------------------- STATUS ---------------------------"));
  for (Motor &m : motores) {
    Saida.printf("Motor %c [%s]: duty=%.0f%%  freq=%lu Hz  sentido=%s\n",
                  m.nome, m.pidAtivo ? "PID" : "manual", m.dutyPct,
                  (unsigned long)m.freq, nomeSentido(m.sentido));
    Saida.printf("   setpoint=%.1f RPM  medido=%.1f RPM  |  %.3f m/s  |  odometria=%.3f m\n",
                  m.pidSetpoint, m.rpm, m.velocidade, m.distancia);
    Saida.printf("   roda=%.1f mm  disco=%u ppr  |  Kp=%.3f Ki=%.3f Kd=%.3f\n",
                  m.diametroMm, m.ppr, m.kp, m.ki, m.kd);
  }
  Saida.printf("Andar reto: Kp=%.3f Ki=%.3f Kd=%.3f  |  duty max=%.0f%%\n", kpReto, kiReto, kdReto, dutyMax);
  Saida.printf("EXPERIMENTO: controle %s | estado %s | ref %.3f m | Kp=%.3f Ki=%.3f Kd=%.3f\n",
               NOMES_MODO[ex.modo], NOMES_ESTADO[ex.estado], ex.refM,
               ex.ganhos[ex.modo][0], ex.ganhos[ex.modo][1], ex.ganhos[ex.modo][2]);
  Saida.printf("   VMAX %.0f RPM | TOL %.0f mm | TMAX %.0f s | aberta: %s %.0f, %.2f s | reto %s\n",
               ex.vmaxRpm, ex.tolMm, ex.tmaxS, ex.maTipo ? "PWM%" : "RPM",
               ex.maTipo ? ex.maDuty : ex.maRpm, ex.maTempoS, ex.reto ? "on" : "off");
  Saida.println(F("Experimento: MODO x | GANHO kp ki kd | REF m | VMAX | TOL | TMAX | ABERTA t v s | RETO"));
  Saida.println(F("             INICIAR | PARAR | REINICIAR | LOG | CFG | TEL | SALVAR | PADRAO"));
  Saida.println(F("Motor: A/B D duty | R rpm | F freq | S 1/-1/0 | W mm | P ppr | K kp ki kd"));
  Saida.println(F("Geral: G m | V rpm | R rpm | K | H | L pct | J f g | I | E | X | M | Z | ?"));
}

void mostrarMonitor() {
  Motor &a = motores[0];
  Motor &b = motores[1];
  if (experimentoAtivo()) {
    Serial.printf("[%s %s] t=%.1fs ref=%.3f pos=%.3f erro=%+.1fmm u=%.1f | A %.0f/%.0f B %.0f/%.0f RPM\n",
                  NOMES_MODO[ex.modo], NOMES_ESTADO[ex.estado], (millis() - ex.t0) / 1000.0f,
                  ex.refM, ex.pos, ex.erro * 1000.0f, ex.u,
                  ex.cmdA, rpmComSinal(a), ex.cmdB, rpmComSinal(b));
    return;
  }
  Serial.printf("A: sp%6.1f rpm%6.1f duty%4.0f%% | B: sp%6.1f rpm%6.1f duty%4.0f%%",
                a.pidSetpoint, a.rpm, a.dutyPct, b.pidSetpoint, b.rpm, b.dutyPct);
  if (nav.estado == NAV_ANDANDO) {
    float dA, dB;
    distanciasNav(dA, dB);
    Serial.printf(" | %.3f/%.3f m  dif %+.0f mm",
                  (dA + dB) / 2.0f, nav.alvoM, (dA - dB) * 1000.0f);
  }
  Serial.println();
}

// ---------------- Linhas CSV para a IHM ----------------
// Linha C: configuração das rodas.
void enviarConfig() {
  Motor &a = motores[0];
  Motor &b = motores[1];
  Saida.printf("C,%.0f,%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%u,%u,%.0f,%d,%d\n",
               dutyMax, nav.rpmCruzeiro, a.kp, a.ki, a.kd, b.kp, b.ki, b.kd,
               kpReto, kiReto, kdReto, a.diametroMm, b.diametroMm, a.ppr, b.ppr, DUTY_MIN, invertido ? 1 : 0, ladosTrocados ? 1 : 0);
}

// Linha P: parâmetros do experimento e os 7 x 3 ganhos.
void enviarParametros() {
  char l[400];
  int n = snprintf(l, sizeof(l), "P,%u,%.3f,%.0f,%.0f,%.0f,%u,%.0f,%.0f,%.2f,%d,%d",
                   ex.modo, ex.refM, ex.vmaxRpm, ex.tolMm, ex.tmaxS, ex.maTipo,
                   ex.maRpm, ex.maDuty, ex.maTempoS, ex.reto ? 1 : 0, telSerial ? 1 : 0);
  for (uint8_t m = 0; m < N_MODOS && n < (int)sizeof(l) - 40; m++) {
    n += snprintf(l + n, sizeof(l) - n, ",%.3f,%.3f,%.3f", ex.ganhos[m][0], ex.ganhos[m][1], ex.ganhos[m][2]);
  }
  snprintf(l + n, sizeof(l) - n, "\n");
  Saida.print(l);
}

// Linha T: telemetria das rodas (a cada 200 ms).
void enviarTelemetria() {
  Motor &a = motores[0];
  Motor &b = motores[1];
  float dA, dB;
  distanciasNav(dA, dB);
  char linha[160];
  snprintf(linha, sizeof(linha),
           "T,%.1f,%.1f,%.1f,%.1f,%.0f,%.0f,%.3f,%.3f,%d,%.3f,%.3f,%.0f,%d,%d\n",
           a.rpm, b.rpm, a.pidSetpoint, b.pidSetpoint, a.dutyPct, b.dutyPct,
           a.distancia, b.distancia, (int)nav.estado, (dA + dB) / 2.0f, nav.alvoM,
           (dA - dB) * 1000.0f, a.sentidoEnc, b.sentidoEnc);
  enviarDados(linha);
}

// Linha E: telemetria do experimento (100 ms rodando, 200 ms parado).
void enviarTelemetriaExp() {
  char linha[160];
  float t = experimentoAtivo() ? (millis() - ex.t0) / 1000.0f
                               : (logN ? logBuf[logN - 1].t / 1000.0f : 0);
  float pos = experimentoAtivo() ? ex.pos : posicaoAtual();
  snprintf(linha, sizeof(linha), "E,%.2f,%d,%u,%.3f,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u\n",
           t, (int)ex.estado, ex.modo, ex.refM, pos, (ex.refM - pos) * 1000.0f, ex.u,
           ex.cmdA, ex.cmdB, rpmComSinal(motores[0]), rpmComSinal(motores[1]), logN);
  enviarDados(linha);
}

// Resposta padrão depois de um comando que muda parâmetros:
// IHM (BLE ou TEL 1) recebe as linhas C e P; no monitor serial, o texto de status.
void concluirComando() {
  if (comandoViaBle || telSerial) { enviarConfig(); enviarParametros(); }
  else mostrarStatus();
}

// =====================================================================
// 11. COMANDOS (parser e execução)
// =====================================================================

// Lê até "max" números de uma string. Retorna a quantidade lida,
// ou -1 se sobrar texto que não é número.
int lerNumeros(const char *p, float *v, int max) {
  int nv = 0;
  while (nv < max) {
    char *fim;
    float x = strtof(p, &fim);
    if (fim == p) break;
    v[nv++] = x;
    p = fim;
  }
  while (*p == ' ') p++;
  return (*p == '\0') ? nv : -1;
}

// ---------------- Comandos por palavra (experimento) ----------------
// Retorna true se a linha era um comando por palavra.
bool processarPalavra(const String &linha) {
  int esp = linha.indexOf(' ');
  String chave = esp < 0 ? linha : linha.substring(0, esp);
  String resto = esp < 0 ? String("") : linha.substring(esp + 1);
  resto.trim();
  float v[4];
  int nv;

  if (chave == "MODO") {
    if (experimentoAtivo()) { Saida.println(F("Pare o experimento antes de trocar o controle.")); return true; }
    int m = -1;
    for (int i = 0; i < N_MODOS; i++) if (resto == NOMES_MODO[i]) m = i;
    if (m < 0 && resto.length() == 1 && isdigit((unsigned char)resto[0])) m = resto.toInt();
    if (m < 0 || m >= N_MODOS) { Saida.println(F("Uso: MODO MA|P|I|D|PI|PD|PID")); return true; }
    ex.modo = m;
    if (ex.estado != EXP_PRONTO) ex.estado = EXP_PRONTO;
    Saida.printf("Controle selecionado: %s\n", NOMES_MODO[m]);
  } else if (chave == "GANHO") {
    nv = lerNumeros(resto.c_str(), v, 3);
    if (nv != 3 || v[0] < 0 || v[1] < 0 || v[2] < 0) { Saida.println(F("Uso: GANHO kp ki kd (>= 0)")); return true; }
    for (int k = 0; k < 3; k++) ex.ganhos[ex.modo][k] = v[k];
    Saida.printf("Ganhos %s: Kp=%.3f Ki=%.3f Kd=%.3f\n", NOMES_MODO[ex.modo], v[0], v[1], v[2]);
  } else if (chave == "REF") {
    nv = lerNumeros(resto.c_str(), v, 1);
    if (nv != 1 || v[0] < 0 || v[0] > 1.5f) { Saida.println(F("Uso: REF metros (0..1,5)")); return true; }
    ex.refM = v[0];
    if (experimentoAtivo() && ex.modo != MODO_MA) logRef = ex.refM;   // troca de referência em execução
  } else if (chave == "VMAX") {
    nv = lerNumeros(resto.c_str(), v, 1);
    if (nv != 1 || v[0] < 20 || v[0] > 300) { Saida.println(F("Uso: VMAX rpm (20..300)")); return true; }
    ex.vmaxRpm = v[0];
  } else if (chave == "TOL") {
    nv = lerNumeros(resto.c_str(), v, 1);
    if (nv != 1 || v[0] < 1 || v[0] > 100) { Saida.println(F("Uso: TOL mm (1..100)")); return true; }
    ex.tolMm = v[0];
  } else if (chave == "TMAX") {
    nv = lerNumeros(resto.c_str(), v, 1);
    if (nv != 1 || v[0] < 2 || v[0] > 60) { Saida.println(F("Uso: TMAX s (2..60)")); return true; }
    ex.tmaxS = v[0];
  } else if (chave == "ABERTA") {
    nv = lerNumeros(resto.c_str(), v, 3);
    if (nv != 3 || (v[0] != 0 && v[0] != 1) || v[2] < 0 || v[2] > 30) {
      Saida.println(F("Uso: ABERTA tipo(0=RPM,1=PWM) valor tempo_s(0=auto)")); return true;
    }
    if (v[0] == 0 && (v[1] < 10 || v[1] > 300)) { Saida.println(F("RPM: 10..300")); return true; }
    if (v[0] == 1 && (v[1] <= DUTY_MIN || v[1] > dutyMax)) { Saida.printf("PWM: %.0f..%.0f %%\n", DUTY_MIN + 1, dutyMax); return true; }
    ex.maTipo = (uint8_t)v[0];
    if (ex.maTipo == 0) ex.maRpm = v[1]; else ex.maDuty = v[1];
    ex.maTempoS = v[2];
  } else if (chave == "RETO") {
    nv = lerNumeros(resto.c_str(), v, 1);
    if (nv != 1) { Saida.println(F("Uso: RETO 1|0")); return true; }
    ex.reto = v[0] != 0;
  } else if (chave == "INICIAR") {
    iniciarExperimento();
  } else if (chave == "PARAR") {
    pararTudo();
  } else if (chave == "REINICIAR") {
    reiniciarExperimento();
  } else if (chave == "LOG") {
    iniciarDump(comandoViaBle);
    return true;
  } else if (chave == "CFG") {
    enviarConfig(); enviarParametros();
    return true;
  } else if (chave == "TEL") {
    nv = lerNumeros(resto.c_str(), v, 1);
    if (nv != 1) { Saida.println(F("Uso: TEL 1|0")); return true; }
    telSerial = v[0] != 0;
    if (telSerial) monitorAtivo = false;   // não mistura o monitor com a telemetria
  } else if (chave == "SALVAR") {
    salvarFlash();
  } else if (chave == "PADRAO") {
    if (experimentoAtivo()) { Saida.println(F("Pare o experimento primeiro.")); return true; }
    carregarPadroes();
    aplicarGanhosRpm();
    Saida.println(F("Valores de fabrica carregados (use SALVAR para gravar)."));
  } else {
    return false;
  }
  concluirComando();
  return true;
}

// ---------------- Roteador de comandos ----------------
// Palavra com 3+ letras no início => comando do experimento (processarPalavra).
// 1 letra => comando geral (v1). 2 letras (A/B + comando) => comando de uma roda.
void processarComando(String linha) {
  linha.trim();
  linha.toUpperCase();
  if (linha.length() == 0) return;
  if (linha == "?") { mostrarStatus(); return; }

  // Palavra com 3+ letras no início => comando do experimento
  int nAlfa = 0;
  while (nAlfa < (int)linha.length() && isalpha((unsigned char)linha[nAlfa])) nAlfa++;
  if (nAlfa >= 3) {
    if (!processarPalavra(linha)) Saida.println(F("Comando invalido. Digite ? para ajuda."));
    return;
  }

  // Separa as letras (até 2) dos números (até 3)
  const char *p = linha.c_str();
  char letras[3] = {0};
  uint8_t nl = 0;
  while (*p && (isalpha((unsigned char)*p) || *p == ' ')) {
    if (*p != ' ') {
      if (nl >= 2) { Saida.println(F("Comando invalido.")); return; }
      letras[nl++] = *p;
    }
    p++;
  }
  float v[3];
  uint8_t nv = 0;
  while (nv < 3) {
    char *fim;
    float x = strtof(p, &fim);
    if (fim == p) break;
    v[nv++] = x;
    p = fim;
  }
  while (*p == ' ') p++;
  if (*p != '\0' || nl == 0) { Saida.println(F("Formato invalido. Digite ? para ajuda.")); return; }

  // ---------- Comandos gerais de uma letra (v1) ----------
  if (nl == 1) {
    switch (letras[0]) {
      case 'M':
        monitorAtivo = !monitorAtivo;
        Saida.printf("Monitor %s\n", monitorAtivo ? "ligado" : "desligado");
        return;
      case 'Z':
        if (nav.estado != NAV_PARADO || experimentoAtivo()) { Saida.println(F("Nao disponivel em movimento.")); return; }
        zerarOdometria();
        Saida.println(F("Odometria zerada."));
        return;
      case 'X':
        pararTudo();
        return;
      case 'C':
        enviarConfig();
        enviarParametros();
        return;
      case 'J':
        if (nv != 2) { Saida.println(F("Uso: J frente giro (RPM)")); return; }
        dirigir(v[0], v[1]);
        return;
      case 'E':
        if (nv != 1) { Saida.println(F("Uso: E 1 (lados trocados) ou E 0 (normal)")); return; }
        ladosTrocados = (v[0] != 0);
        if (dirigindo) dirigir(0, 0);
        Saida.printf("Esquerda e direita %s\n", ladosTrocados ? "TROCADAS" : "normais");
        break;
      case 'I':
        if (nv != 1) { Saida.println(F("Uso: I 1 (invertido) ou I 0 (normal)")); return; }
        if (experimentoAtivo()) { Saida.println(F("Pare o experimento primeiro.")); return; }
        invertido = (v[0] != 0);
        if (dirigindo) dirigir(0, 0);
        Saida.printf("Controle %s\n", invertido ? "INVERTIDO (traseira = frente)" : "normal");
        break;
      case 'G':
        if (nv != 1 || fabsf(v[0]) < 0.01f || fabsf(v[0]) > 20.0f) {
          Saida.println(F("Uso: G metros (0.01..20, negativo = re)"));
          return;
        }
        iniciarNavegacao(v[0]);
        return;
      case 'V':
        if (nv != 1 || v[0] < 10 || v[0] > 300) { Saida.println(F("Uso: V rpm (10..300)")); return; }
        nav.rpmCruzeiro = v[0];
        break;
      case 'R':
        if (nv != 1 || fabsf(v[0]) > 300) { Saida.println(F("Uso: R rpm (-300..300)")); return; }
        cancelarManual();
        definirRpm(0, v[0]);
        definirRpm(1, v[0]);
        break;
      case 'K':
        if (nv != 3) { Saida.println(F("Uso: K kp ki kd")); return; }
        for (uint8_t i = 0; i < 2; i++) {
          motores[i].kp = v[0]; motores[i].ki = v[1]; motores[i].kd = v[2];
          pids[i]->SetTunings(v[0], v[1], v[2]);
        }
        break;
      case 'L':
        if (nv != 1 || v[0] <= DUTY_MIN || v[0] > 100) {
          Saida.printf("Uso: L pct (%.0f..100)\n", DUTY_MIN + 1);
          return;
        }
        dutyMax = v[0];
        for (Motor &mt : motores) aplicarDuty(mt);
        break;
      case 'H':
        if (nv != 3) { Saida.println(F("Uso: H kp ki kd")); return; }
        kpReto = v[0]; kiReto = v[1]; kdReto = v[2];
        pidReto.SetTunings(kpReto, kiReto, kdReto);
        break;
      default:
        Saida.println(F("Comando invalido. Digite ? para ajuda."));
        return;
    }
    concluirComando();
    return;
  }

  // ---------- Comandos de uma roda (A ou B + comando) ----------
  uint8_t i;
  if (letras[0] == 'A')      i = 0;
  else if (letras[0] == 'B') i = 1;
  else { Saida.println(F("Motor invalido (use A ou B).")); return; }
  Motor &m = motores[i];
  char cmd = letras[1];
  if (nv < 1) { Saida.println(F("Falta o valor do comando.")); return; }
  float valor = v[0];

  switch (cmd) {
    case 'D':
      if (valor < 0 || valor > dutyMax) {
        Saida.printf("Duty deve ser 0..%.0f (limite definido com L)\n", dutyMax);
        return;
      }
      cancelarManual();
      m.pidAtivo = false;
      m.pidSetpoint = 0;
      m.dutyPct = valor;
      aplicarDuty(m);
      break;

    case 'R':
      if (fabsf(valor) > 300) { Saida.println(F("RPM deve ser -300..300")); return; }
      cancelarManual();
      definirRpm(i, valor);
      break;

    case 'F':
      if (valor < FREQ_MIN || valor > FREQ_MAX) {
        Saida.printf("Frequencia deve ser %lu..%lu Hz\n",
                      (unsigned long)FREQ_MIN, (unsigned long)FREQ_MAX);
        return;
      }
      if (!pwmMudarFrequencia(m, (uint32_t)valor)) { Saida.println(F("Falha ao mudar frequencia.")); return; }
      m.freq = (uint32_t)valor;
      aplicarDuty(m);
      break;

    case 'S':
      if (valor != 1 && valor != -1 && valor != 0) { Saida.println(F("Sentido: 1, -1 ou 0")); return; }
      cancelarManual();
      m.sentido = (int8_t)valor;
      aplicarSentido(m);
      aplicarDuty(m);
      break;

    case 'W':
      if (valor < 10 || valor > 500) { Saida.println(F("Diametro deve ser 10..500 mm")); return; }
      m.diametroMm = valor;
      break;

    case 'P':
      if (valor < 1 || valor > 1000) { Saida.println(F("Pulsos/volta deve ser 1..1000")); return; }
      if (nav.estado != NAV_PARADO || experimentoAtivo()) { Saida.println(F("Nao disponivel em movimento.")); return; }
      m.ppr = (uint16_t)valor;
      m.rpm = 0;
      m.temRef = false;
      break;

    case 'K':
      if (nv != 3) { Saida.println(F("Uso: A K kp ki kd")); return; }
      m.kp = v[0]; m.ki = v[1]; m.kd = v[2];
      pids[i]->SetTunings(m.kp, m.ki, m.kd);
      break;

    default:
      Saida.println(F("Comando invalido (use D, R, F, S, W, P ou K)."));
      return;
  }
  concluirComando();
}

// =====================================================================
// 12. SETUP E LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  delay(300);

  carregarFlash();

  // Pinos, PWM, encoders e PIDs de rotação de cada roda
  for (uint8_t i = 0; i < 2; i++) {
    Motor &m = motores[i];
    m.sentidoEnc = 1;
    pinMode(m.pinIN1, OUTPUT);
    pinMode(m.pinIN2, OUTPUT);
    pwmIniciar(m);
    m.sentido = 0;
    aplicarSentido(m);
    aplicarDuty(m);

    // GPIO 34/35 não têm pull-up interno; o módulo LM393 já tem resistor na saída D0.
    pinMode(m.pinEnc, INPUT);
    attachInterruptArg(digitalPinToInterrupt(m.pinEnc), isrEncoder, &m, FALLING);

    pids[i]->SetOutputLimits(0, 100);
    pids[i]->SetSampleTimeUs(INTERVALO_CALC_MS * 1000UL);
    pids[i]->SetMode(QuickPID::Control::manual);
  }
  pidReto.SetOutputLimits(-CORRECAO_MAX_RPM, CORRECAO_MAX_RPM);
  pidReto.SetSampleTimeUs(INTERVALO_CALC_MS * 1000UL);
  pidReto.SetMode(QuickPID::Control::manual);
  aplicarGanhosRpm();

  bleIniciar();
  Saida.println(F("\nRobo ESP32 - controle de posicao (MA, P, I, D, PI, PD, PID) + BLE"));
  Saida.println(F("BLE anunciando como \"" BLE_NOME "\""));
  mostrarStatus();
}

void loop() {
  static uint32_t tCalc = 0, tPrint = 0;
  uint32_t agora = millis();

  // ---------- Ciclo de controle (100 ms) ----------
  if (agora - tCalc >= INTERVALO_CALC_MS) {
    tCalc = agora;
    for (Motor &m : motores) calcularVelocidade(m);
    atualizarNavegacao();
    atualizarExperimento();          // malha externa (posição) antes da interna (RPM)
    controlarRPM(0);
    controlarRPM(1);
    if (experimentoAtivo() || ex.estado == EXP_CONCLUIDO || ex.estado == EXP_TEMPO_ESGOTADO || ex.estado == EXP_PARADO) {
      // telemetria do experimento a cada 100 ms enquanto roda
      static uint8_t div = 0;
      if (experimentoAtivo() || ++div >= 2) { div = 0; enviarTelemetriaExp(); }
    }
  }

  verificarChegada();

  // ---------- Monitor contínuo na Serial ----------
  if (monitorAtivo && agora - tPrint >= INTERVALO_PRINT_MS) {
    tPrint = agora;
    mostrarMonitor();
  }

  // ---------- Comandos (Serial e BLE) ----------
  if (Serial.available()) {
    comandoViaBle = false;
    processarComando(Serial.readStringUntil('\n'));
  }

  char cmd[CMD_TAM];
  while (xQueueReceive(filaComandos, cmd, 0) == pdTRUE) {
    comandoViaBle = true;
    processarComando(String(cmd));
    comandoViaBle = false;
  }

  // ---------- Telemetria (T e, parado, E) a cada 200 ms ----------
  static uint32_t tTelemetria = 0, tDesconexao = 0;
  if ((bleConectado || telSerial) && agora - tTelemetria >= INTERVALO_TELEMETRIA_MS) {
    tTelemetria = agora;
    enviarTelemetria();
    if (ex.estado == EXP_PRONTO) enviarTelemetriaExp();
  }

  // ---------- Desconexão BLE: para o robô e volta a anunciar ----------
  if (bleDesconectou) {
    bleDesconectou = false;
    tDesconexao = agora;
    Serial.println(F("BLE desconectado."));
    if (PARAR_AO_DESCONECTAR) pararTudo();
  }
  if (tDesconexao && agora - tDesconexao > 500) {
    tDesconexao = 0;
    BLEDevice::startAdvertising();
  }

  // ---------- Segurança da direção manual ----------
  if (dirigindo && millis() - tUltimoJ > TIMEOUT_DIRECAO_MS) {
    dirigir(0, 0);
    Saida.println(F("Direcao: sem comando recente, parando."));
  }

  // ---------- Envio em partes (LOG e fila do BLE) ----------
  processarDump();
  bleEnviarPendentes();
}
