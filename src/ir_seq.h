/* Sequencias de codigos IR: gravacao, montagem do envio e formato de
 * armazenamento.
 *
 * Alguns aparelhos (ex.: projetor Epson) mandam, a cada toque, dois
 * quadros com codigos diferentes, como capturas separadas. Este modulo
 * junta os quadros de um toque numa sequencia de ate IR_SEQ_MAX codigos
 * e remonta o envio com o mesmo intervalo entre eles.
 *
 * C puro, sem dependencias do Zephyr, como o ir_decode: o main.c so
 * entrega os quadros e o horario, e os testes no PC exercitam a mesma
 * logica que roda na placa.
 *
 * Todos os horarios sao em microssegundos, num relogio de 32 bits que
 * pode dar a volta: so diferencas (a - b) sao usadas.
 */
#ifndef IR_SEQ_H
#define IR_SEQ_H

#include <stdbool.h>
#include <stdint.h>

#include "ir_decode.h"

/* Maximo de codigos por sequencia (por slot). */
#define IR_SEQ_MAX 3

/* O primeiro codigo aceito abre uma janela: quadros que comecam ate esse
 * tempo depois dele fazem parte da mesma sequencia. Medido no Epson:
 * 108-109 ms entre os dois quadros. */
#define IR_SEQ_WINDOW_US 300000u

/* Depois do fim da janela, espera um quadro que comecou dentro dela
 * terminar e ser entregue: ~80 ms de quadro NEC + 20 ms de silencio
 * que encerra a captura + folga. */
#define IR_SEQ_SETTLE_US 120000u

/* Quadros do mesmo codigo dentro dessa janela sao o mesmo toque. */
#define IR_SEQ_REPEAT_US 200000u

/* Atraso do aviso de falha: se outro quadro do toque decodificar nesse
 * tempo, o aviso e cancelado. */
#define IR_SEQ_WARN_DELAY_US 200000u

/* Silencio minimo entre dois quadros de uma sequencia no envio. */
#define IR_SEQ_MIN_GAP_US 10000u

/* Maior envio de uma sequencia: IR_SEQ_MAX quadros e os silencios. Um
 * codigo so usa o ir_encode, que cabe em IR_ENCODE_MAX. */
#define IR_SEQ_TX_MAX                                                     \
  (IR_SEQ_MAX * IR_ENCODE_FRAME_MAX + (IR_SEQ_MAX - 1) > IR_ENCODE_MAX     \
       ? IR_SEQ_MAX * IR_ENCODE_FRAME_MAX + (IR_SEQ_MAX - 1)               \
       : IR_ENCODE_MAX)

/* ---- Gravacao crua ----
 *
 * Quando nenhum quadro de um toque decodifica (protocolo desconhecido,
 * ar-condicionado, quadros longos), o toque e guardado como os tempos que
 * chegaram, ja sem a portadora (ir_edges.h), e reenviado do mesmo jeito.
 * Funciona com qualquer controle de 38 kHz, mas reproduz tambem a
 * distorcao do receptor (nesta placa, com o filtro, ~0 us). */

/* Maximo de intervalos de uma gravacao crua (ar-condicionado: ~200 a
 * ~600). */
#define IR_RAW_MAX 1024

/* Captura que pode comecar uma gravacao crua: pelo menos isso de
 * intervalos, termina num pulso e o 1o pulso tem pelo menos
 * IR_RAW_MIN_PULSE_US (descarta ruido curto). */
#define IR_RAW_MIN_LEN 11
#define IR_RAW_MIN_PULSE_US 300

/* Capturas com menos que isso de silencio entre elas sao o mesmo toque
 * (ar-condicionado as vezes manda 2 ou 3 quadros com 5 a 40 ms entre
 * eles, e o fim de captura e de 20 ms). */
#define IR_RAW_JOIN_US 200000u

/* Silencio depois da ultima captura para fechar a gravacao crua. */
#define IR_RAW_SETTLE_US 250000u

/* Cada intervalo e guardado em 16 bits; silencios maiores sao cortados
 * aqui (continua sendo silencio entre quadros). */
#define IR_RAW_DUR_MAX 65535u

struct ir_raw {
  int n;
  uint16_t d[IR_RAW_MAX]; /* pulso, espaco, pulso... termina num pulso */
};

struct ir_seq {
  int count; /* 0 = vazio */
  struct ir_code codes[IR_SEQ_MAX];
  /* gap_us[i]: do inicio do codigo i - 1 ao inicio do codigo i.
   * gap_us[0] vale sempre 0. */
  uint32_t gap_us[IR_SEQ_MAX];
};

/* ---- Envio ---- */

/* Monta o envio de uma sequencia.
 *  - Um codigo: ir_encode (quadro + repeticao), igual ao controle.
 *  - Varios: um quadro por codigo, com o intervalo gravado entre os
 *    inicios (no minimo IR_SEQ_MIN_GAP_US de silencio).
 * O primeiro valor e um pulso; o vetor alterna pulso e espaco, como o
 * ir_send espera. Devolve quantos valores foram escritos, ou 0. */
int ir_seq_encode(const struct ir_seq *seq, uint32_t *dur, int max);

/* ---- Armazenamento ---- */

#define IR_SEQ_STORE_VERSION 1
/* versao + quantidade + (protocolo + codigo + intervalo) por codigo */
#define IR_SEQ_STORE_MAX (2 + 9 * IR_SEQ_MAX)

/* Serializa num formato fixo, independente do compilador (little
 * endian, sem padding). Devolve o tamanho, ou 0 se a sequencia e
 * invalida. */
int ir_seq_pack(const struct ir_seq *seq, uint8_t *buf, int max);

/* Le o formato acima e valida tudo (versao, tamanho, protocolo, codigo,
 * intervalos). Devolve 0, ou -1 se os dados nao servem. */
int ir_seq_unpack(const uint8_t *buf, int len, struct ir_seq *seq);

/* true se a sequencia e coerente (o que o unpack aceitaria). */
bool ir_seq_is_valid(const struct ir_seq *seq);

/* Gravacao crua no mesmo armazenamento: versao 2, quantidade (16 bits) e
 * os intervalos (16 bits cada), little endian. */
#define IR_RAW_STORE_VERSION 2
#define IR_RAW_STORE_MAX (3 + 2 * IR_RAW_MAX)

/* true se a gravacao crua e coerente: tamanho impar dentro do limite e
 * nenhum intervalo zero. */
bool ir_raw_is_valid(const struct ir_raw *raw);

int ir_raw_pack(const struct ir_raw *raw, uint8_t *buf, int max);
int ir_raw_unpack(const uint8_t *buf, int len, struct ir_raw *raw);

/* Versao do dado guardado (primeiro byte), ou -1. */
int ir_store_version(const uint8_t *buf, int len);

/* ---- Gravacao ---- */

/* O que aconteceu com um quadro entregue ao gravador. */
enum ir_rec_result {
  IR_REC_ADDED,     /* codigo novo entrou na sequencia */
  IR_REC_SAME,      /* outro quadro do mesmo codigo (mesmo toque) */
  IR_REC_REPEAT,    /* repeticao NEC */
  IR_REC_CYCLE,     /* codigo ja na sequencia: o controle recomecou */
  IR_REC_FULL,      /* sequencia cheia: codigo ignorado */
  IR_REC_LATE,      /* comecou depois da janela: ignorado */
  IR_REC_PENDING,   /* leitura com pulso perdido, esperando confirmacao */
  IR_REC_REJECTED,  /* quadro nao decodificou */
};

/* O que ir_rec_poll tem para o main fazer. */
enum ir_rec_poll {
  IR_REC_POLL_NONE,
  IR_REC_POLL_WARN, /* falha sem nenhum codigo aceito: avisar */
  IR_REC_POLL_DONE, /* janela fechou: sequencia pronta */
  IR_REC_POLL_RAW,  /* nenhum codigo decodificou: gravacao crua pronta em
                     * ir_rec_raw(); o main copia e chama ir_rec_reset */
};

struct ir_recorder {
  /* Ultimo codigo aceito, para reconhecer os outros quadros do toque. */
  struct ir_code last;
  bool has_last;
  uint32_t last_us;

  /* Leitura que supos pulsos perdidos: so vale se outro quadro do
   * mesmo toque confirmar. */
  struct ir_code pending;
  bool has_pending;
  uint32_t pending_us;

  /* Aviso de falha adiado. */
  bool warn_pending;
  uint32_t warn_us;
  const char *warn_reason;

  /* Ultimo quadro rejeitado, para avisar se ele veio logo antes do
   * primeiro codigo aceito (o primeiro quadro de um par pode ter se
   * perdido). */
  bool has_reject;
  uint32_t reject_us;

  /* Sequencia em gravacao. */
  bool open;
  bool cycled; /* o controle recomecou a sequencia: nada mais entra */
  uint32_t start_us;
  struct ir_seq seq;

  /* Algum quadro perto da sequencia falhou: ela pode estar incompleta. */
  bool suspect;

  /* Algum quadro deste toque decodificou (mesmo sem confirmacao): o
   * controle e de um protocolo conhecido, entao nao vale gravar cru. */
  bool known_seen;

  /* Gravacao crua em montagem (so vale se nenhum codigo decodificar). */
  bool raw_on;
  bool raw_overflow;
  uint32_t raw_end_us;
  struct ir_raw raw;
};

/* Zera tudo (ao entrar ou sair do modo gravar). */
void ir_rec_reset(struct ir_recorder *rec);

/* Entrega um quadro. st e o resultado de ir_decode_frame; code e
 * lost_pulses so valem com IR_OK. t_us e o instante da primeira borda
 * do quadro. */
enum ir_rec_result ir_rec_frame(struct ir_recorder *rec, enum ir_status st,
                                const struct ir_code *code, int lost_pulses,
                                uint32_t t_us);

/* Chamar depois de entregar os quadros disponiveis. Com DONE, a
 * sequencia vai para out e o gravador e zerado; com WARN, o motivo
 * fica em rec->warn_reason. */
enum ir_rec_poll ir_rec_poll(struct ir_recorder *rec, uint32_t now_us,
                             struct ir_seq *out, bool *suspect);

/* Entrega a captura inteira (depois dos quadros dela). Se nenhum codigo
 * foi aceito, ela entra na gravacao crua. t_us e o inicio da captura;
 * truncated: a captura encheu o buffer de bordas. */
void ir_rec_capture(struct ir_recorder *rec, const uint32_t *dur, int n,
                    uint32_t t_us, bool truncated);

/* Gravacao crua pronta (depois de IR_REC_POLL_RAW). raw_overflow diz se
 * o toque passou de IR_RAW_MAX e foi cortado. */
const struct ir_raw *ir_rec_raw(const struct ir_recorder *rec);

#endif /* IR_SEQ_H */
