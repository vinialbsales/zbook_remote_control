/* Decodificador de IR robusto para NEC e Samsung32.
 *
 * C puro, sem dependencias do Zephyr: compila no PC (testes) e na placa.
 *
 * Entrada: duracoes em microssegundos, alternando pulso (portadora
 * ligada), espaco, pulso... sempre comecando por um pulso.
 *
 * O receptor da placa estica os pulsos e encurta os espacos por uma
 * quantidade variavel (o "esticamento"), mas preserva o periodo entre o
 * inicio de um pulso e o inicio do seguinte. O decodificador usa esse
 * periodo, e nao a largura do pulso, para contar os bits.
 */
#ifndef IR_DECODE_H
#define IR_DECODE_H

#include <stdint.h>

/* Espaco acima disso separa dois quadros. */
#define IR_FRAME_GAP_US 10000

/* Quadro ideal: cabecalho (2) + 32 bits (64) + pulso final (1). */
#define IR_FRAME_LEN 67

/* Maior sequencia que ir_encode gera (dois quadros e o intervalo). */
#define IR_ENCODE_MAX (2 * IR_FRAME_LEN + 1)

enum ir_protocol {
  IR_PROTO_NONE = 0,
  IR_PROTO_NEC,
  IR_PROTO_SAMSUNG32,
};

enum ir_status {
  IR_OK = 0,
  IR_REPEAT,        /* codigo de repeticao do NEC (botao segurado) */
  IR_ERR_HEADER,    /* nao comeca com um cabecalho conhecido */
  IR_ERR_TRUNCATED, /* quadro cortado: termina num espaco */
  IR_ERR_DECODE,    /* nenhuma interpretacao com 32 bits coerentes */
  IR_ERR_CHECK,     /* ha 32 bits, mas nenhum passa na validacao */
  IR_ERR_AMBIGUOUS, /* mais de um codigo valido possivel */
};

struct ir_code {
  enum ir_protocol protocol;
  /* Bits na ordem em que chegam, o primeiro no bit 31 (MSB primeiro).
   * Ex.: liga/desliga Samsung = 0xE0E040BF. */
  uint32_t code;
};

/* Informacoes extras de uma decodificacao bem sucedida. */
struct ir_decode_info {
  int32_t stretch_avg_us; /* esticamento medio estimado no quadro */
  int32_t stretch_min_us;
  int32_t stretch_max_us;
  int lost_pulses;  /* pulsos que a leitura escolhida supoe perdidos */
  int candidates;   /* leituras com 32 bits examinadas */
  int search_steps; /* passos da busca (mede o custo da decodificacao) */
};

/* Trecho [start, start + len) de um vetor de duracoes. */
struct ir_span {
  int start;
  int len;
};

/* Separa uma captura em quadros nos espacos maiores que IR_FRAME_GAP_US.
 * Devolve quantos quadros foram escritos em spans (no maximo max). */
int ir_split_frames(const uint32_t *dur, int n, struct ir_span *spans,
                    int max);

/* Decodifica um unico quadro. info pode ser NULL. */
enum ir_status ir_decode_frame(const uint32_t *dur, int n,
                               struct ir_code *out,
                               struct ir_decode_info *info);

/* Separa a captura em quadros e devolve o primeiro que decodificar.
 * Se nenhum decodificar, devolve o erro do ultimo quadro. */
enum ir_status ir_decode(const uint32_t *dur, int n, struct ir_code *out,
                         struct ir_decode_info *info);

/* Monta a transmissao com os tempos ideais, como o controle original
 * faria num toque: Samsung32 envia o quadro duas vezes; NEC envia o
 * quadro e um codigo de repeticao. Ambos a cada 108 ms.
 * Devolve o numero de duracoes escritas, ou 0 se nao couber. */
int ir_encode(const struct ir_code *code, uint32_t *dur, int max);

/* Valida o codigo de 32 bits segundo o protocolo. */
int ir_code_is_valid(enum ir_protocol protocol, uint32_t code);

const char *ir_protocol_name(enum ir_protocol protocol);
const char *ir_status_name(enum ir_status status);

#endif /* IR_DECODE_H */
