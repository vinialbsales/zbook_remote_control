/* Decodificador de IR hibrido para NEC, Samsung32 e Coolix.
 *
 * C puro, sem dependencias do Zephyr: compila no PC (testes) e na placa.
 *
 * Entrada: duracoes em microssegundos, alternando pulso (portadora
 * ligada), espaco, pulso... sempre comecando por um pulso.
 *
 * Primeiro tenta o caminho convencional (cada tempo conferido contra o
 * nominal), que le o sinal limpo do receptor. Se ele nao ler o quadro, o
 * caminho robusto (reserva, CONFIG_APP_IR_ROBUST_FALLBACK) le NEC e
 * Samsung32 mesmo com os pulsos esticados e os espacos encurtados por uma
 * quantidade variavel (o "esticamento"), usando o periodo entre o inicio
 * de um pulso e o inicio do seguinte, que se preserva. O esticamento era
 * causado por um capacitor de 100 nF na linha do receptor, ja removido;
 * sem ele, fica entre 0 e 20 us. Detalhes em ir_decode.c.
 */
#ifndef IR_DECODE_H
#define IR_DECODE_H

#include <stdint.h>

/* Espaco acima disso separa dois quadros. */
#define IR_FRAME_GAP_US 10000

/* Quadro ideal: cabecalho (2) + 32 bits (64) + pulso final (1). */
#define IR_FRAME_LEN 67

/* Quadro do Coolix: cabecalho (2) + 48 bits (96) + pulso final (1). */
#define IR_COOLIX_FRAME_LEN 99

/* Maior sequencia que ir_encode gera (dois quadros e o intervalo). */
#define IR_ENCODE_MAX (2 * IR_COOLIX_FRAME_LEN + 1)

/* Maior quadro que ir_encode_frame gera. */
#define IR_ENCODE_FRAME_MAX IR_COOLIX_FRAME_LEN

enum ir_protocol {
  IR_PROTO_NONE = 0,
  IR_PROTO_NEC,
  IR_PROTO_SAMSUNG32,
  /* Ar-condicionado da familia Midea (Springer Midea, Carrier, Comfee,
   * Beko, Toshiba...): 24 bits de estado, cada byte seguido do seu
   * inverso. Envio e gravacao: so o caminho convencional le este
   * protocolo (o cabecalho cai na tolerancia do Samsung32; quem separa e
   * o numero de intervalos, 99 ou 199 contra 67). Gravacoes viram uma
   * sequencia normal de um codigo, igual a da biblioteca. Com o sinal
   * distorcido (receptor com o capacitor), o convencional nao le e a
   * gravacao continua crua. */
  IR_PROTO_COOLIX,
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

/* Caminho do decodificador que leu o quadro (ver ir_decode.c). */
enum ir_decode_path {
  IR_PATH_NONE = 0,
  IR_PATH_CONVENTIONAL, /* tempos conferidos um a um, sinal limpo */
  IR_PATH_ROBUST,       /* blocos e busca, sinal distorcido */
};

/* Informacoes extras de uma decodificacao bem sucedida. No caminho
 * convencional, lost_pulses = 0, candidates = 1 e search_steps = 0. */
struct ir_decode_info {
  int32_t stretch_avg_us; /* esticamento medio estimado no quadro */
  int32_t stretch_min_us;
  int32_t stretch_max_us;
  int lost_pulses;  /* pulsos que a leitura escolhida supoe perdidos */
  int candidates;   /* leituras com 32 bits examinadas */
  int search_steps; /* passos da busca (mede o custo da decodificacao) */
  enum ir_decode_path path;
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

/* Decodifica um unico quadro: caminho convencional e, se ele nao ler,
 * o robusto (se ligado). info pode ser NULL; info->path diz qual leu. */
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

/* Monta um unico quadro ideal, sem repeticao. Usado para enviar
 * sequencias de codigos diferentes (ex.: projetor Epson).
 * Devolve o tamanho do quadro (IR_FRAME_LEN, ou IR_COOLIX_FRAME_LEN), ou
 * 0 se nao couber ou o protocolo e desconhecido. */
int ir_encode_frame(const struct ir_code *code, uint32_t *dur, int max);

/* Duracao do quadro ideal, do inicio do cabecalho ao fim do pulso
 * final, em us. 0 se o protocolo e desconhecido. */
uint32_t ir_frame_duration_us(const struct ir_code *code);

/* Valida o codigo de 32 bits segundo o protocolo. */
int ir_code_is_valid(enum ir_protocol protocol, uint32_t code);

const char *ir_protocol_name(enum ir_protocol protocol);
const char *ir_status_name(enum ir_status status);

#endif /* IR_DECODE_H */
