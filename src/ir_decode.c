/* Decodificador de IR hibrido para NEC, Samsung32 e Coolix. Ver
 * ir_decode.h.
 *
 * Dois caminhos, nesta ordem (ir_decode_frame):
 *
 *   1. Convencional: confere cabecalho, cada pulso e cada espaco contra o
 *      nominal (+-CONV_TOL_PCT %), com a estrutura exata do protocolo.
 *      Rapido, iterativo e com pouca pilha. Le NEC, Samsung32, a
 *      repeticao NEC e o Coolix (so este caminho le Coolix).
 *   2. Robusto (reserva, CONFIG_APP_IR_ROBUST_FALLBACK): o modelo de
 *      blocos descrito abaixo, so para NEC e Samsung32. Roda quando o
 *      convencional nao le o quadro, com o mesmo comportamento de antes.
 *
 * Historico: o esticamento descrito abaixo NAO e uma caracteristica do
 * receptor. A causa era um capacitor de 100 nF na linha do receptor, que
 * nao devia estar ali. Sem ele (descoberto no laboratorio), o esticamento caiu
 * para 0 a 20 us e os quadros saem pelo caminho convencional. O caminho
 * robusto ficou como reserva para placas que ainda tenham o capacitor ou
 * sinais muito fracos.
 *
 * Como o sinal e codificado (NEC e Samsung32 usam o mesmo esquema):
 *
 *   cabecalho: pulso longo + espaco longo
 *   bit 0:     pulso 560 + espaco  560  (periodo 1120)
 *   bit 1:     pulso 560 + espaco 1690  (periodo 2250)
 *   fim:       pulso 560
 *
 * O que o receptor com o capacitor fazia com isso (e o que o caminho
 * robusto supoe): cada pulso chega esticado por
 * uma quantidade "e" (o esticamento) e o espaco seguinte encolhe na mesma
 * quantidade, de modo que pulso + espaco (o periodo) continua certo.
 * Quando e > 560, o espaco de um bit 0 deixa de existir e o pulso desse
 * bit se funde com o pulso seguinte. Com sinal fraco acontece o contrario
 * (e negativo) e as vezes um pulso inteiro some.
 *
 * Nos dois casos, quem desaparece e um pulso que vem logo apos um espaco
 * curto (bit 0); o pulso que vem depois de um espaco longo (bit 1)
 * sempre aparece. Entao cada "bloco" visivel (um pulso e o espaco que o
 * segue) representa:
 *
 *   N bits 0 seguidos de um bit final, que pode ser 0 ou 1.
 *
 * O periodo do bloco vale N * 1120 + (1120 ou 2250). Como 2250 e quase
 * 2 * 1120, o periodo sozinho nao diz se o bloco termina em 1 ou em dois
 * zeros. Quem decide e o espaco:
 *
 *   - Pulsos fundidos: se o bloco termina em 1, o espaco vale 1690 - e;
 *     se termina em 0, 560 - e. Cada hipotese implica um esticamento
 *     diferente (separados por ~1130 us) e o esticamento varia devagar ao
 *     longo do quadro, entao so uma hipotese fica coerente com os
 *     vizinhos.
 *   - Pulso perdido (sinal fraco): o espaco engole o pulso que sumiu e
 *     deixa de dizer qual e o bit final. As duas hipoteses ficam abertas.
 *
 * Testamos as hipoteses abertas com uma busca em profundidade e ficamos
 * com a que forma exatamente 32 bits e passa na validacao do protocolo.
 * Se duas leituras validas derem codigos diferentes, o quadro e rejeitado.
 */
#include "ir_decode.h"

#include <stddef.h>

/* Caminho robusto como reserva do convencional. Na placa vem do Kconfig
 * (CONFIG_APP_IR_ROBUST_FALLBACK, que o Zephyr define em todo arquivo);
 * no PC fica ligado, salvo -DIR_ROBUST_FALLBACK=0. */
#ifndef IR_ROBUST_FALLBACK
#if defined(__ZEPHYR__) && !defined(CONFIG_APP_IR_ROBUST_FALLBACK)
#define IR_ROBUST_FALLBACK 0
#else
#define IR_ROBUST_FALLBACK 1
#endif
#endif

/* Tempos nominais dos bits, em us. */
#define BIT_PULSE 560
#define BIT_SPACE_0 560
#define BIT_SPACE_1 1690
#define PERIOD_0 (BIT_PULSE + BIT_SPACE_0)
#define PERIOD_1 (BIT_PULSE + BIT_SPACE_1)
#define FRAME_BITS 32

/* Intervalo entre inicios de quadro dos controles NEC e Samsung. */
#define REPEAT_PERIOD_US 108000

/* Faixa de esticamento aceita nos bits. Medido com o capacitor: de -400
 * (sinal fraco) ate ~1600 (pino com pull-up). O maximo fisico e um pouco
 * menos que 1690: acima disso ate o espaco do bit 1 some. Fora da faixa,
 * a hipotese e descartada. */
#define STRETCH_MIN (-550)
#define STRETCH_MAX 1650

/* Variacao maxima de esticamento entre blocos vizinhos. Medido: < 200. */
#define STRETCH_JUMP 400

/* So se considera pulso perdido quando o pulso visivel e curto
 * (esticamento abaixo disso), que e quando o sinal esta fraco. Medido:
 * nos quadros com pulso perdido, o esticamento fica abaixo de 200; nos
 * quadros fortes, acima de 380. */
#define WEAK_STRETCH_MAX 300

/* Maximo de pulsos perdidos aceitos num quadro. */
#define MAX_LOST_PULSES 6

/* Erro aceito no periodo de um bloco. Medido: < 200. Precisa ficar bem
 * abaixo de PERIOD_0 / 2 para nao confundir N com N + 1 zeros. */
#define PERIOD_TOL 350

/* Cabecalho: tolerancia de 12% no periodo e faixa de esticamento maior,
 * porque o receptor demora mais para reagir ao primeiro pulso. */
#define HEADER_TOL_PCT 12
#define HEADER_STRETCH_MIN (-1000)
#define HEADER_STRETCH_MAX 2000

/* Limite de passos da busca, para o tempo de decodificacao ser limitado
 * mesmo com um sinal muito ruim (cada passo custa poucos us na placa). */
#define MAX_NODES 50000

/* Um quadro tem no maximo 32 blocos de dados (um por bit). */
#define MAX_BLOCKS FRAME_BITS

/* Ate 4 hipoteses por bloco: bit final 0 ou 1, com ou sem pulso perdido. */
#define MAX_HYP 4

#define NO_STRETCH INT32_MIN

struct protocol_timing {
  enum ir_protocol protocol;
  uint32_t header_pulse;
  uint32_t header_space;
};

static const struct protocol_timing timings[] = {
    {IR_PROTO_SAMSUNG32, 4500, 4500},
    {IR_PROTO_NEC, 9000, 4500},
};

/* Coolix (IRremoteESP8266, ir_Coolix.cpp): tick de 276 us. */
#define COOLIX_HDR_PULSE 4692
#define COOLIX_HDR_SPACE 4416
#define COOLIX_BIT_PULSE 552
#define COOLIX_SPACE_1 1656
#define COOLIX_SPACE_0 552
#define COOLIX_GAP 5244 /* entre os dois quadros */
#define COOLIX_BITS 24

/* Codigo de repeticao do NEC: 9000 + 2250 + 560. */
#define NEC_REPEAT_PULSE 9000
#define NEC_REPEAT_SPACE 2250

/* Uma leitura possivel de um bloco: "zeros" bits 0 e depois last_bit. */
struct hypothesis {
  int8_t zeros;
  uint8_t last_bit;
  uint8_t lost; /* 1 se a leitura supoe um pulso perdido */
  int32_t stretch;
};

struct block {
  struct hypothesis hyp[MAX_HYP];
  int n_hyp;
};

/* Estado da busca em profundidade. */
struct search {
  const struct block *blocks;
  int n_blocks;
  uint32_t final_pulse;
  enum ir_protocol protocol;
  int max_lost;

  uint8_t choice[MAX_BLOCKS]; /* hipotese escolhida em cada bloco */
  int nodes;
  int candidates;

  int found;
  int ambiguous;
  int weak_check; /* houve leitura que so passou numa validacao fraca */
  uint32_t best_code;
  int32_t best_cost;
  struct ir_decode_info best_info;
};

static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

/* Divisao arredondada para o inteiro mais proximo (num pode ser < 0). */
static int32_t div_round(int32_t num, int32_t den) {
  return num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
}

/* Quantos zeros cabem num periodo, dado o periodo do bit final.
 * Devolve -1 se o periodo nao fecha com nenhuma quantidade inteira. */
static int zeros_in_period(int32_t period, int32_t last_bit_period) {
  int32_t rest = period - last_bit_period;
  int32_t zeros = div_round(rest, PERIOD_0);

  if (zeros < 0 || zeros >= FRAME_BITS) {
    return -1;
  }
  if (abs32(rest - zeros * PERIOD_0) > PERIOD_TOL) {
    return -1;
  }
  return (int)zeros;
}

static int stretch_ok(int32_t s) {
  return s >= STRETCH_MIN && s <= STRETCH_MAX;
}

static void add_hypothesis(struct block *b, int zeros, int last_bit, int lost,
                           int32_t stretch) {
  struct hypothesis *h = &b->hyp[b->n_hyp++];

  h->zeros = (int8_t)zeros;
  h->last_bit = (uint8_t)last_bit;
  h->lost = (uint8_t)lost;
  h->stretch = stretch;
}

/* Lista as leituras possiveis de um bloco (pulso + espaco). */
static void analyze_block(uint32_t pulse, uint32_t space, struct block *b) {
  static const int32_t last_period[2] = {PERIOD_0, PERIOD_1};
  static const int32_t last_space[2] = {BIT_SPACE_0, BIT_SPACE_1};
  int32_t period = (int32_t)(pulse + space);

  b->n_hyp = 0;
  for (int x = 0; x < 2; x++) {
    int zeros = zeros_in_period(period, last_period[x]);
    int32_t s;

    if (zeros < 0) {
      continue;
    }

    /* Sem pulso perdido: o espaco que sobrou e o do bit final,
     * encurtado pelo esticamento. */
    s = last_space[x] - (int32_t)space;
    if (stretch_ok(s)) {
      add_hypothesis(b, zeros, x, 0, s);
    }

    /* Com pulso perdido: o pulso visivel e um pulso isolado (do primeiro
     * zero) e o esticamento vem dele; o espaco nao diz nada sobre o bit
     * final. Precisa de ao menos um zero (o dono do pulso visivel) e de
     * sinal fraco. */
    s = (int32_t)pulse - BIT_PULSE;
    if (zeros >= 1 && stretch_ok(s) && s <= WEAK_STRETCH_MAX) {
      add_hypothesis(b, zeros, x, 1, s);
    }
  }
}

static int header_matches(uint32_t pulse, uint32_t space, uint32_t nom_pulse,
                          uint32_t nom_space) {
  uint32_t nominal = nom_pulse + nom_space;
  uint32_t tol = nominal * HEADER_TOL_PCT / 100;
  int32_t s = (int32_t)pulse - (int32_t)nom_pulse;

  return pulse + space >= nominal - tol && pulse + space <= nominal + tol &&
         s >= HEADER_STRETCH_MIN && s <= HEADER_STRETCH_MAX;
}

int ir_code_is_valid(enum ir_protocol protocol, uint32_t code) {
  uint8_t b1 = (uint8_t)(code >> 24);
  uint8_t b2 = (uint8_t)(code >> 16);
  uint8_t b3 = (uint8_t)(code >> 8);
  uint8_t b4 = (uint8_t)code;

  switch (protocol) {
  case IR_PROTO_COOLIX:
    /* 24 bits; o nibble de cima e sempre 0xB. */
    return (code >> 24) == 0 && (code >> 20) == 0xB;
  case IR_PROTO_SAMSUNG32:
    /* Endereco repetido nos bytes 1 e 2; comando e seu inverso. */
    return b1 == b2 && b4 == (uint8_t)~b3;
  case IR_PROTO_NEC:
    /* Endereco normal (b2 = ~b1) ou estendido (16 bits livres); o
     * comando sempre vem seguido do seu inverso. */
    return b4 == (uint8_t)~b3;
  default:
    return 0;
  }
}

/* Quantos bits do codigo a validacao confere. Uma leitura errada passa
 * por acaso com chance de 1 em 2^bits. */
static int check_bits(enum ir_protocol protocol, uint32_t code) {
  uint8_t b1 = (uint8_t)(code >> 24);
  uint8_t b2 = (uint8_t)(code >> 16);

  if (protocol == IR_PROTO_NEC && b2 != (uint8_t)~b1) {
    return 8; /* NEC estendido: so o comando e conferido */
  }
  return 16;
}

/* ------------------------------------------------------------------ */
/* Caminho convencional                                                */
/* ------------------------------------------------------------------ */

/* Tolerancia de cada intervalo, em % do nominal: pulso e espaco sao
 * conferidos um a um, sem nenhuma suposicao de pulso perdido.
 *
 * Por que 30%:
 *   - Receptores de 38 kHz alargam ou encurtam o pulso em alguns ciclos
 *     da portadora (26 us cada); 30% de 560 = 168 us cobre ~6 ciclos, e
 *     a placa sem o capacitor mede 0 a 20 us.
 *   - Os espacos do bit 0 e do bit 1 continuam bem separados: 0 vai de
 *     392 a 728 us e 1 de 1183 a 2197 us. Um espaco no meio (ex.: 1000)
 *     nao e nem um nem outro e o quadro vai para o caminho robusto.
 *   - Os pulsos esticados do receptor antigo (~850 us ou mais) ficam fora
 *     de 392..728, entao esses quadros tambem vao para o robusto.
 *   - Os cabecalhos continuam separados: 4500 (3150..5850) contra 9000
 *     (6300..11700) no pulso; 4500 (3150..5850) contra 2250 da repeticao
 *     (1575..2925) no espaco. */
#define CONV_TOL_PCT 30

/* Tempos de um protocolo no caminho convencional. */
struct conv_timing {
  enum ir_protocol protocol;
  uint32_t header_pulse;
  uint32_t header_space;
  uint32_t bit_pulse;
  uint32_t space_0;
  uint32_t space_1;
  int bits; /* bits de dados por quadro */
};

/* NEC e Samsung32 com os mesmos cabecalhos do timings[] do caminho
 * robusto. Coolix com os tempos do put_coolix: 48 bits por quadro (cada
 * byte do estado seguido do inverso). O cabecalho do Coolix (4692/4416)
 * cai na tolerancia do Samsung32 (4500/4500); quem separa os dois e o
 * numero de intervalos: 99 (ou 199, duas copias) contra 67. */
static const struct conv_timing conv_timings[] = {
    {IR_PROTO_SAMSUNG32, 4500, 4500, BIT_PULSE, BIT_SPACE_0, BIT_SPACE_1,
     FRAME_BITS},
    {IR_PROTO_NEC, 9000, 4500, BIT_PULSE, BIT_SPACE_0, BIT_SPACE_1,
     FRAME_BITS},
    {IR_PROTO_COOLIX, COOLIX_HDR_PULSE, COOLIX_HDR_SPACE, COOLIX_BIT_PULSE,
     COOLIX_SPACE_0, COOLIX_SPACE_1, 2 * COOLIX_BITS},
};

/* Intervalo entre as duas copias do Coolix. Sai do ir_encode com
 * COOLIX_GAP (5244 us); acima de IR_FRAME_GAP_US, as copias ja chegam
 * como quadros separados e cada uma e lida sozinha. */
#define COOLIX_GAP_MIN (COOLIX_GAP * (100 - CONV_TOL_PCT) / 100)

/* Estado Coolix de 48 bits lidos: cada byte tem que vir seguido do seu
 * inverso. Devolve 0 se algum inverso nao bate. */
static int coolix_state(uint64_t bits, uint32_t *state) {
  uint32_t v = 0;

  for (int byte = 2; byte >= 0; byte--) {
    uint8_t b = (uint8_t)(bits >> (16 * byte + 8));
    uint8_t inv = (uint8_t)(bits >> (16 * byte));

    if (inv != (uint8_t)~b) {
      return 0;
    }
    v = v << 8 | b;
  }
  *state = v;
  return 1;
}

/* Esticamento de cada bit, calculado como no caminho robusto (espaco
 * nominal - espaco medido; no pulso final, pulso medido - nominal), para
 * o log mostrar a mesma coisa nos dois caminhos. */
struct conv_stats {
  int32_t sum;
  int32_t min;
  int32_t max;
  int count;
};

static void conv_stats_add(struct conv_stats *st, int32_t s) {
  if (st->count == 0 || s < st->min) {
    st->min = s;
  }
  if (st->count == 0 || s > st->max) {
    st->max = s;
  }
  st->sum += s;
  st->count++;
}

/* v dentro de nominal +- CONV_TOL_PCT %. Em 64 bits: um pulso preso pode
 * durar segundos. */
static int conv_near(uint32_t v, uint32_t nominal) {
  uint64_t v100 = (uint64_t)v * 100;

  return v100 >= (uint64_t)nominal * (100 - CONV_TOL_PCT) &&
         v100 <= (uint64_t)nominal * (100 + CONV_TOL_PCT);
}

/* Le um quadro de t->bits bits que comeca em d[0] (cabecalho) e tem
 * exatamente 2 + 2 * bits + 1 intervalos. Os bits saem em *bits, o
 * primeiro que chegou no mais significativo. */
static enum ir_status conv_read_frame(const struct conv_timing *t,
                                      const uint32_t *d, uint64_t *bits,
                                      struct conv_stats *st) {
  uint64_t v = 0;

  if (!conv_near(d[0], t->header_pulse) || !conv_near(d[1], t->header_space)) {
    return IR_ERR_HEADER;
  }
  for (int i = 0; i < t->bits; i++) {
    uint32_t pulse = d[2 + 2 * i];
    uint32_t space = d[3 + 2 * i];
    int bit;

    if (!conv_near(pulse, t->bit_pulse)) {
      return IR_ERR_DECODE;
    }
    if (conv_near(space, t->space_1)) {
      bit = 1;
    } else if (conv_near(space, t->space_0)) {
      bit = 0;
    } else {
      return IR_ERR_DECODE;
    }
    conv_stats_add(st, (int32_t)(bit ? t->space_1 : t->space_0) -
                           (int32_t)space);
    v = v << 1 | (uint64_t)bit;
  }
  uint32_t final_pulse = d[2 + 2 * t->bits];

  if (!conv_near(final_pulse, t->bit_pulse)) {
    return IR_ERR_DECODE;
  }
  conv_stats_add(st, (int32_t)final_pulse - (int32_t)t->bit_pulse);
  *bits = v;
  return IR_OK;
}

/* Le os bits de um quadro e valida: NEC e Samsung32 pelo
 * ir_code_is_valid; Coolix pelos bytes inversos e pelo ir_code_is_valid
 * (nibble 0xB). */
static enum ir_status conv_frame_code(const struct conv_timing *t,
                                      const uint32_t *d, uint32_t *code,
                                      struct conv_stats *st) {
  uint64_t bits;
  enum ir_status res = conv_read_frame(t, d, &bits, st);

  if (res != IR_OK) {
    return res;
  }
  if (t->protocol == IR_PROTO_COOLIX) {
    if (!coolix_state(bits, code)) {
      return IR_ERR_CHECK;
    }
  } else {
    *code = (uint32_t)bits;
  }
  return ir_code_is_valid(t->protocol, *code) ? IR_OK : IR_ERR_CHECK;
}

/* Decodificador convencional: cabecalho e bits conferidos intervalo a
 * intervalo, com a estrutura exata do protocolo (cabecalho + bits +
 * pulso final). Sem busca e sem recursao. Para NEC e Samsung32, aplica a
 * mesma validacao do caminho robusto (ir_code_is_valid). O check_bits do
 * robusto so pesa em leituras com pulso perdido, que este caminho nunca
 * supoe.
 *
 * Coolix: o controle manda o quadro duas vezes com ~5,2 ms entre elas, o
 * que fica abaixo de IR_FRAME_GAP_US, entao as duas copias chegam num
 * quadro so de 199 intervalos. As duas tem que dar o mesmo estado. Um
 * quadro de 99 (uma copia) tambem vale. */
static enum ir_status decode_conventional(const uint32_t *dur, int n,
                                          struct ir_code *out,
                                          struct ir_decode_info *info) {
  const struct conv_timing *t = NULL;
  int header_seen = 0;

  if (n < 2) {
    return IR_ERR_HEADER;
  }

  /* NEC repetido: so cabecalho curto e o pulso final. */
  if (n == 3 && conv_near(dur[0], NEC_REPEAT_PULSE) &&
      conv_near(dur[1], NEC_REPEAT_SPACE) && conv_near(dur[2], BIT_PULSE)) {
    return IR_REPEAT;
  }

  /* O protocolo e escolhido pelo cabecalho e pelo numero de intervalos. */
  for (size_t p = 0; p < sizeof(conv_timings) / sizeof(conv_timings[0]);
       p++) {
    const struct conv_timing *c = &conv_timings[p];

    if (!conv_near(dur[0], c->header_pulse) ||
        !conv_near(dur[1], c->header_space)) {
      continue;
    }
    header_seen = 1;
    int len = 2 + 2 * c->bits + 1;

    if (n == len ||
        (c->protocol == IR_PROTO_COOLIX && n == 2 * len + 1)) {
      t = c;
      break;
    }
  }
  if (!header_seen) {
    return IR_ERR_HEADER;
  }
  if (n % 2 == 0) {
    return IR_ERR_TRUNCATED;
  }
  if (t == NULL) {
    return IR_ERR_DECODE; /* cabecalho conhecido, tamanho errado */
  }

  struct conv_stats st = {0};
  uint32_t code;
  int len = 2 + 2 * t->bits + 1;
  enum ir_status res = conv_frame_code(t, dur, &code, &st);

  if (res != IR_OK) {
    return res;
  }
  if (n > len) {
    /* Segunda copia do Coolix. */
    uint32_t again;

    if (dur[len] < COOLIX_GAP_MIN || dur[len] > IR_FRAME_GAP_US) {
      return IR_ERR_DECODE;
    }
    res = conv_frame_code(t, dur + len + 1, &again, &st);
    if (res != IR_OK) {
      return res;
    }
    if (again != code) {
      return IR_ERR_AMBIGUOUS;
    }
  }

  out->protocol = t->protocol;
  out->code = code;
  if (info != NULL) {
    info->stretch_avg_us = st.sum / st.count;
    info->stretch_min_us = st.min;
    info->stretch_max_us = st.max;
    info->lost_pulses = 0;
    info->candidates = 1;
    info->search_steps = 0;
    info->path = IR_PATH_CONVENTIONAL;
  }
  return IR_OK;
}

/* ------------------------------------------------------------------ */
/* Caminho robusto                                                     */
/* ------------------------------------------------------------------ */

/* Chamado quando todos os blocos receberam uma leitura: monta os bits,
 * confere o pulso final e a validacao, e compara com o melhor ate agora. */
static void evaluate(struct search *sr, int n_bits, int32_t last_s,
                     int lost) {
  int final_zeros = FRAME_BITS - n_bits;

  /* O pulso final pode ter engolido os ultimos zeros do codigo:
   * pulso = zeros * 1120 + 560 + esticamento. */
  int32_t final_s =
      (int32_t)sr->final_pulse - BIT_PULSE - final_zeros * PERIOD_0;
  if (!stretch_ok(final_s) ||
      (last_s != NO_STRETCH && abs32(final_s - last_s) > STRETCH_JUMP)) {
    return;
  }

  /* Monta o codigo e as estatisticas do esticamento. O custo soma as
   * variacoes de esticamento entre blocos: quanto menor, mais coerente. */
  uint64_t code = 0; /* 64 bits: deslocar um uint32_t por 32 e indefinido */
  int32_t prev = NO_STRETCH, cost = 0, sum = 0, min = 0, max = 0;
  int count = 0;

  for (int i = 0; i <= sr->n_blocks; i++) {
    int32_t s;

    if (i < sr->n_blocks) {
      const struct hypothesis *h = &sr->blocks[i].hyp[sr->choice[i]];

      code = (code << (h->zeros + 1)) | h->last_bit;
      s = h->stretch;
    } else {
      code <<= final_zeros;
      s = final_s;
    }
    if (prev != NO_STRETCH) {
      cost += abs32(s - prev);
    }
    if (count == 0 || s < min) {
      min = s;
    }
    if (count == 0 || s > max) {
      max = s;
    }
    sum += s;
    count++;
    prev = s;
  }

  sr->candidates++;
  if (!ir_code_is_valid(sr->protocol, (uint32_t)code)) {
    return;
  }

  /* Com pulsos perdidos ha muitas leituras possiveis; uma validacao de
   * 8 bits deixaria passar uma errada com facilidade. */
  if (lost > 0 && check_bits(sr->protocol, (uint32_t)code) < 16) {
    sr->weak_check = 1;
    return;
  }

  if (sr->found && (uint32_t)code != sr->best_code) {
    sr->ambiguous = 1;
  }
  if (!sr->found || cost < sr->best_cost) {
    sr->found = 1;
    sr->best_code = (uint32_t)code;
    sr->best_cost = cost;
    sr->best_info.stretch_avg_us = sum / count;
    sr->best_info.stretch_min_us = min;
    sr->best_info.stretch_max_us = max;
    sr->best_info.lost_pulses = lost;
  }
}

/* Busca em profundidade: escolhe a leitura do bloco i e segue para o
 * proximo. Os cortes (mais de 32 bits, salto de esticamento, pulsos
 * perdidos demais) eliminam quase todos os ramos. */
static void explore(struct search *sr, int i, int n_bits, int32_t last_s,
                    int lost) {
  if (++sr->nodes > MAX_NODES) {
    return;
  }
  if (i == sr->n_blocks) {
    evaluate(sr, n_bits, last_s, lost);
    return;
  }

  const struct block *b = &sr->blocks[i];

  for (int k = 0; k < b->n_hyp; k++) {
    const struct hypothesis *h = &b->hyp[k];

    if (n_bits + h->zeros + 1 > FRAME_BITS || lost + h->lost > sr->max_lost) {
      continue;
    }
    if (last_s != NO_STRETCH && abs32(h->stretch - last_s) > STRETCH_JUMP) {
      continue;
    }
    sr->choice[i] = (uint8_t)k;
    explore(sr, i + 1, n_bits + h->zeros + 1, h->stretch, lost + h->lost);
  }
}

/* Decodificador robusto (blocos, esticamento e busca). Comportamento
 * igual ao de antes do caminho convencional. */
static enum ir_status decode_robust(const uint32_t *dur, int n,
                                    struct ir_code *out,
                                    struct ir_decode_info *info) {
  struct search sr = {0};
  struct block blocks[MAX_BLOCKS];

  if (n < 2) {
    return IR_ERR_HEADER;
  }

  /* NEC repetido: so cabecalho curto e o pulso final. */
  if (n == 3 &&
      header_matches(dur[0], dur[1], NEC_REPEAT_PULSE, NEC_REPEAT_SPACE)) {
    return IR_REPEAT;
  }

  for (size_t p = 0; p < sizeof(timings) / sizeof(timings[0]); p++) {
    if (header_matches(dur[0], dur[1], timings[p].header_pulse,
                       timings[p].header_space)) {
      sr.protocol = timings[p].protocol;
      break;
    }
  }
  if (sr.protocol == IR_PROTO_NONE) {
    return IR_ERR_HEADER;
  }

  /* Um quadro completo termina com o pulso final, entao tem tamanho
   * impar. Se termina num espaco, a captura foi cortada no meio. */
  if (n % 2 == 0) {
    return IR_ERR_TRUNCATED;
  }

  /* Depois do cabecalho: pares (pulso, espaco) e o pulso final. */
  sr.n_blocks = (n - 3) / 2;
  if (sr.n_blocks > MAX_BLOCKS) {
    return IR_ERR_DECODE;
  }
  for (int i = 0; i < sr.n_blocks; i++) {
    analyze_block(dur[2 + 2 * i], dur[3 + 2 * i], &blocks[i]);
  }
  sr.blocks = blocks;
  sr.final_pulse = dur[n - 1];

  /* Prefere a explicacao mais simples: primeiro tenta sem nenhum pulso
   * perdido, depois com 1, com 2... e para no primeiro nivel em que
   * alguma leitura passa na validacao. */
  for (sr.max_lost = 0; sr.max_lost <= MAX_LOST_PULSES; sr.max_lost++) {
    explore(&sr, 0, 0, NO_STRETCH, 0);
    if (sr.found || sr.weak_check || sr.nodes > MAX_NODES) {
      break;
    }
  }

  if (sr.nodes > MAX_NODES || sr.ambiguous || sr.weak_check) {
    return IR_ERR_AMBIGUOUS;
  }
  if (sr.candidates == 0) {
    return IR_ERR_DECODE;
  }
  if (!sr.found) {
    return IR_ERR_CHECK;
  }

  out->protocol = sr.protocol;
  out->code = sr.best_code;
  if (info != NULL) {
    *info = sr.best_info;
    info->candidates = sr.candidates;
    info->search_steps = sr.nodes;
    info->path = IR_PATH_ROBUST;
  }
  return IR_OK;
}

/* Primeiro o caminho convencional; se ele nao ler o quadro, o robusto
 * (se ligado), sem nenhuma mudanca no comportamento dele. */
enum ir_status ir_decode_frame(const uint32_t *dur, int n,
                               struct ir_code *out,
                               struct ir_decode_info *info) {
  enum ir_status st = decode_conventional(dur, n, out, info);

  if (st == IR_OK || st == IR_REPEAT || !IR_ROBUST_FALLBACK) {
    return st;
  }
  return decode_robust(dur, n, out, info);
}

int ir_split_frames(const uint32_t *dur, int n, struct ir_span *spans,
                    int max) {
  int count = 0;
  int start = 0;

  for (int i = 0; i <= n && count < max; i++) {
    /* Indices impares sao espacos (a captura comeca num pulso). */
    int gap = i < n && (i % 2) == 1 && dur[i] > IR_FRAME_GAP_US;

    if (i == n || gap) {
      if (i > start) {
        spans[count].start = start;
        spans[count].len = i - start;
        count++;
      }
      start = i + 1;
    }
  }
  return count;
}

enum ir_status ir_decode(const uint32_t *dur, int n, struct ir_code *out,
                         struct ir_decode_info *info) {
  struct ir_span spans[8];
  int frames = ir_split_frames(dur, n, spans, 8);
  enum ir_status result = IR_ERR_HEADER;
  int only_repeats = frames > 0;

  for (int f = 0; f < frames; f++) {
    enum ir_status st =
        ir_decode_frame(dur + spans[f].start, spans[f].len, out, info);

    if (st == IR_OK) {
      return IR_OK;
    }
    if (st != IR_REPEAT) {
      only_repeats = 0;
      result = st;
    }
  }
  return only_repeats ? IR_REPEAT : result;
}

/* Escreve um quadro ideal. Devolve a nova posicao, ou -1 se nao couber. */
static int put_frame(const struct protocol_timing *t, uint32_t code,
                     uint32_t *dur, int pos, int max, uint32_t *elapsed) {
  if (pos + IR_FRAME_LEN > max) {
    return -1;
  }
  dur[pos++] = t->header_pulse;
  dur[pos++] = t->header_space;
  for (int i = FRAME_BITS - 1; i >= 0; i--) {
    dur[pos++] = BIT_PULSE;
    dur[pos++] = (code >> i) & 1 ? BIT_SPACE_1 : BIT_SPACE_0;
  }
  dur[pos++] = BIT_PULSE;

  for (int i = pos - IR_FRAME_LEN; i < pos; i++) {
    *elapsed += dur[i];
  }
  return pos;
}

/* Tempos do protocolo, ou NULL se ele nao e conhecido. */
static const struct protocol_timing *find_timing(enum ir_protocol protocol) {
  for (size_t p = 0; p < sizeof(timings) / sizeof(timings[0]); p++) {
    if (timings[p].protocol == protocol) {
      return &timings[p];
    }
  }
  return NULL;
}

/* Quadro Coolix: cada byte do estado (MSB primeiro) e depois o inverso. */
static int put_coolix(uint32_t state, uint32_t *dur, int max) {
  int pos = 0;

  if (max < IR_COOLIX_FRAME_LEN) {
    return 0;
  }
  dur[pos++] = COOLIX_HDR_PULSE;
  dur[pos++] = COOLIX_HDR_SPACE;
  for (int byte = 2; byte >= 0; byte--) {
    uint8_t v = (uint8_t)(state >> (8 * byte));

    for (int inv = 0; inv < 2; inv++) {
      uint8_t b = inv ? (uint8_t)~v : v;

      for (int i = 7; i >= 0; i--) {
        dur[pos++] = COOLIX_BIT_PULSE;
        dur[pos++] = (b >> i) & 1 ? COOLIX_SPACE_1 : COOLIX_SPACE_0;
      }
    }
  }
  dur[pos++] = COOLIX_BIT_PULSE;
  return pos;
}

int ir_encode_frame(const struct ir_code *code, uint32_t *dur, int max) {
  const struct protocol_timing *t = find_timing(code->protocol);
  uint32_t elapsed = 0;

  if (code->protocol == IR_PROTO_COOLIX) {
    return put_coolix(code->code, dur, max);
  }
  if (t == NULL) {
    return 0;
  }
  return put_frame(t, code->code, dur, 0, max, &elapsed) < 0 ? 0
                                                             : IR_FRAME_LEN;
}

uint32_t ir_frame_duration_us(const struct ir_code *code) {
  const struct protocol_timing *t = find_timing(code->protocol);
  uint32_t total;

  if (code->protocol == IR_PROTO_COOLIX) {
    uint32_t buf[IR_COOLIX_FRAME_LEN];
    int n = put_coolix(code->code, buf, IR_COOLIX_FRAME_LEN);

    total = 0;
    for (int i = 0; i < n; i++) {
      total += buf[i];
    }
    return total;
  }
  if (t == NULL) {
    return 0;
  }
  total = t->header_pulse + t->header_space + BIT_PULSE;
  for (int i = 0; i < FRAME_BITS; i++) {
    total += (code->code >> i) & 1 ? PERIOD_1 : PERIOD_0;
  }
  return total;
}

int ir_encode(const struct ir_code *code, uint32_t *dur, int max) {
  const struct protocol_timing *t = find_timing(code->protocol);
  uint32_t elapsed = 0;
  int pos;

  /* Coolix: o controle manda o quadro duas vezes, 5,2 ms de intervalo. */
  if (code->protocol == IR_PROTO_COOLIX) {
    int n = put_coolix(code->code, dur, max);

    if (n == 0 || n + 1 + IR_COOLIX_FRAME_LEN > max) {
      return 0;
    }
    dur[n] = COOLIX_GAP;
    return n + 1 + put_coolix(code->code, dur + n + 1, max - n - 1);
  }
  if (t == NULL) {
    return 0;
  }

  pos = put_frame(t, code->code, dur, 0, max, &elapsed);
  if (pos < 0 || pos + 1 > max) {
    return 0;
  }

  /* Silencio ate completar 108 ms desde o inicio do primeiro quadro. */
  dur[pos++] = REPEAT_PERIOD_US - elapsed;

  if (code->protocol == IR_PROTO_SAMSUNG32) {
    /* O controle Samsung repete o quadro inteiro. */
    pos = put_frame(t, code->code, dur, pos, max, &elapsed);
  } else if (pos + 3 <= max) {
    /* O NEC manda so o codigo de repeticao: reenviar o quadro inteiro
     * poderia ser lido como um segundo toque. */
    dur[pos++] = NEC_REPEAT_PULSE;
    dur[pos++] = NEC_REPEAT_SPACE;
    dur[pos++] = BIT_PULSE;
  } else {
    pos = -1;
  }
  return pos < 0 ? 0 : pos;
}

const char *ir_protocol_name(enum ir_protocol protocol) {
  switch (protocol) {
  case IR_PROTO_NEC:
    return "NEC";
  case IR_PROTO_SAMSUNG32:
    return "Samsung32";
  case IR_PROTO_COOLIX:
    return "Coolix";
  default:
    return "nenhum";
  }
}

const char *ir_status_name(enum ir_status status) {
  switch (status) {
  case IR_OK:
    return "ok";
  case IR_REPEAT:
    return "repeticao NEC";
  case IR_ERR_HEADER:
    return "sem cabecalho";
  case IR_ERR_TRUNCATED:
    return "quadro cortado";
  case IR_ERR_DECODE:
    return "tempos incoerentes";
  case IR_ERR_CHECK:
    return "falhou na validacao";
  case IR_ERR_AMBIGUOUS:
    return "ambiguo";
  default:
    return "?";
  }
}
