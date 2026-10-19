/* Decodificador de IR robusto para NEC e Samsung32. Ver ir_decode.h.
 *
 * Como o sinal e codificado (NEC e Samsung32 usam o mesmo esquema):
 *
 *   cabecalho: pulso longo + espaco longo
 *   bit 0:     pulso 560 + espaco  560  (periodo 1120)
 *   bit 1:     pulso 560 + espaco 1690  (periodo 2250)
 *   fim:       pulso 560
 *
 * O que o receptor da placa faz com isso: cada pulso chega esticado por
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

/* Tempos nominais dos bits, em us. */
#define BIT_PULSE 560
#define BIT_SPACE_0 560
#define BIT_SPACE_1 1690
#define PERIOD_0 (BIT_PULSE + BIT_SPACE_0)
#define PERIOD_1 (BIT_PULSE + BIT_SPACE_1)
#define FRAME_BITS 32

/* Intervalo entre inicios de quadro dos controles NEC e Samsung. */
#define REPEAT_PERIOD_US 108000

/* Faixa de esticamento aceita nos bits. Medido: de -400 (sinal fraco)
 * ate ~1600 (pino com pull-up). O maximo fisico e um pouco menos que
 * 1690: acima disso ate o espaco do bit 1 some. Fora da faixa, a
 * hipotese e descartada. */
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

enum ir_status ir_decode_frame(const uint32_t *dur, int n,
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
  }
  return IR_OK;
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

int ir_encode(const struct ir_code *code, uint32_t *dur, int max) {
  const struct protocol_timing *t = NULL;
  uint32_t elapsed = 0;
  int pos;

  for (size_t p = 0; p < sizeof(timings) / sizeof(timings[0]); p++) {
    if (timings[p].protocol == code->protocol) {
      t = &timings[p];
    }
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
