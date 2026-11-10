/* Sequencias de codigos IR. Ver ir_seq.h. */
#include "ir_seq.h"

#include <stddef.h>
#include <string.h>

/* (a - b) >= d num relogio de 32 bits que pode dar a volta. */
static bool elapsed(uint32_t now, uint32_t since, uint32_t d) {
  return (uint32_t)(now - since) >= d;
}

static bool same_code(const struct ir_code *a, const struct ir_code *b) {
  return a->protocol == b->protocol && a->code == b->code;
}

/* ------------------------------------------------------------------ */
/* Envio                                                               */
/* ------------------------------------------------------------------ */

int ir_seq_encode(const struct ir_seq *seq, uint32_t *dur, int max) {
  int pos = 0;

  if (!ir_seq_is_valid(seq)) {
    return 0;
  }
  /* Um codigo: exatamente o que o controle faz num toque. */
  if (seq->count == 1) {
    return ir_encode(&seq->codes[0], dur, max);
  }

  for (int i = 0; i < seq->count; i++) {
    if (i > 0) {
      /* O intervalo gravado e de inicio a inicio; o silencio e o que
       * sobra depois do quadro anterior. Sem o minimo, um intervalo
       * menor que o quadro faria a subtracao sem sinal dar a volta e
       * virar mais de uma hora de espera com o escalonador travado. */
      uint32_t frame = ir_frame_duration_us(&seq->codes[i - 1]);
      uint32_t gap = seq->gap_us[i];
      uint32_t silence = gap > frame + IR_SEQ_MIN_GAP_US
                             ? gap - frame
                             : IR_SEQ_MIN_GAP_US;

      if (pos + 1 > max) {
        return 0;
      }
      /* O quadro tem tamanho impar, entao esta posicao e um espaco e o
       * proximo quadro comeca num pulso, como o ir_send espera. */
      dur[pos++] = silence;
    }
    int n = ir_encode_frame(&seq->codes[i], dur + pos, max - pos);

    if (n == 0) {
      return 0;
    }
    pos += n;
  }
  return pos;
}

/* ------------------------------------------------------------------ */
/* Armazenamento                                                       */
/* ------------------------------------------------------------------ */

static void put_u32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p) {
  return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
         (uint32_t)p[3] << 24;
}

bool ir_seq_is_valid(const struct ir_seq *seq) {
  if (seq->count < 1 || seq->count > IR_SEQ_MAX || seq->gap_us[0] != 0) {
    return false;
  }
  for (int i = 0; i < seq->count; i++) {
    const struct ir_code *c = &seq->codes[i];

    if (ir_frame_duration_us(c) == 0 || !ir_code_is_valid(c->protocol, c->code)) {
      return false;
    }
    if (i > 0 && (seq->gap_us[i] == 0 || seq->gap_us[i] > IR_SEQ_WINDOW_US)) {
      return false;
    }
  }
  return true;
}

int ir_seq_pack(const struct ir_seq *seq, uint8_t *buf, int max) {
  int len = 2 + 9 * seq->count;

  if (!ir_seq_is_valid(seq) || len > max) {
    return 0;
  }
  buf[0] = IR_SEQ_STORE_VERSION;
  buf[1] = (uint8_t)seq->count;
  for (int i = 0; i < seq->count; i++) {
    uint8_t *p = buf + 2 + 9 * i;

    p[0] = (uint8_t)seq->codes[i].protocol;
    put_u32(p + 1, seq->codes[i].code);
    put_u32(p + 5, seq->gap_us[i]);
  }
  return len;
}

int ir_seq_unpack(const uint8_t *buf, int len, struct ir_seq *seq) {
  struct ir_seq s;

  memset(&s, 0, sizeof(s));
  if (len < 2 || buf[0] != IR_SEQ_STORE_VERSION || buf[1] < 1 ||
      buf[1] > IR_SEQ_MAX || len != 2 + 9 * buf[1]) {
    return -1;
  }
  s.count = buf[1];
  for (int i = 0; i < s.count; i++) {
    const uint8_t *p = buf + 2 + 9 * i;

    s.codes[i].protocol = (enum ir_protocol)p[0];
    s.codes[i].code = get_u32(p + 1);
    s.gap_us[i] = get_u32(p + 5);
  }
  if (!ir_seq_is_valid(&s)) {
    return -1;
  }
  *seq = s;
  return 0;
}

static void put_u16(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static uint32_t get_u16(const uint8_t *p) {
  return (uint32_t)p[0] | (uint32_t)p[1] << 8;
}

bool ir_raw_is_valid(const struct ir_raw *raw) {
  if (raw->n < IR_RAW_MIN_LEN || raw->n > IR_RAW_MAX || raw->n % 2 == 0) {
    return false;
  }
  for (int i = 0; i < raw->n; i++) {
    if (raw->d[i] == 0) {
      return false;
    }
  }
  return true;
}

int ir_raw_pack(const struct ir_raw *raw, uint8_t *buf, int max) {
  int len = 3 + 2 * raw->n;

  if (!ir_raw_is_valid(raw) || len > max) {
    return 0;
  }
  buf[0] = IR_RAW_STORE_VERSION;
  put_u16(buf + 1, (uint32_t)raw->n);
  for (int i = 0; i < raw->n; i++) {
    put_u16(buf + 3 + 2 * i, raw->d[i]);
  }
  return len;
}

int ir_raw_unpack(const uint8_t *buf, int len, struct ir_raw *raw) {
  int n;

  if (len < 3 || buf[0] != IR_RAW_STORE_VERSION) {
    return -1;
  }
  n = (int)get_u16(buf + 1);
  if (n < 1 || n > IR_RAW_MAX || len != 3 + 2 * n) {
    return -1;
  }
  for (int i = 0; i < n; i++) {
    raw->d[i] = (uint16_t)get_u16(buf + 3 + 2 * i);
  }
  raw->n = n;
  if (!ir_raw_is_valid(raw)) {
    raw->n = 0;
    return -1;
  }
  return 0;
}

int ir_store_version(const uint8_t *buf, int len) {
  return len >= 1 ? buf[0] : -1;
}

/* ------------------------------------------------------------------ */
/* Gravacao                                                            */
/* ------------------------------------------------------------------ */

void ir_rec_reset(struct ir_recorder *rec) { memset(rec, 0, sizeof(*rec)); }

/* true se um codigo foi aceito ha pouco (este quadro e do mesmo toque). */
static bool recently_accepted(const struct ir_recorder *rec, uint32_t t) {
  return rec->has_last && !elapsed(t, rec->last_us, IR_SEQ_REPEAT_US);
}

/* Um quadro falhou (ou ficou sem confirmacao). */
static void note_failure(struct ir_recorder *rec, const char *reason,
                         uint32_t t) {
  rec->has_reject = true;
  rec->reject_us = t;

  /* Ja existe codigo aceito: nada de "Nao reconhecido", mas a sequencia
   * pode ter ficado incompleta. */
  if (rec->open) {
    rec->suspect = true;
    return;
  }
  if (recently_accepted(rec, t)) {
    return;
  }
  rec->warn_pending = true;
  rec->warn_us = t;
  rec->warn_reason = reason;
}

static bool in_sequence(const struct ir_recorder *rec,
                        const struct ir_code *code) {
  for (int i = 0; i < rec->seq.count; i++) {
    if (same_code(&rec->seq.codes[i], code)) {
      return true;
    }
  }
  return false;
}

enum ir_rec_result ir_rec_frame(struct ir_recorder *rec, enum ir_status st,
                                const struct ir_code *code, int lost_pulses,
                                uint32_t t_us) {
  if (st == IR_OK || st == IR_REPEAT) {
    rec->known_seen = true;
  }
  if (st == IR_REPEAT) {
    /* Botao NEC segurado: so prolonga a janela de repeticao. */
    if (recently_accepted(rec, t_us)) {
      rec->last_us = t_us;
    }
    return IR_REC_REPEAT;
  }
  if (st != IR_OK) {
    note_failure(rec, ir_status_name(st), t_us);
    return IR_REC_REJECTED;
  }


  /* Instante que vale para o codigo: o do primeiro quadro que o leu. */
  uint32_t t_code = t_us;

  if (lost_pulses > 0) {
    bool by_pending = rec->has_pending && same_code(&rec->pending, code) &&
                      !elapsed(t_us, rec->pending_us, IR_SEQ_REPEAT_US);
    bool by_last = recently_accepted(rec, t_us) && same_code(&rec->last, code);

    if (!by_pending && !by_last) {
      rec->pending = *code;
      rec->pending_us = t_us;
      rec->has_pending = true;
      note_failure(rec, "sinal fraco e sem confirmacao", t_us);
      return IR_REC_PENDING;
    }
    /* Confirmado: o codigo comecou no quadro pendente, se ele for da
     * janela (ou se a janela ainda nao abriu). */
    if (by_pending &&
        (!rec->open || (int32_t)(rec->pending_us - rec->start_us) >= 0)) {
      t_code = rec->pending_us;
    }
  }
  rec->has_pending = false;
  rec->warn_pending = false;

  /* Segundo quadro do mesmo toque, ou botao segurado. */
  if (recently_accepted(rec, t_us) && same_code(&rec->last, code)) {
    rec->last_us = t_us;
    return IR_REC_SAME;
  }
  /* A janela vale pelo inicio do quadro: um quadro que comecou dentro
   * dela entra, mesmo que o main o veja depois do prazo. */
  if (rec->open && elapsed(t_us, rec->start_us, IR_SEQ_WINDOW_US)) {
    return IR_REC_LATE;
  }
  rec->last = *code;
  rec->has_last = true;
  rec->last_us = t_us;

  if (!rec->open) {
    rec->open = true;
    rec->cycled = false;
    rec->start_us = t_code;
    rec->seq.count = 1;
    rec->seq.codes[0] = *code;
    rec->seq.gap_us[0] = 0;
    /* Um quadro rejeitado logo antes pode ter sido o primeiro codigo da
     * sequencia; o slot ficaria so com o segundo. */
    if (rec->has_reject &&
        !elapsed(t_code, rec->reject_us, IR_SEQ_WINDOW_US)) {
      rec->suspect = true;
    }
    return IR_REC_ADDED;
  }

  /* Um codigo que ja esta na sequencia quer dizer que o controle
   * recomecou (botao segurado: A, B, A, B...). Dai em diante nada entra:
   * gravar A, B, A tornaria o envio diferente do toque original. */
  if (rec->cycled || in_sequence(rec, code)) {
    rec->cycled = true;
    return IR_REC_CYCLE;
  }
  if (rec->seq.count >= IR_SEQ_MAX) {
    return IR_REC_FULL;
  }
  rec->seq.codes[rec->seq.count] = *code;
  rec->seq.gap_us[rec->seq.count] = t_code - rec->start_us;
  rec->seq.count++;
  return IR_REC_ADDED;
}

static void raw_put(struct ir_recorder *rec, uint32_t v) {
  if (rec->raw.n >= IR_RAW_MAX) {
    rec->raw_overflow = true;
    return;
  }
  rec->raw.d[rec->raw.n++] =
      (uint16_t)(v > IR_RAW_DUR_MAX ? IR_RAW_DUR_MAX : (v == 0 ? 1 : v));
}

void ir_rec_capture(struct ir_recorder *rec, const uint32_t *dur, int n,
                    uint32_t t_us, bool truncated) {
  uint32_t total = 0;

  /* Um codigo ja foi aceito: o toque e uma sequencia, nao cru. */
  if (rec->open || n < 1) {
    return;
  }
  for (int i = 0; i < n; i++) {
    total += dur[i];
  }
  if (!rec->raw_on) {
    if (n < IR_RAW_MIN_LEN || n % 2 == 0 || dur[0] < IR_RAW_MIN_PULSE_US ||
        truncated) {
      return;
    }
    rec->raw_on = true;
    rec->raw_overflow = false;
    rec->raw.n = 0;
  } else {
    uint32_t gap = t_us - rec->raw_end_us;

    if (gap >= IR_RAW_JOIN_US) {
      return; /* outro toque: a gravacao atual ja vai fechar */
    }
    /* O silencio entre as capturas vira um espaco. Uma captura que nao
     * termina num pulso (cortada) nao pode continuar. */
    if (rec->raw.n % 2 == 0) {
      rec->raw_overflow = true;
      return;
    }
    raw_put(rec, gap);
  }
  for (int i = 0; i < n; i++) {
    raw_put(rec, dur[i]);
  }
  if (truncated || n % 2 == 0) {
    rec->raw_overflow = true;
  }
  rec->raw_end_us = t_us + total;
}

const struct ir_raw *ir_rec_raw(const struct ir_recorder *rec) {
  return &rec->raw;
}

enum ir_rec_poll ir_rec_poll(struct ir_recorder *rec, uint32_t now_us,
                             struct ir_seq *out, bool *suspect) {
  /* Com uma gravacao crua em montagem, a falha de decodificacao e
   * esperada: nada de "Nao reconhecido". */
  if (rec->warn_pending && rec->raw_on && !rec->open) {
    rec->warn_pending = false;
  }
  if (rec->warn_pending &&
      elapsed(now_us, rec->warn_us, IR_SEQ_WARN_DELAY_US)) {
    rec->warn_pending = false;
    return IR_REC_POLL_WARN;
  }
  if (rec->open &&
      elapsed(now_us, rec->start_us, IR_SEQ_WINDOW_US + IR_SEQ_SETTLE_US)) {
    *out = rec->seq;
    if (suspect != NULL) {
      *suspect = rec->suspect || rec->has_pending;
    }
    ir_rec_reset(rec);
    return IR_REC_POLL_DONE;
  }
  /* Comparacao com sinal: um "agora" anterior ao fim da captura (que o
   * main nunca passa, mas um chamador poderia) nao fecha a gravacao. */
  if (!rec->open && rec->raw_on &&
      (int32_t)(now_us - (rec->raw_end_us + IR_RAW_SETTLE_US)) >= 0) {
    rec->raw_on = false;
    /* Um quadro de protocolo conhecido sem confirmacao (sinal fraco):
     * melhor pedir outro toque do que gravar o sinal fraco cru. */
    if (rec->known_seen) {
      rec->known_seen = false;
      rec->warn_reason = "sinal fraco e sem confirmacao";
      return IR_REC_POLL_WARN;
    }
    if (suspect != NULL) {
      *suspect = rec->raw_overflow;
    }
    /* Uma gravacao que nao termina num pulso nao pode ser reenviada. */
    if (rec->raw.n % 2 == 0 && rec->raw.n > 0) {
      rec->raw.n--;
    }
    return IR_REC_POLL_RAW;
  }
  return IR_REC_POLL_NONE;
}
