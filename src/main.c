#include <string.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

/* API do Pico SDK (hal_rpi_pico), usada so para a histerese do pino,
 * que o driver GPIO do Zephyr nao expoe. */
#include <hardware/gpio.h>
#include <hardware/structs/pads_bank0.h>

#include "ir_decode.h"
#include "ir_edges.h"
#include "ir_seq.h"
#include "slot_store.h"
#include "ui.h"

/* Bordas por captura. Um quadro NEC tem 68; ar-condicionado chega a
 * centenas (gravacao crua). */
#define MAX_EDGES IR_RAW_MAX

/* Mais quadros que isso numa captura sao ignorados (MAX_EDGES cabe ~3). */
#define MAX_FRAMES 8

/* Silencio que encerra uma captura. Menor que o intervalo entre os
 * quadros de um toque (~46 ms), para cada quadro chegar separado. */
#define END_OF_CAPTURE_MS 20

/* 1 imprime as duracoes brutas de cada captura no modo gravar. Com
 * CONFIG_APP_IR_DEBUG, elas saem sempre, na impressao de depuracao. */
#define PRINT_RAW 1

/* Histerese (schmitt trigger) do pino do receptor, escolhida no
 * app.overlay pela propriedade ir-hysteresis do no zephyr,user. O padrao
 * do RP2350 apos o reset e ligada. */
#define ZEPHYR_USER DT_PATH(zephyr_user)
#define IR_HYSTERESIS DT_PROP_OR(ZEPHYR_USER, ir_hysteresis, 1)

/* Receptor (entrada) e portadora do emissor (saida). */
static const struct gpio_dt_spec ir =
    GPIO_DT_SPEC_GET(DT_NODELABEL(ir_receiver), gpios);
static const struct pwm_dt_spec carrier =
    PWM_DT_SPEC_GET(DT_NODELABEL(ir_carrier));
static struct gpio_callback ir_cb;

/* Captura: preenchido pela interrupcao, pelo filtro do ir_edges.h, que
 * le o nivel do pino e apaga os intervalos da portadora. */
static volatile uint32_t edges[MAX_EDGES];
static struct ir_edges cap = {.t = edges, .max = MAX_EDGES};
static volatile uint32_t last_edge;
static uint32_t ir_pin; /* numero absoluto do pino (GPIO47) */
static uint32_t copy[MAX_EDGES];
static uint32_t durations[MAX_EDGES];

/* Junta os quadros de um toque numa sequencia (ir_seq.c). So os codigos
 * vao para os slots; os tempos sao remontados na hora de enviar. */
static struct ir_recorder rec;

/* Transmissao montada a partir da sequencia ou da gravacao crua. */
#define TX_MAX (IR_SEQ_TX_MAX > IR_RAW_MAX ? IR_SEQ_TX_MAX : IR_RAW_MAX)
static uint32_t tx_buf[TX_MAX];

/* Copia da gravacao crua pronta (grande demais para a pilha). */
static struct ir_raw raw_done;

/* Resultado de um quadro, guardado para a impressao de depuracao. */
struct frame_result {
  struct ir_span span;
  enum ir_status status;
  struct ir_code code;
  struct ir_decode_info info;
  uint32_t t_us;      /* inicio do quadro (primeira borda) */
  uint32_t decode_us; /* tempo do ir_decode_frame na placa */
};

static void on_edge(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins) {
  uint32_t now = k_cycle_get_32();

  /* Leitura direta do SIO (inline, poucos ciclos). */
  ir_edges_on_edge(&cap, now, gpio_get(ir_pin));
  last_edge = now;
}

/* Reproduz um sinal: nos pulsos liga a portadora (duty ~1/3),
 * nos espacos desliga. */
static void ir_send(const uint32_t *dur_us, int count) {
  /* Sem isso, o proprio receptor da placa capturaria o envio. */
  gpio_pin_interrupt_configure_dt(&ir, GPIO_INT_DISABLE);

  /* Evita que outra thread interrompa o envio no meio. */
  k_sched_lock();

  uint32_t t = k_cycle_get_32();

  for (int i = 0; i < count; i++) {
    bool pulse = (i % 2) == 0;

    pwm_set_pulse_dt(&carrier, pulse ? carrier.period / 3 : 0);

    /* Espera ate o instante exato do fim deste intervalo.
     * Usar um alvo absoluto evita acumular atraso. */
    t += k_us_to_cyc_ceil32(dur_us[i]);
    while ((int32_t)(k_cycle_get_32() - t) < 0) {
    }
  }
  pwm_set_pulse_dt(&carrier, 0);

  k_sched_unlock();

  /* Descarta o que tiver sido capturado e volta a escutar. */
  k_msleep(50);
  cap.n = 0;
  gpio_pin_interrupt_configure_dt(&ir, GPIO_INT_EDGE_BOTH);
}

/* Se a captura terminou (silencio de END_OF_CAPTURE_MS), copia as bordas
 * para out e devolve quantas sao; senao devolve 0.
 *
 * Tudo acontece com as interrupcoes travadas. Antes, o relogio era lido
 * primeiro e last_edge depois; se uma borda chegasse entre as duas
 * leituras, last_edge ficava maior que o "agora", a subtracao sem sinal
 * dava ~4 bilhoes e a captura era encerrada no meio do quadro. Com
 * irq_lock, on_edge nao roda entre as leituras, entao agora >= last_edge,
 * e a decisao de encerrar e a copia ficam consistentes: nenhuma borda
 * nova entra depois da decisao. */
static int take_capture(uint32_t *out) {
  int n = 0;
  unsigned int key = irq_lock();
  uint32_t idle = k_cycle_get_32() - last_edge;

  if (cap.n > 0 && k_cyc_to_ms_floor32(idle) >= END_OF_CAPTURE_MS) {
    n = cap.n;
    for (int i = 0; i < n; i++) {
      out[i] = edges[i];
    }
    cap.n = 0;
  }
  irq_unlock(key);
  return n;
}

/* Relogio do gravador: us de 32 bits, tirado do contador de ciclos de
 * 64 bits (da a volta a cada ~71 min, sem salto). */
static uint32_t now_us(void) {
  return (uint32_t)k_cyc_to_us_floor64(k_cycle_get_64());
}

/* Instante de uma borda no relogio do gravador. A interrupcao guarda so
 * os 32 bits de baixo do contador (que da a volta a cada ~28 s); a idade
 * da borda, medida contra o mesmo "agora", reconstroi os 64 bits. Assim
 * vale o horario em que o sinal chegou, nao a hora em que o main viu a
 * captura (que atrasa com o desenho da tela ou um printk). */
static uint32_t edge_time_us(uint32_t edge_cyc, uint64_t now_cyc) {
  uint32_t age = (uint32_t)now_cyc - edge_cyc;

  return (uint32_t)k_cyc_to_us_floor64(now_cyc - age);
}

/* Envia a sequencia de um slot gravado, com "Enviando..." na tela. */
static void send_slot(int slot) {
  const struct ir_seq *seq = ui_slot_sequence(slot);
  const uint16_t *raw;
  int raw_n = ui_slot_raw(slot, &raw);

  if (raw_n > 0 && raw_n <= (int)ARRAY_SIZE(tx_buf)) {
    uint32_t total = 0;

    for (int i = 0; i < raw_n; i++) {
      tx_buf[i] = raw[i];
      total += raw[i];
    }
    printk("Enviando \"%s / %s\" (%s, %d intervalos, %u ms)\n",
           ui_device_name(ui_device()), ui_slot_name(slot),
           ui_slot_source(slot) == UI_SRC_RECORDED ? "gravacao crua"
                                                   : "de fabrica",
           raw_n, total / 1000);
    ui_set_sending(slot);
    ui_draw();
    ir_send(tx_buf, raw_n);
    ui_set_sending(-1);
    printk("Enviado\n\n");
    return;
  }
  if (seq == NULL) {
    return;
  }
  int n = ir_seq_encode(seq, tx_buf, ARRAY_SIZE(tx_buf));

  if (n == 0) {
    printk("Slot \"%s\": sequencia invalida, nada enviado\n",
           ui_slot_name(slot));
    return;
  }
  printk("Enviando \"%s / %s\" (%s, %d codigo(s)):\n",
         ui_device_name(ui_device()), ui_slot_name(slot),
         ui_slot_source(slot) == UI_SRC_RECORDED ? "gravado" : "de fabrica",
         seq->count);
  for (int i = 0; i < seq->count; i++) {
    printk("  %s 0x%08X (+%u us)\n", ir_protocol_name(seq->codes[i].protocol),
           seq->codes[i].code, seq->gap_us[i]);
  }
  /* ir_send trava o escalonador, entao a tela e desenhada antes. */
  ui_set_sending(slot);
  ui_draw();
  ir_send(tx_buf, n);
  ui_set_sending(-1);
  printk("Enviado\n\n");
}

static void print_durations(const uint32_t *dur, int n) {
  printk("Bruto: %d intervalos\n", n);
  for (int i = 0; i < n; i++) {
    printk("%c%u ", (i % 2) ? '-' : '+', dur[i]);
    if ((i + 1) % 8 == 0) {
      printk("\n");
    }
  }
  printk("\n");
}

/* Mostra no console o que o gravador fez com um quadro. */
static void report_frame(const struct frame_result *f,
                         enum ir_rec_result r) {
  const struct ir_seq *seq = &rec.seq;

  switch (r) {
  case IR_REC_ADDED:
    printk("Gravado: %s 0x%08X (codigo %d, +%u us)\n",
           ir_protocol_name(f->code.protocol), f->code.code, seq->count,
           seq->gap_us[seq->count - 1]);
    break;
  case IR_REC_PENDING:
    printk("Quadro %s 0x%08X com pulso perdido: esperando confirmacao\n",
           ir_protocol_name(f->code.protocol), f->code.code);
    break;
  case IR_REC_CYCLE:
    printk("Codigo 0x%08X ja esta na sequencia (controle recomecou): "
           "ignorado\n", f->code.code);
    break;
  case IR_REC_FULL:
    printk("Sequencia cheia (%d codigos): 0x%08X ignorado\n", IR_SEQ_MAX,
           f->code.code);
    break;
  case IR_REC_LATE:
    printk("Quadro 0x%08X comecou depois da janela: ignorado\n",
           f->code.code);
    break;
  case IR_REC_REJECTED:
    printk("Quadro rejeitado: %s\n", ir_status_name(f->status));
    break;
  case IR_REC_SAME:
  case IR_REC_REPEAT:
    break;
  }
}

/* Impressao de depuracao de toda captura, com ou sem modo gravar
 * (CONFIG_APP_IR_DEBUG). Feita depois do gravador, para nao atrasar
 * nada que dependa de tempo; o gravador usa o horario das bordas. */
static void debug_capture(int n_edges_copied, int n, const struct frame_result *fr,
                          int frames, uint32_t t0_us) {
  static uint32_t count;
  static uint32_t prev_t0_us;
  static uint32_t prev_end_us;
  uint32_t end_us = t0_us + k_cyc_to_us_floor32(copy[n_edges_copied - 1] - copy[0]);

  count++;
  printk("== IR captura %u: t=%u ms", count, t0_us / 1000);
  if (count > 1) {
    printk(", +%u ms desde a anterior (silencio %u ms)",
           (t0_us - prev_t0_us) / 1000, (t0_us - prev_end_us) / 1000);
  }
  printk(", modo gravar: %s\n", ui_is_recording() ? "sim" : "nao");
  printk("Limite MAX_EDGES (%d bordas): %s\n", MAX_EDGES,
         n_edges_copied >= MAX_EDGES ? "SIM, captura cortada" : "nao");
  printk("Filtro: %u espacos de portadora apagados, %u bordas fora de "
         "ordem ignoradas (desde a partida)\n", cap.merged, cap.ignored);
  prev_t0_us = t0_us;
  prev_end_us = end_us;

  print_durations(durations, n);
  for (int f = 0; f < frames; f++) {
    const struct frame_result *r = &fr[f];

    printk("quadro %d: %d intervalos, +%u us: ", f + 1, r->span.len,
           r->t_us - t0_us);
    if (r->status == IR_OK) {
      printk("%s 0x%08X, esticamento %d us (%d a %d), pulsos perdidos %d",
             ir_protocol_name(r->code.protocol), r->code.code,
             r->info.stretch_avg_us, r->info.stretch_min_us,
             r->info.stretch_max_us, r->info.lost_pulses);
      printk(", %d passos, caminho %s", r->info.search_steps,
             r->info.path == IR_PATH_CONVENTIONAL ? "convencional"
                                                  : "robusto");
    } else {
      printk("%s", ir_status_name(r->status));
    }
    printk(", decodificado em %u us\n", r->decode_us);
  }
  printk("\n");
}

/* Separa a captura em quadros, decodifica cada um e entrega ao gravador.
 * O ir_decode para no primeiro quadro valido; aqui cada quadro conta,
 * porque se o main atrasar dois codigos diferentes podem cair na mesma
 * captura. */
static void handle_capture(int n_edges_copied) {
  static struct frame_result fr[MAX_FRAMES];
  struct ir_span spans[MAX_FRAMES];
  int n = n_edges_copied - 1;
  uint64_t now_cyc = k_cycle_get_64();
  bool recording = ui_is_recording();
  bool debug = IS_ENABLED(CONFIG_APP_IR_DEBUG);

  if (!recording && !debug) {
    return;
  }

  /* Converte instantes em duracoes: pulso, espaco, pulso... */
  for (int i = 0; i < n; i++) {
    durations[i] = k_cyc_to_us_floor32(copy[i + 1] - copy[i]);
  }
  if (PRINT_RAW && recording && !debug) {
    print_durations(durations, n);
  }

  int frames = ir_split_frames(durations, n, spans, MAX_FRAMES);

  for (int f = 0; f < frames; f++) {
    struct frame_result *r = &fr[f];
    uint32_t c0 = k_cycle_get_32();

    r->span = spans[f];
    r->status = ir_decode_frame(durations + spans[f].start, spans[f].len,
                                &r->code, &r->info);
    r->decode_us = k_cyc_to_us_floor32(k_cycle_get_32() - c0);
    /* durations[i] comeca na borda copy[i]. */
    r->t_us = edge_time_us(copy[spans[f].start], now_cyc);

    if (recording) {
      enum ir_rec_result res = ir_rec_frame(
          &rec, r->status, &r->code,
          r->status == IR_OK ? r->info.lost_pulses : 0, r->t_us);

      if (!debug && r->status == IR_OK) {
        printk("Quadro: %s 0x%08X, esticamento %d us (%d a %d), "
               "pulsos perdidos %d\n",
               ir_protocol_name(r->code.protocol), r->code.code,
               r->info.stretch_avg_us, r->info.stretch_min_us,
               r->info.stretch_max_us, r->info.lost_pulses);
      }
      report_frame(r, res);
    }
  }

  /* A captura inteira vai para a gravacao crua, que so vale se nenhum
   * codigo for reconhecido no toque. */
  if (recording) {
    ir_rec_capture(&rec, durations, n, edge_time_us(copy[0], now_cyc),
                   n_edges_copied >= MAX_EDGES);
  }
  if (debug) {
    debug_capture(n_edges_copied, n, fr, frames,
                  edge_time_us(copy[0], now_cyc));
  }
}

/* Confere o gravador: aviso de falha adiado e fim da janela. */
static void poll_recorder(void) {
  struct ir_seq seq;
  bool suspect = false;

  switch (ir_rec_poll(&rec, now_us(), &seq, &suspect)) {
  case IR_REC_POLL_WARN:
    printk("Aviso: sinal nao reconhecido (%s); nada foi gravado.\n"
           "Tente de novo com o controle a uns 30 cm a 1 m da placa.\n\n",
           rec.warn_reason);
    ui_record_failed();
    break;

  case IR_REC_POLL_DONE: {
    int slot = ui_record_sequence(&seq);

    if (slot < 0) {
      break;
    }
    printk("Slot \"%s\" gravado com %d codigo(s)\n", ui_slot_name(slot),
           seq.count);
    if (suspect) {
      printk("Aviso: um quadro perto deste toque falhou; a sequencia pode "
             "estar incompleta.\nSe o aparelho nao reagir, grave de novo.\n");
    }
    int err = slot_store_save(ui_device(), slot, &seq);

    if (err != 0) {
      printk("Aviso: falha ao salvar na flash (%d); o slot vale ate "
             "reiniciar\n", err);
    }
    printk("\n");
    break;
  }

  case IR_REC_POLL_RAW: {
    const struct ir_raw *raw = ir_rec_raw(&rec);

    raw_done.n = raw->n;
    memcpy(raw_done.d, raw->d, sizeof(raw->d[0]) * raw->n);
    ir_rec_reset(&rec);

    int slot = ui_record_raw(&raw_done);

    if (slot < 0) {
      printk("Gravacao crua nao salva (%d intervalos): sinal invalido ou "
             "sem espaco (%d gravacoes cruas no maximo)\n\n", raw_done.n,
             UI_RAW_SLOTS);
      break;
    }
    printk("Slot \"%s\" gravado cru: %d intervalos (protocolo nao "
           "reconhecido; reenvia os tempos como chegaram)\n",
           ui_slot_name(slot), raw_done.n);
    if (suspect) {
      printk("Aviso: o toque passou de %d intervalos e foi cortado.\n",
             IR_RAW_MAX);
    }
    int err = slot_store_save_raw(ui_device(), slot, &raw_done);

    if (err != 0) {
      printk("Aviso: falha ao salvar na flash (%d); o slot vale ate "
             "reiniciar\n", err);
    }
    printk("\n");
    break;
  }

  case IR_REC_POLL_NONE:
    break;
  }
}

static void on_slot_loaded(int dev, int slot, const struct ir_seq *seq) {
  ui_restore_slot(dev, slot, seq);
  printk("Flash: \"%s / %s\" gravado com %d codigo(s)\n",
         ui_device_name(dev), ui_device_slot_name(dev, slot),
         seq->count);
}

static void on_raw_loaded(int dev, int slot, const struct ir_raw *raw) {
  ui_restore_raw(dev, slot, raw);
  printk("Flash: \"%s / %s\" gravado cru (%d intervalos)\n",
         ui_device_name(dev), ui_device_slot_name(dev, slot), raw->n);
}

static int saved_device;

static void on_device_loaded(int dev) {
  ui_set_device(dev);
  saved_device = dev;
}

static const struct slot_store_cb store_cb = {
    .num_devices = IR_LIB_NUM_DEVICES,
    .max_slots = IR_LIB_MAX_SLOTS,
    /* Os slots das versoes anteriores (um aparelho so) vao para "Livre". */
    .legacy_device = IR_LIB_LIVRE,
    .on_slot = on_slot_loaded,
    .on_raw = on_raw_loaded,
    .on_device = on_device_loaded,
};

/* Leva para a flash o que a interface mudou: aparelho escolhido e
 * gravacoes apagadas. */
static void sync_store(void) {
  int dev, slot;

  if (ui_device() != saved_device) {
    saved_device = ui_device();
    if (slot_store_save_device(saved_device) != 0) {
      printk("Aviso: falha ao salvar o aparelho na flash\n");
    }
  }
  if (ui_take_erased(&dev, &slot)) {
    int err = slot_store_erase(dev, slot);

    /* -ENOENT: a gravacao ainda nao tinha chegado a flash. */
    if (err != 0 && err != -ENOENT) {
      printk("Aviso: falha ao apagar da flash (%d)\n", err);
    }
    printk("\n");
  }
}

/* Numero absoluto do pino (0..47): o banco gpio0_hi comeca no GPIO32. */
static uint32_t ir_pad_pin(void) {
  return ir.pin + (ir.port == DEVICE_DT_GET(DT_NODELABEL(gpio0_hi)) ? 32 : 0);
}

static void print_receiver_config(void) {
  uint32_t pin = ir_pad_pin();
  uint32_t pad = pads_bank0_hw->io[pin];

  printk("Receptor GPIO%u: pull %s, histerese %s, nivel em repouso %d\n",
         pin,
         (pad & PADS_BANK0_GPIO0_PUE_BITS)   ? "up"
         : (pad & PADS_BANK0_GPIO0_PDE_BITS) ? "down"
                                             : "nenhum",
         (pad & PADS_BANK0_GPIO0_SCHMITT_BITS) ? "ligada" : "desligada",
         gpio_pin_get_raw(ir.port, ir.pin));
}

int main(void) {
  if (!gpio_is_ready_dt(&ir) || !pwm_is_ready_dt(&carrier)) {
    printk("Receptor ou PWM nao esta pronto\n");
    return 0;
  }
  pwm_set_pulse_dt(&carrier, 0);

  /* GPIO_INPUT se junta as flags do devicetree (pull-up/down). */
  if (gpio_pin_configure_dt(&ir, GPIO_INPUT) < 0 ||
      gpio_pin_interrupt_configure_dt(&ir, GPIO_INT_EDGE_BOTH) < 0) {
    printk("Falha ao configurar o pino do receptor\n");
    return 0;
  }
  gpio_set_input_hysteresis_enabled(ir_pad_pin(), IR_HYSTERESIS);
  ir_pin = ir_pad_pin();
  cap.merge = k_us_to_cyc_ceil32(IR_EDGE_MERGE_US);
  gpio_init_callback(&ir_cb, on_edge, BIT(ir.pin));
  gpio_add_callback(ir.port, &ir_cb);

  if (ui_init() < 0) {
    printk("Display nao esta pronto; seguindo sem tela\n");
  }
  int loaded = slot_store_init(&store_cb);

  if (loaded < 0) {
    printk("Flash: settings indisponivel (%d); gravacoes so na RAM\n",
           loaded);
  } else {
    printk("Flash: %d gravacao(oes) carregada(s)\n", loaded);
  }
  printk("Aparelho: %s\n", ui_device_name(ui_device()));
  ui_draw();

  print_receiver_config();
  if (IS_ENABLED(CONFIG_APP_IR_DEBUG)) {
    printk("CONFIG_APP_IR_DEBUG ligada: toda captura e impressa\n");
  }
  printk("Gire o encoder para escolher um slot. Clique envia; segure\n"
         "0,8 s para gravar. No nome do aparelho (acima do 1o slot), o\n"
         "clique troca de aparelho. BUTTON0..3 enviam os slots 1..4.\n"
         "Na lista: fb = codigo de fabrica, gr = gravado, -- = vazio.\n\n");

  bool was_recording = false;

  while (1) {
    struct ui_event evt;

    /* Espera ate 10 ms por um evento de entrada; o timeout faz o papel do
     * tick que verifica a captura. Depois esvazia o que mais tiver na
     * fila, para girar rapido nao gerar um redesenho por clique. */
    if (k_msgq_get(&ui_msgq, &evt, K_MSEC(10)) == 0) {
      do {
        int slot = ui_handle_event(&evt);

        if (slot >= 0) {
          send_slot(slot);
        }
      } while (k_msgq_get(&ui_msgq, &evt, K_NO_WAIT) == 0);
    }
    ui_tick();

    /* Entrar ou sair do modo gravar (inclusive cancelar) zera o
     * gravador: nada de uma gravacao anterior vale depois. */
    bool recording = ui_is_recording();

    if (recording != was_recording) {
      ir_rec_reset(&rec);
      was_recording = recording;
    }

    /* A captura sempre e retirada do buffer; handle_capture so
     * decodifica no modo gravar (ou com a depuracao ligada). O gravador
     * e conferido depois, para um quadro que comecou dentro da janela
     * entrar mesmo que o main o veja depois do prazo. */
    int n = take_capture(copy);

    if (n >= 2) {
      handle_capture(n);
    }
    if (recording) {
      poll_recorder();
    }
    sync_store();
    ui_draw();
  }
  return 0;
}
