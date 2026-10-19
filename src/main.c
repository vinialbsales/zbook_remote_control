#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>

/* API do Pico SDK (hal_rpi_pico), usada so para a histerese do pino,
 * que o driver GPIO do Zephyr nao expoe. */
#include <hardware/gpio.h>
#include <hardware/structs/pads_bank0.h>

#include "ir_decode.h"

#define MAX_EDGES 200

/* Silencio que encerra uma captura. Menor que o intervalo entre os
 * quadros de um toque (~46 ms), para cada quadro chegar separado. */
#define END_OF_CAPTURE_MS 20

/* Quadros do mesmo codigo dentro dessa janela sao o mesmo toque. */
#define REPEAT_WINDOW_MS 200

/* 1 imprime as duracoes brutas de cada captura (util para depurar). */
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

/* Captura: preenchido pela interrupcao. */
static volatile uint32_t edges[MAX_EDGES];
static volatile int n_edges;
static volatile uint32_t last_edge;
static uint32_t copy[MAX_EDGES];
static uint32_t durations[MAX_EDGES];

/* Ultimo codigo gravado. So o protocolo e o codigo sao guardados; os
 * tempos sao remontados na hora de enviar. */
static struct ir_code saved;
static bool has_saved;
static uint32_t last_accept_ms;

/* Leitura que supos pulsos perdidos: so e gravada se outro quadro do
 * mesmo toque der o mesmo codigo. */
static struct ir_code pending;
static bool has_pending;
static uint32_t pending_ms;

/* Aviso de falha adiado: se outro quadro do mesmo toque decodificar
 * logo em seguida, o aviso e cancelado. */
static bool warn_pending;
static uint32_t warn_ms;
static const char *warn_reason;

/* Transmissao montada a partir do codigo gravado. */
static uint32_t tx_buf[IR_ENCODE_MAX];

/* Sinalizado quando o BUTTON0 e apertado. */
K_SEM_DEFINE(tx_sem, 0, 1);

static void on_edge(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins) {
  uint32_t now = k_cycle_get_32();

  if (n_edges < MAX_EDGES) {
    edges[n_edges++] = now;
  }
  last_edge = now;
}

/* Chamado pelo subsistema de entrada a cada evento de botao/encoder. */
static void on_input(struct input_event *evt, void *user_data) {
  if (evt->type == INPUT_EV_KEY && evt->code == INPUT_BTN_0 &&
      evt->value == 1) {
    k_sem_give(&tx_sem);
  }
}
INPUT_CALLBACK_DEFINE(NULL, on_input, NULL);

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
  n_edges = 0;
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

  if (n_edges > 0 && k_cyc_to_ms_floor32(idle) >= END_OF_CAPTURE_MS) {
    n = n_edges;
    for (int i = 0; i < n; i++) {
      out[i] = edges[i];
    }
    n_edges = 0;
  }
  irq_unlock(key);
  return n;
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

static bool same_code(const struct ir_code *a, const struct ir_code *b) {
  return a->protocol == b->protocol && a->code == b->code;
}

/* true se um codigo foi aceito ha pouco (este quadro e do mesmo toque). */
static bool recently_accepted(uint32_t now) {
  return has_saved && now - last_accept_ms < REPEAT_WINDOW_MS;
}

static void schedule_warning(const char *reason, uint32_t now) {
  if (recently_accepted(now)) {
    return;
  }
  warn_pending = true;
  warn_ms = now;
  warn_reason = reason;
}

/* Trata um quadro decodificado com sucesso. */
static void on_code(const struct ir_code *code, int lost_pulses,
                    uint32_t now) {
  if (lost_pulses > 0) {
    bool confirmed = (has_pending && same_code(&pending, code) &&
                      now - pending_ms < REPEAT_WINDOW_MS) ||
                     (recently_accepted(now) && same_code(&saved, code));

    if (!confirmed) {
      pending = *code;
      pending_ms = now;
      has_pending = true;
      schedule_warning("sinal fraco e sem confirmacao", now);
      return;
    }
  }
  has_pending = false;
  warn_pending = false;

  /* Segundo quadro do mesmo toque, ou botao segurado. */
  if (recently_accepted(now) && same_code(&saved, code)) {
    last_accept_ms = now;
    return;
  }

  saved = *code;
  has_saved = true;
  last_accept_ms = now;
  printk("Gravado: %s 0x%08X\n\n", ir_protocol_name(code->protocol),
         code->code);
}

static void handle_capture(int n_edges_copied) {
  int n = n_edges_copied - 1;
  struct ir_code code;
  struct ir_decode_info info;
  uint32_t now = k_uptime_get_32();

  /* Converte instantes em duracoes: pulso, espaco, pulso... */
  for (int i = 0; i < n; i++) {
    durations[i] = k_cyc_to_us_floor32(copy[i + 1] - copy[i]);
  }
  if (PRINT_RAW) {
    print_durations(durations, n);
  }

  enum ir_status st = ir_decode(durations, n, &code, &info);

  if (st == IR_OK) {
    printk("Quadro: %s 0x%08X, esticamento %d us (%d a %d), "
           "pulsos perdidos %d\n",
           ir_protocol_name(code.protocol), code.code, info.stretch_avg_us,
           info.stretch_min_us, info.stretch_max_us, info.lost_pulses);
    on_code(&code, info.lost_pulses, now);
  } else if (st == IR_REPEAT) {
    /* Botao NEC segurado: so prolonga a janela de repeticao. */
    if (recently_accepted(now)) {
      last_accept_ms = now;
    }
  } else {
    printk("Quadro rejeitado: %s\n", ir_status_name(st));
    schedule_warning(ir_status_name(st), now);
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
  gpio_init_callback(&ir_cb, on_edge, BIT(ir.pin));
  gpio_add_callback(ir.port, &ir_cb);

  print_receiver_config();
  printk("Aponte um controle e aperte um botao para gravar.\n");
  printk("Aperte BUTTON0 para reenviar o ultimo codigo gravado.\n\n");

  while (1) {
    /* Espera ate 10 ms pelo botao; se nao vier, segue pra captura. */
    if (k_sem_take(&tx_sem, K_MSEC(10)) == 0) {
      if (!has_saved) {
        printk("Nada gravado ainda\n");
        continue;
      }
      int n = ir_encode(&saved, tx_buf, IR_ENCODE_MAX);

      printk("Enviando %s 0x%08X...\n", ir_protocol_name(saved.protocol),
             saved.code);
      ir_send(tx_buf, n);
      printk("Enviado\n\n");
      continue;
    }

    if (warn_pending &&
        k_uptime_get_32() - warn_ms >= REPEAT_WINDOW_MS) {
      warn_pending = false;
      printk("Aviso: sinal nao reconhecido (%s); nada foi gravado.\n"
             "Tente de novo com o controle a uns 30 cm a 1 m da placa.\n\n",
             warn_reason);
    }

    int n = take_capture(copy);
    if (n >= 2) {
      handle_capture(n);
    }
  }
  return 0;
}
