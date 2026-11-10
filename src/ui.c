#include <string.h>

#include <zephyr/device.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/display.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>

#include "ui.h"

/* Fonte 10x16: no display de 128x64 cabem 4 linhas de 12 caracteres. */
#define FONT_W 10
#define FONT_H 16
#define SCREEN_W 128

/* Linhas de slots visiveis abaixo do cabecalho. */
#define VISIBLE_ROWS 3

/* Segurar o botao do encoder por esse tempo entra no modo gravar (na
 * lista) ou apaga a gravacao do slot (no modo gravar). */
#define LONG_PRESS_MS 800

#define FAIL_MSG_MS 2000
#define EMPTY_MSG_MS 1000
#define DONE_MSG_MS 1500

/* Selecao no cabecalho (nome do aparelho): clique troca de aparelho. */
#define SEL_HEADER (-1)

static const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
static bool display_ok;

/* Aparelho atual e as gravacoes de todos (count == 0: sem gravacao). A
 * gravacao tem prioridade sobre o codigo de fabrica. */
static int dev;
static struct ir_seq recorded[IR_LIB_NUM_DEVICES][IR_LIB_MAX_SLOTS];

/* Gravacoes cruas: poucas e grandes, entao ficam num conjunto separado. */
struct raw_slot {
  bool used;
  int8_t dev;
  int8_t slot;
  struct ir_raw raw;
};
static struct raw_slot raw_pool[UI_RAW_SLOTS];

static struct raw_slot *find_raw(int d, int slot) {
  for (int i = 0; i < UI_RAW_SLOTS; i++) {
    if (raw_pool[i].used && raw_pool[i].dev == d && raw_pool[i].slot == slot) {
      return &raw_pool[i];
    }
  }
  return NULL;
}

static void free_raw(int d, int slot) {
  struct raw_slot *r = find_raw(d, slot);

  if (r != NULL) {
    r->used = false;
  }
}

/* Guarda a gravacao crua do slot (substitui a anterior). */
static bool put_raw(int d, int slot, const struct ir_raw *raw) {
  struct raw_slot *r = find_raw(d, slot);

  for (int i = 0; r == NULL && i < UI_RAW_SLOTS; i++) {
    if (!raw_pool[i].used) {
      r = &raw_pool[i];
    }
  }
  if (r == NULL) {
    return false;
  }
  r->used = true;
  r->dev = (int8_t)d;
  r->slot = (int8_t)slot;
  r->raw.n = raw->n;
  memcpy(r->raw.d, raw->d, sizeof(raw->d[0]) * raw->n);
  return true;
}

/* Slot selecionado (ou SEL_HEADER) e o primeiro slot visivel. */
static int sel;
static int top;

/* Slot no modo gravar, ou -1 fora dele. */
static int rec_slot = -1;

/* Slot sendo enviado, ou -1. */
static int send_slot = -1;

/* Botao do encoder. "consumed" marca que este aperto ja teve efeito
 * (clique longo), entao soltar o botao nao faz nada. */
static bool enc_down;
static bool enc_consumed;
static uint32_t enc_down_ms;

/* Gravacao apagada esperando o main tirar da flash. */
static bool erased;
static int erased_dev;
static int erased_slot;

/* Aviso temporario desenhado no lugar da tela atual (line2 pode ser
 * NULL). */
static bool msg_on;
static const char *msg_line1;
static const char *msg_line2;
static uint32_t msg_until_ms;

/* Texto montado na hora para a mensagem ("Gravado (2)"). */
static char msg_buf[16];

/* true quando o estado mudou e a tela precisa ser redesenhada. */
static bool dirty = true;

static const struct ir_lib_device *cur(void) { return &ir_lib_devices[dev]; }

/* Ate 16 eventos esperando o main; se encher, os novos sao descartados. */
K_MSGQ_DEFINE(ui_msgq, sizeof(struct ui_event), 16, 4);

/* Chamado pela thread do subsistema de entrada. So traduz o evento e
 * coloca na fila: quem decide o que fazer (e desenha) e o main. */
static void on_input(struct input_event *evt, void *user_data) {
  struct ui_event ev = {.value = evt->value, .time_ms = k_uptime_get_32()};

  if (evt->type == INPUT_EV_REL && evt->code == INPUT_REL_WHEEL) {
    ev.type = UI_EV_ROTATE;
  } else if (evt->type != INPUT_EV_KEY) {
    return;
  } else if (evt->code == INPUT_KEY_ENTER) {
    /* value: 1 = apertado, 0 = solto. */
    ev.type = evt->value ? UI_EV_ENC_DOWN : UI_EV_ENC_UP;
  } else if (evt->code >= INPUT_BTN_0 && evt->code <= INPUT_BTN_3 &&
             evt->value == 1) {
    ev.type = UI_EV_BUTTON;
    ev.value = evt->code - INPUT_BTN_0;
  } else {
    return;
  }
  k_msgq_put(&ui_msgq, &ev, K_NO_WAIT);
}
INPUT_CALLBACK_DEFINE(NULL, on_input, NULL);

/* Escolhe a fonte de FONT_W x FONT_H entre as fontes da CFB. */
static int set_font(void) {
  int n = cfb_get_numof_fonts(display);

  for (int i = 0; i < n; i++) {
    uint8_t w, h;

    if (cfb_get_font_size(display, i, &w, &h) == 0 && w == FONT_W &&
        h == FONT_H) {
      return cfb_framebuffer_set_font(display, i);
    }
  }
  return -ENOENT;
}

int ui_init(void) {
  if (!device_is_ready(display)) {
    return -ENODEV;
  }
  /* Mesmo formato que o exemplo samples/subsys/display/cfb tenta. */
  if (display_set_pixel_format(display, PIXEL_FORMAT_MONO10) != 0 &&
      display_set_pixel_format(display, PIXEL_FORMAT_MONO01) != 0) {
    return -ENOTSUP;
  }
  int err = cfb_framebuffer_init(display);

  if (err == 0) {
    err = set_font();
  }
  if (err == 0) {
    err = cfb_framebuffer_clear(display, true);
  }
  if (err == 0) {
    err = display_blanking_off(display);
    if (err == -ENOSYS) {
      err = 0;
    }
  }
  display_ok = (err == 0);
  dirty = true;
  return err;
}

/* Texto alinhado a direita, com 2 pixels de margem. */
static void draw_right(const char *text, int y) {
  cfb_draw_text(display, text, SCREEN_W - 2 - FONT_W * (int)strlen(text), y);
}

static const char *source_tag(enum ui_source src) {
  switch (src) {
  case UI_SRC_RECORDED:
    return "gr";
  case UI_SRC_FACTORY:
    return "fb";
  default:
    return "--";
  }
}

static void draw_list(void) {
  cfb_draw_text(display, cur()->name, 2, 0);
  if (sel == SEL_HEADER) {
    cfb_invert_area(display, 0, 0, SCREEN_W, FONT_H);
  }

  for (int row = 0; row < VISIBLE_ROWS && top + row < cur()->n_slots; row++) {
    int i = top + row;
    int y = FONT_H * (row + 1);

    cfb_draw_text(display, cur()->slots[i].name, 2, y);
    draw_right(source_tag(ui_slot_source(i)), y);
    if (i == sel) {
      cfb_invert_area(display, 0, y, SCREEN_W, FONT_H);
    }
  }
}

static void show_message(const char *line1, const char *line2,
                         uint32_t duration_ms) {
  msg_on = true;
  msg_line1 = line1;
  msg_line2 = line2;
  msg_until_ms = k_uptime_get_32() + duration_ms;
  dirty = true;
}

static void hide_message(void) {
  if (msg_on) {
    msg_on = false;
    dirty = true;
  }
}

/* Move a selecao (o cabecalho fica acima do primeiro slot) e rola a
 * lista para a selecao ficar visivel. */
static void move_selection(int delta) {
  int new_sel = CLAMP(sel + delta, SEL_HEADER, cur()->n_slots - 1);

  if (new_sel == sel) {
    return;
  }
  hide_message();
  sel = new_sel;
  if (sel >= 0 && sel < top) {
    top = sel;
  } else if (sel >= top + VISIBLE_ROWS) {
    top = sel - VISIBLE_ROWS + 1;
  } else if (sel == SEL_HEADER) {
    top = 0;
  }
  dirty = true;
}

void ui_set_device(int d) {
  if (d < 0 || d >= IR_LIB_NUM_DEVICES) {
    return;
  }
  dev = d;
  top = 0;
  if (sel >= cur()->n_slots) {
    sel = cur()->n_slots - 1;
  }
  dirty = true;
}

static void next_device(void) {
  ui_set_device((dev + 1) % IR_LIB_NUM_DEVICES);
  sel = SEL_HEADER;
  printk("Aparelho: %s\n", cur()->name);
}

static void start_recording(void) {
  rec_slot = sel;
  enc_consumed = true;
  hide_message();
  dirty = true;
  printk("Gravar \"%s / %s\": aponte o controle (clique cancela; segurar "
         "0,8 s apaga a gravacao)\n",
         cur()->name, cur()->slots[rec_slot].name);
}

static void stop_recording(void) {
  rec_slot = -1;
  hide_message();
  dirty = true;
}

/* Segurar no modo gravar: apaga a gravacao do slot. */
static void erase_recording(void) {
  int slot = rec_slot;

  enc_consumed = true;
  stop_recording();
  if (recorded[dev][slot].count == 0 && find_raw(dev, slot) == NULL) {
    show_message("Nada para", "apagar", EMPTY_MSG_MS);
    printk("\"%s\" nao tem gravacao\n\n", cur()->slots[slot].name);
    return;
  }
  recorded[dev][slot].count = 0;
  free_raw(dev, slot);
  erased = true;
  erased_dev = dev;
  erased_slot = slot;
  if (cur()->slots[slot].seq.count > 0 || cur()->slots[slot].raw != NULL) {
    show_message("Volta ao", "de fabrica", DONE_MSG_MS);
  } else {
    show_message("Gravacao", "apagada", DONE_MSG_MS);
  }
  printk("Gravacao de \"%s / %s\" apagada\n", cur()->name,
         cur()->slots[slot].name);
}

/* Pede o envio de um slot: so vale se ele tiver codigo. */
static int request_send(int slot) {
  if (slot < 0 || slot >= cur()->n_slots) {
    return -1;
  }
  if (ui_slot_source(slot) == UI_SRC_NONE) {
    printk("Slot \"%s\" vazio\n", cur()->slots[slot].name);
    show_message(cur()->slots[slot].name, "vazio", EMPTY_MSG_MS);
    return -1;
  }
  return slot;
}

int ui_handle_event(const struct ui_event *evt) {
  switch (evt->type) {
  case UI_EV_ROTATE:
    if (rec_slot < 0) {
      move_selection(evt->value);
    }
    break;

  case UI_EV_ENC_DOWN:
    enc_down = true;
    enc_down_ms = evt->time_ms;
    enc_consumed = false;
    break;

  case UI_EV_ENC_UP: {
    bool was_down = enc_down;
    bool long_press = evt->time_ms - enc_down_ms >= LONG_PRESS_MS;

    enc_down = false;
    if (!was_down || enc_consumed) {
      break;
    }
    /* O main pode ter atrasado o ui_tick (desenho, envio); o tempo vem
     * do proprio evento, entao a decisao curto/longo continua certa. */
    if (rec_slot >= 0) {
      if (long_press) {
        erase_recording();
      } else {
        printk("Gravacao cancelada\n\n");
        stop_recording();
      }
      break;
    }
    if (sel == SEL_HEADER) {
      next_device();
      break;
    }
    if (long_press) {
      start_recording();
      break;
    }
    return request_send(sel);
  }

  case UI_EV_BUTTON:
    if (rec_slot < 0) {
      return request_send(evt->value);
    }
    break;
  }
  return -1;
}

void ui_tick(void) {
  uint32_t now = k_uptime_get_32();

  /* O clique longo vale sem esperar soltar: a tela muda com o dedo ainda
   * no botao. */
  if (enc_down && !enc_consumed && now - enc_down_ms >= LONG_PRESS_MS) {
    if (rec_slot >= 0) {
      erase_recording();
    } else if (sel >= 0) {
      start_recording();
    }
  }
  /* Comparacao com sinal: funciona mesmo quando o uptime de 32 bits
   * da a volta (a cada ~49 dias). */
  if (msg_on && (int32_t)(now - msg_until_ms) >= 0) {
    hide_message();
  }
}

bool ui_is_recording(void) { return rec_slot >= 0; }

int ui_record_sequence(const struct ir_seq *seq) {
  int slot = rec_slot;

  if (slot < 0 || seq->count < 1 || seq->count > IR_SEQ_MAX) {
    return -1;
  }
  recorded[dev][slot] = *seq;
  free_raw(dev, slot);
  stop_recording();
  snprintk(msg_buf, sizeof(msg_buf), "Gravado (%d)", seq->count);
  show_message(msg_buf, NULL, DONE_MSG_MS);
  return slot;
}

int ui_record_raw(const struct ir_raw *raw) {
  int slot = rec_slot;

  if (slot < 0 || !ir_raw_is_valid(raw)) {
    return -1;
  }
  if (!put_raw(dev, slot, raw)) {
    stop_recording();
    show_message("Memoria", "cheia", FAIL_MSG_MS);
    return -1;
  }
  recorded[dev][slot].count = 0;
  stop_recording();
  show_message("Gravado cru", NULL, DONE_MSG_MS);
  return slot;
}

void ui_record_failed(void) {
  if (rec_slot >= 0) {
    show_message("Nao", "reconhecido", FAIL_MSG_MS);
  }
}

bool ui_take_erased(int *d, int *slot) {
  if (!erased) {
    return false;
  }
  erased = false;
  *d = erased_dev;
  *slot = erased_slot;
  return true;
}

int ui_device(void) { return dev; }

const char *ui_device_name(int d) { return ir_lib_devices[d].name; }

const char *ui_device_slot_name(int d, int slot) {
  return ir_lib_devices[d].slots[slot].name;
}

void ui_restore_slot(int d, int slot, const struct ir_seq *seq) {
  if (d < 0 || d >= IR_LIB_NUM_DEVICES || slot < 0 ||
      slot >= ir_lib_devices[d].n_slots) {
    return;
  }
  recorded[d][slot] = *seq;
  free_raw(d, slot);
  dirty = true;
}

void ui_restore_raw(int d, int slot, const struct ir_raw *raw) {
  if (d < 0 || d >= IR_LIB_NUM_DEVICES || slot < 0 ||
      slot >= ir_lib_devices[d].n_slots) {
    return;
  }
  if (put_raw(d, slot, raw)) {
    recorded[d][slot].count = 0;
    dirty = true;
  }
}

const char *ui_slot_name(int slot) { return cur()->slots[slot].name; }

enum ui_source ui_slot_source(int slot) {
  if (recorded[dev][slot].count > 0 || find_raw(dev, slot) != NULL) {
    return UI_SRC_RECORDED;
  }
  if (cur()->slots[slot].seq.count > 0 || cur()->slots[slot].raw != NULL) {
    return UI_SRC_FACTORY;
  }
  return UI_SRC_NONE;
}

int ui_slot_raw(int slot, const uint16_t **d) {
  struct raw_slot *r = find_raw(dev, slot);

  /* Gravacao (crua ou nao) vale mais que o codigo de fabrica. */
  if (r != NULL) {
    *d = r->raw.d;
    return r->raw.n;
  }
  if (recorded[dev][slot].count == 0 && cur()->slots[slot].raw != NULL) {
    *d = cur()->slots[slot].raw;
    return cur()->slots[slot].raw_n;
  }
  return 0;
}

const struct ir_seq *ui_slot_sequence(int slot) {
  switch (ui_slot_source(slot)) {
  case UI_SRC_RECORDED:
    return recorded[dev][slot].count > 0 ? &recorded[dev][slot] : NULL;
  case UI_SRC_FACTORY:
    return cur()->slots[slot].seq.count > 0 ? &cur()->slots[slot].seq : NULL;
  default:
    return NULL;
  }
}

void ui_set_sending(int slot) {
  send_slot = slot;
  dirty = true;
}

static void draw_centered(const char *text, int y) {
  cfb_draw_text(display, text, (SCREEN_W - FONT_W * (int)strlen(text)) / 2, y);
}

/* Uma linha fica no meio da tela; duas ficam nas linhas 1 e 2. */
static void draw_message(void) {
  if (msg_line2 == NULL) {
    draw_centered(msg_line1, (64 - FONT_H) / 2);
  } else {
    draw_centered(msg_line1, FONT_H);
    draw_centered(msg_line2, 2 * FONT_H);
  }
}

static void draw_recording(void) {
  /* Cabecalho invertido deixa claro que o modo mudou. */
  cfb_draw_text(display, "Gravando", 2, 0);
  cfb_invert_area(display, 0, 0, SCREEN_W, FONT_H);
  cfb_draw_text(display, cur()->slots[rec_slot].name, 2, FONT_H);
  cfb_draw_text(display, "Aponte o", 2, 2 * FONT_H);
  cfb_draw_text(display, "controle", 2, 3 * FONT_H);
}

void ui_draw(void) {
  if (!dirty || !display_ok) {
    return;
  }
  dirty = false;

  /* Desenha no buffer em RAM e depois envia tudo de uma vez pelo I2C. */
  cfb_framebuffer_clear(display, false);
  if (send_slot >= 0) {
    draw_centered("Enviando...", FONT_H);
    draw_centered(cur()->slots[send_slot].name, 2 * FONT_H);
  } else if (msg_on) {
    draw_message();
  } else if (rec_slot >= 0) {
    draw_recording();
  } else {
    draw_list();
  }
  cfb_framebuffer_finalize(display);
}
