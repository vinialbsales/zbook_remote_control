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

/* Segurar o botao do encoder por esse tempo entra no modo gravar. */
#define LONG_PRESS_MS 800

#define FAIL_MSG_MS 2000
#define EMPTY_MSG_MS 1000

struct slot {
  const char *name;
  struct ir_code code;
  bool has_code;
};

static struct slot slots[UI_NUM_SLOTS] = {
    {.name = "Ligar"},   {.name = "Vol +"}, {.name = "Vol -"},
    {.name = "Canal +"}, {.name = "Canal -"}, {.name = "Mudo"},
    {.name = "Entrada"}, {.name = "Menu"},
};

static const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
static bool display_ok;

/* Slot selecionado e o primeiro slot visivel na tela. */
static int sel;
static int top;

/* Slot no modo gravar, ou -1 fora dele. */
static int rec_slot = -1;

/* Slot sendo enviado, ou -1. */
static int send_slot = -1;

/* Botao do encoder. "consumed" marca que este aperto ja teve efeito
 * (clique longo ou cancelamento), entao soltar o botao nao faz nada. */
static bool enc_down;
static bool enc_consumed;
static uint32_t enc_down_ms;

/* Aviso temporario desenhado no lugar da tela atual (line2 pode ser
 * NULL). */
static bool msg_on;
static const char *msg_line1;
static const char *msg_line2;
static uint32_t msg_until_ms;

/* true quando o estado mudou e a tela precisa ser redesenhada. */
static bool dirty = true;

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

static void draw_list(void) {
  char pos[24]; /* cabe "%d/%d" com dois int quaisquer */

  cfb_draw_text(display, "Controle", 2, 0);
  snprintk(pos, sizeof(pos), "%d/%d", sel + 1, UI_NUM_SLOTS);
  draw_right(pos, 0);

  for (int row = 0; row < VISIBLE_ROWS; row++) {
    int i = top + row;
    int y = FONT_H * (row + 1);

    cfb_draw_text(display, slots[i].name, 2, y);
    draw_right(slots[i].has_code ? "ok" : "--", y);
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

/* Move a selecao e rola a lista para a selecao ficar visivel. */
static void move_selection(int delta) {
  int new_sel = CLAMP(sel + delta, 0, UI_NUM_SLOTS - 1);

  if (new_sel == sel) {
    return;
  }
  hide_message();
  sel = new_sel;
  if (sel < top) {
    top = sel;
  } else if (sel >= top + VISIBLE_ROWS) {
    top = sel - VISIBLE_ROWS + 1;
  }
  dirty = true;
}

static void start_recording(void) {
  rec_slot = sel;
  enc_consumed = true;
  hide_message();
  dirty = true;
  printk("Gravar \"%s\": aponte o controle (clique no encoder cancela)\n",
         slots[rec_slot].name);
}

static void stop_recording(void) {
  rec_slot = -1;
  hide_message();
  dirty = true;
}

/* Pede o envio de um slot: so vale se ele estiver gravado. */
static int request_send(int slot) {
  if (!slots[slot].has_code) {
    printk("Slot \"%s\" vazio\n", slots[slot].name);
    show_message(slots[slot].name, "vazio", EMPTY_MSG_MS);
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
    /* No modo gravar, o clique cancela ja ao apertar. */
    if (rec_slot >= 0) {
      printk("Gravacao cancelada\n\n");
      stop_recording();
      enc_consumed = true;
    }
    break;

  case UI_EV_ENC_UP:
    if (!enc_down || enc_consumed) {
      enc_down = false;
      break;
    }
    enc_down = false;
    /* O main pode ter atrasado o ui_tick (desenho, envio); o tempo vem
     * do proprio evento, entao a decisao curto/longo continua certa. */
    if (evt->time_ms - enc_down_ms >= LONG_PRESS_MS) {
      start_recording();
      break;
    }
    return request_send(sel);

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

  if (enc_down && !enc_consumed && rec_slot < 0 &&
      now - enc_down_ms >= LONG_PRESS_MS) {
    start_recording();
  }
  /* Comparacao com sinal: funciona mesmo quando o uptime de 32 bits
   * da a volta (a cada ~49 dias). */
  if (msg_on && (int32_t)(now - msg_until_ms) >= 0) {
    hide_message();
  }
}

bool ui_is_recording(void) { return rec_slot >= 0; }

void ui_record_code(const struct ir_code *code) {
  if (rec_slot < 0) {
    return;
  }
  slots[rec_slot].code = *code;
  slots[rec_slot].has_code = true;
  printk("Slot \"%s\" gravado\n\n", slots[rec_slot].name);
  stop_recording();
}

void ui_record_failed(void) {
  if (rec_slot >= 0) {
    show_message("Nao", "reconhecido", FAIL_MSG_MS);
  }
}

const struct ir_code *ui_slot_code(int slot) {
  return slots[slot].has_code ? &slots[slot].code : NULL;
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
  cfb_draw_text(display, slots[rec_slot].name, 2, FONT_H);
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
    draw_centered(slots[send_slot].name, 2 * FONT_H);
  } else if (msg_on) {
    draw_message();
  } else if (rec_slot >= 0) {
    draw_recording();
  } else {
    draw_list();
  }
  cfb_framebuffer_finalize(display);
}
