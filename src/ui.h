/* Interface no display OLED: lista de slots, modo gravar e avisos.
 *
 * Toda funcao daqui deve ser chamada so pela thread do main. */
#ifndef UI_H
#define UI_H

#include <zephyr/kernel.h>

#include "ir_decode.h"

#define UI_NUM_SLOTS 8

/* Evento de entrada ja traduzido pelo callback do subsistema de entrada. */
enum ui_event_type {
  UI_EV_ROTATE,   /* encoder girado: value = +1 ou -1 por clique do eixo */
  UI_EV_ENC_DOWN, /* botao do encoder apertado */
  UI_EV_ENC_UP,   /* botao do encoder solto */
  UI_EV_BUTTON,   /* BUTTON0..3 apertado: value = 0..3 */
};

struct ui_event {
  enum ui_event_type type;
  int32_t value;
  uint32_t time_ms; /* k_uptime_get_32() no momento do evento */
};

/* Fila onde o callback de entrada coloca os eventos; o main tira. */
extern struct k_msgq ui_msgq;

/* Inicializa o display. Devolve 0 ou um erro negativo; sem display a
 * interface segue funcionando, so nao desenha. */
int ui_init(void);

/* Trata um evento tirado da ui_msgq. Devolve o slot que deve ser
 * enviado agora (sempre um slot gravado), ou -1. */
int ui_handle_event(const struct ui_event *evt);

/* Chamada a cada volta do loop: detecta o clique longo enquanto o botao
 * ainda esta apertado e apaga avisos vencidos. */
void ui_tick(void);

/* true enquanto um slot espera um codigo (modo gravar). */
bool ui_is_recording(void);

/* Salva o codigo no slot do modo gravar e volta para a lista. */
void ui_record_code(const struct ir_code *code);

/* Mostra "Nao reconhecido" por 2 s; o modo gravar continua. */
void ui_record_failed(void);

/* Codigo gravado no slot, ou NULL se o slot esta vazio. */
const struct ir_code *ui_slot_code(int slot);

/* Mostra "Enviando..." para o slot dado; -1 volta a tela anterior. */
void ui_set_sending(int slot);

/* Redesenha a tela se algo mudou desde o ultimo desenho. */
void ui_draw(void);

#endif /* UI_H */
