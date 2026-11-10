/* Gravacoes na flash. Ver slot_store.h. */
#include "slot_store.h"

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>

#define STORE_PARTITION DT_FIXED_PARTITION_ID(DT_CHOSEN(zephyr_settings_partition))

#define STORE_ROOT "remote"

static const struct slot_store_cb *cbs;
static int loaded;

/* Buffers grandes fora da pilha (so a thread do main usa). */
static uint8_t store_buf[IR_RAW_STORE_MAX];
static struct ir_raw raw_tmp;

/* Le "<num>" no inicio de s; devolve o numero e avanca s, ou -1. */
static int parse_num(const char **s) {
  int v = 0, digits = 0;

  while (**s >= '0' && **s <= '9' && digits < 3) {
    v = v * 10 + (**s - '0');
    (*s)++;
    digits++;
  }
  return digits > 0 ? v : -1;
}

/* Nome relativo -> aparelho e slot. "d1s3" ou o antigo "s3". */
static bool parse_slot_key(const char *name, int *dev, int *slot) {
  const char *p = name;

  if (*p == 'd') {
    p++;
    *dev = parse_num(&p);
    if (*dev < 0 || *p != 's') {
      return false;
    }
  } else if (*p == 's') {
    *dev = cbs->legacy_device;
  } else {
    return false;
  }
  p++;
  *slot = parse_num(&p);
  return *slot >= 0 && *p == '\0' && *dev < cbs->num_devices &&
         *slot < cbs->max_slots;
}

static int on_set(const char *name, size_t len, settings_read_cb read_cb,
                  void *cb_arg) {
  uint8_t *buf = store_buf;
  const char *next;
  int dev, slot;

  if (settings_name_steq(name, "dev", &next) && next == NULL) {
    uint8_t d;

    if (len == 1 && read_cb(cb_arg, &d, 1) == 1 && d < cbs->num_devices &&
        cbs->on_device != NULL) {
      cbs->on_device(d);
    }
    return 0;
  }
  if (!parse_slot_key(name, &dev, &slot)) {
    return 0; /* chave desconhecida: ignora */
  }
  if (len > sizeof(store_buf)) {
    printk("Flash: %s com tamanho invalido (%u), ignorado\n", name,
           (unsigned int)len);
    return 0;
  }
  ssize_t n = read_cb(cb_arg, buf, len);
  struct ir_seq seq;

  if (n > 0 && ir_store_version(buf, (int)n) == IR_RAW_STORE_VERSION) {
    if (ir_raw_unpack(buf, (int)n, &raw_tmp) != 0) {
      printk("Flash: %s invalido, ignorado\n", name);
      return 0;
    }
    if (cbs->on_raw != NULL) {
      cbs->on_raw(dev, slot, &raw_tmp);
    }
    loaded++;
    return 0;
  }
  if (n < 0 || ir_seq_unpack(buf, (int)n, &seq) != 0) {
    printk("Flash: %s invalido, ignorado\n", name);
    return 0;
  }
  if (cbs->on_slot != NULL) {
    cbs->on_slot(dev, slot, &seq);
  }
  loaded++;
  return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(remote, STORE_ROOT, NULL, on_set, NULL, NULL);

/* Apaga a particao inteira. So quando o NVS nao consegue montar: os
 * dados ja estao ilegiveis (por exemplo, restos de outro firmware nos
 * setores que o NVS passou a usar). */
static int format_partition(void) {
  const struct flash_area *fa;
  int err = flash_area_open(STORE_PARTITION, &fa);

  if (err != 0) {
    return err;
  }
  err = flash_area_erase(fa, 0, fa->fa_size);
  flash_area_close(fa);
  return err;
}

int slot_store_init(const struct slot_store_cb *cb) {
  int err = settings_subsys_init();

  if (err != 0) {
    printk("Flash: area de gravacoes ilegivel (%d); formatando...\n", err);
    err = format_partition();
    if (err == 0) {
      err = settings_subsys_init();
    }
    if (err != 0) {
      return err;
    }
    printk("Flash: area formatada (gravacoes anteriores perdidas)\n");
  }
  cbs = cb;
  loaded = 0;
  err = settings_load_subtree(STORE_ROOT);
  return err != 0 ? err : loaded;
}

/* A chave antiga do mesmo slot sai, para nao competir com a nova na
 * proxima partida. */
static void drop_legacy(int dev, int slot) {
  char key[24];

  if (cbs != NULL && dev == cbs->legacy_device) {
    snprintk(key, sizeof(key), STORE_ROOT "/s%d", slot);
    settings_delete(key);
  }
}

int slot_store_save(int dev, int slot, const struct ir_seq *seq) {
  uint8_t buf[IR_SEQ_STORE_MAX];
  char key[24];
  int len = ir_seq_pack(seq, buf, sizeof(buf));

  if (len == 0) {
    return -EINVAL;
  }
  drop_legacy(dev, slot);
  snprintk(key, sizeof(key), STORE_ROOT "/d%ds%d", dev, slot);
  return settings_save_one(key, buf, len);
}

int slot_store_save_raw(int dev, int slot, const struct ir_raw *raw) {
  char key[24];
  int len = ir_raw_pack(raw, store_buf, sizeof(store_buf));

  if (len == 0) {
    return -EINVAL;
  }
  drop_legacy(dev, slot);
  snprintk(key, sizeof(key), STORE_ROOT "/d%ds%d", dev, slot);
  return settings_save_one(key, store_buf, len);
}

int slot_store_erase(int dev, int slot) {
  char key[24];

  drop_legacy(dev, slot);
  snprintk(key, sizeof(key), STORE_ROOT "/d%ds%d", dev, slot);
  return settings_delete(key);
}

int slot_store_save_device(int dev) {
  uint8_t d = (uint8_t)dev;

  return settings_save_one(STORE_ROOT "/dev", &d, 1);
}
