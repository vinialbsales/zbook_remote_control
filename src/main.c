#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define MAX_EDGES 200

static const struct gpio_dt_spec ir =
	GPIO_DT_SPEC_GET(DT_NODELABEL(ir_receiver), gpios);
static struct gpio_callback ir_cb;

static volatile uint32_t edges[MAX_EDGES];
static volatile int n_edges;
static volatile uint32_t last_edge;
static uint32_t copy[MAX_EDGES];

/* Roda a cada borda (subida ou descida) no pino do receptor. */
static void on_edge(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	uint32_t now = k_cycle_get_32();

	if (n_edges < MAX_EDGES) {
		edges[n_edges++] = now;
	}
	last_edge = now;
}

int main(void)
{
	if (!gpio_is_ready_dt(&ir)) {
		printk("Receptor IR nao esta pronto\n");
		return 0;
	}
	if (gpio_pin_configure_dt(&ir, GPIO_INPUT) < 0 ||
	    gpio_pin_interrupt_configure_dt(&ir, GPIO_INT_EDGE_BOTH) < 0) {
		printk("Falha ao configurar o pino do receptor\n");
		return 0;
	}
	gpio_init_callback(&ir_cb, on_edge, BIT(ir.pin));
	gpio_add_callback(ir.port, &ir_cb);

	printk("Aponte um controle para a placa e aperte um botao\n");

	while (1) {
		k_msleep(10);

		/* Espera o sinal ficar 50 ms parado antes de imprimir. */
		if (n_edges == 0 ||
		    k_cyc_to_ms_floor32(k_cycle_get_32() - last_edge) < 50) {
			continue;
		}

		unsigned int key = irq_lock();
		int n = n_edges;
		for (int i = 0; i < n; i++) {
			copy[i] = edges[i];
		}
		n_edges = 0;
		irq_unlock(key);

		printk("%d bordas:\n", n);
		for (int i = 1; i < n; i++) {
			/* + = pulso (luz IR), - = espaco (sem luz) */
			printk("%c%u ", (i % 2) ? '+' : '-',
			       k_cyc_to_us_floor32(copy[i] - copy[i - 1]));
			if (i % 8 == 0) {
				printk("\n");
			}
		}
		printk("\n\n");
	}
	return 0;
}
