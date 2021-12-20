#include "spi.h"
#include "stm32_spi.h"
#include "gpio.h"
#include "stm32_gpio.h"
#include "irq.h"
//#include "stm32_irq.h"

#if 0
int32_t spi_init(struct spi_desc **desc,
		 const struct spi_init_param *param)
{
	return stm32_spi_init(desc, param);
}

int32_t spi_remove(struct spi_desc *desc)
{
	return stm32_spi_remove(desc);
}

int32_t spi_write_and_read(struct spi_desc *desc,
			   uint8_t *data,
			   uint16_t bytes_number)
{
	return stm32_spi_write_and_read(desc, data, bytes_number);
}

/* Obtain the GPIO decriptor. */
int32_t gpio_get(struct gpio_desc **desc,
		 const struct gpio_init_param *param)
{
	return stm32_gpio_get(desc, param);
}

/* Obtain optional GPIO descriptor. */
int32_t gpio_get_optional(struct gpio_desc **desc,
			  const struct gpio_init_param *param)
{
	return stm32_gpio_get_optional(desc, param);
}

/* Free the resources allocated by gpio_get() */
int32_t gpio_remove(struct gpio_desc *desc)
{
	return stm32_gpio_remove(desc);
}

/* Enable the input direction of the specified GPIO. */
int32_t gpio_direction_input(struct gpio_desc *desc)
{
	return stm32_gpio_direction_input(desc);
}

/* Enable the output direction of the specified GPIO. */
int32_t gpio_direction_output(struct gpio_desc *desc,
			      uint8_t value)
{
	return stm32_gpio_direction_output(desc, value);
}

/* Get the direction of the specified GPIO. */
int32_t gpio_get_direction(struct gpio_desc *desc,
			   uint8_t *direction)
{
	return stm32_gpio_get_direction(desc, direction);
}

/* Set the value of the specified GPIO. */
int32_t gpio_set_value(struct gpio_desc *desc,
		       uint8_t value)
{
	return stm32_gpio_set_value(desc, value);
}

/* Get the value of the specified GPIO. */
int32_t gpio_get_value(struct gpio_desc *desc,
		       uint8_t *value)
{
	return stm32_gpio_get_value(desc, value);
}

/* Initialize a interrupt controller peripheral. */
int32_t irq_ctrl_init(struct irq_ctrl_desc **desc,
		      const struct irq_init_param *param)
{
	return stm32_irq_ctrl_init(desc, param);
}

int32_t irq_register_callback(struct irq_ctrl_desc *desc, uint32_t irq_id,
			      struct callback_desc *callback_desc)
{
	stm32_irq_register_callback(desc, irq_id, callback_desc);
}

/* Global interrupt enable */
int32_t irq_global_enable(struct irq_ctrl_desc *desc)
{
	stm32_irq_global_enable(desc);
}

/* Global interrupt disable */
int32_t irq_global_disable(struct irq_ctrl_desc *desc)
{
	stm32_irq_global_disable(desc);
}

/* Enable specific interrupt */
int32_t irq_enable(struct irq_ctrl_desc *desc, uint32_t irq_id)
{
	stm32_irq_enable(desc, irq_id);
}

/* Disable specific interrupt */
int32_t irq_disable(struct irq_ctrl_desc *desc, uint32_t irq_id)
{
	stm32_irq_disable(desc, irq_id);
}
#endif

uint32_t get_timer_clock(void)
{
	return 216000000;
}
