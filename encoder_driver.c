#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>

/**
 * Borrows code from
 * https://github.com/Johannes4Linux/Linux_Driver_Tutorial/blob/main/11_gpio_irq/gpio_irq.c
 */

// Interrupt request number.
static int irq_number;

// GPIO descriptors for LED and button.
static struct gpio_desc *button_desc;

// Current time
ktime_t currTime;
// Previous time
ktime_t prevTime;
//Difference in time
int diff;

/**
 * Interrupt service routine triggered on button press.
 */
static irq_handler_t gpiod_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {
    //Count how long its been since last ESC has been triggered
    currTime = ktime_get();
    int temp = ktime_to_ns(currTime - prevTime) / 1000000;
    if (temp > 1)
        diff = temp;
    
    printk("Received difference of: %d\n", diff);
    
    prevTime = currTime;
    
    // finished handling
    return (irq_handler_t) IRQ_HANDLED; 
}

/**
 * Inserts the kernel module.
 */
static int led_probe(struct platform_device *pdev) {
    printk("Loading module...");

    // Get GPIO descriptor for the button (encoder).
    button_desc = devm_gpiod_get(&pdev->dev, "button", GPIOD_IN);

    // Setup the IRQ handler.
    irq_number = gpiod_to_irq(button_desc);
    if (request_irq(irq_number, (irq_handler_t) gpiod_irq_handler, IRQF_TRIGGER_FALLING, "my_gpio_irq", NULL) != 0) {
            printk("Error! Cannot get IRQ no. %d.\n", irq_number);
            return -1;
    }

    // Set the debounce for the encoder.
    gpiod_set_debounce(button_desc, 1000000);

    // Output that we are done.
    printk("Done!\n");
    printk("GPIO 6 is mapped to IRQ no. %d.\n", irq_number);

    return 0;
}

/**
 * Removes the kernel module.
 */
static int led_remove(struct platform_device *pdev) {
    printk("Unloading module... ");

    // Free the IRQ number to be used by another program.
    free_irq(irq_number, NULL);

    printk("Done!\n");

    return 0;
}

// Of match table.
static struct of_device_id of_match_table[] = {
    { .compatible = "comp424p2", },
    {/* leave alone - keep this here (end node) */},
};

// Platform driver object.
static struct platform_driver gpio_driver = {
    .probe = led_probe,
    .remove = led_remove,
    .driver =
        {
            .name = "The Rock: this name doesn't even matter",
            .owner = THIS_MODULE,
            .of_match_table = of_match_table,
        },
};

// Specify the driver to be inserted.
module_platform_driver(gpio_driver);

MODULE_DESCRIPTION("424\'s finest");
MODULE_AUTHOR("GOAT");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adam_driver");
