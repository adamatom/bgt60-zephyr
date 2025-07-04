#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>

#include <zephyr/drivers/sensor/bgt60.h>

const struct device *const dev = DEVICE_DT_GET_ANY(infineon_bgt60);

SENSOR_DT_READ_IODEV(iodev, DT_COMPAT_GET_ANY_STATUS_OKAY(infineon_bgt60),
		{SENSOR_CHAN_RADAR_CHIRP, 0});

RTIO_DEFINE(ctx, 1, 1);

#define BLOCK_BYTES		3 // 3 bytes per fifo block
#define BUF_SIZE		BLOCK_BYTES * CONFIG_BGT60_RADAR_SENSOR_DEFAULT_FIFO_BLOCKS

uint8_t buf[BUF_SIZE];

int main(void)
{
	while (1) {
		printk("\nrequesting radar data\n");
		const int rc = sensor_read(&iodev, &ctx, buf, sizeof(buf));
		if (rc != 0) {
			printk("sensor_read() failed %d\n", rc);
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* due to limited ram on the hifive1, print the bytes received for offline processing */
		for (size_t i = 0; i < BUF_SIZE; i++) {
			printk("%02X ", buf[i]);
		}
	}

	return 0;
}
