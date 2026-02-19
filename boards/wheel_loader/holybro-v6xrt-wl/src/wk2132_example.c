/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * WK2132 I2C-to-UART Serial Driver Usage Example
 *
 ****************************************************************************/

/**
 * @file wk2132_example.c
 * @author PX4 Development Team
 * @brief Example usage of WK2132 I2C-to-UART bridge
 */

#include <nuttx/config.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <syslog.h>

#ifdef CONFIG_WK2132_SERIAL

/**
 * @brief Example function showing how to use WK2132 serial ports
 */
void wk2132_usage_example(void)
{
	int fd;
	struct termios config;
	const char *test_message = "Hello from WK2132!\n";
	char read_buffer[128];
	ssize_t bytes_read;

	/* Open WK2132 port 0 (assuming it's registered as /dev/ttyS6) */
	fd = open("/dev/ttyS6", O_RDWR | O_NOCTTY);

	if (fd < 0) {
		syslog(LOG_ERR, "WK2132: Failed to open /dev/ttyS6: %s", strerror(errno));
		return;
	}

	/* Get current configuration */
	if (tcgetattr(fd, &config) != 0) {
		syslog(LOG_ERR, "WK2132: Failed to get terminal attributes: %s", strerror(errno));
		close(fd);
		return;
	}

	/* Configure for 9600 baud, 8N1 */
	cfsetispeed(&config, B9600);
	cfsetospeed(&config, B9600);

	config.c_cflag &= ~CSIZE;    /* Clear size bits */
	config.c_cflag |= CS8;       /* 8 data bits */
	config.c_cflag &= ~PARENB;   /* No parity */
	config.c_cflag &= ~CSTOPB;   /* 1 stop bit */
	config.c_cflag &= ~CRTSCTS;  /* No hardware flow control */

	/* Set local mode */
	config.c_cflag |= CREAD | CLOCAL; /* Enable receiver, ignore modem control lines */

	/* Set input mode (non-canonical, no echo) */
	config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	/* Set output mode */
	config.c_oflag &= ~OPOST;

	/* Set input mode */
	config.c_iflag &= ~(IXON | IXOFF | IXANY); /* Turn off software flow control */
	config.c_iflag &= ~(INLCR | IGNCR | ICRNL); /* Turn off character processing */

	/* Apply configuration */
	if (tcsetattr(fd, TCSANOW, &config) != 0) {
		syslog(LOG_ERR, "WK2132: Failed to set terminal attributes: %s", strerror(errno));
		close(fd);
		return;
	}

	/* Send test message */
	ssize_t bytes_written = write(fd, test_message, strlen(test_message));

	if (bytes_written < 0) {
		syslog(LOG_ERR, "WK2132: Failed to write data: %s", strerror(errno));

	} else {
		syslog(LOG_INFO, "WK2132: Sent %d bytes", (int)bytes_written);
	}

	/* Try to read data (non-blocking) */
	bytes_read = read(fd, read_buffer, sizeof(read_buffer) - 1);

	if (bytes_read > 0) {
		read_buffer[bytes_read] = '\0';
		syslog(LOG_INFO, "WK2132: Received: %s", read_buffer);

	} else if (bytes_read < 0 && errno != EAGAIN) {
		syslog(LOG_ERR, "WK2132: Failed to read data: %s", strerror(errno));
	}

	/* Close the port */
	close(fd);
}

/**
 * @brief Example configuration for GPS module on WK2132 port
 */
void wk2132_gps_example(void)
{
	int fd;
	struct termios config;
	const char *gps_init_cmd = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";

	/* Open WK2132 port for GPS (e.g., /dev/ttyS7) */
	fd = open("/dev/ttyS7", O_RDWR | O_NOCTTY);

	if (fd < 0) {
		syslog(LOG_ERR, "WK2132: Failed to open GPS port: %s", strerror(errno));
		return;
	}

	/* Configure for GPS (typically 9600 baud) */
	if (tcgetattr(fd, &config) == 0) {
		cfsetispeed(&config, B9600);
		cfsetospeed(&config, B9600);

		config.c_cflag = CS8 | CREAD | CLOCAL; /* 8N1, enable receiver */
		config.c_lflag = 0;  /* Raw mode */
		config.c_oflag = 0;  /* Raw output */
		config.c_iflag = 0;  /* Raw input */

		tcsetattr(fd, TCSANOW, &config);

		/* Send GPS initialization command */
		write(fd, gps_init_cmd, strlen(gps_init_cmd));

		syslog(LOG_INFO, "WK2132: GPS port configured and initialized");
	}

	close(fd);
}

/**
 * @brief Test function to verify all WK2132 ports
 */
void wk2132_test_all_ports(void)
{
	char device_path[16];
	int fd;
	int port;

	for (port = 0; port < 4; port++) {
		snprintf(device_path, sizeof(device_path), "/dev/ttyS%d", 6 + port);

		fd = open(device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (fd >= 0) {
			syslog(LOG_INFO, "WK2132: Port %d (%s) is available", port + 1, device_path);
			close(fd);

		} else {
			syslog(LOG_WARNING, "WK2132: Port %d (%s) is not available: %s",
			       port + 1, device_path, strerror(errno));
		}
	}
}

#endif /* CONFIG_WK2132_SERIAL */
