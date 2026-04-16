// SPDX-License-Identifier: Apache-2.0

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sim_pipe_reader);

#ifdef CONFIG_BOARD_NATIVE_SIM

#include <core/sim/sim_pipe_protocol.h>
#include <core/sim/sim_sensor_state.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <zephyr/drivers/sensor.h>

#define SIM_PIPE_READER_STACK_SIZE 2048
#define SIM_PIPE_READER_PRIORITY   5
#define SIM_PIPE_SOCKET_PATH       "/tmp/marshal_sim.sock"

K_THREAD_STACK_DEFINE(sim_pipe_reader_stack, SIM_PIPE_READER_STACK_SIZE);
static struct k_thread sim_pipe_reader_thread;
static bool sim_pipe_reader_started;

static ssize_t sim_pipe_read_exact(int fd, void *buffer, size_t size) {
    uint8_t *cursor = buffer;
    size_t total = 0U;

    while (total < size) {
        const ssize_t bytes_read = recv(fd, cursor + total, size - total, 0);
        if (bytes_read > 0) {
            total += (size_t) bytes_read;
            continue;
        }

        if (bytes_read < 0 && errno == EINTR) {
            continue;
        }

        break;
    }

    return (ssize_t) total;
}

static void sim_pipe_update_state(const struct SimFrame *frame) {
    k_mutex_lock(&g_sim_sensor_mutex, K_FOREVER);

    (void) sensor_value_from_float(&g_sim_sensor_state.accel_x, frame->accel_x_mps2);
    (void) sensor_value_from_float(&g_sim_sensor_state.accel_y, frame->accel_y_mps2);
    (void) sensor_value_from_float(&g_sim_sensor_state.accel_z, frame->accel_z_mps2);
    (void) sensor_value_from_float(&g_sim_sensor_state.gyro_x, frame->gyro_x_rads);
    (void) sensor_value_from_float(&g_sim_sensor_state.gyro_y, frame->gyro_y_rads);
    (void) sensor_value_from_float(&g_sim_sensor_state.gyro_z, frame->gyro_z_rads);
    (void) sensor_value_from_float(&g_sim_sensor_state.pressure, frame->pressure_pa);
    (void) sensor_value_from_float(&g_sim_sensor_state.temperature, frame->temp_c);

    k_mutex_unlock(&g_sim_sensor_mutex);
}

static void sim_pipe_reader_thread_entry(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    struct sockaddr_un address;
    const int backlog = 1;

    const int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        LOG_ERR("socket(%s) failed: %d", SIM_PIPE_SOCKET_PATH, errno);
        return;
    }

    (void) unlink(SIM_PIPE_SOCKET_PATH);

    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strncpy(address.sun_path, SIM_PIPE_SOCKET_PATH, sizeof(address.sun_path) - 1U);

    if (bind(server_fd, (struct sockaddr *) &address, sizeof(address)) < 0) {
        LOG_ERR("bind(%s) failed: %d", SIM_PIPE_SOCKET_PATH, errno);
        (void) close(server_fd);
        return;
    }

    if (listen(server_fd, backlog) < 0) {
        LOG_ERR("listen(%s) failed: %d", SIM_PIPE_SOCKET_PATH, errno);
        (void) close(server_fd);
        (void) unlink(SIM_PIPE_SOCKET_PATH);
        return;
    }

    LOG_INF("sim sensor socket ready at %s", SIM_PIPE_SOCKET_PATH);

    while (true) {
        const int client_fd = accept(server_fd, NULL, NULL);
        if (client_fd < 0) {
            if (errno == EINTR) {
                continue;
            }

            LOG_ERR("accept(%s) failed: %d", SIM_PIPE_SOCKET_PATH, errno);
            k_msleep(100);
            continue;
        }

        while (true) {
            struct SimFrame frame;
            const ssize_t bytes_read = sim_pipe_read_exact(client_fd, &frame, sizeof(frame));

            if (bytes_read != (ssize_t) sizeof(frame)) {
                LOG_INF("sim stream client disconnected");
                break;
            }

            if (frame.magic != SIM_FRAME_MAGIC) {
                LOG_WRN("dropping sim frame with bad magic: 0x%04x", frame.magic);
                continue;
            }

            if (frame.version != SIM_FRAME_VERSION) {
                LOG_WRN("dropping sim frame with unsupported version: %u", frame.version);
                continue;
            }

            sim_pipe_update_state(&frame);
        }

        (void) close(client_fd);
    }
}

void sim_pipe_reader_start(void) {
    if (sim_pipe_reader_started) {
        return;
    }

    sim_pipe_reader_started = true;

    k_tid_t tid =
        k_thread_create(&sim_pipe_reader_thread, sim_pipe_reader_stack, K_THREAD_STACK_SIZEOF(sim_pipe_reader_stack),
                        sim_pipe_reader_thread_entry, NULL, NULL, NULL, SIM_PIPE_READER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(tid, "sim_pipe_reader");
}

#else

void sim_pipe_reader_start(void) {}

#endif
