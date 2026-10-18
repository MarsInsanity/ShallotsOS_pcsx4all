/* tf_driver.h — device-correct driver.so path from /tmp/tfdevice.env.
 *
 * One SD image, N devices (R36SX / SF3000 / SF3500). The boot detector
 * (cubegm/tf_detect.sh) writes TF_DRIVER=<path to the matching driver_*.so>.
 * Loading the wrong driver.so segfaults the proprietary audio/video init, so
 * every frontend must resolve its driver this way. Falls back to the generic
 * driver.so when the env file is absent (dev / pre-detect boot).
 *
 * Uses POSIX open/read (not stdio) on purpose: some frontends (lgpt) #define
 * fopen/fread to a custom VFS, which would break a stdio-based reader. */
#ifndef TF_DRIVER_H
#define TF_DRIVER_H
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
static inline const char *tf_driver_path(void) {
    static char path[128];
    if (path[0]) return path;
    int fd = open("/tmp/tfdevice.env", O_RDONLY);
    if (fd >= 0) {
        char buf[512];
        int n = (int)read(fd, buf, sizeof buf - 1);
        close(fd);
        if (n > 0) {
            buf[n] = '\0';
            /* find "TF_DRIVER=" at start of file or start of a line */
            char *p = strstr(buf, "TF_DRIVER=");
            while (p && p != buf && p[-1] != '\n') p = strstr(p + 1, "TF_DRIVER=");
            if (p) {
                p += 10;
                char *e = p;
                while (*e && *e != '\r' && *e != '\n') e++;
                int len = (int)(e - p);
                if (len > 0 && len < (int)sizeof path) { memcpy(path, p, len); path[len] = '\0'; }
            }
        }
    }
    if (!path[0]) strcpy(path, "/mnt/sdcard/cubegm/driver.so");
    return path;
}
#endif /* TF_DRIVER_H */
