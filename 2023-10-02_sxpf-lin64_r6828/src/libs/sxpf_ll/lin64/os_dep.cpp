/** @file os_dep.cpp
 *
 * Operating-system-dependent functions implementations for Linux.
 */
/* for scandir */
#define _XOPEN_SOURCE   700
#define _SVID_SOURCE
#define _DEFAULT_SOURCE

#include "sxpf.h"
#include "svnversion.h"
#include "sxpf_version.h"

#include <sys/types.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>


static int filter(const struct dirent *ent)
{
    int n, l;
    int r = sscanf(ent->d_name, "sxpf%d%n", &n, &l);

    return (r == 1) && (ent->d_name[l] == 0);
}


/** Linux-specific function to return the number of available SX proFRAME cards
 *  in the system.
 *
 * @return The number of available sxpf devices
 */
int sxpf_get_num_cards(void)
{
    struct dirent **namelist = nullptr;
    int             n = scandir("/sys/class/sxpf", &namelist, filter, nullptr);

    if (n < 0)
    {
        // try to get number of sxpf devices from /dev, which may be
        // misleading, if nodes have been created using mknod without the
        // driver being loaded...
        n = scandir("/dev", &namelist, filter, nullptr);
    }

    if (n < 0)
        return 0;

    free(namelist);

    return n;
}


static char *get_version_string()
{
    static char     ver_string[128];

    if (!ver_string[0])
    {
        FILE    *f = fopen("/sys/module/sxpf/version", "r");

        if (f)
        {
            char *s = fgets(ver_string, sizeof(ver_string), f);

            fclose(f);

            return s;
        }
    }

    return ver_string;
}


/** Linux-specific function to get Software revision information.
 *
 * @param library_revision  Pointer to library's SVN revision
 * @param driver_revision   Pointer to driver's SVN revision. Will be written
 *                          as 0, if the driver isn't recent enough to provide
 *                          this information.
 *
 * @return 0 on success; 1 on parameter error; -1 no driver version available
 */
int sxpf_get_revisions(__u32 *library_revision, __u32 *driver_revision)
{
    if (!(library_revision && driver_revision))
        return 1;

    *library_revision = SVNVERSION;
    *driver_revision = 0;

    char *s = get_version_string();

    auto res = sscanf(s, "r%u", driver_revision);

    if (res == 1)
        return 0;

    unsigned major;
    unsigned minor;
    unsigned issue;
    res = sscanf(s, "%u.%u.%u.r%u", &major, &minor, &issue, driver_revision);

    if (res != 4)
        return -1;

    return 0;
}


/** Linux-specific function to get Software version information.
 *
 * The version numbers returned by this function are constructed like this:
 * (major_version << 24) | (minor_version << 16) | (issue_number)
 *
 * @param library_version  Pointer to library's version
 * @param driver_version   Pointer to driver's version. Will be written
 *                         as 0, if the driver isn't recent enough to provide
 *                         this information.
 *
 * @return 0 on success; 1 on parameter error; -1 no driver version available
 */
int sxpf_get_sw_versions(__u32 *library_version, __u32 *driver_version)
{
    if (!(library_version && driver_version))
        return 1;

    *library_version =
        (SXPF_SW_MAJOR << 24) | (SXPF_SW_MINOR << 16) | SXPF_SW_ISSUE;
    *driver_version = 0;

    char *s = get_version_string();

    if (!s)
        return -1;

    unsigned major;
    unsigned minor;
    unsigned issue;
    unsigned revision;
    auto res = sscanf(s, "%u.%u.%u.r%u", &major, &minor, &issue, &revision);

    if (res != 4)
        return -1;

    *driver_version = (major << 24) | (minor << 16) | issue;

    return 0;
}
