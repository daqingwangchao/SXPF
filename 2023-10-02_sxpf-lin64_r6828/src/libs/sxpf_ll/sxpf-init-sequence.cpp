#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// SXPF low-level API
#include "sxpf.h"
#include "sxpf-init-sequence.h"
#include "sxpf-init-sequence-parser.h"
#include "sxpf_regs.h"

// ----------------------------------------------------------------------------
// - private configuration
// ----------------------------------------------------------------------------

#define CONFIG_USE_LOW_LEVEL_REGISTER_ACCESS    (1)


#if 1
#define LOG_ERR(args, ...) \
    fprintf(stderr, "\nERROR! " args "\n\n", ##__VA_ARGS__)
#else
#define LOG_ERR(args, ...) (void)0
#endif

void sxpf_free_init_sequence(uint8_t* ini_buffer)
{
    free(ini_buffer);
}


uint8_t* sxpf_load_init_sequence(const char* iniFileName, uint32_t* data_size)
{
    sxpf_init_sequence_t    initSequence;

    uint8_t* ini_data = NULL;

#define MAX_TMP_SIZE (1*1024*1024)

    char* tmp = NULL;
    int tmpSize = 0;

    uint32_t unused = 0;

    if (!data_size)
        return NULL;
    *data_size = 0;

    if (sxpf_init_sequence_parser_open(iniFileName, &initSequence) < 0)
        return NULL;

    ini_data = sxpf_init_sequence_parser_createUpdateBinary2(&initSequence,
        data_size,
        1048576);
    if (!ini_data)
        return NULL;

    for (uint32_t i = 0; i < initSequence.sequenceListCount; i++)
    {
        if (!initSequence.sequenceList[i].used)
        {
            printf("\nNOTICE! sequence '%s' not used and will not be placed "
                "in binary!\n", initSequence.sequenceList[i].eventname);
            unused++;
        }
    }

    if (ini_data)
    {
        tmp = (char*)malloc(MAX_TMP_SIZE);
        if (!tmp)
        {
            LOG_ERR("Out of memory during allocating memory for temporary "
                "decoding ini_data!");
            free(ini_data);
            return NULL;
        }
        tmpSize = sxpf_decode_init_sequence(ini_data, *data_size,
            tmp, MAX_TMP_SIZE);
        free(tmp);
        if (tmpSize < 0)
        {
            free(ini_data);
            return NULL;
        }
    }

    if (iniFileName)
    {
        sxpf_init_sequence_parser_close(&initSequence);
    }

    return ini_data;
}


int sxpf_send_init_sequence_verify(sxpf_hdl fg, uint8_t* ini_data,
    uint32_t data_size, uint32_t port,
    uint32_t execute_event_id,
    uint8_t* regsize_table,
    int (*log_cb)(void *log_ctx, char const* fmt, va_list args), void *log_ctx)
{
    uint32_t portBackup = port;
    int ret = 0;

    if (log_cb == NULL)
    {
        log_cb = [](void *, char const* fmt, va_list args)
        {
            vprintf(fmt, args);
            return 0;
        };
    }

    // call user-provided log function and exit early if it returns an error
    auto log_bk = [log_ctx, &ret, log_cb](char const* fmt, ...) -> int
    {
        va_list va;

        va_start(va, fmt);
        int r = log_cb(log_ctx, fmt, va);
        va_end(va);
        if (r < 0)
            ret = 1;
        return r;
    };

    {
        uint32_t* data32 = (uint32_t*)ini_data, size, crc32, tmp;
        uint8_t* p, * pEndOfData;
        uint32_t count = 0;

        if (!ini_data) {
            LOG_ERR("No init sequence given! can't execute anything!");
            return 8;
        }

        if (!fg) {
            LOG_ERR("No device given! can't execute anything!");
            return 9;
        }

        log_bk("port %u: execute event %d\n", port, execute_event_id);

        // check the least number of bytes must exists
        if (data_size < (16 + 4)) { // 16=Header, 4=Null-Event
            LOG_ERR("Binary size of %d bytes can not be a valid init sequence!",
                data_size);
            return 10;
        }

        if (0xC0D1F1ED != *data32++ || 0x00010000 != *data32++) {
            LOG_ERR("raw ini_data is no init sequence format!");
            return 11;
        }

        // CRC32 and SIZE information
        size = *data32++;
        crc32 = *data32++; (void)crc32; // avoid compile time warning
        if (size > data_size) {
            LOG_ERR("size of header (%d) not match binary-blob size (%d)!", size,
                data_size);
            return 12;
        }

        pEndOfData = ini_data + size;

        do {
            tmp = *data32++;
            if (tmp) {
                uint32_t tmp_id, tmp_offset;
                tmp_id = tmp >> 24;
                tmp_offset = tmp & 0xFFFFFF;
                if (tmp_id == execute_event_id) {
                    uint8_t tag, length, retryCount = 1;
                    log_bk("port %u: run sequence at offset 0x%08x\n", port, tmp_offset);
                    count++;
                    p = ini_data + tmp_offset;
                    if (p >= pEndOfData) {
                        LOG_ERR("end of ini_data");
                        return 14;
                    }
                    tag = *p++;
                    while (tag && p < pEndOfData) {
                        length = *p++;
                        switch (tag) {
                        case 0x01: // CMD_INVALIDATE_CACHE
                            // not needed
                            break;
                        case 0x02: { // CMD_WAIT_MILLIS
                            uint32_t ms;
                            switch (length) {
                            case 1: ms = p[0]; break;
                            case 2: ms = p[1] << 8 | p[0]; break;
                            case 3: ms = p[2] << 16 | p[1] << 8 | p[0]; break;
                            case 4: ms = p[3] << 24 | p[2] << 16 | p[1] << 8 | p[0]; break;
                            default:
                                LOG_ERR("Invalid time value");
                                return 14;
                            }
                            log_bk("port %u: wait %ums\n", port, ms);
                            usleep(ms * 1000);
                            break;
                        }
                        case 0x03: { // CMD_WRITE_REGISTER
                            int result;
                            uint32_t reg = 0, val = 0, inc = 0;
                            // supported length values are 2, 3, 8 and 12 bytes
                            uint32_t numargs = (length % 3 == 0) ? 3 : 2;
                            uint32_t argsize = length / numargs;
                            if ((numargs != 2 && numargs != 3) ||
                                (argsize != 1 && argsize != 4)) {
                                LOG_ERR("Invalid register args");
                                return 14;
                            }
                            // parse arguments in a byte-wise fashion
                            for (uint32_t b = 0; b < argsize; b++) {
                                reg += p[b] << (8 * b);             // register
                                val += p[argsize + b] << (8 * b);   // value
                                if (3 == numargs) {         // channel-increment
                                    inc += p[2 * argsize + b] << (8 * b);
                                }
                            }
                            reg = (reg + (inc * port)) & 0x00fffffc;
                            log_bk("port %u: write-reg: 0x%x:=0x%x\n", port,
                                reg, val);

                            if (CONFIG_USE_LOW_LEVEL_REGISTER_ACCESS &&
                                reg < REG_SPECIAL_BASE_OFFSET)
                            {
                                result = sxpf_write_register(fg, PLASMA_REGION,
                                    reg, val);
                            }
                            else
                            {
                                result = sxpf_plasma_writeRegister(fg, reg, val);
                            }

                            if (result) {
                                log_bk(" --> failed : 0x%x\n", result);
                                return 15;
                            }
                            break;
                        }
                        case 0x04: { // CMD_I2C_BAUDRATE
                            uint32_t br;
                            int result;
                            switch (length) {
                            case 1: br = p[0]; break;
                            case 2: br = p[1] << 8 | p[0]; break;
                            case 3: br = p[2] << 16 | p[1] << 8 | p[0]; break;
                            case 4: br = p[3] << 24 | p[2] << 16 | p[1] << 8 | p[0]; break;
                            default:
                                LOG_ERR("Invalid baudrate value");
                                return 15;
                            }
                            log_bk("port %u: set baudrate: %d \n", port, br);
                            result = sxpf_i2c_baudrate(fg, port, br);
                            if (result) {
                                log_bk(" --> failed\n");
                                return 16;
                            }
                            break;
                        }
                        case 0x05: // CMD_I2C_RETRYCOUNT
                            retryCount = (0 == p[0]) ? 1 : p[0];
                            break;
                        case 0x06: // HUB
                            log_bk("HUB-CMD not implemented! -> ignore it!\n");
                            break;
                        case 0x07: // GPIO
                            log_bk("GPIO-CMD not implemented! -> ignore it!\n");
                            break;
                        case 0x08:
                        case 0x09:
                        case 0x0A:
                        case 0x0B:
                        case 0x0C:
                            log_bk("unsupported tag 0x%x -> ignore it\n", tag);
                            break;
                        case 0x0D: // CMD_SWITCH_I2C_PORT
                        {
                            uint8_t secondary;
                            if (1 != length) {
                                LOG_ERR("Invalid command args");
                                return 18;
                            }
                            secondary = *(uint8_t*)&p[0];
                            switch (portBackup)
                            {
                            case 0:
                                port = (secondary != 0) ? 1 : portBackup;
                                break;
                            case 1:
                                port = (secondary != 0) ? 0 : portBackup;
                                break;
                            case 2:
                                port = (secondary != 0) ? 3 : portBackup;
                                break;
                            case 3:
                                port = (secondary != 0) ? 2 : portBackup;
                                break;
                            }
                            log_bk("switching to i2c port: %d\n", port);
                            break;
                        }
                        case 0x0E:
                            log_bk("unsupported tag 0x%02x -> ignore it\n", tag);
                            break;
                        case 0x0F: {
                            switch (p[0]) {
                            case 0x00:
                                log_bk("JUMP-CMD not implemented! -> ignore it!\n");
                                break;
                            case 0x01:
                                log_bk("RETURN-CMD not implemented! -> ignore it!\n");
                                break;
                            case 0x02: { // CMD_MASKED_I2C_WRITE
                                int retry, result = 0;
                                if (length < 4 || (p[2] + 2 * p[3]) != (length - 4)) {
                                    log_bk("invalid masked-i2c-transfer format --> giving up!\n");
                                    return 21;
                                }
                                log_bk("port %u: masked-i2c-transfer: 0x%02x %d ", port, p[1], length);
                                for (int i = 2; i < length; i++) {
                                    log_bk("0x%02x ", p[i]);
                                }

                                for (retry = 0; retry < retryCount; retry++) {
                                    if (retry > 0)
                                        log_bk("port %d: retry", port);
                                    uint8_t data[128];
                                    for (int i = 0; i < p[1]; i++) {
                                        data[i] = p[4 + i];
                                    }
                                    result = sxpf_i2c_xfer(fg, port, p[1], data, p[2], data + p[2], p[3]);
                                    if (result) {
                                        log_bk(" --> failed!\n");
                                    }
                                    else {
                                        for (int i = 0; i < p[3]; i++) {
                                            uint8_t mask = p[p[2] + 4 + i];
                                            uint8_t write = p[p[2] + p[3] + 4 + i];
                                            log_bk(" 0x%02x ", data[p[2] + i]);
                                            data[p[2] + i] = (data[p[2] + i] & ~mask) | (write & mask);
                                            log_bk("-> 0x%02x", data[p[2] + i]);
                                        }
                                        result = sxpf_i2c_xfer(fg, port, p[1], data, p[2] + p[3], NULL, 0);
                                        if (result)
                                            log_bk(" --> failed!\n");
                                        if (0 == result) { break; }
                                    }
                                }
                                if (0 != result) {
                                    log_bk("port %d:  --> giving up!\n", port);
                                    return 23;
                                }
                                log_bk("\n");
                                break;
                            }
                            case 0x03: { // CMD_CSI2_LANE_MAP
                                int      res        = 0;
                                uint8_t  adapt_type = 0;
                                uint32_t bus_config = 6;
                                uint8_t  wbuf[2]    = { 0x0B, 0x00 };
                                uint32_t reg0x90    = 0;
                                uint32_t reg0x94    = 0;
                                uint32_t reg0x98    = 0;
                                uint32_t reg0x9c    = 0;

                                log_bk("port %u: configure CSI-2 lane mapping", port);

                                // enable I2C bus segment to access MachXO FPGA
                                res  = sxpf_read_register(fg, PLASMA_REGION, 0x20 + 4 * port, &bus_config);
                                res |= sxpf_write_register(fg, PLASMA_REGION, 0x20 + 4 * port, 6);

                                // set baudrate to 100kBaud to make sure to reach MachXO
                                if (sxpf_i2c_baudrate(fg, port, 100000) != 0)
                                {
                                    res |= -1;
                                }

                                res |= sxpf_i2c_xfer(fg, port, 0x84, wbuf, 2, &adapt_type, 1);

                                sxpf_read_register(fg, 0, 0x90 + 0x20 * port, &reg0x90);
                                sxpf_read_register(fg, 0, 0x94 + 0x20 * port, &reg0x94);
                                sxpf_read_register(fg, 0, 0x98 + 0x20 * port, &reg0x98);
                                sxpf_read_register(fg, 0, 0x9c + 0x20 * port, &reg0x9c);

                                reg0x90 &= 0xff00ffff;
                                reg0x94 &= 0xff00ffff;
                                reg0x98 &= 0xff00ffff;
                                reg0x9c &= 0xff00ffff;

                                switch (adapt_type) {
                                case 21: // SX camAD3 DUAL MAX9296A or MAX96716A or MAX96792A
                                    sxpf_write_register(fg, 0, 0x90 + 0x20 * port, reg0x90 | 0x00d80000);
                                    sxpf_write_register(fg, 0, 0x94 + 0x20 * port, reg0x94 | 0x00d80000);
                                    sxpf_write_register(fg, 0, 0x98 + 0x20 * port, reg0x98 | 0x00e40000);
                                    sxpf_write_register(fg, 0, 0x9c + 0x20 * port, reg0x9c | 0x001b0000);
                                    break;
                                case 23: // SX camAD3 TI954 dual (or TI936 dual)
                                    sxpf_write_register(fg, 0, 0x90 + 0x20 * port, reg0x90 | 0x001b0000);
                                    sxpf_write_register(fg, 0, 0x94 + 0x20 * port, reg0x94 | 0x001b0000);
                                    sxpf_write_register(fg, 0, 0x98 + 0x20 * port, reg0x98 | 0x001b0000);
                                    sxpf_write_register(fg, 0, 0x9c + 0x20 * port, reg0x9c | 0x001b0000);
                                    break;
                                case 28: // SX camAD3 DUAL MAX9295A/MAX9296A or MAX96717/MAX96716A or MAX96793/MAX96792A
                                    sxpf_write_register(fg, 0, 0x90 + 0x20 * port, reg0x90 | 0x004e0000);
                                    sxpf_write_register(fg, 0, 0x94 + 0x20 * port, reg0x94 | 0x001b0000);
                                    sxpf_write_register(fg, 0, 0x98 + 0x20 * port, reg0x98 | 0x004e0000);
                                    sxpf_write_register(fg, 0, 0x9c + 0x20 * port, reg0x9c | 0x00930000);
                                    break;
                                case 29: // SX camAD3 TI953/954 (or TI935/936)
                                    sxpf_write_register(fg, 0, 0x90 + 0x20 * port, reg0x90 | 0x001b0000);
                                    sxpf_write_register(fg, 0, 0x94 + 0x20 * port, reg0x94 | 0x00e40000);
                                    sxpf_write_register(fg, 0, 0x98 + 0x20 * port, reg0x98 | 0x001b0000);
                                    sxpf_write_register(fg, 0, 0x9c + 0x20 * port, reg0x9c | 0x001b0000);
                                    break;
                                case 30: // SX camAD3 TI9702 dual
                                    sxpf_write_register(fg, 0, 0x90 + 0x20 * port, reg0x90 | 0x001b0000);
                                    sxpf_write_register(fg, 0, 0x94 + 0x20 * port, reg0x94 | 0x00720000);
                                    sxpf_write_register(fg, 0, 0x98 + 0x20 * port, reg0x98 | 0x00e40000);
                                    sxpf_write_register(fg, 0, 0x9c + 0x20 * port, reg0x9c | 0x001b0000);
                                    break;
                                case 31: // SX camAD3 DUAL MAX96705/96706
                                case 32:
                                    sxpf_write_register(fg, 0, 0x90 + 0x20 * port, reg0x90 | 0x00e40000);
                                    sxpf_write_register(fg, 0, 0x94 + 0x20 * port, reg0x94 | 0x00e40000);
                                    sxpf_write_register(fg, 0, 0x98 + 0x20 * port, reg0x98 | 0x00e40000);
                                    sxpf_write_register(fg, 0, 0x9c + 0x20 * port, reg0x9c | 0x00e40000);
                                    break;
                                case 33: // SX camAD3 QUAD IPEX
                                    sxpf_write_register(fg, 0, 0x90 + 0x20 * port, reg0x90 | 0x00360000);
                                    sxpf_write_register(fg, 0, 0x94 + 0x20 * port, reg0x94 | 0x00360000);
                                    sxpf_write_register(fg, 0, 0x98 + 0x20 * port, reg0x98 | 0x009c0000);
                                    sxpf_write_register(fg, 0, 0x9c + 0x20 * port, reg0x9c | 0x00390000);
                                    break;
                                default:
                                    if (res == 0)
                                    {
                                        log_bk(" --> not required!\n");
                                    }
                                    break;
                                }

                                res |= sxpf_write_register(fg, PLASMA_REGION, 0x20 + 4 * port, bus_config);

                                if (res) {
                                    log_bk(" --> failed!\n");
                                    return 24;
                                }

                                log_bk("\n");
                                break;
                            }
                            case 0x04: { // CMD_SET_POC
                                int result = 0;

                                if (length != 3) {
                                    log_bk("invalid set poc format --> giving up!\n");
                                    return 21;
                                }

                                if (p[2] < 83) {
                                    log_bk("configured poc voltage of %.1f V to low (must be >= 8.3 V)\n", p[2]/10.f);
                                    return 21;
                                }

                                log_bk("port %d: set port %d poc to %.1f V", port, p[1], p[2]/10.f);
                                uint8_t wbuf[2] = {
                                    p[1] == 0 ? (uint8_t)0x11 : (uint8_t)0x12,
                                    (uint8_t)((10e6 - 39000 * (p[2] - 10)) / (390 * (p[2] - 10)))
                                };
                                result = sxpf_i2c_xfer(fg, port, 0x50, wbuf, 2, NULL, 0);
                                if (result) {
                                    log_bk(" --> failed!\n");
                                    return 21;
                                }
                                log_bk("\n");

                                break;
                            }
                            default:
                                log_bk("unsupported subcommand 0x%02x -> ignore it!\n", p[0]);
                                break;
                            }
                            break;
                        }
                        default: {
                            int retry, result = 0;
                            log_bk("port %u: i2c-transfer: 0x%02x %d ", port, tag, length);
                            for (int i = 0; i < length; i++) {
                                log_bk("0x%02x ", p[i]);
                            }
                            for (retry = 0; retry < retryCount; retry++) {
                                if (retry > 0)
                                    log_bk("port %d: retry", port);
                                result = sxpf_i2c_xfer(fg, port, tag, p, length, NULL, 0);
                                if (result)
                                    log_bk(" --> failed!\n");
                                if (0 == result) { break; }
                            }
                            if (0 != result) {
                                log_bk("port %d:  --> giving up!\n", port);
                                return 17;
                            }
                            if (regsize_table && regsize_table[tag] != 0) {
                                uint8_t readback[256];
                                int verify_errors = 0;
                                int regsize = regsize_table[tag];

                                // send register address until it is
                                // acknowledged or timeout occurs
                                for (int i = 0; i < 100; i++)
                                {
                                    result = sxpf_i2c_xfer(fg, port, tag, p,
                                        regsize, NULL, 0);
                                    if (!result)
                                        break;
                                    usleep(1000);   // wait 1ms
                                }

                                // readd bytes from selected register address
                                if (!result)
                                    result = sxpf_i2c_xfer(fg, port, tag,
                                        NULL, 0, readback,
                                        length - regsize);
                                if (result != 0) {
                                    log_bk(" --> readback error.\n");
                                    return 19;  // readback error
                                }
                                for (int i = 0; i < length - regsize; i++)
                                {
                                    if (readback[i] != p[regsize + i]) {
                                        log_bk("\n --> verify error: seq[i]=%02x,"
                                            " readback[i]=%02x",
                                            p[regsize + i], readback[i]);
                                        verify_errors++;
                                    }
                                }
                                if (verify_errors == 0)
                                    log_bk(" --> verify OK.\n");
                                else
                                {
                                    log_bk("\n");
                                    return 20;
                                }
                            }
                            else
                                log_bk("\n");
                            break;
                        }
                        }
                        p += length;
                        tag = *p++;
                    }
                }
            }
        } while (ret == 0 && tmp && (uint8_t*)data32 < pEndOfData);

        if (!count) {
            log_bk("no event-id %d found. nothing executed\n", execute_event_id);
        }
    }

    return ret;
}


int sxpf_send_init_sequence(sxpf_hdl fg, uint8_t* ini_data, uint32_t data_size,
    uint32_t port, uint32_t execute_event_id)
{
    return sxpf_send_init_sequence_verify(fg, ini_data, data_size, port,
                                          execute_event_id, NULL, NULL, NULL);
}
