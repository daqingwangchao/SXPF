/**
 * @file    sxpf.cpp
 *
 * SX proFRAME frame grabber user space library implementation.
 *
 * The basic usage scenario of this low-level part of the library looks like
 * this: @image html grabber-API.png
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "sxpf.h"
#include "sxpf_regs.h"
#include "driver_acc.h"

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <limits.h>
#include <assert.h>
#include <string.h>
#include <time.h>
#include <dirent.h>
#include <initializer_list>

static int write_lattice_i2c_register(sxpf_hdl pf, uint8_t port, int id, uint8_t reg, uint8_t val);
static int read_lattice_i2c_register(sxpf_hdl pf, uint8_t port, int id, uint8_t reg);
static int write_machxo_register(sxpf_hdl pf, unsigned port, uint8_t reg, uint8_t val);
static int read_machxo_register(sxpf_hdl pf, unsigned port, uint8_t reg);
//static int read_crosslink_register(sxpf_hdl pf, unsigned port, uint8_t reg);
static int read_lattice_i2c_date(sxpf_hdl pf, unsigned port, uint8_t id,
                                 uint8_t date[4]);
static int read_gmsl2_3_device_identifier(sxpf_hdl pf, unsigned port, int bus, int id);

static int get_adapter_info(sxpf_hdl pf,
                            sxpf_card_info_t *card_info,
                            unsigned port_idx);


static int get_adapt_crosslink_info(sxpf_hdl pf,
                                    sxpf_card_info_t *card_info,
                                    unsigned port_idx);


//static void get_adapt_eeprom_info(sxpf_hdl pf, cards_info_container_t* sx_cards_info,
//                                int card_idx, unsigned port_idx);


/** Help funtion to get card name based on firmware version
 *
 * @param card_info     SXPF card struct
 * @param fw_ver_major     FW version
 * @param fw_ver_minor     FW version
 *
 */
static void get_card_name(uint32_t fw_ver_major, uint32_t fw_ver_minor , sxpf_card_info_t *card_info)
{
    if (fw_ver_major == 3 || fw_ver_major == 9)   // proFRAME 2.0
    {
        switch (fw_ver_minor)
        {
        case 1:
        case 5:
        case 10:
        case 11:
            card_info->model = "SX proFRAME 2.0";
            break;
        case 7:
        case 8:
        case 9:
            card_info->model =  "SX proFRAME 2.0 LVDS";
            break;
        case 100:
        case 101:
            card_info->model = "SX proFRAME 2.0 sinaSCOPE w/ dual 4K ISP";
            break;
        default:
            card_info->model = "Undefined Card";
            break;
        }
    }
    else if (fw_ver_major == 4 || fw_ver_major == 10) // proFRAME 3.0
    {
        switch (fw_ver_minor)
        {
        case 1:
            card_info->model = "SX proFRAME 3.0 cPCI-S";
            break;
        case 2:
        case 9:
            card_info->model = "SX proFRAME 3.0 PCIe";
            break;
        case 3:
            card_info->model = "SX proFRAME 3.0 PCIe openLDI";
            break;
        case 4:
            card_info->model = "SX proFRAME 3.0 PCIe TAP";
            break;
        default:
            card_info->model = "Undefined Card";
            break;
        }
    }
}

/** Help funtion to get card temperature based on firmware version
 *
 * @param pf                      Grabber handle
 * @param fpga_temp_celsius       FPGA temperature in celsius
 *
 * @return 0 on success, non-null on error
 */
int sxpf_get_card_temperature(sxpf_hdl pf,
                              double *fpga_temp_celsius)
{
    assert(pf && DEV_VALID(pf->dev) && fpga_temp_celsius != NULL);

    uint32_t local_fpga_temperature = 0U;

    uint32_t fw_ver = ((pf->props.fw_version >> 24) & 255);

    int res = 0;

    if (!pf)
    {
        res = -1;
    }
    else
    {
        if (fw_ver == 3 || fw_ver == 9 || fw_ver == 4 || fw_ver == 10)
        {
            if(fw_ver == 4 || fw_ver == 10)
            {
                res = sxpf_plasma_readRegister(pf, 0xc400, &local_fpga_temperature);

                if (local_fpga_temperature > 0)
                {
                   *fpga_temp_celsius =
                       ((local_fpga_temperature/65536.0)/0.00199451786 - 273.76);
                }
            }
            else
            {
                res = sxpf_plasma_readRegister(pf, 0xc200, &local_fpga_temperature);

                if (local_fpga_temperature > 0)
                {
                   *fpga_temp_celsius    =
                       ((local_fpga_temperature * 503.975) / 65536.0 - 273.15);
                }
            }
        }
    }

    return res;
}

/** Fill SXPF card data structure with information about the queried cards.
 *
 * @param pf              Grabber handle.
 * @param card_info       SXPF card info
 *
 * @return 0 on success, -1 on error
 */
int sxpf_get_single_card_info(sxpf_hdl pf, sxpf_card_info_t *card_info)
{
    // check if card properties are valid
    int ret =
        sxpf_get_card_properties(pf, &card_info->properties);

    ret |= sxpf_plasma_readRegister(pf, 0x0c, &card_info->plasma_version);

    if(ret)
    {
        return -1;
    }
    else
    {
        // set the usage mode
        if (card_info->properties.capabilities & SXPF_CAP_VIDEO_PLAYBACK)
        {
            card_info->style = "REPLAY";
        }
        else
        {
            card_info->style = "CAPTURE";
        }

        // Set firmware build date
        card_info->fw_build_date.year =
            ((card_info->properties.fw_date) & 65535);

        card_info->fw_build_date.month =
            (((card_info->properties.fw_date) >> 16) & 255);

        card_info->fw_build_date.day =
            (((card_info->properties.fw_date) >> 24) & 255);


        //Check if SG is active
        uint32_t    reg_feature = 0;
        sxpf_read_register(pf, 0, REG_FEATURE0, &reg_feature);

         (   (reg_feature & (1 << SXPF_FEATURE_SG_PLDA))
            || (reg_feature & (1 << SXPF_FEATURE_SG_XDMA))) ?
              card_info->is_scatter_gather = 1 :
              card_info->is_scatter_gather = 0;


        // Set the card name and firmware version
        int      num_adapters           = 4U;
        uint32_t local_ver_minor        = 0U;
        uint32_t local_ver_major        = 0U;

        local_ver_minor =
            (((card_info->properties.fw_version) >> 16) & 255);

        local_ver_major =
            (((card_info->properties.fw_version) >> 24) & 255);

        sprintf(card_info->fw_version, "%d.%d.%d", local_ver_major,
                local_ver_minor, ((card_info->properties.fw_version) & 65535));

        if (local_ver_major >= 8)
        {
            card_info->card_generation = (local_ver_major - 7U); // playback card
        }
        else
        {
            card_info->card_generation = (local_ver_major - 1U); // capture card
        }

        //uint32_t local_ver = card_info->properties.fw_version >>24;

        // get card temperature
        double temp_celsius    = 0U;

        if(sxpf_get_card_temperature(pf, &temp_celsius) == 0)
        {
            card_info->status.fpga_temp_celsius    = (float)temp_celsius;
        }

        // get card name
        get_card_name(local_ver_major, local_ver_minor, card_info);

        if (card_info->card_generation >= 3)
        {
            num_adapters = 2U;
        }
        for(int port_idx = 0; port_idx < num_adapters; ++port_idx)
        {
            get_adapter_info(pf, card_info, port_idx);
        }

        // get device id
        memset(card_info->device_dna, 0xff, sizeof(uint32_t)*3);
        if (card_info->card_generation >= 3)
        {
            for(int i=0; i<3; i++)
            {
                sxpf_read_register(pf, 1, REG_DEVICE_DNA0 + i*4, &card_info->device_dna[i]);
            }
        }
    }

    return 0;
}

/** Fill SXPF adapters data structure with information about the queried adpaters.
 *
 * @param pf              Grabber handle.
 * @param sx_cards_info   SXPF tree data structure for storing cards information
 * @param card_idx        SXPF card id
 * @param port_idx        SXPF index of the used slot
 *
 * @return 0 on success, else on error
 */
static int get_adapter_info(sxpf_hdl pf, sxpf_card_info_t *card_info, unsigned port_idx)
{
    int       adapt_type    = -1;
    int       adapt_version = -1;
    uint32_t  bus_config    = 0;
    int res = 0;

    // set baudrate to 100kBaud to make sure to reach MachXO
    if (sxpf_i2c_baudrate(pf, port_idx, 100000) != 0)
    {
        res |= -1;
    }

    // store I2C bus configuration
    res  = sxpf_plasma_readRegister(pf, 0x20 + 4 * port_idx, &bus_config);

    for (auto config: { 2, 4 })
    {
        // enable I2C bus segment to access MachXO FPGA
        res |= sxpf_plasma_writeRegister(pf, 0x20 + 4 * port_idx, config);

        // read type & version registers
        adapt_type    = read_machxo_register(pf, port_idx, 0x80);
        adapt_version = read_machxo_register(pf, port_idx, 0x81);

        // if MachXO FPGA access failed try on other I2C bus segment
        if (adapt_type >= 0 && adapt_version >= 0)
        {
            break;
        }
    }

    sxpf_adapt_info_t *adapters_info =
        &card_info->adapters[port_idx];

    if (adapt_type < 0 || adapt_version < 0)
    {
        adapters_info->name    = "No adapter connected";
        adapters_info->type    = adapt_type;
        adapters_info->version = adapt_version;

    }
    else
    {
        const char  *name;
        uint8_t     xo_date[4] = { 0 };
        switch (adapt_type)
        {
        case 24: name = "Customer-specific";                                 break;

        // proFRAME 2.0
        case  6: name = "SX camAD TI953/TI960";                              break;
        case  7: name = "SX camAD MAX9240a/MAX9271";                         break;
        case  8: name = "SX camAD TI913/TI914";                              break;
        case  9: name = "SX camAD TI914 dual";                               break;
        case 10: name = "SX camAD TI953 dual (or TI935 dual)";               break;
        case 11: name = "SX camAD MAX9275/MAX9276";                          break;
        case 12: name = "SX camAD MAX9240a dual";                            break;
        case 14: name = "SX camAD TI953/954 (or TI935/936)";                 break;
        case 15: name = "SX camAD TI954 (or TI936)";                         break;
        case 16: name = "SX camAD MAX96705/MAX96706";                        break;
        case 17: name = "SX camAD MAX9295/MAX9296A (or MAX96716/MAX96717)";  break;

        // proFRAME 3.0
        case 21:
        {
            name = "SX camAD3 DUAL MAX9296A or MAX96716A or MAX96792A (auto-detect failed)";

            int reg0 = read_machxo_register(pf, port_idx, 0x00);
            int reg3 = read_machxo_register(pf, port_idx, 0x03);
            if (res == 0 && reg0 != -1 && reg3 != -1)
            {
                res |= write_machxo_register(pf, port_idx, 0x00, reg0 | 0x08);
                res |= write_machxo_register(pf, port_idx, 0x03, reg3 | 0x08);

                int des0 = read_gmsl2_3_device_identifier(pf, port_idx, 0x08, 0x90);
                int des1 = read_gmsl2_3_device_identifier(pf, port_idx, 0x20, 0x90);
                if (des0 == 0x94 && des1 == 0x94)
                {
                    name = "SX camAD3 DUAL MAX9296A";
                }
                else
                {
                    des0 = read_gmsl2_3_device_identifier(pf, port_idx, 0x08, 0x50);
                    des1 = read_gmsl2_3_device_identifier(pf, port_idx, 0x20, 0x50);
                    if (des0 == 0xBE && des1 == 0xBE)
                    {
                        name = "SX camAD3 DUAL MAX96716A";
                    }
                    else if (des0 == 0xB6 && des1 == 0xB6)
                    {
                        name = "SX camAD3 DUAL MAX96792A";
                    }
                }

                res |= write_machxo_register(pf, port_idx, 0x00, reg0);
                res |= write_machxo_register(pf, port_idx, 0x03, reg3);
            }
            break;
        }
        case 23: name = "SX camAD3 TI954 dual (or TI936 dual)";              break;
        case 27: name = "SX camAD3 MAX96878 dual H-MTD";                     break;
        case 28:
        {
            name = "SX camAD3 DUAL MAX9295A/MAX9296A or MAX96717/MAX96716A or MAX96793/MAX96792A (auto-detect failed)";

            int reg0 = read_machxo_register(pf, port_idx, 0x00);
            int reg3 = read_machxo_register(pf, port_idx, 0x03);
            if (res == 0 && reg0 != -1 && reg3 != -1)
            {
                res |= write_machxo_register(pf, port_idx, 0x00, reg0 | 0x0C);
                res |= write_machxo_register(pf, port_idx, 0x03, reg3 | 0x0C);

                int des0 = read_gmsl2_3_device_identifier(pf, port_idx, 0x08, 0x90);
                int des1 = read_gmsl2_3_device_identifier(pf, port_idx, 0x20, 0x90);
                int ser0 = read_gmsl2_3_device_identifier(pf, port_idx, 0x10, 0x80);
                int ser1 = read_gmsl2_3_device_identifier(pf, port_idx, 0x40, 0x80);
                if (des0 == 0x94 && des1 == 0x94 && ser0 == 0x91 && ser1 == 0x91)
                {
                    name = "SX camAD3 DUAL MAX9295A/MAX9296A";
                }
                else
                {
                    des0 = read_gmsl2_3_device_identifier(pf, port_idx, 0x08, 0x50);
                    des1 = read_gmsl2_3_device_identifier(pf, port_idx, 0x20, 0x50);
                    ser0 = read_gmsl2_3_device_identifier(pf, port_idx, 0x10, 0x80);
                    ser1 = read_gmsl2_3_device_identifier(pf, port_idx, 0x40, 0x80);
                    if (des0 == 0xBE && des1 == 0xBE && ser0 == 0xBF && ser1 == 0xBF)
                    {
                        name = "SX camAD3 DUAL MAX96717/MAX96716A";
                    }
                    else if (des0 == 0xB6 && des1 == 0xB6 && ser0 == 0xB7 && ser1 == 0xB7)
                    {
                        name = "SX camAD3 DUAL MAX96793/MAX96792A";
                    }
                }

                res |= write_machxo_register(pf, port_idx, 0x00, reg0);
                res |= write_machxo_register(pf, port_idx, 0x03, reg3);
            }
            break;
        }
        case 29: name = "SX camAD3 TI953/954 (or TI935/936)";                break;
        case 30: name = "SX camAD3 TI9702 dual";                             break;
        case 31: name = "SX camAD3 DUAL MAX96705/96706 Capture";             break;
        case 32: name = "SX camAD3 DUAL MAX96705/96706 Replay";              break;
        case 33: name = "SX camAD3 QUAD IPEX";                               break;
        case 34: name = "SX camAD3 DUAL CXD4967/4966";                       break;
        case 35: name = "SX ledAD3 Base";                                    break;

        default: name = "unknown";                                           break;
        }

        // Print and save the adapters info
        adapters_info->name    = name;
        adapters_info->type    = adapt_type;
        adapters_info->version = adapt_version;

        for (auto config: { 2, 4 })
        {
            // enable I2C bus segment to access MachXO FPGA
            res |= sxpf_plasma_writeRegister(pf, 0x20 + 4 * port_idx, config);

            if (read_lattice_i2c_date(pf, port_idx, 0x84, xo_date) >= 0)
            {
                adapters_info->machxo_build_num        = xo_date[0];
                adapters_info->machxo_creat_date.day   = xo_date[1];
                adapters_info->machxo_creat_date.month = xo_date[2];
                adapters_info->machxo_creat_date.year  = xo_date[3];

                break;
            }
        }

        // Get Crosslink information
        get_adapt_crosslink_info(pf, card_info, port_idx);

    }
    // restore I2C bus config
    sxpf_plasma_writeRegister(pf, 0x20 + 4 * port_idx, bus_config);

    return res;
}

/** Fill SXPF crosslink data structure with information about the queried adapters.
 *
 * @param pf              Grabber handle.
 * @param sx_cards_info   SXPF tree data structure for storing cards information
 * @param card_idx        SXPF card id
 * @param port_idx        SXPF index of the used slot
 * @param adpater_type    SXPF cam adapter type
 *
 * @return 0 on success, else on error
 */
static int get_adapt_crosslink_info(sxpf_hdl pf, sxpf_card_info_t *card_info, unsigned port_idx)
{
    sxpf_adapt_info_t *adapters_info =
        &card_info->adapters[port_idx];

    switch (adapters_info->type)
    {
    default:
        break;

    case 17:
        break;
    }

    int     cl_type    = read_lattice_i2c_register(pf, port_idx, 0x86, 0x80);
    int     cl_version = read_lattice_i2c_register(pf, port_idx, 0x86, 0x81);
    uint8_t cl_date[4] = { 0 };
    int     res        = 0;
    res = read_lattice_i2c_date(pf, port_idx, 0x86, cl_date);

    if (cl_type < 0 || cl_version < 0 || res < 0)
    {
        adapters_info->cl_info.cl_type     = cl_type;
        adapters_info->cl_info.cl_version  = cl_version;
    }
    else
    {
        const char  *name;

        switch (cl_type)
        {
        case 0x11:  name = "Record + Playback (x6)"; break;
        case 0x13:  name = "Record only (x12)";      break;
        case 0x14:  name = "Playback only (x12)";    break;
        default:    name = "unknown";                break;
        }

        adapters_info->cl_info.cl_name               = name;
        adapters_info->cl_info.cl_type               = cl_type;
        adapters_info->cl_info.cl_version            = cl_version;
        adapters_info->cl_info.cl_build_num          = cl_date[0];
        adapters_info->cl_info.cl_creat_date.year    = cl_date[3];
        adapters_info->cl_info.cl_creat_date.day     = cl_date[2];
        adapters_info->cl_info.cl_creat_date.month   = cl_date[1];
    }

    return res;
}

#if 0
/** Fill SXPF eeprom data structure with information about the queried adapters.
 *
 * @param pf              Grabber handle.
 * @param sx_cards_info   SXPF tree data structure for storing cards information
 * @param card_idx        SXPF card id
 * @param port_idx        SXPF index of the used slot
 *
 */
static void get_adapt_eeprom_info(sxpf_hdl pf, cards_info_container_t* sx_cards_info, int card_idx, unsigned port_idx)
{
    // TBC: do we need to make a eeeprom structure ? In the eeprom function
    // As I've seen in the eeprom function, we only read out registers, nothing is saved.
}
#endif

static int write_lattice_i2c_register(sxpf_hdl pf, uint8_t port,
                                      int id, uint8_t reg, uint8_t val)
{
    uint8_t wbuf[3];
    int     res = 0;

    if (reg >= 0x80)
    {
        wbuf[0] = 0x02;
        wbuf[1] = reg - 0x80;
    }
    else
    {
        wbuf[0] = 0x01;
        wbuf[1] = reg;
    }
    wbuf[2] = val;

    res = sxpf_i2c_xfer(pf, port, id, wbuf, 3, NULL, 0);

    if (res != 0)
    {
        return -1;
    }

    return 0;
}

static int read_lattice_i2c_register(sxpf_hdl pf, uint8_t port,
                                     int id, uint8_t reg)
{
    uint8_t wbuf[2];
    uint8_t val = 0;
    int     res = 0;

    if (reg >= 0x80)
    {
        wbuf[0] = 0x0B;
        wbuf[1] = reg - 0x80;
    }
    else
    {
        wbuf[0] = 0x05;
        wbuf[1] = reg;
    }

    res = sxpf_i2c_xfer(pf, port, id, wbuf, 2, &val, 1);

    if (res != 0)
    {
        return -1;
    }

    return val;
}

static int write_machxo_register(sxpf_hdl pf, unsigned port, uint8_t reg, uint8_t val)
{
    return write_lattice_i2c_register(pf, port, 0x84, reg, val);
}

static int read_machxo_register(sxpf_hdl pf, unsigned port, uint8_t reg)
{
    return read_lattice_i2c_register(pf, port, 0x84, reg);
}

#if 0
static int read_crosslink_register(sxpf_hdl pf, unsigned port, uint8_t reg)
{
    return read_lattice_i2c_register(pf, port, 0x86, reg);
}
#endif

static int read_lattice_i2c_date(sxpf_hdl pf, unsigned port, uint8_t id,
                                 uint8_t date[4])
{
    int res = 0;

    for (uint8_t reg = 0; reg < 4; reg++)
    {
        res = read_lattice_i2c_register(pf, port, id, 4 + reg + 0x80);

        if (res < 0)
            break;

        date[reg] = res;
    }

    return res;
}

static int read_gmsl2_3_device_identifier(sxpf_hdl pf, unsigned port, int bus, int id)
{
    uint8_t        wbuf[2];
    uint8_t        val = 0;
    int            res = -1;

    sxpf_plasma_writeRegister(pf, 0x20 + 4 * port, bus);

    // MAX96716A/F: 0x000D = 0xBE
    // MAX96717:    0x000D = 0xBF
    // MAX9296A:    0x000D = 0x94
    // MAX9295A:    0x000D = 0x91
    // MAX96792A:   0x000D = 0xB6
    // MAX96793:    0x000D = 0xB7
    wbuf[0] = 0x00;
    wbuf[1] = 0x0D;

    for (int i=0; i<10; i++)
    {
        res = sxpf_i2c_xfer(pf, port, id, wbuf, 2, &val, 1);
        if (res == 0)
        {
            return val;
        }
        usleep(1000);
    }

    return -1;
}
