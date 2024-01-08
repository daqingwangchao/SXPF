/** @file adapter-osc.cpp
 *
 * Implementation of the adapter oscillator frequency setup.
 */
#include "sxpf.h"

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>


// SI 570 I2C Id
#define SI570_ID 0xAA

// SI 570 Register
#define SI570_CONFIG_START 7
#define SI570_CTRL         135
#define SI570_CTRL2        137

#define POW_2_28 268435456.0;

const uint8_t HS_DIV[6] = { 11, 9, 7, 6, 5, 4 };
const double FDCO_MAX = 5.67e9;
const double FDCO_MIN = 4.85e9;

struct si570_config
{
    uint64_t rfreq;
    uint8_t  hsdiv;
    uint8_t  n1;
};

struct si570_transfer
{
    uint32_t fout;        /* rw */
    uint8_t  config[6];   /* rw */
    uint8_t  smallchange; /* w  */
};


struct si570_data
{
    struct   si570_config factory;
    uint64_t xtal;
    uint8_t  config_regs[6];
    uint32_t fout;
};


static int si570_read_startup_configuration(sxpf_hdl pf, uint8_t channel,
                                            struct si570_data *data);
static void si570_get_trans(struct si570_data *data,
                            struct si570_transfer *trans);
static int si570_calc_divider(uint32_t fout, uint8_t* n1, uint8_t* hsdiv);
static int si570_write_trans_adapter(sxpf_hdl pf, uint8_t channel,
                                     struct si570_data *data,
                                     struct si570_transfer *ptrans);

uint8_t SetBits(uint8_t original, uint8_t reset_mask, uint8_t new_val);


int sxpf_adapter_get_osc_freq(sxpf_hdl pf, uint8_t channel, uint32_t *freq,
                              uint32_t factoryFreq)
{
    struct si570_data       data;
    struct si570_transfer   trans;

    if (0 != sxpf_plasma_writeRegister(pf, (channel * 4) + 0x00000020, 0x02))
    {
        fprintf(stderr, "Error writing oscillator Port-Enable register\n");
        return -1;
    }

    data.fout = factoryFreq;

    if (0 != si570_read_startup_configuration(pf, channel, &data))
    {
        fprintf(stderr, "Error reading oscillator startup configuration\n");
        return -2;
    }

    si570_get_trans(&data, &trans);

    *freq = data.fout;

    return 0;
}


int sxpf_osc_modify_adapter(sxpf_hdl pf, uint8_t channel, double newFreq,
                            uint32_t factoryFreq)
{
    int ret = 0;
    struct si570_data data;
    struct si570_transfer trans;

    if (newFreq < 10e6 || newFreq > 160e6)
    {
        fprintf(stderr,
                "Requested oscillator frequency of %.3fMHz is out of range!\n",
                newFreq / 1e6);
        return -1;
    }

    if (0 != sxpf_plasma_writeRegister(pf, (channel * 4) + 0x00000020, 0x02))
    {
        fprintf(stderr, "Error writing oscillator Port-Enable register\n");
        return -1;
    }

    data.fout = factoryFreq;

    if (0 != si570_read_startup_configuration(pf, channel, &data))
    {
        fprintf(stderr, "Error reading oscillator startup configuration\n");
        return -2;
    }

    si570_get_trans(&data, &trans);

    uint8_t hsdiv_0 = ((trans.config[0] & 0xE0) >> 5) + 4;
    uint8_t n1_0 =
        ((trans.config[0] & 0x1F) << 2) + ((trans.config[1] & 0xC0 ) >> 6);

    if (n1_0 == 0)
    {
        n1_0 = 1;
    }
    else if ((n1_0 & 1) != 0)
    {
        n1_0++;
    }

    uint64_t rfreq_ = trans.config[1] & 0x3F;
    rfreq_ = (rfreq_ << 8) + trans.config[2];
    rfreq_ = (rfreq_ << 8) + trans.config[3];
    rfreq_ = (rfreq_ << 8) + trans.config[4];
    rfreq_ = (rfreq_ << 8) + trans.config[5];

    // calc new parameters
    uint8_t n1 = n1_0;
    uint8_t hsdiv = hsdiv_0;
    double ratio = newFreq / trans.fout;
    double fdco_new = newFreq * n1 * hsdiv;

#if USE_SMALL_CHANGES
    // check if change < 3500 ppm
    if (((fabs(newFreq - trans.fout) / trans.fout) < (3500.0 / 1000000.0)) &&
        (fdco_new >= FDCO_MIN) && (fdco_new <= FDCO_MAX))
    {
        trans.smallchange = 1;
    }
    else
#endif
    {
        // Big change: calc new n1, hsdiv
        trans.smallchange = 0;

        if (si570_calc_divider((uint32_t)newFreq, &n1, &hsdiv) < 0)
        {
            fprintf(stderr, "Could not calculate new parameter");
            return -1;
        }
        else
        {
            fdco_new = newFreq * n1 * hsdiv;
            if ((fdco_new < FDCO_MIN) || (fdco_new > FDCO_MAX))
            {
                fprintf(stderr, "fdco_new out of valid range");
                return -1;
            }

            // Calculate RFREQ organizing the float variables to save precision;
            // RFREQ is kept as an unsigned long
            // only 32 bits are available in the long format
            // RFREQ in the device has 34 bits of precision
            // only 34 of the 38 bits are needed since RFREQ is between 42.0 and
            // 50.0 for fxtal of 114.285MHz (nominal)

            // Try to keep ration near 1
            // to maintain precision
            ratio = ratio * (((double)n1)/((double)n1_0));
            ratio = ratio * (((double)hsdiv)/((double)hsdiv_0));

        }

    }
    uint64_t final_rfreq_long = uint64_t(ratio * rfreq_);  // Calculate final RFREQ
    // value using ratio
    // computed above

    // transmit new values
    for(int i = 0; i < 6; i++)
    {
        trans.config[i] = 0;                //clear registers
    }

    hsdiv = hsdiv - 4;  // Subtract 4 because of the offset of HS_DIV.
    // Ex: "000" maps to 4, "001" maps to 5

    //set the top 3 bits of REG[0] which will correspond to Register 7 on Si57x
    trans.config[0] = (hsdiv << 5);


    // convert new N1 to the binary representation
    if(n1 == 1)
    {
        n1 = 0;  //Corner case for N1. If N1=1, it is represented as "00000000"
    }
    else if((n1 & 1) == 0)
    {
        n1 = n1 - 1; // If n1 is even, round down to closest odd number. See the
    }                // Si57x datasheet for more information.

    // Write correct new values to REG[0] through REG[6]
    // These will be sent to the Si57x and will update the output frequency
    trans.config[0] = SetBits(trans.config[0], 0xE0, (n1 >> 2));// N1 of REG[0]
    trans.config[1] = (n1 & 3) << 6;                   // Set N1 part of REG[1]
    //Write new version of RFREQ to corresponding registers
    trans.config[1] = trans.config[1] | ((final_rfreq_long >> 32) & 0xff);
    trans.config[2] = (final_rfreq_long >> 24) & 0xff;
    trans.config[3] = (final_rfreq_long >> 16) & 0xff;
    trans.config[4] = (final_rfreq_long >> 8) & 0xff;
    trans.config[5] = (final_rfreq_long) & 0xff;

    trans.fout = (uint32_t)newFreq;

    if (0 != si570_write_trans_adapter(pf, channel, &data, &trans))
    {
        fprintf(stderr, "\nError writing configuration\n\n");
        return -2;
    }

    return ret;
}


/* from SI reference implementation AN334 */
int si570_read_startup_configuration(sxpf_hdl pf, uint8_t channel,
                                     struct si570_data *data)
{
    int ret = 0;

    /* Recall NVM bits into RAM */
#define RECALL_FACTORY 1
#if RECALL_FACTORY
    uint8_t cmdFactoryReset[2];
    cmdFactoryReset[0] = SI570_CTRL;
    cmdFactoryReset[1] = 0x01;
    ret = sxpf_i2c_xfer(pf, channel, SI570_ID, cmdFactoryReset, 2, NULL, 0);
    if (ret != 0)
    {
        return ret;
    }
#endif

    /* read register 7 to 12 */
    uint8_t cmdReadParams[1];
    cmdReadParams[0] = SI570_CONFIG_START;
    ret = sxpf_i2c_xfer(pf, channel, SI570_ID, cmdReadParams, 1,
                        data->config_regs, 6);
    if (ret != 0)
    {
        return ret;
    }

    data->factory.hsdiv = ((data->config_regs[0] & 0xE0) >> 5) + 4;
    data->factory.n1 =
        ((data->config_regs[0] & 0x1F) << 2) |
        ((data->config_regs[1] & 0xC0) >> 6);

    /* handle 0 and illegal odd values */
    if (data->factory.n1 == 0)
    {
        data->factory.n1 = 1;
    }
    else if ((data->factory.n1 & 1) != 0)
    {
        data->factory.n1++;
    }

    data->factory.rfreq = data->config_regs[1] & 0x3F;
    data->factory.rfreq = (data->factory.rfreq << 8) + data->config_regs[2];
    data->factory.rfreq = (data->factory.rfreq << 8) + data->config_regs[3];
    data->factory.rfreq = (data->factory.rfreq << 8) + data->config_regs[4];
    data->factory.rfreq = (data->factory.rfreq << 8) + data->config_regs[5];

    return ret;
}


void si570_get_trans(struct si570_data *data, struct si570_transfer *trans)
{
    trans->fout = data->fout;
    for (int i = 0; i < 6; i++)
    {
        trans->config[i] = data->config_regs[i];
    }
}


int si570_calc_divider(uint32_t fout, uint8_t* n1, uint8_t* hsdiv)
{
    const uint16_t divider_max = (uint16_t)(floor(FDCO_MAX / (double)fout));
    uint16_t curr_div = (uint16_t)(ceil(FDCO_MIN / (double)fout));
    int validCombo = 0;
    int ret = 0;

    while (curr_div <= divider_max)
    {
        for (unsigned i = 0; i < 6 /*(sizeof(HS_DIV)/sizeof(*HS_DIV))*/; i++)
        {
            // get the next possible n1 value
            *hsdiv = HS_DIV[i];
            double curr_n1 = (double)curr_div / (double)(*hsdiv);

            // Determine if curr_n1 is an integer and an even number or one
            // then it will be a valid divider option for the new frequency
            double n1_tmp = floor(curr_n1);
            n1_tmp = curr_n1 - n1_tmp;
            if (n1_tmp == 0.0)  // Then curr_n1 is an integer
            {
                *n1 = (uint8_t)curr_n1;

                if ((*n1 == 1) || ((*n1 & 1) == 0))// Then the calculated N1 is
                {                                  // either 1 or an even number
                    validCombo = 1;
                }
            }
            if (validCombo)
                break;      // Divider was found, exit loop
        }
        if (validCombo)
            break;          // Divider was found, exit loop

        // If a valid divider is not found, increment curr_div and loop
        curr_div++;
    }

    // If validCombo == 0 at this point, then there is an error
    // in the calculation. Check if the provided FOUT0 and FOUT1
    // are valid frequencies
    if (!validCombo)
    {
        fprintf(stderr,
                "Error in calculation. Check if provider fout is valid\n");
        ret = -1;
    }

    return ret;
}


uint8_t SetBits(uint8_t original, uint8_t reset_mask, uint8_t new_val)
{
    return (( original & reset_mask ) | new_val );
}


int si570_write_trans_adapter(sxpf_hdl pf, uint8_t channel,
                              struct si570_data *data,
                              struct si570_transfer *ptrans)
{
    int i;
    int ret = 0;

    uint8_t writeData[10];
    uint8_t readData[10];

    if (ptrans->smallchange) // change << 3500 ppm
    {
        writeData[0] = SI570_CTRL;
        writeData[1] = 1 << 5;
        ret = sxpf_i2c_xfer(pf, channel, SI570_ID, writeData, 2, NULL, 0);
        if (ret < 0)
        {
            goto exit;
        }

        writeData[0] = SI570_CONFIG_START;
        for (i = 0; i < 6; ++i)
        {
            writeData[i+1] = ptrans->config[i];
            data->config_regs[i] = ptrans->config[i];
        }
        ret = sxpf_i2c_xfer(pf, channel, SI570_ID, writeData, 7, NULL, 0);
        if (ret < 0)
        {
            goto exit;
        }

        writeData[0] = SI570_CTRL;
        writeData[1] = 0;
        ret = sxpf_i2c_xfer(pf, channel, SI570_ID, writeData, 2, NULL, 0);
        if (ret < 0)
        {
            goto exit;
        }
    }
    else
    {
        int32_t reg137;

        writeData[0] = SI570_CTRL2;
        ret = sxpf_i2c_xfer(pf, channel, SI570_ID, writeData, 1, readData, 1);
        if (ret < 0)
        {
            fprintf(stderr, "error read reg137\n");
            goto exit;
        }
        reg137 = readData[0];

        usleep(5000);

        // freeze
        writeData[0] = SI570_CTRL2;
        writeData[1] = reg137 | (1 << 4);
        ret = sxpf_i2c_xfer(pf, channel, SI570_ID, writeData, 2, NULL, 0);
        if (ret < 0)
        {
            fprintf(stderr, "error freeze\n");
            goto exit;
        }

        usleep(5000);

        writeData[0] = SI570_CONFIG_START;
        for (i = 0; i < 6; ++i)
        {
            writeData[i + 1] = ptrans->config[i];
            data->config_regs[i] = ptrans->config[i];
        }
        ret = sxpf_i2c_xfer(pf, channel, SI570_ID, writeData, 7, NULL, 0);
        if (ret < 0)
        {
            fprintf(stderr, "error write data\n");
            goto exit;
        }

        usleep(1000);

        writeData[0] = SI570_CTRL2;
        ret = sxpf_i2c_xfer(pf, channel, SI570_ID, writeData, 1, readData, 1);
        if (ret < 0)
        {
            fprintf(stderr, "error read reg137\n");
            goto exit;
        }
        reg137 = readData[0];

        writeData[0] = SI570_CTRL2;
        writeData[1] = reg137 & ~(1 << 4);
        ret = sxpf_i2c_xfer(pf, channel, SI570_ID, writeData, 2, NULL, 0);
        if (ret < 0)
        {
            fprintf(stderr, "error write reg137\n");
            goto exit;
        }

        usleep(1000);
        // alert NewFreq
        writeData[0] = SI570_CTRL;
        writeData[1] = 0x40;
        ret = sxpf_i2c_xfer(pf, channel, SI570_ID, writeData, 2, NULL, 0);
    }

    if (ret >= 0)
    {
        for (i = 0; i < 6; ++i)
        {
            data->config_regs[i] = ptrans->config[i];
        }

        data->fout = ptrans->fout;
    }
exit:
    return ret;
}

