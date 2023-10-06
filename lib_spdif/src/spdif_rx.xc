// Copyright 2014-2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#include <xs1.h>
#include <stdint.h>
#include <stddef.h>
#include <xclib.h>
#include <stdio.h>

#include "spdif.h"

void spdif_receive_sample(streaming chanend c, int32_t &sample, size_t &index)
{
    uint32_t v;
    c :> v;
    index = (v & SPDIF_RX_PREAMBLE_MASK) == SPDIF_FRAME_Y ? 1 : 0;
    sample = SPDIF_RX_EXTRACT_SAMPLE(v);
}

#if (LEGACY_SPDIF_RECEIVER)

void SpdifReceive(in buffered port:4 p, streaming chanend c, int initial_divider, clock clk);

void spdif_rx(streaming chanend c, in port p, clock clk, unsigned sample_freq_estimate)
{
    int initial_divider;
    in port * movable pp = &p;
    in buffered port:4 * movable p_buf = reconfigure_port(move(pp), in buffered port:4);
    if (sample_freq_estimate > 96000)
    {
        initial_divider = 1;
    }
    else if (sample_freq_estimate > 48000)
    {
        initial_divider = 2;
    }
    else
    {
        initial_divider = 4;
    }

    SpdifReceive(*p_buf, c, initial_divider, clk);

    // Set pointers and ownership back to original state if SpdifReceive() exits
    pp = reconfigure_port(move(p_buf), in port);
}

void spdif_receive_shutdown(streaming chanend c)
{
    soutct (c, XS1_CT_END);
}

#else

void spdif_rx_441(streaming chanend c, buffered in port:32 p);
void spdif_rx_441_2X(streaming chanend c, buffered in port:32 p);
void spdif_rx_48(streaming chanend c, buffered in port:32 p);
void spdif_rx_48_2X(streaming chanend c, buffered in port:32 p);
int check_clock_div(buffered in port:32 p, unsigned decode_2x);

void spdif_rx(streaming chanend c, in port p, clock clk, unsigned sample_freq_estimate)
{
    unsigned sample_rate = sample_freq_estimate;
    unsigned clock_div, decode_2x, next_sr;

    in port * movable pp = &p;
    in buffered port:32 * movable p_buf = reconfigure_port(move(pp), in buffered port:32);

    // Configure spdif rx port to be clocked from spdif_rx clock defined below.
    configure_in_port(*p_buf, clk);

    while(1)
    {
        // Determine operating mode and next sample rate to try from current sample rate.
        switch(sample_rate)
        {
            case 32000:  clock_div = 3; decode_2x = 0; next_sr = 44100;  break;
            case 44100:  clock_div = 1; decode_2x = 1; next_sr = 48000;  break;
            case 48000:  clock_div = 1; decode_2x = 1; next_sr = 88200;  break;
            case 88200:  clock_div = 0; decode_2x = 1; next_sr = 96000;  break;
            case 96000:  clock_div = 0; decode_2x = 1; next_sr = 176400; break;
            case 176400: clock_div = 0; decode_2x = 0; next_sr = 192000; break;
            case 192000: clock_div = 0; decode_2x = 0; next_sr = 32000;  break;
            default:     clock_div = 0; decode_2x = 0; next_sr = 48000;  break;
        }

        // Stop clock so we can reconfigure it
        stop_clock(clk);
        // Set the desired clock div
        configure_clock_ref(clk, clock_div);
        // Start the clock block running. Port timer will be reset here.
        start_clock(clk);

        if (check_clock_div(*p_buf, decode_2x) == 0) // Check the input signal is roughly matched to the decode rate specified
        {
            if(sample_rate % 44100)
            {
                if (decode_2x)
                    spdif_rx_48_2X(c, *p_buf);
                else
                    spdif_rx_48(c, *p_buf);
            }
            else
            {
                if (decode_2x)
                    spdif_rx_441_2X(c, *p_buf);
                else
                    spdif_rx_441(c, *p_buf);
            }
        }

        // Update sample rate
        sample_rate = next_sr;
    }

    // Set pointers and ownership back to original state if SpdifReceive() exits (currently unreachable)
    pp = reconfigure_port(move(p_buf), in port);
}

#endif

