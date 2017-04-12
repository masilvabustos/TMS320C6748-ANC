/**
 * \file  aic31.h
 *
 * \brief The macro definitions and function prototypes for
 *        configuring AIC31 codec
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _AIC31_H_
#define _AIC31_H_

/**************************************************************************
**                       Macro Definitions
**************************************************************************/
/*
** Macros for the dataType variable to pass to AIC31DataConfig function
*/
#define AIC31_DATATYPE_I2S             (0u << 6u) /* I2S Mode */
#define AIC31_DATATYPE_DSP             (1u << 6u) /* DSP Mode */
#define AIC31_DATATYPE_RIGHTJ          (2u << 6u) /* Right Aligned Mode */
#define AIC31_DATATYPE_LEFTJ           (3u << 6u) /* Left Aligned Mode */

/*
** Macros for the mode variable for the AIC31SampleRateConfig function
*/
#define AIC31_MODE_ADC                 (0xF0u)
#define AIC31_MODE_DAC                 (0x0Fu)
#define AIC31_MODE_BOTH                (0xFFu)

/**************************************************************************
**                      API function Prototypes
**************************************************************************/
extern void AIC31Reset(unsigned int baseAddr);
extern void AIC31DataConfig(unsigned int baseAddr, unsigned char dataType,
                            unsigned char slotWidth, unsigned char dataOff);
extern void AIC31SampleRateConfig(unsigned int baseAddr, unsigned int mode,
                                  unsigned int sampleRate);
extern void AIC31ADCInit(unsigned int baseAddr);
extern void AIC31DACInit(unsigned int baseAddr);

#endif
