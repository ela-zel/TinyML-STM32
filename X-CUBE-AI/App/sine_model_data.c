/**
******************************************************************************
* @file    sine_model_data.c
* @author  AST Embedded Analytics Research Platform
* @date    Thu May 27 21:11:39 2021
* @brief   AI Tool Automatic Code Generator for Embedded NN computing
******************************************************************************
* @attention
*
* Copyright (c) 2021 STMicroelectronics.
* All rights reserved.
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/

#include "sine_model_data.h"

ai_handle ai_sine_model_data_weights_get(void)
{
  AI_ALIGNED(32)
  static const ai_u8 s_sine_model_weights[1284] = {
    0x47, 0x92, 0xba, 0x3d, 0xf9, 0x4b, 0x71, 0x3e, 0xe4, 0x89,
    0x8b, 0x3e, 0xc0, 0x3a, 0x9a, 0x3e, 0x62, 0x15, 0x8f, 0xbe,
    0xea, 0x7e, 0xcc, 0xbe, 0xfc, 0xe9, 0xa7, 0xbe, 0x4a, 0x24,
    0x2b, 0xbe, 0xae, 0xb5, 0xc7, 0x3e, 0x6f, 0x09, 0xe0, 0x3e,
    0x57, 0xd4, 0xf1, 0x3e, 0xa6, 0x27, 0x19, 0x3e, 0x48, 0xd4,
    0x06, 0xbe, 0x82, 0xbf, 0x8a, 0x3e, 0xe1, 0x21, 0x15, 0xbe,
    0xb6, 0xf4, 0x22, 0xbe, 0x0a, 0x35, 0x24, 0xbe, 0x8b, 0xde,
    0x30, 0xbf, 0x7c, 0x75, 0x1b, 0xbf, 0xeb, 0xd2, 0xd7, 0xbe,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x58, 0x5f, 0xbf,
    0xdf, 0x6a, 0x0e, 0x3e, 0x54, 0x95, 0x91, 0xbe, 0x75, 0x5e,
    0xd5, 0xbe, 0x00, 0x00, 0x00, 0x00, 0x6a, 0x52, 0x42, 0x3f,
    0x69, 0x1c, 0x29, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x9b, 0x03,
    0xbb, 0x3d, 0x63, 0xc3, 0xd9, 0xbe, 0x43, 0x34, 0x80, 0x3d,
    0x5c, 0x2a, 0x13, 0xbe, 0x98, 0x8d, 0x7e, 0xbd, 0xcc, 0xd0,
    0x67, 0xbe, 0xca, 0x99, 0x7c, 0xbe, 0xa6, 0x16, 0xb7, 0xbe,
    0x2c, 0x29, 0x05, 0xbe, 0x16, 0xef, 0x3f, 0xbf, 0x87, 0xb3,
    0x02, 0xbf, 0x09, 0x1f, 0xaa, 0x3e, 0x59, 0xf4, 0xbe, 0x3e,
    0xff, 0x24, 0xcf, 0x3e, 0x70, 0xfd, 0x0b, 0x3f, 0xde, 0x85,
    0x46, 0x3e, 0xc0, 0x45, 0xcd, 0xbc, 0x67, 0x4e, 0x71, 0xbf,
    0x27, 0xa5, 0xa3, 0xbe, 0x3e, 0x54, 0x86, 0x3d, 0xca, 0x5d,
    0x1a, 0x3e, 0x2f, 0xa1, 0xc4, 0xbe, 0xf4, 0x7c, 0xe8, 0x3d,
    0x90, 0x98, 0xcb, 0xbe, 0x67, 0x31, 0x07, 0xbe, 0x5b, 0x0e,
    0x91, 0xbb, 0xb7, 0x64, 0xce, 0xbe, 0x8d, 0xae, 0x5a, 0xbd,
    0x72, 0x2d, 0x5c, 0x3e, 0x91, 0x78, 0xec, 0x3e, 0x40, 0x81,
    0x4f, 0xbe, 0x9a, 0xf5, 0x0f, 0xbe, 0x2e, 0x7d, 0x0c, 0x3e,
    0xd1, 0x85, 0x1a, 0xbf, 0x79, 0xb2, 0xdb, 0xbe, 0xe8, 0x59,
    0xd7, 0xbe, 0xca, 0xce, 0x71, 0x3e, 0x49, 0xe3, 0xdb, 0x3e,
    0x78, 0xd1, 0xb6, 0xbd, 0xe8, 0x81, 0x72, 0xbd, 0x1a, 0x53,
    0x63, 0xbe, 0x5a, 0xa2, 0xa4, 0x3e, 0xc8, 0xd6, 0xe5, 0xbe,
    0x76, 0x76, 0x6b, 0xbc, 0x70, 0x61, 0x89, 0xbe, 0x07, 0x5d,
    0xe0, 0x3e, 0x78, 0x72, 0xa6, 0x3e, 0x49, 0x50, 0xcf, 0x3e,
    0x97, 0xe1, 0x5c, 0x3e, 0x40, 0x61, 0xcc, 0xbd, 0x3e, 0x97,
    0xab, 0x3e, 0x40, 0x84, 0xc4, 0x3b, 0xc7, 0x87, 0x87, 0xbe,
    0x6e, 0x37, 0x0b, 0x3e, 0x21, 0xf6, 0xc0, 0x3e, 0xac, 0xf6,
    0x27, 0xbe, 0x45, 0xe7, 0x8d, 0xbe, 0x5e, 0x59, 0xb6, 0xbe,
    0x43, 0xae, 0x82, 0x3e, 0x37, 0x30, 0x87, 0xbe, 0x1c, 0xa4,
    0xb6, 0xbd, 0x9e, 0xc8, 0x6f, 0xbd, 0x08, 0x19, 0xa6, 0x3d,
    0xd6, 0x07, 0x84, 0xbe, 0x2d, 0xef, 0x96, 0x3e, 0x93, 0xc8,
    0xc3, 0xbe, 0x6f, 0x93, 0xde, 0xbe, 0xba, 0xfe, 0xfa, 0x3d,
    0x90, 0x58, 0x8c, 0x3c, 0x1d, 0x20, 0xc0, 0xbe, 0x0e, 0xe4,
    0x78, 0x3e, 0xe7, 0x87, 0xa6, 0x3e, 0x63, 0x8e, 0xed, 0xbe,
    0x37, 0x5a, 0xb9, 0x3d, 0x7d, 0x76, 0xc2, 0x3e, 0x14, 0xf1,
    0xf9, 0x3d, 0x60, 0xe6, 0x67, 0xbd, 0xca, 0x02, 0xc0, 0x3e,
    0x1d, 0x92, 0x15, 0x3f, 0x24, 0xd0, 0xba, 0x3d, 0xa2, 0x40,
    0x47, 0x3e, 0x4b, 0xe4, 0xfd, 0x3e, 0x22, 0xb9, 0xc3, 0x3e,
    0xab, 0xdd, 0x8d, 0xbe, 0x99, 0x6d, 0x92, 0xbe, 0xd0, 0x13,
    0xec, 0xbc, 0x22, 0x34, 0xbf, 0xbe, 0xaa, 0xb8, 0x18, 0x3e,
    0x0e, 0x78, 0xea, 0x3e, 0x96, 0xfd, 0x47, 0xbe, 0xb8, 0x89,
    0xc3, 0x3e, 0x8c, 0x9e, 0xc2, 0x3e, 0x51, 0xb3, 0xd8, 0x3e,
    0x6e, 0xad, 0xcc, 0xbe, 0x6e, 0x05, 0xba, 0xbc, 0x60, 0x43,
    0x8a, 0xbe, 0xfc, 0x02, 0xb2, 0xbe, 0xea, 0x87, 0x19, 0x3e,
    0xa6, 0xb4, 0x4d, 0xbe, 0xa2, 0x25, 0xd7, 0xbe, 0x80, 0xe9,
    0x49, 0x3b, 0xca, 0x7d, 0x9b, 0xbe, 0xac, 0x5e, 0xb6, 0xbe,
    0x89, 0x04, 0x55, 0xbe, 0xd0, 0xb0, 0x97, 0x3c, 0x8a, 0x05,
    0x48, 0x3e, 0x1c, 0xac, 0x5c, 0xbe, 0x00, 0x63, 0xc8, 0x3c,
    0xc5, 0x2a, 0xcc, 0xbe, 0x81, 0xe4, 0x7a, 0xbe, 0xbe, 0xd4,
    0xd4, 0xbe, 0x58, 0x67, 0xd1, 0xbd, 0x35, 0xe9, 0x44, 0xbe,
    0xec, 0x1d, 0x08, 0xbe, 0x8f, 0x46, 0x36, 0xbe, 0xdc, 0xc0,
    0x00, 0xbf, 0xe2, 0xd3, 0x46, 0x3e, 0x54, 0x82, 0x07, 0xbe,
    0x56, 0x8e, 0x74, 0x3e, 0x30, 0x1e, 0x4a, 0x3d, 0xe4, 0x23,
    0x9c, 0x3d, 0xdb, 0xcb, 0x0c, 0x3d, 0x97, 0xe8, 0x90, 0x3e,
    0x88, 0x60, 0xe0, 0x3e, 0xae, 0x64, 0x09, 0x3e, 0xb6, 0x6d,
    0xa1, 0xbc, 0xcc, 0xc6, 0x42, 0xbd, 0x5b, 0x93, 0xa2, 0x3e,
    0xa3, 0x37, 0x8d, 0x3e, 0x2b, 0xc2, 0xc0, 0xbe, 0xda, 0x26,
    0x14, 0x3e, 0x9c, 0xcb, 0x08, 0x3e, 0x22, 0x2e, 0x6f, 0x3e,
    0x56, 0xd9, 0x9a, 0xbe, 0xa4, 0x96, 0xed, 0x3d, 0xa0, 0x1d,
    0x48, 0xbd, 0xf3, 0x42, 0x84, 0x3d, 0xc4, 0x08, 0x5b, 0xbf,
    0xc0, 0x71, 0x11, 0xbe, 0x60, 0x33, 0xdc, 0xbe, 0x0c, 0x07,
    0xae, 0xbd, 0x29, 0xdd, 0x2e, 0x3d, 0xdd, 0x0f, 0xe8, 0x3e,
    0xf7, 0xe4, 0x4c, 0xbe, 0x6e, 0xbc, 0x6f, 0x3e, 0x19, 0x2b,
    0xab, 0xbe, 0x74, 0xac, 0xc5, 0x3d, 0x50, 0x46, 0x52, 0x3d,
    0xed, 0xf0, 0xa2, 0xbe, 0x7e, 0xb8, 0x12, 0xbe, 0xe8, 0xa7,
    0x88, 0x3d, 0xf2, 0xa1, 0x20, 0xbe, 0xbf, 0xd3, 0x74, 0xbe,
    0xcf, 0x53, 0x41, 0xbe, 0x88, 0x29, 0x38, 0xbe, 0xc5, 0xd6,
    0xba, 0x3e, 0xba, 0x29, 0xa5, 0xbe, 0xf4, 0x76, 0x81, 0xbe,
    0x48, 0x79, 0x93, 0xbe, 0xb0, 0xf2, 0x04, 0xbe, 0x93, 0x75,
    0x2b, 0xbe, 0xc6, 0x29, 0xe5, 0xbe, 0x4f, 0xb3, 0x85, 0xbe,
    0x4d, 0x59, 0xb5, 0xbd, 0x7f, 0x3c, 0xa2, 0x3e, 0x34, 0x5a,
    0xc3, 0x3d, 0x90, 0xc1, 0xe1, 0xbc, 0xdb, 0x43, 0xaa, 0xbe,
    0xa4, 0xf9, 0x81, 0xbe, 0x3c, 0x84, 0x77, 0x3e, 0xc5, 0xdc,
    0x88, 0x3e, 0x64, 0x44, 0xb3, 0x3d, 0x62, 0x21, 0x1e, 0xbe,
    0xf4, 0x80, 0xab, 0x3d, 0x39, 0x84, 0x28, 0xbd, 0xbd, 0x3a,
    0xa6, 0x3e, 0x90, 0xf6, 0x1f, 0x3e, 0x67, 0x49, 0x00, 0xbf,
    0x0c, 0x68, 0x5f, 0xbe, 0x92, 0x8e, 0x0d, 0xbf, 0x9e, 0x15,
    0x80, 0xbe, 0xe6, 0xdd, 0x1b, 0x3e, 0x62, 0x7d, 0xdc, 0xbe,
    0xcb, 0x98, 0xa9, 0x3e, 0xe6, 0x90, 0x19, 0xbf, 0xd7, 0x0a,
    0x56, 0xbe, 0x01, 0x9a, 0x63, 0x3e, 0x62, 0x00, 0x68, 0x3e,
    0x50, 0x46, 0xb9, 0xbd, 0x3e, 0x9c, 0xa4, 0x3e, 0x77, 0xc6,
    0x00, 0x3c, 0x60, 0x33, 0x14, 0xbd, 0xb2, 0x02, 0x33, 0x3e,
    0x5c, 0x76, 0x12, 0xbf, 0xf2, 0x96, 0xe0, 0xbd, 0x51, 0xe8,
    0xbf, 0xbe, 0x06, 0x41, 0x3b, 0x3e, 0xc8, 0xf5, 0xe1, 0xbd,
    0x5d, 0xeb, 0xad, 0x3e, 0x4e, 0x76, 0x76, 0x3e, 0xf2, 0xdd,
    0xed, 0x3d, 0x53, 0xc5, 0xd2, 0x3c, 0xbc, 0xc0, 0x50, 0x3e,
    0x18, 0xdf, 0x6f, 0xbe, 0xc4, 0x96, 0x0d, 0xbe, 0x78, 0xb7,
    0xd7, 0x3e, 0x6c, 0x81, 0xa4, 0x3e, 0x70, 0x60, 0x98, 0xbe,
    0x3f, 0xce, 0x11, 0xbd, 0x86, 0x0b, 0x1d, 0x3f, 0x31, 0x77,
    0xf9, 0x3e, 0x9b, 0x83, 0xb5, 0x3d, 0xe3, 0x3f, 0xa9, 0xbe,
    0xcc, 0xc4, 0xea, 0x3d, 0x00, 0xce, 0x07, 0xba, 0xa4, 0xda,
    0x81, 0xbe, 0xd4, 0x70, 0xb4, 0xbb, 0xd4, 0xde, 0x85, 0x3e,
    0xa1, 0x05, 0x66, 0x3a, 0xc3, 0x6e, 0x21, 0x3b, 0x8c, 0xe4,
    0x92, 0xbd, 0x7f, 0x90, 0xe6, 0xbe, 0xd0, 0xe1, 0x08, 0xbf,
    0x83, 0x34, 0xbc, 0xbe, 0xcd, 0xc0, 0x37, 0x3e, 0xc0, 0xbe,
    0x77, 0x3e, 0x24, 0xda, 0x68, 0x3e, 0xf1, 0x85, 0xc2, 0x3e,
    0x57, 0xe2, 0xd9, 0x3e, 0x52, 0x7e, 0x3b, 0x3e, 0x81, 0xdb,
    0x8d, 0x3e, 0x1f, 0x5d, 0xad, 0xbe, 0x88, 0x78, 0xe6, 0xbc,
    0x30, 0x9a, 0x10, 0x3d, 0xfa, 0x98, 0xcf, 0x3d, 0x5a, 0x00,
    0xf2, 0x3d, 0x68, 0x05, 0x32, 0x3d, 0xd6, 0xdc, 0x68, 0xbd,
    0x16, 0x45, 0x8e, 0xbe, 0x2f, 0xa3, 0xcb, 0x3e, 0x55, 0xc8,
    0x8e, 0x3e, 0x20, 0xfc, 0x7a, 0xbc, 0x21, 0x9b, 0x8c, 0x3e,
    0x14, 0x6b, 0x9e, 0xbe, 0x66, 0x05, 0x08, 0x3e, 0xfa, 0xc7,
    0x0f, 0xbe, 0x13, 0xb3, 0xd1, 0x3e, 0xea, 0x0e, 0x51, 0x3e,
    0x1c, 0xc4, 0xc1, 0xbe, 0x70, 0xb1, 0xd9, 0xbe, 0x8b, 0x41,
    0xaa, 0xbe, 0xce, 0xb4, 0x8e, 0xbe, 0x7c, 0xee, 0xd3, 0x3d,
    0x7d, 0x8b, 0x67, 0xbe, 0x0d, 0x06, 0x82, 0xbe, 0xb4, 0x4d,
    0x81, 0x3d, 0x6e, 0x9c, 0xd8, 0x3e, 0x16, 0xa9, 0xaa, 0x3e,
    0xe9, 0x87, 0xa3, 0x3e, 0x6d, 0x5e, 0x13, 0xbd, 0xf9, 0xa3,
    0xb3, 0x3e, 0x57, 0xe9, 0xfd, 0xbe, 0x00, 0x00, 0x00, 0x00,
    0x8c, 0x1b, 0x67, 0x3e, 0x4b, 0xd5, 0xd0, 0x3e, 0x00, 0x00,
    0x00, 0x00, 0x20, 0xe2, 0x7b, 0x3e, 0x10, 0xed, 0x93, 0x3e,
    0x35, 0x76, 0x86, 0x3e, 0x42, 0x85, 0xcd, 0xbe, 0xf2, 0x5e,
    0xba, 0xbe, 0x00, 0x00, 0x00, 0x00, 0x81, 0x46, 0x81, 0xbf,
    0x1e, 0x48, 0x79, 0x3f, 0x2e, 0x8e, 0x87, 0x3f, 0x91, 0x9e,
    0xc2, 0x3e, 0x28, 0x78, 0x9c, 0xbe, 0x6b, 0x3d, 0xba,
    0x3f, 0x6e, 0xf6, 0xce, 0xbe, 0x96, 0xe2, 0x25, 0xbe,
    0xbc, 0x6e, 0x30, 0xbf, 0x80, 0x16, 0xe2, 0xbe, 0x5d,
    0xdd, 0x8b, 0xbe, 0x80, 0x49, 0x53, 0x3f, 0x05, 0x90,
    0xd3, 0xbd, 0x51, 0xf5, 0x2b, 0x3f, 0x41, 0x70, 0xca,
    0x3e, 0x66, 0xc0, 0xe6, 0xbe, 0xf9, 0xce, 0xc5, 0xbe  };
  return AI_HANDLE_PTR(s_sine_model_weights);
}
