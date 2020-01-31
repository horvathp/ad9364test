// ==============================================================
// Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC v2019.1 (64-bit)
// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// CONFIG_A
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of version_info_V
//        bit 31~0 - version_info_V[31:0] (Read)
// 0x14 : Control signal of version_info_V
//        bit 0  - version_info_V_ap_vld (Read/COR)
//        others - reserved
// 0x18 : Data signal of td_gain_V
//        bit 7~0 - td_gain_V[7:0] (Read/Write)
//        others  - reserved
// 0x1c : reserved
// 0x20 : Data signal of windowing_V
//        bit 0  - windowing_V[0] (Read/Write)
//        others - reserved
// 0x24 : reserved
// 0x28 : Data signal of r_ap_rst_n_V
//        bit 0  - r_ap_rst_n_V[0] (Read/Write)
//        others - reserved
// 0x2c : reserved
// 0x30 : Data signal of r_ap_start_V
//        bit 0  - r_ap_start_V[0] (Read/Write)
//        others - reserved
// 0x34 : reserved
// 0x38 : Data signal of r_standalone_gen_V
//        bit 0  - r_standalone_gen_V[0] (Read/Write)
//        others - reserved
// 0x3c : reserved
// 0x40 : Data signal of r_standalone_frame_len_V
//        bit 15~0 - r_standalone_frame_len_V[15:0] (Read/Write)
//        others   - reserved
// 0x44 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XPHY_CONTROL_CONFIG_A_ADDR_VERSION_INFO_V_DATA           0x10
#define XPHY_CONTROL_CONFIG_A_BITS_VERSION_INFO_V_DATA           32
#define XPHY_CONTROL_CONFIG_A_ADDR_VERSION_INFO_V_CTRL           0x14
#define XPHY_CONTROL_CONFIG_A_ADDR_TD_GAIN_V_DATA                0x18
#define XPHY_CONTROL_CONFIG_A_BITS_TD_GAIN_V_DATA                8
#define XPHY_CONTROL_CONFIG_A_ADDR_WINDOWING_V_DATA              0x20
#define XPHY_CONTROL_CONFIG_A_BITS_WINDOWING_V_DATA              1
#define XPHY_CONTROL_CONFIG_A_ADDR_R_AP_RST_N_V_DATA             0x28
#define XPHY_CONTROL_CONFIG_A_BITS_R_AP_RST_N_V_DATA             1
#define XPHY_CONTROL_CONFIG_A_ADDR_R_AP_START_V_DATA             0x30
#define XPHY_CONTROL_CONFIG_A_BITS_R_AP_START_V_DATA             1
#define XPHY_CONTROL_CONFIG_A_ADDR_R_STANDALONE_GEN_V_DATA       0x38
#define XPHY_CONTROL_CONFIG_A_BITS_R_STANDALONE_GEN_V_DATA       1
#define XPHY_CONTROL_CONFIG_A_ADDR_R_STANDALONE_FRAME_LEN_V_DATA 0x40
#define XPHY_CONTROL_CONFIG_A_BITS_R_STANDALONE_FRAME_LEN_V_DATA 16

