/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "mdp.h"

/* mdp primary csc limit vector */
uint32 mdp_plv[] = { 0x10, 0xeb, 0x10, 0xf0 };

/* Color Coefficient matrix for YUV -> RGB */
struct mdp_ccs mdp_ccs_yuv2rgb = {
	MDP_CCS_YUV2RGB,
	{
		0x254,
		0x000,
		0x331,
		0x254,
		0xff38,
		0xfe61,
		0x254,
		0x409,
		0x000,
	},
	{
#ifdef CONFIG_FB_MSM_MDP31
		0x1f0,
		0x180,
		0x180
#else
		0x10,
		0x80,
		0x80
#endif
	}
};

/* Color Coefficient matrix for RGB -> YUV */
struct mdp_ccs mdp_ccs_rgb2yuv = {
	MDP_CCS_RGB2YUV,
	{
		0x83,
		0x102,
		0x32,
		0xffb5,
		0xff6c,
		0xe1,
		0xe1,
		0xff45,
		0xffdc,
	},
#ifdef CONFIG_FB_MSM_MDP31
	{
		0x10,
		0x80,
		0x80
	}
#endif
};

static void mdp_load_lut_param(void)
{
	outpdw(MDP_BASE + 0x40800, 0x0);
	outpdw(MDP_BASE + 0x40804, 0x151515);
	outpdw(MDP_BASE + 0x40808, 0x1d1d1d);
	outpdw(MDP_BASE + 0x4080c, 0x232323);
	outpdw(MDP_BASE + 0x40810, 0x272727);
	outpdw(MDP_BASE + 0x40814, 0x2b2b2b);
	outpdw(MDP_BASE + 0x40818, 0x2f2f2f);
	outpdw(MDP_BASE + 0x4081c, 0x333333);
	outpdw(MDP_BASE + 0x40820, 0x363636);
	outpdw(MDP_BASE + 0x40824, 0x393939);
	outpdw(MDP_BASE + 0x40828, 0x3b3b3b);
	outpdw(MDP_BASE + 0x4082c, 0x3e3e3e);
	outpdw(MDP_BASE + 0x40830, 0x404040);
	outpdw(MDP_BASE + 0x40834, 0x434343);
	outpdw(MDP_BASE + 0x40838, 0x454545);
	outpdw(MDP_BASE + 0x4083c, 0x474747);
	outpdw(MDP_BASE + 0x40840, 0x494949);
	outpdw(MDP_BASE + 0x40844, 0x4b4b4b);
	outpdw(MDP_BASE + 0x40848, 0x4d4d4d);
	outpdw(MDP_BASE + 0x4084c, 0x4f4f4f);
	outpdw(MDP_BASE + 0x40850, 0x515151);
	outpdw(MDP_BASE + 0x40854, 0x535353);
	outpdw(MDP_BASE + 0x40858, 0x555555);
	outpdw(MDP_BASE + 0x4085c, 0x565656);
	outpdw(MDP_BASE + 0x40860, 0x585858);
	outpdw(MDP_BASE + 0x40864, 0x5a5a5a);
	outpdw(MDP_BASE + 0x40868, 0x5b5b5b);
	outpdw(MDP_BASE + 0x4086c, 0x5d5d5d);
	outpdw(MDP_BASE + 0x40870, 0x5e5e5e);
	outpdw(MDP_BASE + 0x40874, 0x606060);
	outpdw(MDP_BASE + 0x40878, 0x616161);
	outpdw(MDP_BASE + 0x4087c, 0x636363);
	outpdw(MDP_BASE + 0x40880, 0x646464);
	outpdw(MDP_BASE + 0x40884, 0x666666);
	outpdw(MDP_BASE + 0x40888, 0x676767);
	outpdw(MDP_BASE + 0x4088c, 0x686868);
	outpdw(MDP_BASE + 0x40890, 0x6a6a6a);
	outpdw(MDP_BASE + 0x40894, 0x6b6b6b);
	outpdw(MDP_BASE + 0x40898, 0x6c6c6c);
	outpdw(MDP_BASE + 0x4089c, 0x6e6e6e);
	outpdw(MDP_BASE + 0x408a0, 0x6f6f6f);
	outpdw(MDP_BASE + 0x408a4, 0x707070);
	outpdw(MDP_BASE + 0x408a8, 0x717171);
	outpdw(MDP_BASE + 0x408ac, 0x727272);
	outpdw(MDP_BASE + 0x408b0, 0x747474);
	outpdw(MDP_BASE + 0x408b4, 0x757575);
	outpdw(MDP_BASE + 0x408b8, 0x767676);
	outpdw(MDP_BASE + 0x408bc, 0x777777);
	outpdw(MDP_BASE + 0x408c0, 0x787878);
	outpdw(MDP_BASE + 0x408c4, 0x797979);
	outpdw(MDP_BASE + 0x408c8, 0x7a7a7a);
	outpdw(MDP_BASE + 0x408cc, 0x7c7c7c);
	outpdw(MDP_BASE + 0x408d0, 0x7d7d7d);
	outpdw(MDP_BASE + 0x408d4, 0x7e7e7e);
	outpdw(MDP_BASE + 0x408d8, 0x7f7f7f);
	outpdw(MDP_BASE + 0x408dc, 0x808080);
	outpdw(MDP_BASE + 0x408e0, 0x818181);
	outpdw(MDP_BASE + 0x408e4, 0x828282);
	outpdw(MDP_BASE + 0x408e8, 0x838383);
	outpdw(MDP_BASE + 0x408ec, 0x848484);
	outpdw(MDP_BASE + 0x408f0, 0x858585);
	outpdw(MDP_BASE + 0x408f4, 0x868686);
	outpdw(MDP_BASE + 0x408f8, 0x878787);
	outpdw(MDP_BASE + 0x408fc, 0x888888);
	outpdw(MDP_BASE + 0x40900, 0x898989);
	outpdw(MDP_BASE + 0x40904, 0x8a8a8a);
	outpdw(MDP_BASE + 0x40908, 0x8b8b8b);
	outpdw(MDP_BASE + 0x4090c, 0x8c8c8c);
	outpdw(MDP_BASE + 0x40910, 0x8d8d8d);
	outpdw(MDP_BASE + 0x40914, 0x8e8e8e);
	outpdw(MDP_BASE + 0x40918, 0x8f8f8f);
	outpdw(MDP_BASE + 0x4091c, 0x8f8f8f);
	outpdw(MDP_BASE + 0x40920, 0x909090);
	outpdw(MDP_BASE + 0x40924, 0x919191);
	outpdw(MDP_BASE + 0x40928, 0x929292);
	outpdw(MDP_BASE + 0x4092c, 0x939393);
	outpdw(MDP_BASE + 0x40930, 0x949494);
	outpdw(MDP_BASE + 0x40934, 0x959595);
	outpdw(MDP_BASE + 0x40938, 0x969696);
	outpdw(MDP_BASE + 0x4093c, 0x969696);
	outpdw(MDP_BASE + 0x40940, 0x979797);
	outpdw(MDP_BASE + 0x40944, 0x989898);
	outpdw(MDP_BASE + 0x40948, 0x999999);
	outpdw(MDP_BASE + 0x4094c, 0x9a9a9a);
	outpdw(MDP_BASE + 0x40950, 0x9b9b9b);
	outpdw(MDP_BASE + 0x40954, 0x9c9c9c);
	outpdw(MDP_BASE + 0x40958, 0x9c9c9c);
	outpdw(MDP_BASE + 0x4095c, 0x9d9d9d);
	outpdw(MDP_BASE + 0x40960, 0x9e9e9e);
	outpdw(MDP_BASE + 0x40964, 0x9f9f9f);
	outpdw(MDP_BASE + 0x40968, 0xa0a0a0);
	outpdw(MDP_BASE + 0x4096c, 0xa0a0a0);
	outpdw(MDP_BASE + 0x40970, 0xa1a1a1);
	outpdw(MDP_BASE + 0x40974, 0xa2a2a2);
	outpdw(MDP_BASE + 0x40978, 0xa3a3a3);
	outpdw(MDP_BASE + 0x4097c, 0xa4a4a4);
	outpdw(MDP_BASE + 0x40980, 0xa4a4a4);
	outpdw(MDP_BASE + 0x40984, 0xa5a5a5);
	outpdw(MDP_BASE + 0x40988, 0xa6a6a6);
	outpdw(MDP_BASE + 0x4098c, 0xa7a7a7);
	outpdw(MDP_BASE + 0x40990, 0xa7a7a7);
	outpdw(MDP_BASE + 0x40994, 0xa8a8a8);
	outpdw(MDP_BASE + 0x40998, 0xa9a9a9);
	outpdw(MDP_BASE + 0x4099c, 0xaaaaaa);
	outpdw(MDP_BASE + 0x409a0, 0xaaaaaa);
	outpdw(MDP_BASE + 0x409a4, 0xababab);
	outpdw(MDP_BASE + 0x409a8, 0xacacac);
	outpdw(MDP_BASE + 0x409ac, 0xadadad);
	outpdw(MDP_BASE + 0x409b0, 0xadadad);
	outpdw(MDP_BASE + 0x409b4, 0xaeaeae);
	outpdw(MDP_BASE + 0x409b8, 0xafafaf);
	outpdw(MDP_BASE + 0x409bc, 0xafafaf);
	outpdw(MDP_BASE + 0x409c0, 0xb0b0b0);
	outpdw(MDP_BASE + 0x409c4, 0xb1b1b1);
	outpdw(MDP_BASE + 0x409c8, 0xb2b2b2);
	outpdw(MDP_BASE + 0x409cc, 0xb2b2b2);
	outpdw(MDP_BASE + 0x409d0, 0xb3b3b3);
	outpdw(MDP_BASE + 0x409d4, 0xb4b4b4);
	outpdw(MDP_BASE + 0x409d8, 0xb4b4b4);
	outpdw(MDP_BASE + 0x409dc, 0xb5b5b5);
	outpdw(MDP_BASE + 0x409e0, 0xb6b6b6);
	outpdw(MDP_BASE + 0x409e4, 0xb6b6b6);
	outpdw(MDP_BASE + 0x409e8, 0xb7b7b7);
	outpdw(MDP_BASE + 0x409ec, 0xb8b8b8);
	outpdw(MDP_BASE + 0x409f0, 0xb8b8b8);
	outpdw(MDP_BASE + 0x409f4, 0xb9b9b9);
	outpdw(MDP_BASE + 0x409f8, 0xbababa);
	outpdw(MDP_BASE + 0x409fc, 0xbababa);
	outpdw(MDP_BASE + 0x40a00, 0xbbbbbb);
	outpdw(MDP_BASE + 0x40a04, 0xbcbcbc);
	outpdw(MDP_BASE + 0x40a08, 0xbcbcbc);
	outpdw(MDP_BASE + 0x40a0c, 0xbdbdbd);
	outpdw(MDP_BASE + 0x40a10, 0xbebebe);
	outpdw(MDP_BASE + 0x40a14, 0xbebebe);
	outpdw(MDP_BASE + 0x40a18, 0xbfbfbf);
	outpdw(MDP_BASE + 0x40a1c, 0xc0c0c0);
	outpdw(MDP_BASE + 0x40a20, 0xc0c0c0);
	outpdw(MDP_BASE + 0x40a24, 0xc1c1c1);
	outpdw(MDP_BASE + 0x40a28, 0xc1c1c1);
	outpdw(MDP_BASE + 0x40a2c, 0xc2c2c2);
	outpdw(MDP_BASE + 0x40a30, 0xc3c3c3);
	outpdw(MDP_BASE + 0x40a34, 0xc3c3c3);
	outpdw(MDP_BASE + 0x40a38, 0xc4c4c4);
	outpdw(MDP_BASE + 0x40a3c, 0xc5c5c5);
	outpdw(MDP_BASE + 0x40a40, 0xc5c5c5);
	outpdw(MDP_BASE + 0x40a44, 0xc6c6c6);
	outpdw(MDP_BASE + 0x40a48, 0xc6c6c6);
	outpdw(MDP_BASE + 0x40a4c, 0xc7c7c7);
	outpdw(MDP_BASE + 0x40a50, 0xc8c8c8);
	outpdw(MDP_BASE + 0x40a54, 0xc8c8c8);
	outpdw(MDP_BASE + 0x40a58, 0xc9c9c9);
	outpdw(MDP_BASE + 0x40a5c, 0xc9c9c9);
	outpdw(MDP_BASE + 0x40a60, 0xcacaca);
	outpdw(MDP_BASE + 0x40a64, 0xcbcbcb);
	outpdw(MDP_BASE + 0x40a68, 0xcbcbcb);
	outpdw(MDP_BASE + 0x40a6c, 0xcccccc);
	outpdw(MDP_BASE + 0x40a70, 0xcccccc);
	outpdw(MDP_BASE + 0x40a74, 0xcdcdcd);
	outpdw(MDP_BASE + 0x40a78, 0xcecece);
	outpdw(MDP_BASE + 0x40a7c, 0xcecece);
	outpdw(MDP_BASE + 0x40a80, 0xcfcfcf);
	outpdw(MDP_BASE + 0x40a84, 0xcfcfcf);
	outpdw(MDP_BASE + 0x40a88, 0xd0d0d0);
	outpdw(MDP_BASE + 0x40a8c, 0xd0d0d0);
	outpdw(MDP_BASE + 0x40a90, 0xd1d1d1);
	outpdw(MDP_BASE + 0x40a94, 0xd2d2d2);
	outpdw(MDP_BASE + 0x40a98, 0xd2d2d2);
	outpdw(MDP_BASE + 0x40a9c, 0xd3d3d3);
	outpdw(MDP_BASE + 0x40aa0, 0xd3d3d3);
	outpdw(MDP_BASE + 0x40aa4, 0xd4d4d4);
	outpdw(MDP_BASE + 0x40aa8, 0xd4d4d4);
	outpdw(MDP_BASE + 0x40aac, 0xd5d5d5);
	outpdw(MDP_BASE + 0x40ab0, 0xd6d6d6);
	outpdw(MDP_BASE + 0x40ab4, 0xd6d6d6);
	outpdw(MDP_BASE + 0x40ab8, 0xd7d7d7);
	outpdw(MDP_BASE + 0x40abc, 0xd7d7d7);
	outpdw(MDP_BASE + 0x40ac0, 0xd8d8d8);
	outpdw(MDP_BASE + 0x40ac4, 0xd8d8d8);
	outpdw(MDP_BASE + 0x40ac8, 0xd9d9d9);
	outpdw(MDP_BASE + 0x40acc, 0xd9d9d9);
	outpdw(MDP_BASE + 0x40ad0, 0xdadada);
	outpdw(MDP_BASE + 0x40ad4, 0xdbdbdb);
	outpdw(MDP_BASE + 0x40ad8, 0xdbdbdb);
	outpdw(MDP_BASE + 0x40adc, 0xdcdcdc);
	outpdw(MDP_BASE + 0x40ae0, 0xdcdcdc);
	outpdw(MDP_BASE + 0x40ae4, 0xdddddd);
	outpdw(MDP_BASE + 0x40ae8, 0xdddddd);
	outpdw(MDP_BASE + 0x40aec, 0xdedede);
	outpdw(MDP_BASE + 0x40af0, 0xdedede);
	outpdw(MDP_BASE + 0x40af4, 0xdfdfdf);
	outpdw(MDP_BASE + 0x40af8, 0xdfdfdf);
	outpdw(MDP_BASE + 0x40afc, 0xe0e0e0);
	outpdw(MDP_BASE + 0x40b00, 0xe0e0e0);
	outpdw(MDP_BASE + 0x40b04, 0xe1e1e1);
	outpdw(MDP_BASE + 0x40b08, 0xe1e1e1);
	outpdw(MDP_BASE + 0x40b0c, 0xe2e2e2);
	outpdw(MDP_BASE + 0x40b10, 0xe3e3e3);
	outpdw(MDP_BASE + 0x40b14, 0xe3e3e3);
	outpdw(MDP_BASE + 0x40b18, 0xe4e4e4);
	outpdw(MDP_BASE + 0x40b1c, 0xe4e4e4);
	outpdw(MDP_BASE + 0x40b20, 0xe5e5e5);
	outpdw(MDP_BASE + 0x40b24, 0xe5e5e5);
	outpdw(MDP_BASE + 0x40b28, 0xe6e6e6);
	outpdw(MDP_BASE + 0x40b2c, 0xe6e6e6);
	outpdw(MDP_BASE + 0x40b30, 0xe7e7e7);
	outpdw(MDP_BASE + 0x40b34, 0xe7e7e7);
	outpdw(MDP_BASE + 0x40b38, 0xe8e8e8);
	outpdw(MDP_BASE + 0x40b3c, 0xe8e8e8);
	outpdw(MDP_BASE + 0x40b40, 0xe9e9e9);
	outpdw(MDP_BASE + 0x40b44, 0xe9e9e9);
	outpdw(MDP_BASE + 0x40b48, 0xeaeaea);
	outpdw(MDP_BASE + 0x40b4c, 0xeaeaea);
	outpdw(MDP_BASE + 0x40b50, 0xebebeb);
	outpdw(MDP_BASE + 0x40b54, 0xebebeb);
	outpdw(MDP_BASE + 0x40b58, 0xececec);
	outpdw(MDP_BASE + 0x40b5c, 0xececec);
	outpdw(MDP_BASE + 0x40b60, 0xededed);
	outpdw(MDP_BASE + 0x40b64, 0xededed);
	outpdw(MDP_BASE + 0x40b68, 0xeeeeee);
	outpdw(MDP_BASE + 0x40b6c, 0xeeeeee);
	outpdw(MDP_BASE + 0x40b70, 0xefefef);
	outpdw(MDP_BASE + 0x40b74, 0xefefef);
	outpdw(MDP_BASE + 0x40b78, 0xf0f0f0);
	outpdw(MDP_BASE + 0x40b7c, 0xf0f0f0);
	outpdw(MDP_BASE + 0x40b80, 0xf1f1f1);
	outpdw(MDP_BASE + 0x40b84, 0xf1f1f1);
	outpdw(MDP_BASE + 0x40b88, 0xf2f2f2);
	outpdw(MDP_BASE + 0x40b8c, 0xf2f2f2);
	outpdw(MDP_BASE + 0x40b90, 0xf2f2f2);
	outpdw(MDP_BASE + 0x40b94, 0xf3f3f3);
	outpdw(MDP_BASE + 0x40b98, 0xf3f3f3);
	outpdw(MDP_BASE + 0x40b9c, 0xf4f4f4);
	outpdw(MDP_BASE + 0x40ba0, 0xf4f4f4);
	outpdw(MDP_BASE + 0x40ba4, 0xf5f5f5);
	outpdw(MDP_BASE + 0x40ba8, 0xf5f5f5);
	outpdw(MDP_BASE + 0x40bac, 0xf6f6f6);
	outpdw(MDP_BASE + 0x40bb0, 0xf6f6f6);
	outpdw(MDP_BASE + 0x40bb4, 0xf7f7f7);
	outpdw(MDP_BASE + 0x40bb8, 0xf7f7f7);
	outpdw(MDP_BASE + 0x40bbc, 0xf8f8f8);
	outpdw(MDP_BASE + 0x40bc0, 0xf8f8f8);
	outpdw(MDP_BASE + 0x40bc4, 0xf9f9f9);
	outpdw(MDP_BASE + 0x40bc8, 0xf9f9f9);
	outpdw(MDP_BASE + 0x40bcc, 0xfafafa);
	outpdw(MDP_BASE + 0x40bd0, 0xfafafa);
	outpdw(MDP_BASE + 0x40bd4, 0xfafafa);
	outpdw(MDP_BASE + 0x40bd8, 0xfbfbfb);
	outpdw(MDP_BASE + 0x40bdc, 0xfbfbfb);
	outpdw(MDP_BASE + 0x40be0, 0xfcfcfc);
	outpdw(MDP_BASE + 0x40be4, 0xfcfcfc);
	outpdw(MDP_BASE + 0x40be8, 0xfdfdfd);
	outpdw(MDP_BASE + 0x40bec, 0xfdfdfd);
	outpdw(MDP_BASE + 0x40bf0, 0xfefefe);
	outpdw(MDP_BASE + 0x40bf4, 0xfefefe);
	outpdw(MDP_BASE + 0x40bf8, 0xffffff);
	outpdw(MDP_BASE + 0x40bfc, 0xffffff);
	outpdw(MDP_BASE + 0x40c00, 0x0);
	outpdw(MDP_BASE + 0x40c04, 0x0);
	outpdw(MDP_BASE + 0x40c08, 0x0);
	outpdw(MDP_BASE + 0x40c0c, 0x0);
	outpdw(MDP_BASE + 0x40c10, 0x0);
	outpdw(MDP_BASE + 0x40c14, 0x0);
	outpdw(MDP_BASE + 0x40c18, 0x0);
	outpdw(MDP_BASE + 0x40c1c, 0x0);
	outpdw(MDP_BASE + 0x40c20, 0x0);
	outpdw(MDP_BASE + 0x40c24, 0x0);
	outpdw(MDP_BASE + 0x40c28, 0x0);
	outpdw(MDP_BASE + 0x40c2c, 0x0);
	outpdw(MDP_BASE + 0x40c30, 0x0);
	outpdw(MDP_BASE + 0x40c34, 0x0);
	outpdw(MDP_BASE + 0x40c38, 0x0);
	outpdw(MDP_BASE + 0x40c3c, 0x0);
	outpdw(MDP_BASE + 0x40c40, 0x10101);
	outpdw(MDP_BASE + 0x40c44, 0x10101);
	outpdw(MDP_BASE + 0x40c48, 0x10101);
	outpdw(MDP_BASE + 0x40c4c, 0x10101);
	outpdw(MDP_BASE + 0x40c50, 0x10101);
	outpdw(MDP_BASE + 0x40c54, 0x10101);
	outpdw(MDP_BASE + 0x40c58, 0x10101);
	outpdw(MDP_BASE + 0x40c5c, 0x10101);
	outpdw(MDP_BASE + 0x40c60, 0x10101);
	outpdw(MDP_BASE + 0x40c64, 0x10101);
	outpdw(MDP_BASE + 0x40c68, 0x20202);
	outpdw(MDP_BASE + 0x40c6c, 0x20202);
	outpdw(MDP_BASE + 0x40c70, 0x20202);
	outpdw(MDP_BASE + 0x40c74, 0x20202);
	outpdw(MDP_BASE + 0x40c78, 0x20202);
	outpdw(MDP_BASE + 0x40c7c, 0x20202);
	outpdw(MDP_BASE + 0x40c80, 0x30303);
	outpdw(MDP_BASE + 0x40c84, 0x30303);
	outpdw(MDP_BASE + 0x40c88, 0x30303);
	outpdw(MDP_BASE + 0x40c8c, 0x30303);
	outpdw(MDP_BASE + 0x40c90, 0x30303);
	outpdw(MDP_BASE + 0x40c94, 0x40404);
	outpdw(MDP_BASE + 0x40c98, 0x40404);
	outpdw(MDP_BASE + 0x40c9c, 0x40404);
	outpdw(MDP_BASE + 0x40ca0, 0x40404);
	outpdw(MDP_BASE + 0x40ca4, 0x40404);
	outpdw(MDP_BASE + 0x40ca8, 0x50505);
	outpdw(MDP_BASE + 0x40cac, 0x50505);
	outpdw(MDP_BASE + 0x40cb0, 0x50505);
	outpdw(MDP_BASE + 0x40cb4, 0x50505);
	outpdw(MDP_BASE + 0x40cb8, 0x60606);
	outpdw(MDP_BASE + 0x40cbc, 0x60606);
	outpdw(MDP_BASE + 0x40cc0, 0x60606);
	outpdw(MDP_BASE + 0x40cc4, 0x70707);
	outpdw(MDP_BASE + 0x40cc8, 0x70707);
	outpdw(MDP_BASE + 0x40ccc, 0x70707);
	outpdw(MDP_BASE + 0x40cd0, 0x70707);
	outpdw(MDP_BASE + 0x40cd4, 0x80808);
	outpdw(MDP_BASE + 0x40cd8, 0x80808);
	outpdw(MDP_BASE + 0x40cdc, 0x80808);
	outpdw(MDP_BASE + 0x40ce0, 0x90909);
	outpdw(MDP_BASE + 0x40ce4, 0x90909);
	outpdw(MDP_BASE + 0x40ce8, 0xa0a0a);
	outpdw(MDP_BASE + 0x40cec, 0xa0a0a);
	outpdw(MDP_BASE + 0x40cf0, 0xa0a0a);
	outpdw(MDP_BASE + 0x40cf4, 0xb0b0b);
	outpdw(MDP_BASE + 0x40cf8, 0xb0b0b);
	outpdw(MDP_BASE + 0x40cfc, 0xb0b0b);
	outpdw(MDP_BASE + 0x40d00, 0xc0c0c);
	outpdw(MDP_BASE + 0x40d04, 0xc0c0c);
	outpdw(MDP_BASE + 0x40d08, 0xd0d0d);
	outpdw(MDP_BASE + 0x40d0c, 0xd0d0d);
	outpdw(MDP_BASE + 0x40d10, 0xe0e0e);
	outpdw(MDP_BASE + 0x40d14, 0xe0e0e);
	outpdw(MDP_BASE + 0x40d18, 0xe0e0e);
	outpdw(MDP_BASE + 0x40d1c, 0xf0f0f);
	outpdw(MDP_BASE + 0x40d20, 0xf0f0f);
	outpdw(MDP_BASE + 0x40d24, 0x101010);
	outpdw(MDP_BASE + 0x40d28, 0x101010);
	outpdw(MDP_BASE + 0x40d2c, 0x111111);
	outpdw(MDP_BASE + 0x40d30, 0x111111);
	outpdw(MDP_BASE + 0x40d34, 0x121212);
	outpdw(MDP_BASE + 0x40d38, 0x121212);
	outpdw(MDP_BASE + 0x40d3c, 0x131313);
	outpdw(MDP_BASE + 0x40d40, 0x131313);
	outpdw(MDP_BASE + 0x40d44, 0x141414);
	outpdw(MDP_BASE + 0x40d48, 0x151515);
	outpdw(MDP_BASE + 0x40d4c, 0x151515);
	outpdw(MDP_BASE + 0x40d50, 0x161616);
	outpdw(MDP_BASE + 0x40d54, 0x161616);
	outpdw(MDP_BASE + 0x40d58, 0x171717);
	outpdw(MDP_BASE + 0x40d5c, 0x171717);
	outpdw(MDP_BASE + 0x40d60, 0x181818);
	outpdw(MDP_BASE + 0x40d64, 0x191919);
	outpdw(MDP_BASE + 0x40d68, 0x191919);
	outpdw(MDP_BASE + 0x40d6c, 0x1a1a1a);
	outpdw(MDP_BASE + 0x40d70, 0x1b1b1b);
	outpdw(MDP_BASE + 0x40d74, 0x1b1b1b);
	outpdw(MDP_BASE + 0x40d78, 0x1c1c1c);
	outpdw(MDP_BASE + 0x40d7c, 0x1c1c1c);
	outpdw(MDP_BASE + 0x40d80, 0x1d1d1d);
	outpdw(MDP_BASE + 0x40d84, 0x1e1e1e);
	outpdw(MDP_BASE + 0x40d88, 0x1f1f1f);
	outpdw(MDP_BASE + 0x40d8c, 0x1f1f1f);
	outpdw(MDP_BASE + 0x40d90, 0x202020);
	outpdw(MDP_BASE + 0x40d94, 0x212121);
	outpdw(MDP_BASE + 0x40d98, 0x212121);
	outpdw(MDP_BASE + 0x40d9c, 0x222222);
	outpdw(MDP_BASE + 0x40da0, 0x232323);
	outpdw(MDP_BASE + 0x40da4, 0x242424);
	outpdw(MDP_BASE + 0x40da8, 0x242424);
	outpdw(MDP_BASE + 0x40dac, 0x252525);
	outpdw(MDP_BASE + 0x40db0, 0x262626);
	outpdw(MDP_BASE + 0x40db4, 0x272727);
	outpdw(MDP_BASE + 0x40db8, 0x272727);
	outpdw(MDP_BASE + 0x40dbc, 0x282828);
	outpdw(MDP_BASE + 0x40dc0, 0x292929);
	outpdw(MDP_BASE + 0x40dc4, 0x2a2a2a);
	outpdw(MDP_BASE + 0x40dc8, 0x2b2b2b);
	outpdw(MDP_BASE + 0x40dcc, 0x2c2c2c);
	outpdw(MDP_BASE + 0x40dd0, 0x2c2c2c);
	outpdw(MDP_BASE + 0x40dd4, 0x2d2d2d);
	outpdw(MDP_BASE + 0x40dd8, 0x2e2e2e);
	outpdw(MDP_BASE + 0x40ddc, 0x2f2f2f);
	outpdw(MDP_BASE + 0x40de0, 0x303030);
	outpdw(MDP_BASE + 0x40de4, 0x313131);
	outpdw(MDP_BASE + 0x40de8, 0x323232);
	outpdw(MDP_BASE + 0x40dec, 0x333333);
	outpdw(MDP_BASE + 0x40df0, 0x333333);
	outpdw(MDP_BASE + 0x40df4, 0x343434);
	outpdw(MDP_BASE + 0x40df8, 0x353535);
	outpdw(MDP_BASE + 0x40dfc, 0x363636);
	outpdw(MDP_BASE + 0x40e00, 0x373737);
	outpdw(MDP_BASE + 0x40e04, 0x383838);
	outpdw(MDP_BASE + 0x40e08, 0x393939);
	outpdw(MDP_BASE + 0x40e0c, 0x3a3a3a);
	outpdw(MDP_BASE + 0x40e10, 0x3b3b3b);
	outpdw(MDP_BASE + 0x40e14, 0x3c3c3c);
	outpdw(MDP_BASE + 0x40e18, 0x3d3d3d);
	outpdw(MDP_BASE + 0x40e1c, 0x3e3e3e);
	outpdw(MDP_BASE + 0x40e20, 0x3f3f3f);
	outpdw(MDP_BASE + 0x40e24, 0x404040);
	outpdw(MDP_BASE + 0x40e28, 0x414141);
	outpdw(MDP_BASE + 0x40e2c, 0x424242);
	outpdw(MDP_BASE + 0x40e30, 0x434343);
	outpdw(MDP_BASE + 0x40e34, 0x444444);
	outpdw(MDP_BASE + 0x40e38, 0x464646);
	outpdw(MDP_BASE + 0x40e3c, 0x474747);
	outpdw(MDP_BASE + 0x40e40, 0x484848);
	outpdw(MDP_BASE + 0x40e44, 0x494949);
	outpdw(MDP_BASE + 0x40e48, 0x4a4a4a);
	outpdw(MDP_BASE + 0x40e4c, 0x4b4b4b);
	outpdw(MDP_BASE + 0x40e50, 0x4c4c4c);
	outpdw(MDP_BASE + 0x40e54, 0x4d4d4d);
	outpdw(MDP_BASE + 0x40e58, 0x4f4f4f);
	outpdw(MDP_BASE + 0x40e5c, 0x505050);
	outpdw(MDP_BASE + 0x40e60, 0x515151);
	outpdw(MDP_BASE + 0x40e64, 0x525252);
	outpdw(MDP_BASE + 0x40e68, 0x535353);
	outpdw(MDP_BASE + 0x40e6c, 0x545454);
	outpdw(MDP_BASE + 0x40e70, 0x565656);
	outpdw(MDP_BASE + 0x40e74, 0x575757);
	outpdw(MDP_BASE + 0x40e78, 0x585858);
	outpdw(MDP_BASE + 0x40e7c, 0x595959);
	outpdw(MDP_BASE + 0x40e80, 0x5b5b5b);
	outpdw(MDP_BASE + 0x40e84, 0x5c5c5c);
	outpdw(MDP_BASE + 0x40e88, 0x5d5d5d);
	outpdw(MDP_BASE + 0x40e8c, 0x5e5e5e);
	outpdw(MDP_BASE + 0x40e90, 0x606060);
	outpdw(MDP_BASE + 0x40e94, 0x616161);
	outpdw(MDP_BASE + 0x40e98, 0x626262);
	outpdw(MDP_BASE + 0x40e9c, 0x646464);
	outpdw(MDP_BASE + 0x40ea0, 0x656565);
	outpdw(MDP_BASE + 0x40ea4, 0x666666);
	outpdw(MDP_BASE + 0x40ea8, 0x686868);
	outpdw(MDP_BASE + 0x40eac, 0x696969);
	outpdw(MDP_BASE + 0x40eb0, 0x6a6a6a);
	outpdw(MDP_BASE + 0x40eb4, 0x6c6c6c);
	outpdw(MDP_BASE + 0x40eb8, 0x6d6d6d);
	outpdw(MDP_BASE + 0x40ebc, 0x6f6f6f);
	outpdw(MDP_BASE + 0x40ec0, 0x707070);
	outpdw(MDP_BASE + 0x40ec4, 0x717171);
	outpdw(MDP_BASE + 0x40ec8, 0x737373);
	outpdw(MDP_BASE + 0x40ecc, 0x747474);
	outpdw(MDP_BASE + 0x40ed0, 0x767676);
	outpdw(MDP_BASE + 0x40ed4, 0x777777);
	outpdw(MDP_BASE + 0x40ed8, 0x797979);
	outpdw(MDP_BASE + 0x40edc, 0x7a7a7a);
	outpdw(MDP_BASE + 0x40ee0, 0x7c7c7c);
	outpdw(MDP_BASE + 0x40ee4, 0x7d7d7d);
	outpdw(MDP_BASE + 0x40ee8, 0x7f7f7f);
	outpdw(MDP_BASE + 0x40eec, 0x808080);
	outpdw(MDP_BASE + 0x40ef0, 0x828282);
	outpdw(MDP_BASE + 0x40ef4, 0x838383);
	outpdw(MDP_BASE + 0x40ef8, 0x858585);
	outpdw(MDP_BASE + 0x40efc, 0x868686);
	outpdw(MDP_BASE + 0x40f00, 0x888888);
	outpdw(MDP_BASE + 0x40f04, 0x898989);
	outpdw(MDP_BASE + 0x40f08, 0x8b8b8b);
	outpdw(MDP_BASE + 0x40f0c, 0x8d8d8d);
	outpdw(MDP_BASE + 0x40f10, 0x8e8e8e);
	outpdw(MDP_BASE + 0x40f14, 0x909090);
	outpdw(MDP_BASE + 0x40f18, 0x919191);
	outpdw(MDP_BASE + 0x40f1c, 0x939393);
	outpdw(MDP_BASE + 0x40f20, 0x959595);
	outpdw(MDP_BASE + 0x40f24, 0x969696);
	outpdw(MDP_BASE + 0x40f28, 0x989898);
	outpdw(MDP_BASE + 0x40f2c, 0x9a9a9a);
	outpdw(MDP_BASE + 0x40f30, 0x9b9b9b);
	outpdw(MDP_BASE + 0x40f34, 0x9d9d9d);
	outpdw(MDP_BASE + 0x40f38, 0x9f9f9f);
	outpdw(MDP_BASE + 0x40f3c, 0xa1a1a1);
	outpdw(MDP_BASE + 0x40f40, 0xa2a2a2);
	outpdw(MDP_BASE + 0x40f44, 0xa4a4a4);
	outpdw(MDP_BASE + 0x40f48, 0xa6a6a6);
	outpdw(MDP_BASE + 0x40f4c, 0xa7a7a7);
	outpdw(MDP_BASE + 0x40f50, 0xa9a9a9);
	outpdw(MDP_BASE + 0x40f54, 0xababab);
	outpdw(MDP_BASE + 0x40f58, 0xadadad);
	outpdw(MDP_BASE + 0x40f5c, 0xafafaf);
	outpdw(MDP_BASE + 0x40f60, 0xb0b0b0);
	outpdw(MDP_BASE + 0x40f64, 0xb2b2b2);
	outpdw(MDP_BASE + 0x40f68, 0xb4b4b4);
	outpdw(MDP_BASE + 0x40f6c, 0xb6b6b6);
	outpdw(MDP_BASE + 0x40f70, 0xb8b8b8);
	outpdw(MDP_BASE + 0x40f74, 0xbababa);
	outpdw(MDP_BASE + 0x40f78, 0xbbbbbb);
	outpdw(MDP_BASE + 0x40f7c, 0xbdbdbd);
	outpdw(MDP_BASE + 0x40f80, 0xbfbfbf);
	outpdw(MDP_BASE + 0x40f84, 0xc1c1c1);
	outpdw(MDP_BASE + 0x40f88, 0xc3c3c3);
	outpdw(MDP_BASE + 0x40f8c, 0xc5c5c5);
	outpdw(MDP_BASE + 0x40f90, 0xc7c7c7);
	outpdw(MDP_BASE + 0x40f94, 0xc9c9c9);
	outpdw(MDP_BASE + 0x40f98, 0xcbcbcb);
	outpdw(MDP_BASE + 0x40f9c, 0xcdcdcd);
	outpdw(MDP_BASE + 0x40fa0, 0xcfcfcf);
	outpdw(MDP_BASE + 0x40fa4, 0xd1d1d1);
	outpdw(MDP_BASE + 0x40fa8, 0xd3d3d3);
	outpdw(MDP_BASE + 0x40fac, 0xd5d5d5);
	outpdw(MDP_BASE + 0x40fb0, 0xd7d7d7);
	outpdw(MDP_BASE + 0x40fb4, 0xd9d9d9);
	outpdw(MDP_BASE + 0x40fb8, 0xdbdbdb);
	outpdw(MDP_BASE + 0x40fbc, 0xdddddd);
	outpdw(MDP_BASE + 0x40fc0, 0xdfdfdf);
	outpdw(MDP_BASE + 0x40fc4, 0xe1e1e1);
	outpdw(MDP_BASE + 0x40fc8, 0xe3e3e3);
	outpdw(MDP_BASE + 0x40fcc, 0xe5e5e5);
	outpdw(MDP_BASE + 0x40fd0, 0xe7e7e7);
	outpdw(MDP_BASE + 0x40fd4, 0xe9e9e9);
	outpdw(MDP_BASE + 0x40fd8, 0xebebeb);
	outpdw(MDP_BASE + 0x40fdc, 0xeeeeee);
	outpdw(MDP_BASE + 0x40fe0, 0xf0f0f0);
	outpdw(MDP_BASE + 0x40fe4, 0xf2f2f2);
	outpdw(MDP_BASE + 0x40fe8, 0xf4f4f4);
	outpdw(MDP_BASE + 0x40fec, 0xf6f6f6);
	outpdw(MDP_BASE + 0x40ff0, 0xf8f8f8);
	outpdw(MDP_BASE + 0x40ff4, 0xfbfbfb);
	outpdw(MDP_BASE + 0x40ff8, 0xfdfdfd);
	outpdw(MDP_BASE + 0x40ffc, 0xffffff);
}

#if defined(CONFIG_MACH_MSM7X27_THUNDERG) || defined(CONFIG_MACH_MSM7X27_THUNDERC) || defined(CONFIG_MACH_MSM7X27_ALESSI) || defined(CONFIG_MACH_MSM7X27_ELINI)
/* LGE_CHANGE_S
  * Change code to apply new LUT for display quality. 2010-08-03. minjong.gong@lge.com 
  * Below LUT Table was received from taeyun.kim@lge.com.
  */
static u32 thunder_lut_normal[256] = {
	0x00000000, 0x00010101, 0x00010101, 0x00020202, 0x00030303, 0x00040403, 0x00040404, 0x00050505,
		0x00060606, 0x00060706, 0x00070707, 0x00080808, 0x00090908, 0x00090909, 0x000A0A0A, 0x000B0B0B,
		0x000B0C0B, 0x000C0C0C, 0x000D0D0D, 0x000E0E0D, 0x000E0F0E, 0x000F0F0F, 0x00101010, 0x00111110,
		0x00111211, 0x00121312, 0x00131313, 0x00141413, 0x00141514, 0x00151615, 0x00161616, 0x00171716,
		0x00181817, 0x00181918, 0x00191A19, 0x001A1B19, 0x001B1B1A, 0x001C1C1B, 0x001C1D1C, 0x001D1E1D,
		0x001E1F1D, 0x001F201E, 0x0020201F, 0x00212120, 0x00212221, 0x00222322, 0x00232422, 0x00242523,
		0x00252624, 0x00262725, 0x00272726, 0x00282827, 0x00282928, 0x00292A29, 0x002A2B29, 0x002B2C2A,
		0x002C2D2B, 0x002D2E2C, 0x002E2F2D, 0x002F302E, 0x0030312F, 0x00313230, 0x00323331, 0x00333432,
		0x00343533, 0x00353634, 0x00363735, 0x00373836, 0x00383937, 0x00393A38, 0x003A3B39, 0x003B3C3A,
		0x003C3D3B, 0x003D3E3C, 0x003E3F3D, 0x003F403E, 0x0040413F, 0x00414240, 0x00424341, 0x00434442,
		0x00444543, 0x00454744, 0x00464845, 0x00474946, 0x00484A47, 0x004A4B48, 0x004B4C49, 0x004C4D4A,
		0x004D4E4B, 0x004E504D, 0x004F514E, 0x0050524F, 0x00525350, 0x00535451, 0x00545552, 0x00555753,
		0x00565854, 0x00575956, 0x00585A57, 0x005A5B58, 0x005B5D59, 0x005C5E5A, 0x005D5F5B, 0x005E605D,
		0x0060625E, 0x0061635F, 0x00626460, 0x00636561, 0x00646662, 0x00666864, 0x00676965, 0x00686A66,
		0x00696B67, 0x006B6D69, 0x006C6E6A, 0x006D6F6B, 0x006E716C, 0x0070726D, 0x0071736F, 0x00727470,
		0x00737671, 0x00757772, 0x00767873, 0x00777975, 0x00787B76, 0x007A7C77, 0x007B7D78, 0x007C7F7A,
		0x007D807B, 0x007E817C, 0x0080827D, 0x0081847E, 0x00828580, 0x00848681, 0x00858782, 0x00868983,
		0x00878A85, 0x00898B86, 0x008A8D87, 0x008B8E88, 0x008C8F8A, 0x008E908B, 0x008F928C, 0x0090938D,
		0x0091948E, 0x00939690, 0x00949791, 0x00959892, 0x00969993, 0x00989B95, 0x00999C96, 0x009A9D97,
		0x009B9E98, 0x009DA099, 0x009EA19B, 0x009FA29C, 0x00A0A39D, 0x00A1A59E, 0x00A3A69F, 0x00A4A7A1,
		0x00A5A8A2, 0x00A6AAA3, 0x00A7ABA4, 0x00A9ACA5, 0x00AAADA6, 0x00ABAEA8, 0x00ACB0A9, 0x00ADB1AA,
		0x00AFB2AB, 0x00B0B3AC, 0x00B1B4AD, 0x00B2B6AF, 0x00B3B7B0, 0x00B4B8B1, 0x00B5B9B2, 0x00B7BAB3,
		0x00B8BBB4, 0x00B9BDB5, 0x00BABEB6, 0x00BBBFB7, 0x00BCC0B9, 0x00BDC1BA, 0x00BEC2BB, 0x00C0C3BC,
		0x00C1C4BD, 0x00C2C6BE, 0x00C3C7BF, 0x00C4C8C0, 0x00C5C9C1, 0x00C6CAC2, 0x00C7CBC3, 0x00C8CCC4,
		0x00C9CDC5, 0x00CACEC6, 0x00CBCFC7, 0x00CCD0C8, 0x00CDD1C9, 0x00CED2CA, 0x00CFD3CB, 0x00D0D4CC,
		0x00D1D6CD, 0x00D2D7CE, 0x00D3D8CF, 0x00D4D9D0, 0x00D5DAD1, 0x00D6DBD2, 0x00D7DCD3, 0x00D8DCD4,
		0x00D9DDD5, 0x00DADED6, 0x00DBDFD7, 0x00DCE0D8, 0x00DDE1D9, 0x00DEE2D9, 0x00DFE3DA, 0x00E0E4DB,
		0x00E1E5DC, 0x00E2E6DD, 0x00E2E7DE, 0x00E3E8DF, 0x00E4E9E0, 0x00E5EAE1, 0x00E6EBE1, 0x00E7ECE2,
		0x00E8ECE3, 0x00E9EDE4, 0x00EAEEE5, 0x00EAEFE6, 0x00EBF0E7, 0x00ECF1E7, 0x00EDF2E8, 0x00EEF3E9,
		0x00EFF3EA, 0x00F0F4EB, 0x00F0F5EC, 0x00F1F6EC, 0x00F2F7ED, 0x00F3F8EE, 0x00F4F9EF, 0x00F5F9F0,
		0x00F5FAF1, 0x00F6FBF1, 0x00F7FCF2, 0x00F8FDF3, 0x00F9FEF4, 0x00FAFEF5, 0x00FAFFF5, 0x00FBFFF6,
		0x00FCFFF7, 0x00FDFFF8, 0x00FEFFF9, 0x00FEFFF9, 0x00FFFFFA, 0x00FFFFFB, 0x00FFFFFC, 0x00FFFFFD
	
	};

static u32 thunder_lut_camera[256] = {
	
		0x00000000, 0x00010101, 0x00020202, 0x00030303, 0x00040404, 0x00050505, 0x00060606, 0x00070707,
		0x00080808, 0x00090909, 0x000A0A0A, 0x000B0B0B, 0x000C0C0C, 0x000D0D0D, 0x000E0E0E, 0x000F0F0F,
		0x00101010, 0x00111211, 0x00121312, 0x00131413, 0x00141514, 0x00151615, 0x00161716, 0x00171817,
		0x00181918, 0x00191A19, 0x001A1B1A, 0x001B1C1B, 0x001C1D1C, 0x001D1E1D, 0x001E1F1E, 0x001F201F,
		0x00202120, 0x00212221, 0x00222322, 0x00232423, 0x00242524, 0x00252625, 0x00262726, 0x00272827,
		0x00282928, 0x00292A29, 0x002A2B2A, 0x002B2C2B, 0x002C2D2C, 0x002D2E2D, 0x002E2F2E, 0x002F302F,
		0x00303130, 0x00313231, 0x00333432, 0x00343532, 0x00353633, 0x00363734, 0x00373835, 0x00383936,
		0x00393A37, 0x003A3B38, 0x003B3C39, 0x003C3D3A, 0x003D3E3B, 0x003E3F3C, 0x003F403D, 0x0040413E,
		0x0041423F, 0x00424340, 0x00434441, 0x00444542, 0x00454643, 0x00464744, 0x00474845, 0x00484946,
		0x00494A47, 0x004A4B48, 0x004B4C49, 0x004C4D4A, 0x004D4E4B, 0x004E4F4C, 0x004F504D, 0x0050514E,
		0x0051524F, 0x00525350, 0x00535451, 0x00545552, 0x00555753, 0x00565854, 0x00575955, 0x00585A56,
		0x00595B57, 0x005A5C58, 0x005B5D59, 0x005C5E5A, 0x005D5F5B, 0x005E605C, 0x005F615D, 0x0060625E,
		0x0061635F, 0x00626460, 0x00636561, 0x00646662, 0x00656763, 0x00666864, 0x00676965, 0x00686A66,
		0x00696B67, 0x006A6C68, 0x006B6D69, 0x006C6E6A, 0x006D6F6B, 0x006E706C, 0x006F716D, 0x0070726E,
		0x0071736F, 0x00727470, 0x00737571, 0x00747672, 0x00757773, 0x00767974, 0x00777A75, 0x00787B76,
		0x00797C77, 0x007A7D78, 0x007B7E79, 0x007C7F7A, 0x007D807B, 0x007E817C, 0x007F827D, 0x0080837E,
		0x0081847F, 0x00828580, 0x00838681, 0x00848782, 0x00858883, 0x00868984, 0x00878A85, 0x00888B86,
		0x00898C87, 0x008A8D88, 0x008B8E89, 0x008C8F8A, 0x008D908B, 0x008E918C, 0x008F928D, 0x0090938E,
		0x0091948F, 0x00929590, 0x00939691, 0x00949792, 0x00959893, 0x00969994, 0x00989B95, 0x00999C95,
		0x009A9D96, 0x009B9E97, 0x009C9F98, 0x009DA099, 0x009EA19A, 0x009FA29B, 0x00A0A39C, 0x00A1A49D,
		0x00A2A59E, 0x00A3A69F, 0x00A4A7A0, 0x00A5A8A1, 0x00A6A9A2, 0x00A7AAA3, 0x00A8ABA4, 0x00A9ACA5,
		0x00AAADA6, 0x00ABAEA7, 0x00ACAFA8, 0x00ADB0A9, 0x00AEB1AA, 0x00AFB2AB, 0x00B0B3AC, 0x00B1B4AD,
		0x00B2B5AE, 0x00B3B6AF, 0x00B4B7B0, 0x00B5B8B1, 0x00B6B9B2, 0x00B7BAB3, 0x00B8BBB4, 0x00B9BCB5,
		0x00BABEB6, 0x00BBBFB7, 0x00BCC0B8, 0x00BDC1B9, 0x00BEC2BA, 0x00BFC3BB, 0x00C0C4BC, 0x00C1C5BD,
		0x00C2C6BE, 0x00C3C7BF, 0x00C4C8C0, 0x00C5C9C1, 0x00C6CAC2, 0x00C7CBC3, 0x00C8CCC4, 0x00C9CDC5,
		0x00CACEC6, 0x00CBCFC7, 0x00CCD0C8, 0x00CDD1C9, 0x00CED2CA, 0x00CFD3CB, 0x00D0D4CC, 0x00D1D5CD,
		0x00D2D6CE, 0x00D3D7CF, 0x00D4D8D0, 0x00D5D9D1, 0x00D6DAD2, 0x00D7DBD3, 0x00D8DCD4, 0x00D9DDD5,
		0x00DADED6, 0x00DBE0D7, 0x00DCE1D8, 0x00DDE2D9, 0x00DEE3DA, 0x00DFE4DB, 0x00E0E5DC, 0x00E1E6DD,
		0x00E2E7DE, 0x00E3E8DF, 0x00E4E9E0, 0x00E5EAE1, 0x00E6EBE2, 0x00E7ECE3, 0x00E8EDE4, 0x00E9EEE5,
		0x00EAEFE6, 0x00EBF0E7, 0x00ECF1E8, 0x00EDF2E9, 0x00EEF3EA, 0x00EFF4EB, 0x00F0F5EC, 0x00F1F6ED,
		0x00F2F7EE, 0x00F3F8EF, 0x00F4F9F0, 0x00F5FAF1, 0x00F6FBF2, 0x00F7FCF3, 0x00F8FDF4, 0x00F9FEF5,
		0x00FAFFF6, 0x00FBFFF7, 0x00FDFFF8, 0x00FEFFF8, 0x00FFFFF9, 0x00FFFFFA, 0x00FFFFFB, 0x00FFFFFC
	
	};

void mdp_load_thunder_lut(int lut_type)
{
	int i=0;
	if(lut_type != 0)
	{
		/* MDP cmd block enable */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

		/* bit4 	: LUT Table is Post(After matrix convert), bit2:0  : disable 3 components table. */
		MDP_OUTP(MDP_BASE + 0x90070, 0x10);

		if(lut_type == 1)
		{
			for(i=0;i<256;i++) {
				outpdw(MDP_BASE + 0x93800 + i*4, thunder_lut_normal[i]); }
		}
		else if(lut_type ==2)
		{
			for(i=0;i<256;i++) {
				outpdw(MDP_BASE + 0x93800 + i*4, thunder_lut_camera[i]); }			
		}
		
		/* bit4 	: LUT Table is Post(After matrix convert), bit2:0  : enable 3 components table. */
		MDP_OUTP(MDP_BASE + 0x90070, 0x17);

		/* MDP cmd block disable */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);		
	}
}
/* LGE_CHANGE_E */
#endif

#define   IRQ_EN_1__MDP_IRQ___M    0x00000800

void mdp_hw_init(void)
{
	int i;

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	/* debug interface write access */
	outpdw(MDP_BASE + 0x60, 1);

	outp32(MDP_INTR_ENABLE, MDP_ANY_INTR_MASK);
	outp32(MDP_EBI2_PORTMAP_MODE, 0x3);
	outpdw(MDP_CMD_DEBUG_ACCESS_BASE + 0x01f8, 0x0);
	outpdw(MDP_CMD_DEBUG_ACCESS_BASE + 0x01fc, 0x0);
	outpdw(MDP_BASE + 0x60, 0x1);
	mdp_load_lut_param();

	/*
	 * clear up unused fg/main registers
	 */
	/* comp.plane 2&3 ystride */
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0120, 0x0);
	/* unpacked pattern */
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x012c, 0x0);
	/* unpacked pattern */
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0130, 0x0);
	/* unpacked pattern */
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0134, 0x0);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0158, 0x0);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x15c, 0x0);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0160, 0x0);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0170, 0x0);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0174, 0x0);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x017c, 0x0);

	/* comp.plane 2 */
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0114, 0x0);
	/* comp.plane 3 */
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0118, 0x0);

	/* clear up unused bg registers */
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x01c8, 0);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x01d0, 0);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x01dc, 0);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x01e0, 0);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x01e4, 0);

#ifndef CONFIG_FB_MSM_MDP22
	MDP_OUTP(MDP_BASE + 0xE0000, 0);
	MDP_OUTP(MDP_BASE + 0x100, 0xffffffff);
	MDP_OUTP(MDP_BASE + 0x90070, 0);
	MDP_OUTP(MDP_BASE + 0x94010, 1);
	MDP_OUTP(MDP_BASE + 0x9401c, 2);
#endif

#if defined(CONFIG_MACH_MSM7X27_THUNDERG) || defined(CONFIG_MACH_MSM7X27_THUNDERC) || defined(CONFIG_MACH_MSM7X27_ALESSI) || defined(CONFIG_MACH_MSM7X27_ELINI)
	/* LGE_CHANGE_S
	  * Change code to apply new LUT for display quality. 2010-08-03. minjong.gong@lge.com 
	  */
	mdp_load_thunder_lut(1);	// nornal 
#endif

	/*
	 * limit vector
	 * pre gets applied before color matrix conversion
	 * post is after ccs
	 */
	writel(mdp_plv[0], MDP_CSC_PRE_LV1n(0));
	writel(mdp_plv[1], MDP_CSC_PRE_LV1n(1));
	writel(mdp_plv[2], MDP_CSC_PRE_LV1n(2));
	writel(mdp_plv[3], MDP_CSC_PRE_LV1n(3));

#ifdef CONFIG_FB_MSM_MDP31
	writel(mdp_plv[2], MDP_CSC_PRE_LV1n(4));
	writel(mdp_plv[3], MDP_CSC_PRE_LV1n(5));

	writel(0, MDP_CSC_POST_LV1n(0));
	writel(0xff, MDP_CSC_POST_LV1n(1));
	writel(0, MDP_CSC_POST_LV1n(2));
	writel(0xff, MDP_CSC_POST_LV1n(3));
	writel(0, MDP_CSC_POST_LV1n(4));
	writel(0xff, MDP_CSC_POST_LV1n(5));

	writel(0, MDP_CSC_PRE_LV2n(0));
	writel(0xff, MDP_CSC_PRE_LV2n(1));
	writel(0, MDP_CSC_PRE_LV2n(2));
	writel(0xff, MDP_CSC_PRE_LV2n(3));
	writel(0, MDP_CSC_PRE_LV2n(4));
	writel(0xff, MDP_CSC_PRE_LV2n(5));

	writel(mdp_plv[0], MDP_CSC_POST_LV2n(0));
	writel(mdp_plv[1], MDP_CSC_POST_LV2n(1));
	writel(mdp_plv[2], MDP_CSC_POST_LV2n(2));
	writel(mdp_plv[3], MDP_CSC_POST_LV2n(3));
	writel(mdp_plv[2], MDP_CSC_POST_LV2n(4));
	writel(mdp_plv[3], MDP_CSC_POST_LV2n(5));
#endif

	/* primary forward matrix */
	for (i = 0; i < MDP_CCS_SIZE; i++)
		writel(mdp_ccs_rgb2yuv.ccs[i], MDP_CSC_PFMVn(i));

#ifdef CONFIG_FB_MSM_MDP31
	for (i = 0; i < MDP_BV_SIZE; i++)
		writel(mdp_ccs_rgb2yuv.bv[i], MDP_CSC_POST_BV2n(i));

	writel(0, MDP_CSC_PRE_BV2n(0));
	writel(0, MDP_CSC_PRE_BV2n(1));
	writel(0, MDP_CSC_PRE_BV2n(2));
#endif
	/* primary reverse matrix */
	for (i = 0; i < MDP_CCS_SIZE; i++)
		writel(mdp_ccs_yuv2rgb.ccs[i], MDP_CSC_PRMVn(i));

	for (i = 0; i < MDP_BV_SIZE; i++)
		writel(mdp_ccs_yuv2rgb.bv[i], MDP_CSC_PRE_BV1n(i));

#ifdef CONFIG_FB_MSM_MDP31
	writel(0, MDP_CSC_POST_BV1n(0));
	writel(0, MDP_CSC_POST_BV1n(1));
	writel(0, MDP_CSC_POST_BV1n(2));

	outpdw(MDP_BASE + 0x30010, 0x03e0);
	outpdw(MDP_BASE + 0x30014, 0x0360);
	outpdw(MDP_BASE + 0x30018, 0x0120);
	outpdw(MDP_BASE + 0x3001c, 0x0140);
#endif
	mdp_init_scale_table();

#ifndef CONFIG_FB_MSM_MDP31
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0104,
		 ((16 << 6) << 16) | (16) << 6);
#endif

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
}
