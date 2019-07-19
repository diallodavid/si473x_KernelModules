/*
 * Driver for Cirrus Logic CS2300
 *
 * Copyright (C) 2017 rpiReceiver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

#define CS2300_CLK_IN_MIN        50
#define CS2300_CLK_IN_MAX  30000000
#define CS2300_CLK_OUT_MIN  6000000
#define CS2300_CLK_OUT_MAX 75000000
#define CS2300_SKIP_LIMIT     80000
#define CS2300_HIGH_ACCURACY_MULT   20
#define CS2300_HIGH_MULTIPLIER_MULT 12

static const char * const AuxOutSrc[] = {
	"clk-in", "clk-out", "pll-lock-pp", "pll-lock-od"};
enum { CS2300_AUX_OUT_CLK_IN, CS2300_AUX_OUT_CLK_OUT,
		CS2300_AUX_OUT_PLL_LOCK_PP, CS2300_AUX_OUT_PLL_LOCK_OD };

enum cs2300_i2c_registers {
/* Device ID and Revision  */
	CS2300_DEVICE_ID				= 0x0001,
/* Device Ctrl */
	CS2300_DEVICE_CTRL				= 0x0002,
/* Device Cfg 1 */
	CS2300_DEVICE_CFG_1				= 0x0003,
/* Global Cfg */
	CS2300_GLOBAL_CFG				= 0x0005,
/* 32-Bit Ratio */
	CS2300_RATIO_BITS_31_24			= 0x0006,
	CS2300_RATIO_BITS_23_16			= 0x0007,
	CS2300_RATIO_BITS_15_08			= 0x0008,
	CS2300_RATIO_BITS_07_00			= 0x0009,
/* Funct Cfg 1 */
	CS2300_FUNCT_CFG_1				= 0x0016,
/* Funct Cfg 2 */
	CS2300_FUNCT_CFG_2				= 0x0017,
/* Funct Cfg 3 */
	CS2300_FUNCT_CFG_3				= 0x001E,
/* 32-Bit Ratio with auto increment*/
	CS2300_RATIO_BYTES	= 0x80 | CS2300_RATIO_BITS_31_24,
};

enum cs2300_deviceID {
	CS2300_REVISION = BIT(2) | BIT(1) | BIT(0),
	CS2300_DEVICE = BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3),
};

enum cs2300_deviceCtrl {
	CS2300_CLKOUTDIS = BIT(0),
	CS2300_AUXOUTDIS = BIT(1),
	CS2300_UNLOCK = BIT(7),
};

enum cs2300_deviceCfg1 {
	CS2300_ENDEVCFG1 = BIT(0),
	CS2300_AUXOUTSRC0 = BIT(1),
	CS2300_AUXOUTSRC1 = BIT(2),
	CS2300_RMODSEL0 = BIT(5),
	CS2300_RMODSEL1 = BIT(6),
	CS2300_RMODSEL2 = BIT(7),
};

enum cs2300_GlobalCfg {
	CS2300_ENDEVCFG2 = BIT(0),
	CS2300_FREEZE = BIT(3),
};

enum cs2300_FunctCfg1 {
	CS2300_ENDEVCFG3 = BIT(4),
	CS2300_AUXLOCKCFG = BIT(6),
	CS2300_CLKSKIPEN = BIT(7),
};

enum cs2300_FunctCfg2 {
	CS2300_LFRATIOCFG = BIT(3),
	CS2300_CLKOUTUNL = BIT(4),
};

enum cs2300_FunctCfg3 {
	CS2300_CLKIN_BW0 = BIT(4),
	CS2300_CLKIN_BW1 = BIT(5),
	CS2300_CLKIN_BW2 = BIT(6),
};

static const struct regmap_range cs2300_i2c_write_registers[] = {
		regmap_reg_range(CS2300_DEVICE_CTRL, CS2300_DEVICE_CFG_1),
		regmap_reg_range(CS2300_GLOBAL_CFG, CS2300_RATIO_BITS_07_00),
		regmap_reg_range(CS2300_FUNCT_CFG_1, CS2300_FUNCT_CFG_2),
		regmap_reg_range(CS2300_FUNCT_CFG_3, CS2300_FUNCT_CFG_3),
};

static const struct regmap_range cs2300_i2c_read_registers[] = {
		regmap_reg_range(CS2300_DEVICE_ID, CS2300_DEVICE_ID),
};

static const struct regmap_access_table
	cs2300_i2c_registers_write_ranges_table = {
		.yes_ranges     = cs2300_i2c_write_registers,
		.n_yes_ranges   = ARRAY_SIZE(cs2300_i2c_write_registers),
};

static const struct regmap_access_table
	cs2300_i2c_registers_read_ranges_table = {
		.yes_ranges     = cs2300_i2c_read_registers,
		.n_yes_ranges   = ARRAY_SIZE(cs2300_i2c_read_registers),
};

static const struct reg_default cs2300_reg_defaults[] = {
	{ CS2300_DEVICE_CTRL, 0xC0 },
	{ CS2300_DEVICE_CFG_1, 0x00 },
	{ CS2300_GLOBAL_CFG, 0x00 },
	{ CS2300_FUNCT_CFG_1, 0x00 },
	{ CS2300_FUNCT_CFG_2, 0x00 },
	{ CS2300_FUNCT_CFG_3, 0x00 },
};

static const struct regmap_config cs2300_i2c_registers_regmap_config = {
		.name = "cs2300_i2c",
		.reg_bits = 8,
		.val_bits = 8,
		.max_register = CS2300_FUNCT_CFG_3,
		.wr_table = &cs2300_i2c_registers_write_ranges_table,
		.rd_table = &cs2300_i2c_registers_read_ranges_table,
		.reg_defaults = cs2300_reg_defaults,
		.num_reg_defaults = ARRAY_SIZE(cs2300_reg_defaults),
		.can_multi_write = true,
		.cache_type = REGCACHE_RBTREE,
};

struct clk_cs2300 {
	struct clk_hw hw;
	struct regmap *regmap;
	struct i2c_client *i2c_client;
	unsigned long rate;
	unsigned int ratio_eff;
	bool skipping_mode; // defaults to disabled
};

#define to_clk_cs2300(_hw)	container_of(_hw, struct clk_cs2300, hw)

static int cs2300_ratio_set(struct clk_cs2300 *cs2300,
			unsigned long rate, unsigned long parent_rate)
{
	int err;
	u8 cs2300_ratio[4];

	/* Ratio Modifier is not implemented yet */
	if (cs2300->ratio_eff < 4096) { // CS2300_HIGH_ACCURACY
		err = regmap_update_bits(cs2300->regmap, CS2300_FUNCT_CFG_2,
					CS2300_LFRATIOCFG, CS2300_LFRATIOCFG);
		if (err < 0)
			return err;
		cs2300_ratio[0] =
			(cs2300->ratio_eff <<
				CS2300_HIGH_ACCURACY_MULT) >> 24;
		cs2300_ratio[1] =
			(cs2300->ratio_eff <<
				CS2300_HIGH_ACCURACY_MULT) >> 16;
		cs2300_ratio[2] =
			(cs2300->ratio_eff <<
				CS2300_HIGH_ACCURACY_MULT) >>  8;
		cs2300_ratio[3] =
			(cs2300->ratio_eff <<
				CS2300_HIGH_ACCURACY_MULT) >>  0;
	} else { // CS2300_HIGH_MULTIPLIER
		if (cs2300->ratio_eff &
			(BIT(32 - CS2300_HIGH_MULTIPLIER_MULT) - 1)) {
			dev_err(&cs2300->i2c_client->dev,
				"%s, Multiplyer value too high - try implementing Ratio Modifier\n",
				__func__);
		}
		err = regmap_update_bits(cs2300->regmap, CS2300_FUNCT_CFG_2,
					CS2300_LFRATIOCFG, 0);
		if (err < 0)
			return err;
		cs2300_ratio[0] =
			(cs2300->ratio_eff <<
				CS2300_HIGH_MULTIPLIER_MULT) >> 24;
		cs2300_ratio[1] =
			(cs2300->ratio_eff <<
				CS2300_HIGH_MULTIPLIER_MULT) >> 16;
		cs2300_ratio[2] =
			(cs2300->ratio_eff <<
				CS2300_HIGH_MULTIPLIER_MULT) >>  8;
		cs2300_ratio[3] =
			(cs2300->ratio_eff <<
				CS2300_HIGH_MULTIPLIER_MULT) >>  0;
	}
	return regmap_bulk_write(cs2300->regmap, CS2300_RATIO_BYTES,
				cs2300_ratio, ARRAY_SIZE(cs2300_ratio));
}

static unsigned long cs2300_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_cs2300 *cs2300 = to_clk_cs2300(hw);

	dev_dbg(&cs2300->i2c_client->dev,
		"%s, parent_rate: %lu return: %lu\n",
		__func__, parent_rate, cs2300->rate * cs2300->ratio_eff);

		return cs2300->rate * cs2300->ratio_eff;
}

static long cs2300_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	struct clk_cs2300 *cs2300 = to_clk_cs2300(hw);

	dev_dbg(&cs2300->i2c_client->dev,
		"%s, rate: %lu, parent_rate: %lu\n",
		__func__, rate, *parent_rate);

	return rate;
}

static int cs2300_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_cs2300 *cs2300 = to_clk_cs2300(hw);
	int err;

	dev_dbg(&cs2300->i2c_client->dev,
		"%s, rate: %lu, parent_rate: %lu\n",
		__func__, rate, parent_rate);

	if ((rate < CS2300_CLK_IN_MIN) || (rate > CS2300_CLK_IN_MAX)) {
		dev_err(&cs2300->i2c_client->dev,
			"%s: rate %lu outside PLL input range.\n",
			__func__, rate);
		return -EINVAL;
	}
	if (((rate * cs2300->ratio_eff) < CS2300_CLK_OUT_MIN) ||
		((rate * cs2300->ratio_eff) > CS2300_CLK_OUT_MAX)) {
		dev_err(&cs2300->i2c_client->dev,
			"%s: rate %lu outside PLL output range.\n",
			__func__, rate);
		return -EINVAL;
	}
	if (cs2300->skipping_mode && (rate < CS2300_SKIP_LIMIT)) {
		err = regmap_update_bits(cs2300->regmap, CS2300_FUNCT_CFG_1,
					CS2300_CLKSKIPEN, CS2300_CLKSKIPEN);
	} else {
		err = regmap_update_bits(cs2300->regmap, CS2300_FUNCT_CFG_1,
					CS2300_CLKSKIPEN, 0);
	}
	if (err < 0)
		return err;
	cs2300->rate = rate;
	return cs2300_ratio_set(cs2300, rate, parent_rate);
}

static int cs2300_prepare(struct clk_hw *hw)
{
	struct clk_cs2300 *cs2300 = to_clk_cs2300(hw);

	return regmap_update_bits(cs2300->regmap, CS2300_GLOBAL_CFG,
				CS2300_ENDEVCFG2, CS2300_ENDEVCFG2);
}

static void cs2300_unprepare(struct clk_hw *hw)
{
	struct clk_cs2300 *cs2300 = to_clk_cs2300(hw);

	regmap_update_bits(cs2300->regmap, CS2300_GLOBAL_CFG,
				CS2300_ENDEVCFG2, 0);
}

static const struct clk_ops cs2300_clk_ops = {
	.recalc_rate = cs2300_recalc_rate,
	.round_rate = cs2300_round_rate,
	.set_rate = cs2300_set_rate,
	.prepare = cs2300_prepare,
	.unprepare = cs2300_unprepare,
};

static int cs2300_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct clk_cs2300 *cs2300;
	struct clk_init_data init;
	struct clk *clk;
	int cs2300_device_id;
	int err, i, minLoopBandwidth;
	const char *parent_name;

	cs2300 = devm_kzalloc(&client->dev, sizeof(*cs2300), GFP_KERNEL);
	if (!cs2300)
		return -ENOMEM;

	init.name = "clk-cs2300";
	init.ops = &cs2300_clk_ops;
	init.flags = CLK_GET_RATE_NOCACHE;
	parent_name = of_clk_get_parent_name(client->dev.of_node, 0);
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);
	cs2300->hw.init = &init;
	cs2300->i2c_client = client;

	cs2300->regmap = devm_regmap_init_i2c(client,
		&cs2300_i2c_registers_regmap_config);
	if (IS_ERR(cs2300->regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(cs2300->regmap);
	}

	i2c_set_clientdata(client, cs2300);

	regmap_read(cs2300->regmap, CS2300_DEVICE_ID, &cs2300_device_id);
	if ((cs2300_device_id & CS2300_DEVICE) == 0) {
		switch (cs2300_device_id & CS2300_REVISION) {
		case 0x04:
			dev_dbg(&client->dev, "CS2300 Revision: B2 or B3 found\n");
			break;
		case 0x06:
			dev_dbg(&client->dev, "CS2300 Revision: C1 found\n");
			break;
		default:
			dev_err(&client->dev, "CS2300 Revision %X not valid\n",
					cs2300_device_id & CS2300_REVISION);
			return -EINVAL;
		}
	} else {
		dev_err(&client->dev, "CS2300 Device ID not valid\n");
		return -EINVAL;
	}

	if (of_property_read_string(client->dev.of_node, "clock-output-names",
			&init.name))
		init.name = client->dev.of_node->name;

	if (of_property_read_u32(client->dev.of_node,
		"ratio-eff", &cs2300->ratio_eff))
		cs2300->ratio_eff = 256;

	if (!of_property_read_u32(client->dev.of_node,
		"min-loop-bw", &minLoopBandwidth)) {
		for (i = 0; i < 8; i++) {
			if (minLoopBandwidth == 0)
				i = 1;
			if (!(minLoopBandwidth >> i))
				break;
		}
		i--;
		err = regmap_update_bits(cs2300->regmap, CS2300_FUNCT_CFG_3,
			CS2300_CLKIN_BW2 | CS2300_CLKIN_BW1 | CS2300_CLKIN_BW0,
			i << __builtin_ctz(CS2300_CLKIN_BW0));
		if (err < 0)
			return err;
	}

	for (i = 0; i < ARRAY_SIZE(AuxOutSrc); i++) {
		if (of_property_match_string(client->dev.of_node,
			"aux-out-src", AuxOutSrc[i]) >= 0)
			break;
	}
	switch (i) {
	case CS2300_AUX_OUT_CLK_IN:
		err = regmap_update_bits(cs2300->regmap,
			CS2300_DEVICE_CFG_1,
			CS2300_AUXOUTSRC1 | CS2300_AUXOUTSRC0 |
			CS2300_ENDEVCFG1,
			CS2300_AUXOUTSRC0 | CS2300_ENDEVCFG1);
		if (err < 0)
			return err;
		err = regmap_update_bits(cs2300->regmap,
			CS2300_DEVICE_CTRL,
			CS2300_AUXOUTDIS, 0);
		if (err < 0)
			return err;
		break;
	case CS2300_AUX_OUT_CLK_OUT:
		err = regmap_update_bits(cs2300->regmap,
			CS2300_DEVICE_CFG_1,
			CS2300_AUXOUTSRC1 | CS2300_AUXOUTSRC0 |
			CS2300_ENDEVCFG1,
			CS2300_AUXOUTSRC1 | CS2300_ENDEVCFG1);
		if (err < 0)
			return err;
		err = regmap_update_bits(cs2300->regmap,
			CS2300_DEVICE_CTRL,
			CS2300_AUXOUTDIS, 0);
		if (err < 0)
			return err;
		break;
	case CS2300_AUX_OUT_PLL_LOCK_PP:
		err = regmap_update_bits(cs2300->regmap,
			CS2300_DEVICE_CFG_1,
			CS2300_AUXOUTSRC1 | CS2300_AUXOUTSRC0 |
			CS2300_ENDEVCFG1,
			CS2300_AUXOUTSRC1 | CS2300_AUXOUTSRC0 |
			CS2300_ENDEVCFG1);
		if (err < 0)
			return err;
		err = regmap_update_bits(cs2300->regmap,
			CS2300_FUNCT_CFG_1,
			CS2300_AUXLOCKCFG, 0);
		if (err < 0)
			return err;
		err = regmap_update_bits(cs2300->regmap,
			CS2300_DEVICE_CTRL,
			CS2300_AUXOUTDIS, 0);
		if (err < 0)
			return err;
		break;
	case CS2300_AUX_OUT_PLL_LOCK_OD:
		err = regmap_update_bits(cs2300->regmap,
			CS2300_DEVICE_CFG_1,
			CS2300_AUXOUTSRC1 | CS2300_AUXOUTSRC0 |
			CS2300_ENDEVCFG1,
			CS2300_AUXOUTSRC1 | CS2300_AUXOUTSRC0 |
			CS2300_ENDEVCFG1);
		if (err < 0)
			return err;
		err = regmap_update_bits(cs2300->regmap,
			CS2300_FUNCT_CFG_1,
			CS2300_AUXLOCKCFG, CS2300_AUXLOCKCFG);
		if (err < 0)
			return err;
		err = regmap_update_bits(cs2300->regmap,
			CS2300_DEVICE_CTRL,
			CS2300_AUXOUTDIS, 0);
		if (err < 0)
			return err;
		break;
	default:
		err = regmap_update_bits(cs2300->regmap,
			CS2300_DEVICE_CFG_1,
			CS2300_AUXOUTSRC1 | CS2300_AUXOUTSRC0 |
			CS2300_ENDEVCFG1, CS2300_ENDEVCFG1);
		if (err < 0)
			return err;
		err = regmap_update_bits(cs2300->regmap,
			CS2300_DEVICE_CTRL,
			CS2300_AUXOUTDIS, CS2300_AUXOUTDIS);
		if (err < 0)
			return err;
	}
	cs2300->skipping_mode = of_property_read_bool(client->dev.of_node,
				"clock-skip-enable");
	err = regmap_update_bits(cs2300->regmap, CS2300_FUNCT_CFG_1,
				CS2300_ENDEVCFG3, CS2300_ENDEVCFG3);
	if (err < 0)
		return err;

	err = regmap_update_bits(cs2300->regmap, CS2300_DEVICE_CTRL,
				CS2300_CLKOUTDIS, 0);
	if (err < 0)
		return err;

	clk = devm_clk_register(&client->dev, &cs2300->hw);
	if (IS_ERR(clk)) {
		dev_err(&client->dev, "clock registration failed\n");
		return PTR_ERR(clk);
	}
	err = of_clk_add_provider(client->dev.of_node, of_clk_src_simple_get,
			clk);
	if (err) {
		dev_err(&client->dev, "unable to add clk provider\n");
		return err;
	}
	return 0;
}

static int cs2300_remove(struct i2c_client *client)
{
	of_clk_del_provider(client->dev.of_node);

	return 0;
}

static const struct i2c_device_id cs2300_id[] = {
	{ "cs2300", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cs2300_id);

static const struct of_device_id clk_cs2300_of_match[] = {
	{ .compatible = "cirrus,cs2300" },
	{ },
};
MODULE_DEVICE_TABLE(of, clk_cs2300_of_match);

static struct i2c_driver cs2300_driver = {
	.driver = {
		.name = "cs2300-cp",
		.of_match_table = clk_cs2300_of_match,
	},
	.probe		= cs2300_probe,
	.remove		= cs2300_remove,
	.id_table	= cs2300_id,
};
module_i2c_driver(cs2300_driver);

MODULE_AUTHOR("rpiReceiver <rpiReceiver@htl-steyr.ac.at>");
MODULE_DESCRIPTION("CS2300 driver");
MODULE_LICENSE("GPL");
