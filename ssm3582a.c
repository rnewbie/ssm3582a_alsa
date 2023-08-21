// SPDX-License-Identifier: GPL-2.0-only
/*
 * ssm3582a.c -- SSM3582A ALSA SoC codec driver
 *
 * Author:      Alexey Pavlov <rnewbie@hotmail.com>
 *              Copyright 2023
 */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "ssm3582a.h"

struct ssm3582_priv {
	struct regmap *regmap;
	int mute;
	struct mutex mutex;
};

// regmap definitions
const struct regmap_range ssm3582_regmap_rd_regs_range[] = {
	regmap_reg_range(SSM3582_REG_VENDOR_ID, SSM3582_REG_SLOT_RIGHT_CTRL),
	regmap_reg_range(SSM3582_REG_LIM_LEFT_CTRL1, SSM3582_REG_SOFT_RESET),
};

const struct regmap_range ssm3582_regmap_wr_regs_range[] = {
	regmap_reg_range(SSM3582_REG_POWER_CTRL, SSM3582_REG_SLOT_RIGHT_CTRL),
	regmap_reg_range(SSM3582_REG_LIM_LEFT_CTRL1, SSM3582_REG_FAULT_CTRL2),
	regmap_reg_range(SSM3582_REG_SOFT_RESET, SSM3582_REG_SOFT_RESET),
};

const static struct regmap_range ssm3582_regmap_vol_regs_range[] = {
	regmap_reg_range(SSM3582_REG_STATUS1, SSM3582_REG_TEMP),
};

const static struct regmap_access_table ssm3582_regmap_rd_table = {
	.yes_ranges = ssm3582_regmap_rd_regs_range,
	.n_yes_ranges = ARRAY_SIZE(ssm3582_regmap_rd_regs_range),
};

const static struct regmap_access_table ssm3582_regmap_wr_table = {
	.yes_ranges = ssm3582_regmap_wr_regs_range,
	.n_yes_ranges = ARRAY_SIZE(ssm3582_regmap_wr_regs_range),
};

const static struct regmap_access_table ssm3582_regmap_vol_table = {
	.yes_ranges = ssm3582_regmap_vol_regs_range,
	.n_yes_ranges = ARRAY_SIZE(ssm3582_regmap_vol_regs_range),
};

const struct regmap_config ssm3582_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SSM3582_REG_SOFT_RESET,
	.rd_table = &ssm3582_regmap_rd_table,
	.wr_table = &ssm3582_regmap_wr_table,
	.volatile_table = &ssm3582_regmap_vol_table,
};

// sysfs definitions

static ssize_t stdreg_show(struct device *dev, unsigned int reg, char *buf)
{
	struct ssm3582_priv *ssm3582 = dev_get_drvdata(dev);
	unsigned int value;

	regmap_read(ssm3582->regmap, reg, &value);
	return sysfs_emit(buf, "0x%02x\n", value & 0xff);
}

static ssize_t stdreg_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count, unsigned int reg)
{
	struct ssm3582_priv *ssm3582 = dev_get_drvdata(dev);
	int return_code = 0;
	unsigned int reg_value = 0;

	//  return_code = kstrtou8_from_user(buf, count, 0, &reg_value);
	return_code = kstrtouint(buf, 0, &reg_value);

	if (return_code == 0) {
		regmap_write(ssm3582->regmap, reg, reg_value);
	} else {
		dev_err(dev, "Failed to parse input: %i\n", return_code);
	}

	return count;
}

static ssize_t device_id_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct ssm3582_priv *ssm3582 = dev_get_drvdata(dev);
	unsigned int value[2];

	regmap_read(ssm3582->regmap, SSM3582_REG_DEVICE_ID1, &value[0]);
	regmap_read(ssm3582->regmap, SSM3582_REG_DEVICE_ID2, &value[1]);
	return sysfs_emit(buf, "0x%04x\n",
			  (((value[0] & 0xff) << 8) | (value[1] & 0xff)));
}
DEVICE_ATTR_RO(device_id);

#define SSM3582_ATTR_RO(name, register)                                        \
	static ssize_t name##_show(struct device *dev,                         \
				   struct device_attribute *attr, char *buf)   \
	{                                                                      \
		return stdreg_show(dev, register, buf);                        \
	}                                                                      \
	DEVICE_ATTR_RO(name)

#define SSM3582_ATTR_RW(name, register)                                        \
	static ssize_t name##_show(struct device *dev,                         \
				   struct device_attribute *attr, char *buf)   \
	{                                                                      \
		return stdreg_show(dev, register, buf);                        \
	}                                                                      \
                                                                               \
	static ssize_t name##_store(struct device *dev,                        \
				    struct device_attribute *attr,             \
				    const char *buf, size_t count)             \
	{                                                                      \
		return stdreg_store(dev, attr, buf, count, register);          \
	}                                                                      \
	DEVICE_ATTR_RW(name)

SSM3582_ATTR_RO(vendor_id, SSM3582_REG_VENDOR_ID);
SSM3582_ATTR_RO(revision, SSM3582_REG_REVISION);
SSM3582_ATTR_RO(status1, SSM3582_REG_STATUS1);
SSM3582_ATTR_RO(status2, SSM3582_REG_STATUS2);
SSM3582_ATTR_RO(vbat, SSM3582_REG_VBAT);
SSM3582_ATTR_RO(temp, SSM3582_REG_TEMP);
SSM3582_ATTR_RW(power_ctl, SSM3582_REG_POWER_CTRL);
SSM3582_ATTR_RW(dac_ctl, SSM3582_REG_DAC_CTRL);
SSM3582_ATTR_RW(sai_ctrl1, SSM3582_REG_SAI_CTRL1);
SSM3582_ATTR_RW(sai_ctrl2, SSM3582_REG_SAI_CTRL2);
SSM3582_ATTR_RW(amp_dac_ctrl, SSM3582_REG_AMP_DAC_CTRL);
SSM3582_ATTR_RW(slot_left_ctrl, SSM3582_REG_SLOT_LEFT_CTRL);
SSM3582_ATTR_RW(slot_right_ctrl, SSM3582_REG_SLOT_RIGHT_CTRL);
SSM3582_ATTR_RW(fault_ctrl1, SSM3582_REG_FAULT_CTRL1);
SSM3582_ATTR_RW(fault_ctrl2, SSM3582_REG_FAULT_CTRL2);

static struct attribute *ssm3582_sysfs_attrs[] = {
	&dev_attr_vendor_id.attr,
	&dev_attr_device_id.attr,
	&dev_attr_revision.attr,
	&dev_attr_status1.attr,
	&dev_attr_status2.attr,
	&dev_attr_vbat.attr,
	&dev_attr_temp.attr,
	&dev_attr_power_ctl.attr,
	&dev_attr_dac_ctl.attr,
	&dev_attr_sai_ctrl1.attr,
	&dev_attr_sai_ctrl2.attr,
	&dev_attr_amp_dac_ctrl.attr,
	&dev_attr_slot_left_ctrl.attr,
	&dev_attr_slot_right_ctrl.attr,
	&dev_attr_fault_ctrl1.attr,
	&dev_attr_fault_ctrl2.attr,
	NULL,
};

ATTRIBUTE_GROUPS(ssm3582_sysfs);

// DAI

static int ssm3582_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct snd_soc_component *component = dai->component;
	struct ssm3582_priv *ssm3582 = snd_soc_component_get_drvdata(component);
	int return_code;

	mutex_lock(&ssm3582->mutex);
	if (mute) {
		return_code =
			regmap_set_bits(ssm3582->regmap, SSM3582_REG_DAC_CTRL,
					(SSM3582_DAC_CTRL_DAC_MUTE_L |
					 SSM3582_DAC_CTRL_DAC_MUTE_R));
		if (return_code != 0) {
			dev_err(component->dev, "Failed to set DAC mute\n");
		} else {
			ssm3582->mute |= 0x1;
		}
	} else {
		return_code =
			regmap_clear_bits(ssm3582->regmap, SSM3582_REG_DAC_CTRL,
					  (SSM3582_DAC_CTRL_DAC_MUTE_L |
					   SSM3582_DAC_CTRL_DAC_MUTE_R));
		if (return_code != 0) {
			dev_err(component->dev, "Failed to clear DAC mute\n");
		} else {
			ssm3582->mute &= ~0x1;
		}
	}
	mutex_unlock(&ssm3582->mutex);

	return return_code;
}

static int ssm3582_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct ssm3582_priv *ssm3582 = snd_soc_component_get_drvdata(component);
	unsigned int regval = SSM3582_SAI_CTRL1_TDM_BCLKS_32;
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	int slot_size = snd_soc_params_to_frame_size(params);
	int data_width = params_width(params);
	int dac_fs = rate * channels;
	int bclk = snd_soc_params_to_bclk(params);
	int return_code = 0;

	dev_info(component->dev,
		 "hw_params: frame size: %i, width:%i bits, "
		 "rate: %u Hz, %u channels\n",
		 slot_size, data_width, params_rate(params), channels);
	dev_info(
		component->dev,
		"hw_params: bclk: %i, physical_width: %i, period size: %u, periods:%u\n",
		bclk, params_physical_width(params), params_period_size(params),
		params_periods(params));
	dev_info(component->dev, "hw_params: dac_fs: %i\n", dac_fs);

	if (data_width < 0) {
		dev_err(component->dev,
			"hw_params: wrong data_width %i, muting\n", data_width);
		return_code = data_width;
		goto err;
	}

	switch (slot_size) {
	case 16:
		regval = SSM3582_SAI_CTRL1_TDM_BCLKS_16;
		break;
	case 24:
		regval = SSM3582_SAI_CTRL1_TDM_BCLKS_24;
		break;
	case 32:
		regval = SSM3582_SAI_CTRL1_TDM_BCLKS_32;
		break;
	case 48:
		regval = SSM3582_SAI_CTRL1_TDM_BCLKS_48;
		break;
	case 64:
		regval = SSM3582_SAI_CTRL1_TDM_BCLKS_64;
		break;
	default:
		dev_err(component->dev,
			"hw_params: unknown slot size: %i. Falling back to 32\n",
			slot_size);
		break;
	}

	return_code =
		regmap_update_bits(ssm3582->regmap, SSM3582_REG_SAI_CTRL1,
				   SSM3582_SAI_CTRL1_TDM_BCLKS_MASK, regval);
	if (return_code < 0) {
		dev_err(component->dev,
			"hw_params: failed to set block size. Error: %i. Muting\n",
			return_code);
		goto err;
	}

	switch (data_width) {
	case 24:
		return_code = regmap_clear_bits(ssm3582->regmap,
						SSM3582_REG_SAI_CTRL2,
						SSM3582_SAI_CTRL2_DATA_WIDTH);
		break;
	case 16:
		return_code =
			regmap_set_bits(ssm3582->regmap, SSM3582_REG_SAI_CTRL2,
					SSM3582_SAI_CTRL2_DATA_WIDTH);
		break;
	default:
		dev_err(component->dev,
			"hw_params: unsupported width: %i. Falling back to 16\n",
			data_width);
		return_code =
			regmap_set_bits(ssm3582->regmap, SSM3582_REG_SAI_CTRL2,
					SSM3582_SAI_CTRL2_DATA_WIDTH);
	}

	if (return_code < 0) {
		dev_err(component->dev,
			"hw_params: failed to set data width.  Error: %i. Muting\n",
			return_code);
		goto err;
	}

	if ((dac_fs >= 8000) && (dac_fs <= 12000)) {
		regval = SSM3582_DAC_CTRL_DAC_FS_8K_12K;
		goto dac_fs_set;
	}
	if ((dac_fs >= 16000) && (dac_fs <= 24000)) {
		regval = SSM3582_DAC_CTRL_DAC_FS_16K_24K;
		goto dac_fs_set;
	}
	if ((dac_fs >= 32000) && (dac_fs <= 48000)) {
		regval = SSM3582_DAC_CTRL_DAC_FS_32K_48K;
		goto dac_fs_set;
	}
	if ((dac_fs >= 64000) && (dac_fs <= 96000)) {
		regval = SSM3582_DAC_CTRL_DAC_FS_64K_96K;
		goto dac_fs_set;
	}
	if ((dac_fs >= 128000) && (dac_fs <= 192000)) {
		regval = SSM3582_DAC_CTRL_DAC_FS_128K_192K;
		goto dac_fs_set;
	}
	if ((dac_fs >= 48000) && (dac_fs <= 72000)) {
		regval = SSM3582_DAC_CTRL_DAC_FS_48K_72K;
	} else {
		dev_err(component->dev,
			"hw_params: unsupported sample rate: %i, muting\n",
			dac_fs);
		return_code = -EINVAL;
		goto err;
	}

dac_fs_set:
	dev_info(component->dev, "hw_params: dac_fs_reg: %u\n", regval);
	return_code = regmap_write_bits(ssm3582->regmap, SSM3582_REG_DAC_CTRL,
					SSM3582_DAC_CTRL_DAC_FS, regval);
	if (return_code < 0) {
		dev_err(component->dev,
			"hw_params: failed to set block size. Error: %i. Muting\n",
			return_code);
		goto err;
	}

	return 0;
err:
	ssm3582_mute(dai, 1, 0);
	return return_code;
};

static int ssm3582_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_component *component = dai->component;
	struct ssm3582_priv *ssm3582 = snd_soc_component_get_drvdata(component);
	int return_code;

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		break;
	default:
		dev_err(component->dev,
			"set_fmt: unsupported clk/frame cfg: 0x%x\n",
			((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) >> 24));
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dev_info(component->dev, "set_fmt: I2S mode\n");
		return_code = regmap_clear_bits(ssm3582->regmap,
						SSM3582_REG_SAI_CTRL1,
						(SSM3582_SAI_CTRL1_SAI_MODE |
						 SSM3582_SAI_CTRL1_DATA_FMT));
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		dev_info(component->dev, "set_fmt: Left justified mode\n");
		return_code =
			regmap_set_bits(ssm3582->regmap, SSM3582_REG_SAI_CTRL1,
					(SSM3582_SAI_CTRL1_SAI_MODE |
					 SSM3582_SAI_CTRL1_DATA_FMT));
		break;
	default:
		dev_err(component->dev, "set_fmt: unsupported mode: 0x%02x\n",
			(fmt & SND_SOC_DAIFMT_FORMAT_MASK));
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		return_code = regmap_clear_bits(ssm3582->regmap,
						SSM3582_REG_SAI_CTRL1,
						(SSM3582_SAI_CTRL1_BCLK_POL |
						 SSM3582_SAI_CTRL1_FSYNC_MODE));
		break;
	case SND_SOC_DAIFMT_NB_IF:
		return_code =
			regmap_set_bits(ssm3582->regmap, SSM3582_REG_SAI_CTRL1,
					SSM3582_SAI_CTRL1_FSYNC_MODE);
		return_code = regmap_clear_bits(ssm3582->regmap,
						SSM3582_REG_SAI_CTRL1,
						SSM3582_SAI_CTRL1_BCLK_POL);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		return_code =
			regmap_set_bits(ssm3582->regmap, SSM3582_REG_SAI_CTRL1,
					SSM3582_SAI_CTRL1_BCLK_POL);
		return_code = regmap_clear_bits(ssm3582->regmap,
						SSM3582_REG_SAI_CTRL1,
						SSM3582_SAI_CTRL1_FSYNC_MODE);
		break;
	case SND_SOC_DAIFMT_IB_IF:
		return_code =
			regmap_set_bits(ssm3582->regmap, SSM3582_REG_SAI_CTRL1,
					(SSM3582_SAI_CTRL1_BCLK_POL |
					 SSM3582_SAI_CTRL1_FSYNC_MODE));
		break;
	default:
		dev_err(component->dev,
			"set_fmt: unknown BCLK/FSYNC polarity mode: 0x%02x\n",
			(fmt & SND_SOC_DAIFMT_INV_MASK) >> 8);
		return -EINVAL;
		break;
	}

	return 0;
}

int ssm3582_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
			 unsigned int rx_mask, int slots, int slot_width)
{
	struct snd_soc_component *component = dai->component;
	dev_info(component->dev,
		 "ssm3582_set_tdm_slot slots: %i, slot_width: %i\n", slots,
		 slot_width);
	dev_info(component->dev,
		 "ssm3582_set_tdm_slot tx_mask: %x, rx_mask: %x\n", tx_mask,
		 rx_mask);
	return 0;
}

static const struct snd_soc_dai_ops ssm3582_dai_ops = {
	.set_tdm_slot = ssm3582_set_tdm_slot,
	.hw_params = ssm3582_hw_params,
	.set_fmt = ssm3582_set_fmt,
	.mute_stream = ssm3582_mute,
};

static struct snd_soc_dai_driver ssm3582_dai[] = {{
    .name = "ssm3582-hifi",
    .playback =
        {
            .channels_min = 1,
            .channels_max = 2,
            .rates = SNDRV_PCM_RATE_8000_192000,
            .formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE),
        },
    .ops = &ssm3582_dai_ops,
}};

static const char *ssm3582_amp_dac_ana_gains[] = { "13 dB (6.3 V peak)",
						   "16 dB (8.9 V peak)",
						   "19 dB (12.6 V peak)",
						   "21 dB (16 V peak)" };
static const struct soc_enum ssm3582_amp_dac_ana_gain = SOC_ENUM_SINGLE(
	SSM3582_REG_AMP_DAC_CTRL, SSM3582_AMP_DAC_CTRL_ANA_GAIN_POS,
	ARRAY_SIZE(ssm3582_amp_dac_ana_gains), ssm3582_amp_dac_ana_gains);

static const char *ssm3582_limiter_release_rates[] = {
	"3200 ms/dB",
	"1600 ms/dB",
	"1200 ms/dB",
	"800 ms/dB",
};
static const struct soc_enum ssm3582_limiter_release_rate_l =
	SOC_ENUM_SINGLE(SSM3582_REG_LIM_LEFT_CTRL1, SSM3582_LIM_CTRL1_RRT_POS,
			ARRAY_SIZE(ssm3582_limiter_release_rates),
			ssm3582_limiter_release_rates);
static const struct soc_enum ssm3582_limiter_release_rate_r =
	SOC_ENUM_SINGLE(SSM3582_REG_LIM_RIGHT_CTRL1, SSM3582_LIM_CTRL1_RRT_POS,
			ARRAY_SIZE(ssm3582_limiter_release_rates),
			ssm3582_limiter_release_rates);

static const char *ssm3582_limiter_modes[] = {
	"Limiter off",
	"Limiter on",
	"Mute output if VBAT<Limiter Battery Voltage Inflection",
	"Limiter on only if Limiter Battery Voltage Inflection",
};
static const struct soc_enum ssm3582_limiter_mode_l =
	SOC_ENUM_SINGLE(SSM3582_REG_LIM_LEFT_CTRL1, SSM3582_LIM_CTRL1_EN_POS,
			ARRAY_SIZE(ssm3582_limiter_modes),
			ssm3582_limiter_modes);
static const struct soc_enum ssm3582_limiter_mode_r =
	SOC_ENUM_SINGLE(SSM3582_REG_LIM_RIGHT_CTRL1, SSM3582_LIM_CTRL1_EN_POS,
			ARRAY_SIZE(ssm3582_limiter_modes),
			ssm3582_limiter_modes);

static const char *ssm3582_limiter_attack_rates[] = {
	"120 µs/dB",
	"60 µs/dB",
	"30 µs/dB",
	"20 µs/dB",
};

static const struct soc_enum ssm3582_limiter_attack_rate_l =
	SOC_ENUM_SINGLE(SSM3582_REG_LIM_LEFT_CTRL1, SSM3582_LIM_CTRL1_ATR_POS,
			ARRAY_SIZE(ssm3582_limiter_attack_rates),
			ssm3582_limiter_attack_rates);
static const struct soc_enum ssm3582_limiter_attack_rate_r =
	SOC_ENUM_SINGLE(SSM3582_REG_LIM_RIGHT_CTRL1, SSM3582_LIM_CTRL1_ATR_POS,
			ARRAY_SIZE(ssm3582_limiter_attack_rates),
			ssm3582_limiter_attack_rates);

static const struct snd_kcontrol_new ssm3582_controls[] = {
	SOC_DOUBLE_R("Digital Playback Volume", SSM3582_REG_VOL_LEFT_CTRL,
		     SSM3582_REG_VOL_RIGHT_CTRL, SSM3582_VOL_CTRL_POS,
		     SSM3582_VOL_CTRL, 1),
	SOC_ENUM("Amplifier Analog Gain Select", ssm3582_amp_dac_ana_gain),
	SOC_SINGLE("Edge Rate Control (low EMI)", SSM3582_REG_AMP_DAC_CTRL,
		   SSM3582_AMP_DAC_CTRL_EDGE_POS, 1, 0),
	SOC_SINGLE("Right Channel DAC Output Polarity invert",
		   SSM3582_REG_AMP_DAC_CTRL, SSM3582_AMP_DAC_CTRL_DAC_POL_R_POS,
		   1, 0),
	SOC_SINGLE("Left Channel DAC Output Polarity invert",
		   SSM3582_REG_AMP_DAC_CTRL, SSM3582_AMP_DAC_CTRL_DAC_POL_L_POS,
		   1, 0),
	SOC_SINGLE("DAC High-Pass Filter", SSM3582_REG_DAC_CTRL,
		   SSM3582_DAC_CTRL_DAC_HPF_POS, 1, 0),
	/* limiter */
	SOC_ENUM("Left limiter mode", ssm3582_limiter_mode_l),
	SOC_ENUM("Right limiter mode", ssm3582_limiter_mode_r),
	SOC_ENUM("Left limiter release rate", ssm3582_limiter_release_rate_l),
	SOC_ENUM("Right limiter release rate", ssm3582_limiter_release_rate_r),
	SOC_ENUM("Left limiter attack rate", ssm3582_limiter_attack_rate_l),
	SOC_ENUM("Right limiter attack rate", ssm3582_limiter_attack_rate_r),
	SOC_SINGLE("Left Threshold Battery Tracking",
		   SSM3582_REG_LIM_LEFT_CTRL1, SSM3582_LIM_CTRL1_VBAT_TRACK_POS,
		   1, 0),
	SOC_SINGLE("Right Threshold Battery Tracking",
		   SSM3582_REG_LIM_RIGHT_CTRL1,
		   SSM3582_LIM_CTRL1_VBAT_TRACK_POS, 1, 0),
	SOC_SINGLE("Use left limiter settings for both channels",
		   SSM3582_REG_LIM_RIGHT_CTRL1,
		   SSM3582_LIM_CTRL1_VBAT_TRACK_POS, 1, 0),

};

int ssm3582_probe(struct snd_soc_component *component)
{
	struct ssm3582_priv *ssm3582 = snd_soc_component_get_drvdata(component);
	struct device *dev = component->dev;
	int return_code = 0;
	unsigned int value = 0;
	u8 voltage = 12;
	static const u8 voltage_table[] = { 31, 29, 27, 25, 23, 21, 19, 15,
					    12, 10, 8,	6,  4,	2,  0 };

	return_code = regmap_write(ssm3582->regmap, SSM3582_REG_SOFT_RESET,
				   SSM3582_SOFT_RESET_S_RST);
	if (return_code != 0) {
		dev_err(dev, "Unable to issue reset request!\n");
		return return_code;
	}

#ifdef CONFIG_OF
	if (dev->of_node) {
		const struct device_node *dn = dev->of_node;

		if (of_property_read_bool(dn, "stereo")) {
			dev_info(
				dev,
				"Found stereo property, switching to stereo mode\n");
			return_code =
				regmap_clear_bits(ssm3582->regmap,
						  SSM3582_REG_POWER_CTRL,
						  SSM3582_POWER_CTRL_MONO);
			if (return_code != 0) {
				dev_err(dev,
					"Unable to switch to stereo mode!\n");
				return return_code;
			}
			return_code = regmap_clear_bits(
				ssm3582->regmap, SSM3582_REG_SAI_CTRL2,
				(SSM3582_SAI_CTRL2_VOL_LINK |
				 SSM3582_SAI_CTRL2_CLIP_LINK));
			if (return_code != 0) {
				dev_err(dev,
					"Unable to unlink volume and clipping!\n");
				return return_code;
			}
		}

		if (of_property_read_u32(dn, "tdm", &value)) {
			dev_info(dev, "Found property tdm: %u\n", value);
		}

		if (of_property_read_u8(dn, "volts", &voltage)) {
			dev_info(dev,
				 "Found power supply voltage: %u decivolts\n",
				 voltage);
			if (voltage > 16 || voltage < 2) {
				dev_err(dev,
					"Wrong voltage supplied, should be 2-16\n");
				voltage = 12;
			}
		} else {
			dev_info(dev,
				 "Setting default limiter voltage: 12 volts\n");
		}

		voltage = voltage_table[(voltage - 2)];
		return_code =
			regmap_write_bits(ssm3582->regmap,
					  SSM3582_REG_LIM_RIGHT_CTRL2,
					  SSM3582_LIM_CTRL2_THRES, voltage);
		if (return_code != 0) {
			dev_err(dev, "Unable to set right limiter voltage!\n");
			return return_code;
		}
		return_code =
			regmap_write_bits(ssm3582->regmap,
					  SSM3582_REG_LIM_LEFT_CTRL2,
					  SSM3582_LIM_CTRL2_THRES, voltage);
		if (return_code != 0) {
			dev_err(dev, "Unable to set left limiter voltage!\n");
			return return_code;
		}
	}
#endif // CONFIG_OF

	return 0;
}

void ssm3582_remove(struct snd_soc_component *component)
{
	struct ssm3582_priv *ssm3582 = snd_soc_component_get_drvdata(component);

	regmap_write(ssm3582->regmap, SSM3582_REG_SOFT_RESET,
		     SSM3582_SOFT_RESET_S_RST);
}

static const struct snd_soc_component_driver ssm3582_component_driver = {
	.probe = ssm3582_probe,
	.remove = ssm3582_remove,
	.controls = ssm3582_controls,
	.num_controls = ARRAY_SIZE(ssm3582_controls),
};

int ssm3582_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regmap_config config = ssm3582_regmap;
	struct ssm3582_priv *ssm3582;
	int return_code, id;

	ssm3582 = devm_kzalloc(dev, sizeof(struct ssm3582_priv), GFP_KERNEL);
	if (!ssm3582) {
		return -ENOMEM;
	}

	i2c_set_clientdata(client, ssm3582);

	ssm3582->regmap = devm_regmap_init_i2c(client, &config);
	if (IS_ERR(ssm3582->regmap)) {
		return PTR_ERR(ssm3582->regmap);
	}

	return_code = regmap_read(ssm3582->regmap, SSM3582_REG_VENDOR_ID, &id);
	if (return_code == 0) {
		if (id != SSM3582_VENDOR_ID) {
			dev_err(dev,
				"Wrong vendor id. Expected: 0x%02x, obtained: 0x%02x\n",
				SSM3582_VENDOR_ID, id);
			return -ENODEV;
		}
	} else {
		dev_err(dev, "Error obtaining vendor id\n");
		goto err;
	}

	return_code = regmap_read(ssm3582->regmap, SSM3582_REG_DEVICE_ID1, &id);
	if (return_code == 0) {
		if (id != SSM3582_DEVICE_ID1) {
			dev_err(dev,
				"Wrong device id 1. Expected: 0x%02x, obtained: 0x%02x\n",
				SSM3582_DEVICE_ID1, id);
			return -ENODEV;
		}
	} else {
		dev_err(dev, "Error obtaining device id 1\n");
		goto err;
	}

	return_code = regmap_read(ssm3582->regmap, SSM3582_REG_DEVICE_ID2, &id);
	if (return_code == 0) {
		if (id != SSM3582_DEVICE_ID2) {
			dev_err(dev,
				"Wrong device id 2. Expected: 0x%02x, obtained: 0x%02x\n",
				SSM3582_DEVICE_ID2, id);
			return -ENODEV;
		}
	} else {
		dev_err(dev, "Error obtaining device id 2\n");
		goto err;
	}

	return_code =
		devm_snd_soc_register_component(dev, &ssm3582_component_driver,
						ssm3582_dai,
						ARRAY_SIZE(ssm3582_dai));
	if (return_code < 0) {
		dev_err(dev, "Error registering soc component: %d\n",
			return_code);
		goto err;
	}

	return 0;

err:
	return return_code;
}

#ifdef CONFIG_PM
static int ssm3582_suspend(struct device *dev)
{
	struct ssm3582_priv *ssm3582 = dev_get_drvdata(dev);
	int return_code = 0;

	dev_info(dev, "Powering device down\n");
	return_code = regmap_set_bits(ssm3582->regmap, SSM3582_REG_POWER_CTRL,
				      (SSM3582_POWER_CTRL_TEMP_PWDN |
				       SSM3582_POWER_CTRL_SPWDN));
	if (return_code != 0) {
		dev_err(dev, "Failed to power-down device\n");
	}

	return return_code;
}

static int ssm3582_resume(struct device *dev)
{
	struct ssm3582_priv *ssm3582 = dev_get_drvdata(dev);
	int return_code = 0;

	dev_info(dev, "Powering device up\n");
	return_code = regmap_clear_bits(
		ssm3582->regmap, SSM3582_REG_POWER_CTRL,
		(SSM3582_POWER_CTRL_APWDN_EN | SSM3582_POWER_CTRL_L_PWDN |
		 SSM3582_POWER_CTRL_R_PWDN | SSM3582_POWER_CTRL_TEMP_PWDN |
		 SSM3582_POWER_CTRL_SPWDN));
	if (return_code != 0) {
		dev_err(dev, "Failed to power-up device\n");
	}

	return return_code;
}
#endif // CONFIG_PM

static const struct dev_pm_ops ssm3582_pm_ops = { SET_RUNTIME_PM_OPS(
	ssm3582_suspend, ssm3582_resume, NULL) };

const static struct i2c_device_id ssm3582_id_table[] = {
	{ "ssm3582a", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, ssm3582_id_table);

const static struct of_device_id ssm3582_of_match[] = {
	{ .compatible = "adi,ssm3582a" },
	{},
};

static struct i2c_driver ssm3582_i2c_driver = {
    .driver =
        {
            .name = "ssm3582a",
            .of_match_table = of_match_ptr(ssm3582_of_match),
            .dev_groups = ssm3582_sysfs_groups,
            .pm = &ssm3582_pm_ops,
        },
    .id_table = ssm3582_id_table,
    .address_list = I2C_ADDRS(0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                              0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1F),
    .probe_new = ssm3582_i2c_probe,
};

module_i2c_driver(ssm3582_i2c_driver);

MODULE_AUTHOR("Alexey Pavlov <rnewbie@hotmail.com");
MODULE_DESCRIPTION("ALSA driver for SSM3582");
MODULE_LICENSE("GPL");