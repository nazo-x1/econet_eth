// SPDX-License-Identifier: GPL-2.0-only
/*
 * EcoNet EN7528 DSA Switch Driver — self-contained PoC
 *
 * A standalone DSA driver for the MT7530-compatible switch block embedded in the
 * EcoNet EN7528 SoC.  All switch logic is implemented here; the upstream
 * mt7530.ko module is NOT required.
 *
 * The switch register block sits at offset +0x8000 from the FE base
 * (physical 0x1fb58000, size 0x8000) and is accessed through regmap MMIO.
 *
 * Hardware topology (EN7528):
 *   Port 0-4 : Internal GbE PHYs  (user ports)
 *   Port 5   : unused
 *   Port 6   : CPU port, connected to GDM internally
 *
 * Tag protocol: DSA_TAG_PROTO_MTK (4-byte special tag, handled by net/dsa/tag_mtk.c)
 */

#include <linux/bitfield.h>
#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <net/dsa.h>

#include "mt7530.h"

static struct mt753x_pcs *pcs_to_mt753x_pcs(struct phylink_pcs *pcs)
{
	return container_of(pcs, struct mt753x_pcs, pcs);
}

/* ------------------------------------------------------------------ */
/*  EN7528 force-mode uses MT7531-style per-field force bits           */
/* ------------------------------------------------------------------ */
#define EN7528_FORCE_MODE	MT7531_FORCE_MODE_MASK

/* ------------------------------------------------------------------ */
/*  Private driver state                                               */
/* ------------------------------------------------------------------ */
struct en7528_priv {
	struct device		*dev;
	struct dsa_switch	*ds;
	struct regmap		*regmap;
	struct reset_control	*rstc;
	struct mutex		reg_mutex;
	struct mt7530_port	ports[MT7530_NUM_PORTS];
	struct mt753x_pcs	pcs[MT7530_NUM_PORTS];
	u8			mirror_rx;
	u8			mirror_tx;
	int			irq;
	struct irq_domain	*irq_domain;
	u32			irq_enable;
};

/* ------------------------------------------------------------------ */
/*  Regmap-based register access                                       */
/* ------------------------------------------------------------------ */
static int en7528_write(struct en7528_priv *priv, u32 reg, u32 val)
{
	return regmap_write(priv->regmap, reg, val);
}

static u32 en7528_read(struct en7528_priv *priv, u32 reg)
{
	u32 val = 0;
	regmap_read(priv->regmap, reg, &val);
	return val;
}

static void en7528_rmw(struct en7528_priv *priv, u32 reg, u32 mask, u32 set)
{
	mutex_lock(&priv->reg_mutex);
	regmap_update_bits(priv->regmap, reg, mask, set);
	mutex_unlock(&priv->reg_mutex);
}

static void en7528_set(struct en7528_priv *priv, u32 reg, u32 val)
{
	en7528_rmw(priv, reg, val, val);
}

static void en7528_clear(struct en7528_priv *priv, u32 reg, u32 val)
{
	en7528_rmw(priv, reg, val, 0);
}

/* Read helper for readx_poll_timeout */
struct en7528_poll {
	struct en7528_priv *priv;
	u32 reg;
};

static u32 en7528_poll_read(struct en7528_poll *p)
{
	return en7528_read(p->priv, p->reg);
}

/* ------------------------------------------------------------------ */
/*  PHY indirect access via MT7531_PHY_IAC (Clause 22 & 45)           */
/* ------------------------------------------------------------------ */
static int en7528_phy_read_c22(struct en7528_priv *priv, int port, int regnum)
{
	struct en7528_poll p = { priv, MT7531_PHY_IAC };
	u32 val;
	int ret;

	mutex_lock(&priv->reg_mutex);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & MT7531_PHY_ACS_ST), 20, 100000);
	if (ret < 0)
		goto out;

	val = MT7531_MDIO_CL22_READ | MT7531_MDIO_PHY_ADDR(port) |
	      MT7531_MDIO_REG_ADDR(regnum);
	en7528_write(priv, MT7531_PHY_IAC, val | MT7531_PHY_ACS_ST);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & MT7531_PHY_ACS_ST), 20, 100000);
	if (ret < 0)
		goto out;

	ret = val & MT7531_MDIO_RW_DATA_MASK;
out:
	mutex_unlock(&priv->reg_mutex);
	return ret;
}

static int en7528_phy_write_c22(struct en7528_priv *priv, int port,
				int regnum, u16 data)
{
	struct en7528_poll p = { priv, MT7531_PHY_IAC };
	u32 val, reg;
	int ret;

	mutex_lock(&priv->reg_mutex);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & MT7531_PHY_ACS_ST), 20, 100000);
	if (ret < 0)
		goto out;

	reg = MT7531_MDIO_CL22_WRITE | MT7531_MDIO_PHY_ADDR(port) |
	      MT7531_MDIO_REG_ADDR(regnum) | data;
	en7528_write(priv, MT7531_PHY_IAC, reg | MT7531_PHY_ACS_ST);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & MT7531_PHY_ACS_ST), 20, 100000);
out:
	mutex_unlock(&priv->reg_mutex);
	return ret;
}

static int en7528_phy_read_c45(struct en7528_priv *priv, int port,
			       int devad, int regnum)
{
	struct en7528_poll p = { priv, MT7531_PHY_IAC };
	u32 val, reg;
	int ret;

	mutex_lock(&priv->reg_mutex);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & MT7531_PHY_ACS_ST), 20, 100000);
	if (ret < 0)
		goto out;

	reg = MT7531_MDIO_CL45_ADDR | MT7531_MDIO_PHY_ADDR(port) |
	      MT7531_MDIO_DEV_ADDR(devad) | regnum;
	en7528_write(priv, MT7531_PHY_IAC, reg | MT7531_PHY_ACS_ST);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & MT7531_PHY_ACS_ST), 20, 100000);
	if (ret < 0)
		goto out;

	reg = MT7531_MDIO_CL45_READ | MT7531_MDIO_PHY_ADDR(port) |
	      MT7531_MDIO_DEV_ADDR(devad);
	en7528_write(priv, MT7531_PHY_IAC, reg | MT7531_PHY_ACS_ST);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & MT7531_PHY_ACS_ST), 20, 100000);
	if (ret < 0)
		goto out;

	ret = val & MT7531_MDIO_RW_DATA_MASK;
out:
	mutex_unlock(&priv->reg_mutex);
	return ret;
}

static int en7528_phy_write_c45(struct en7528_priv *priv, int port,
				int devad, int regnum, u16 data)
{
	struct en7528_poll p = { priv, MT7531_PHY_IAC };
	u32 val, reg;
	int ret;

	mutex_lock(&priv->reg_mutex);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & MT7531_PHY_ACS_ST), 20, 100000);
	if (ret < 0)
		goto out;

	reg = MT7531_MDIO_CL45_ADDR | MT7531_MDIO_PHY_ADDR(port) |
	      MT7531_MDIO_DEV_ADDR(devad) | regnum;
	en7528_write(priv, MT7531_PHY_IAC, reg | MT7531_PHY_ACS_ST);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & MT7531_PHY_ACS_ST), 20, 100000);
	if (ret < 0)
		goto out;

	reg = MT7531_MDIO_CL45_WRITE | MT7531_MDIO_PHY_ADDR(port) |
	      MT7531_MDIO_DEV_ADDR(devad) | data;
	en7528_write(priv, MT7531_PHY_IAC, reg | MT7531_PHY_ACS_ST);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & MT7531_PHY_ACS_ST), 20, 100000);
out:
	mutex_unlock(&priv->reg_mutex);
	return ret;
}

/* ------------------------------------------------------------------ */
/*  MDIO bus wrappers (for DSA user_mii_bus)                           */
/* ------------------------------------------------------------------ */
static int en7528_mdio_read_c22(struct mii_bus *bus, int port, int regnum)
{
	return en7528_phy_read_c22(bus->priv, port, regnum);
}

static int en7528_mdio_write_c22(struct mii_bus *bus, int port, int regnum,
				 u16 val)
{
	return en7528_phy_write_c22(bus->priv, port, regnum, val);
}

static int en7528_mdio_read_c45(struct mii_bus *bus, int port, int devad,
				int regnum)
{
	return en7528_phy_read_c45(bus->priv, port, devad, regnum);
}

static int en7528_mdio_write_c45(struct mii_bus *bus, int port, int devad,
				 int regnum, u16 val)
{
	return en7528_phy_write_c45(bus->priv, port, devad, regnum, val);
}

/* ------------------------------------------------------------------ */
/*  FDB (Forwarding Database) operations                               */
/* ------------------------------------------------------------------ */
static int en7528_fdb_cmd(struct en7528_priv *priv, enum mt7530_fdb_cmd cmd,
			  u32 *rsp)
{
	struct en7528_poll p = { priv, MT7530_ATC };
	u32 val;
	int ret;

	val = ATC_BUSY | ATC_MAT(0) | cmd;
	en7528_write(priv, MT7530_ATC, val);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & ATC_BUSY), 20, 20000);
	if (ret < 0)
		return ret;

	val = en7528_read(priv, MT7530_ATC);
	if ((cmd == MT7530_FDB_READ) && (val & ATC_INVALID))
		return -EINVAL;

	if (rsp)
		*rsp = val;
	return 0;
}

static void en7528_fdb_read(struct en7528_priv *priv, struct mt7530_fdb *fdb)
{
	u32 reg[3];

	for (int i = 0; i < 3; i++)
		reg[i] = en7528_read(priv, MT7530_TSRA1 + (i * 4));

	fdb->vid = (reg[1] >> CVID) & CVID_MASK;
	fdb->aging = (reg[2] >> AGE_TIMER) & AGE_TIMER_MASK;
	fdb->port_mask = (reg[2] >> PORT_MAP) & PORT_MAP_MASK;
	fdb->mac[0] = (reg[0] >> MAC_BYTE_0) & MAC_BYTE_MASK;
	fdb->mac[1] = (reg[0] >> MAC_BYTE_1) & MAC_BYTE_MASK;
	fdb->mac[2] = (reg[0] >> MAC_BYTE_2) & MAC_BYTE_MASK;
	fdb->mac[3] = (reg[0] >> MAC_BYTE_3) & MAC_BYTE_MASK;
	fdb->mac[4] = (reg[1] >> MAC_BYTE_4) & MAC_BYTE_MASK;
	fdb->mac[5] = (reg[1] >> MAC_BYTE_5) & MAC_BYTE_MASK;
	fdb->noarp = ((reg[2] >> ENT_STATUS) & ENT_STATUS_MASK) == STATIC_ENT;
}

static void en7528_fdb_write(struct en7528_priv *priv, u16 vid, u8 port_mask,
			     const u8 *mac, u8 aging, u8 type)
{
	u32 reg[3] = {};

	reg[1] |= vid & CVID_MASK;
	reg[1] |= ATA2_IVL;
	reg[1] |= ATA2_FID(FID_BRIDGED);
	reg[2] |= (aging & AGE_TIMER_MASK) << AGE_TIMER;
	reg[2] |= (port_mask & PORT_MAP_MASK) << PORT_MAP;
	reg[2] |= (type & ENT_STATUS_MASK) << ENT_STATUS;
	reg[1] |= mac[5] << MAC_BYTE_5;
	reg[1] |= mac[4] << MAC_BYTE_4;
	reg[0] |= mac[3] << MAC_BYTE_3;
	reg[0] |= mac[2] << MAC_BYTE_2;
	reg[0] |= mac[1] << MAC_BYTE_1;
	reg[0] |= mac[0] << MAC_BYTE_0;

	for (int i = 0; i < 3; i++)
		en7528_write(priv, MT7530_ATA1 + (i * 4), reg[i]);
}

/* ------------------------------------------------------------------ */
/*  VLAN table operations                                              */
/* ------------------------------------------------------------------ */
static int en7528_vlan_cmd(struct en7528_priv *priv,
			   enum mt7530_vlan_cmd cmd, u16 vid)
{
	struct en7528_poll p = { priv, MT7530_VTCR };
	u32 val;
	int ret;

	val = VTCR_BUSY | VTCR_FUNC(cmd) | vid;
	en7528_write(priv, MT7530_VTCR, val);

	ret = readx_poll_timeout(en7528_poll_read, &p, val,
				 !(val & VTCR_BUSY), 20, 20000);
	if (ret < 0)
		return ret;

	if (en7528_read(priv, MT7530_VTCR) & VTCR_INVALID)
		return -EINVAL;

	return 0;
}

static int en7528_setup_vlan0(struct en7528_priv *priv)
{
	u32 val = IVL_MAC | EG_CON | PORT_MEM(MT7530_ALL_MEMBERS) |
		  FID(FID_BRIDGED) | VLAN_VALID;
	en7528_write(priv, MT7530_VAWD1, val);
	return en7528_vlan_cmd(priv, MT7530_VTCR_WR_VID, 0);
}

/* ------------------------------------------------------------------ */
/*  MIB counters                                                       */
/* ------------------------------------------------------------------ */
static const struct mt7530_mib_desc en7528_mib[] = {
	MIB_DESC(1, 0x00, "TxDrop"),
	MIB_DESC(1, 0x04, "TxCrcErr"),
	MIB_DESC(1, 0x08, "TxUnicast"),
	MIB_DESC(1, 0x0c, "TxMulticast"),
	MIB_DESC(1, 0x10, "TxBroadcast"),
	MIB_DESC(1, 0x14, "TxCollision"),
	MIB_DESC(1, 0x18, "TxSingleCollision"),
	MIB_DESC(1, 0x1c, "TxMultipleCollision"),
	MIB_DESC(1, 0x20, "TxDeferred"),
	MIB_DESC(1, 0x24, "TxLateCollision"),
	MIB_DESC(1, 0x28, "TxExcessiveCollision"),
	MIB_DESC(1, 0x2c, "TxPause"),
	MIB_DESC(1, 0x30, "TxPktSz64"),
	MIB_DESC(1, 0x34, "TxPktSz65To127"),
	MIB_DESC(1, 0x38, "TxPktSz128To255"),
	MIB_DESC(1, 0x3c, "TxPktSz256To511"),
	MIB_DESC(1, 0x40, "TxPktSz512To1023"),
	MIB_DESC(1, 0x44, "Tx1024ToMax"),
	MIB_DESC(2, 0x48, "TxBytes"),
	MIB_DESC(1, 0x60, "RxDrop"),
	MIB_DESC(1, 0x64, "RxFiltering"),
	MIB_DESC(1, 0x68, "RxUnicast"),
	MIB_DESC(1, 0x6c, "RxMulticast"),
	MIB_DESC(1, 0x70, "RxBroadcast"),
	MIB_DESC(1, 0x74, "RxAlignErr"),
	MIB_DESC(1, 0x78, "RxCrcErr"),
	MIB_DESC(1, 0x7c, "RxUnderSizeErr"),
	MIB_DESC(1, 0x80, "RxFragErr"),
	MIB_DESC(1, 0x84, "RxOverSzErr"),
	MIB_DESC(1, 0x88, "RxJabberErr"),
	MIB_DESC(1, 0x8c, "RxPause"),
	MIB_DESC(1, 0x90, "RxPktSz64"),
	MIB_DESC(1, 0x94, "RxPktSz65To127"),
	MIB_DESC(1, 0x98, "RxPktSz128To255"),
	MIB_DESC(1, 0x9c, "RxPktSz256To511"),
	MIB_DESC(1, 0xa0, "RxPktSz512To1023"),
	MIB_DESC(1, 0xa4, "RxPktSz1024ToMax"),
	MIB_DESC(2, 0xa8, "RxBytes"),
	MIB_DESC(1, 0xb0, "RxCtrlDrop"),
	MIB_DESC(1, 0xb4, "RxIngressDrop"),
	MIB_DESC(1, 0xb8, "RxArlDrop"),
};

/* ------------------------------------------------------------------ */
/*  Switch setup helpers                                               */
/* ------------------------------------------------------------------ */
static void en7528_trap_frames(struct en7528_priv *priv)
{
	/* Trap 802.1X PAE + BPDU to CPU, egress VLAN-untagged */
	en7528_rmw(priv, MT753X_BPC,
		   PAE_BPDU_FR | PAE_EG_TAG_MASK | PAE_PORT_FW_MASK |
		   BPDU_EG_TAG_MASK | BPDU_PORT_FW_MASK,
		   PAE_BPDU_FR | PAE_EG_TAG(MT7530_VLAN_EG_UNTAGGED) |
		   PAE_PORT_FW(TO_CPU_FW_CPU_ONLY) |
		   BPDU_EG_TAG(MT7530_VLAN_EG_UNTAGGED) |
		   TO_CPU_FW_CPU_ONLY);

	/* :01 and :02 */
	en7528_rmw(priv, MT753X_RGAC1,
		   R02_BPDU_FR | R02_EG_TAG_MASK | R02_PORT_FW_MASK |
		   R01_BPDU_FR | R01_EG_TAG_MASK | R01_PORT_FW_MASK,
		   R02_BPDU_FR | R02_EG_TAG(MT7530_VLAN_EG_UNTAGGED) |
		   R02_PORT_FW(TO_CPU_FW_CPU_ONLY) |
		   R01_BPDU_FR | R01_EG_TAG(MT7530_VLAN_EG_UNTAGGED) |
		   TO_CPU_FW_CPU_ONLY);

	/* :03 and :0E */
	en7528_rmw(priv, MT753X_RGAC2,
		   R0E_BPDU_FR | R0E_EG_TAG_MASK | R0E_PORT_FW_MASK |
		   R03_BPDU_FR | R03_EG_TAG_MASK | R03_PORT_FW_MASK,
		   R0E_BPDU_FR | R0E_EG_TAG(MT7530_VLAN_EG_UNTAGGED) |
		   R0E_PORT_FW(TO_CPU_FW_CPU_ONLY) |
		   R03_BPDU_FR | R03_EG_TAG(MT7530_VLAN_EG_UNTAGGED) |
		   TO_CPU_FW_CPU_ONLY);
}

static void en7528_cpu_port_enable(struct en7528_priv *priv,
				   struct dsa_switch *ds, int port)
{
	/* Enable special tag (MTK header) on CPU port */
	en7528_write(priv, MT7530_PVC_P(port), PORT_SPEC_TAG);

	/* Enable flooding on the CPU port */
	en7528_set(priv, MT753X_MFC,
		   BC_FFP(BIT(port)) | UNM_FFP(BIT(port)) |
		   UNU_FFP(BIT(port)));

	/* Add CPU port to the CPU port bitmap (MT7531+ style) */
	en7528_set(priv, MT7531_CFC, MT7531_CPU_PMAP(BIT(port)));

	/* CPU port connects to all user ports */
	en7528_write(priv, MT7530_PCR_P(port),
		     PCR_MATRIX(dsa_user_ports(ds)));

	/* Fallback mode for independent VLAN learning */
	en7528_rmw(priv, MT7530_PCR_P(port), PCR_PORT_VLAN_MASK,
		   MT7530_PORT_FALLBACK_MODE);
}

static void en7528_mib_reset(struct en7528_priv *priv)
{
	en7528_write(priv, MT7530_MIB_CCR, CCR_MIB_FLUSH);
	en7528_write(priv, MT7530_MIB_CCR, CCR_MIB_ACTIVATE);
}

/* ------------------------------------------------------------------ */
/*  DSA switch ops                                                     */
/* ------------------------------------------------------------------ */
static enum dsa_tag_protocol en7528_get_tag_protocol(struct dsa_switch *ds,
						     int port,
						     enum dsa_tag_protocol mp)
{
	return DSA_TAG_PROTO_MTK;
}

static struct dsa_port *
en7528_preferred_default_local_cpu_port(struct dsa_switch *ds)
{
	struct dsa_port *cpu_dp = dsa_to_port(ds, 6);

	if (dsa_port_is_cpu(cpu_dp))
		return cpu_dp;
	return NULL;
}

static void en7528_get_strings(struct dsa_switch *ds, int port,
			       u32 stringset, uint8_t *data)
{
	if (stringset != ETH_SS_STATS)
		return;
	for (int i = 0; i < ARRAY_SIZE(en7528_mib); i++)
		ethtool_puts(&data, en7528_mib[i].name);
}

static void en7528_get_ethtool_stats(struct dsa_switch *ds, int port,
				     uint64_t *data)
{
	struct en7528_priv *priv = ds->priv;

	for (int i = 0; i < ARRAY_SIZE(en7528_mib); i++) {
		u32 reg = MT7530_PORT_MIB_COUNTER(port) + en7528_mib[i].offset;
		data[i] = en7528_read(priv, reg);
		if (en7528_mib[i].size == 2)
			data[i] |= (u64)en7528_read(priv, reg + 4) << 32;
	}
}

static int en7528_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	return sset == ETH_SS_STATS ? ARRAY_SIZE(en7528_mib) : 0;
}

static int en7528_set_ageing_time(struct dsa_switch *ds, unsigned int msecs)
{
	struct en7528_priv *priv = ds->priv;
	unsigned int secs = msecs / 1000;
	unsigned int age_count = 0, age_unit = 0;
	unsigned int error = -1U;

	if (secs < 1 || secs > (AGE_CNT_MAX + 1) * (AGE_UNIT_MAX + 1))
		return -ERANGE;

	for (unsigned int c = 0; c <= AGE_CNT_MAX; c++) {
		unsigned int u = secs / (c + 1) - 1;
		unsigned int e;

		if (u > AGE_UNIT_MAX)
			continue;
		e = secs - (c + 1) * (u + 1);
		if (e < error) {
			error = e;
			age_count = c;
			age_unit = u;
		}
		if (!error)
			break;
	}

	en7528_write(priv, MT7530_AAC, AGE_CNT(age_count) | AGE_UNIT(age_unit));
	return 0;
}

static int en7528_port_enable(struct dsa_switch *ds, int port,
			      struct phy_device *phy)
{
	struct en7528_priv *priv = ds->priv;

	mutex_lock(&priv->reg_mutex);
	if (dsa_port_is_user(dsa_to_port(ds, port))) {
		struct dsa_port *cpu_dp = dsa_to_port(ds, port)->cpu_dp;

		priv->ports[port].pm |= PCR_MATRIX(BIT(cpu_dp->index));
	}
	priv->ports[port].enable = true;
	en7528_rmw(priv, MT7530_PCR_P(port), PCR_MATRIX_MASK,
		   priv->ports[port].pm);
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static void en7528_port_disable(struct dsa_switch *ds, int port)
{
	struct en7528_priv *priv = ds->priv;

	mutex_lock(&priv->reg_mutex);
	priv->ports[port].enable = false;
	en7528_rmw(priv, MT7530_PCR_P(port), PCR_MATRIX_MASK, PCR_MATRIX_CLR);
	mutex_unlock(&priv->reg_mutex);
}

static int en7528_port_change_mtu(struct dsa_switch *ds, int port, int new_mtu)
{
	struct en7528_priv *priv = ds->priv;
	int length;
	u32 val;

	if (!dsa_is_cpu_port(ds, port))
		return 0;

	mutex_lock(&priv->reg_mutex);
	val = en7528_read(priv, MT7530_GMACCR);
	val &= ~MAX_RX_PKT_LEN_MASK;

	length = new_mtu + ETH_HLEN + MTK_HDR_LEN + ETH_FCS_LEN;
	if (length <= 1522)
		val |= MAX_RX_PKT_LEN_1522;
	else if (length <= 1536)
		val |= MAX_RX_PKT_LEN_1536;
	else if (length <= 1552)
		val |= MAX_RX_PKT_LEN_1552;
	else {
		val &= ~MAX_RX_JUMBO_MASK;
		val |= MAX_RX_JUMBO(DIV_ROUND_UP(length, 1024));
		val |= MAX_RX_PKT_LEN_JUMBO;
	}
	en7528_write(priv, MT7530_GMACCR, val);
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static int en7528_port_max_mtu(struct dsa_switch *ds, int port)
{
	return MT7530_MAX_MTU;
}

static void en7528_stp_state_set(struct dsa_switch *ds, int port, u8 state)
{
	struct en7528_priv *priv = ds->priv;
	u32 stp_state;

	switch (state) {
	case BR_STATE_DISABLED:
		stp_state = MT7530_STP_DISABLED;
		break;
	case BR_STATE_BLOCKING:
	case BR_STATE_LISTENING:
		stp_state = MT7530_STP_BLOCKING;
		break;
	case BR_STATE_LEARNING:
		stp_state = MT7530_STP_LEARNING;
		break;
	case BR_STATE_FORWARDING:
	default:
		stp_state = MT7530_STP_FORWARDING;
		break;
	}
	en7528_rmw(priv, MT7530_SSP_P(port), FID_PST_MASK(FID_BRIDGED),
		   FID_PST(FID_BRIDGED, stp_state));
}

static void en7528_update_port_member(struct en7528_priv *priv, int port,
				      const struct net_device *bridge_dev,
				      bool join)
{
	struct dsa_port *dp = dsa_to_port(priv->ds, port), *other_dp;
	struct mt7530_port *p = &priv->ports[port], *other_p;
	u32 port_bitmap = BIT(dp->cpu_dp->index);
	int other_port;

	dsa_switch_for_each_user_port(other_dp, priv->ds) {
		other_port = other_dp->index;
		other_p = &priv->ports[other_port];

		if (dp == other_dp)
			continue;
		if (!dsa_port_offloads_bridge_dev(other_dp, bridge_dev))
			continue;

		if (join && !(p->isolated && other_p->isolated)) {
			other_p->pm |= PCR_MATRIX(BIT(port));
			port_bitmap |= BIT(other_port);
		} else {
			other_p->pm &= ~PCR_MATRIX(BIT(port));
		}

		if (other_p->enable)
			en7528_rmw(priv, MT7530_PCR_P(other_port),
				   PCR_MATRIX_MASK, other_p->pm);
	}

	p->pm = PCR_MATRIX(port_bitmap);
	if (p->enable)
		en7528_rmw(priv, MT7530_PCR_P(port), PCR_MATRIX_MASK, p->pm);
}

static int en7528_port_bridge_join(struct dsa_switch *ds, int port,
				   struct dsa_bridge bridge,
				   bool *tx_fwd_offload,
				   struct netlink_ext_ack *extack)
{
	struct en7528_priv *priv = ds->priv;

	mutex_lock(&priv->reg_mutex);
	en7528_update_port_member(priv, port, bridge.dev, true);
	en7528_rmw(priv, MT7530_PCR_P(port), PCR_PORT_VLAN_MASK,
		   MT7530_PORT_FALLBACK_MODE);
	mutex_unlock(&priv->reg_mutex);
	return 0;
}

static void en7528_port_bridge_leave(struct dsa_switch *ds, int port,
				     struct dsa_bridge bridge)
{
	struct en7528_priv *priv = ds->priv;

	mutex_lock(&priv->reg_mutex);
	en7528_update_port_member(priv, port, bridge.dev, false);
	en7528_rmw(priv, MT7530_PCR_P(port), PCR_PORT_VLAN_MASK,
		   MT7530_PORT_MATRIX_MODE);
	mutex_unlock(&priv->reg_mutex);
}

static int en7528_port_pre_bridge_flags(struct dsa_switch *ds, int port,
					struct switchdev_brport_flags flags,
					struct netlink_ext_ack *extack)
{
	if (flags.mask & ~(BR_LEARNING | BR_FLOOD | BR_MCAST_FLOOD |
			   BR_BCAST_FLOOD | BR_ISOLATED))
		return -EINVAL;
	return 0;
}

static int en7528_port_bridge_flags(struct dsa_switch *ds, int port,
				    struct switchdev_brport_flags flags,
				    struct netlink_ext_ack *extack)
{
	struct en7528_priv *priv = ds->priv;

	if (flags.mask & BR_LEARNING)
		en7528_rmw(priv, MT7530_PSC_P(port), SA_DIS,
			   flags.val & BR_LEARNING ? 0 : SA_DIS);
	if (flags.mask & BR_FLOOD)
		en7528_rmw(priv, MT753X_MFC, UNU_FFP(BIT(port)),
			   flags.val & BR_FLOOD ? UNU_FFP(BIT(port)) : 0);
	if (flags.mask & BR_MCAST_FLOOD)
		en7528_rmw(priv, MT753X_MFC, UNM_FFP(BIT(port)),
			   flags.val & BR_MCAST_FLOOD ? UNM_FFP(BIT(port)) : 0);
	if (flags.mask & BR_BCAST_FLOOD)
		en7528_rmw(priv, MT753X_MFC, BC_FFP(BIT(port)),
			   flags.val & BR_BCAST_FLOOD ? BC_FFP(BIT(port)) : 0);
	if (flags.mask & BR_ISOLATED) {
		struct dsa_port *dp = dsa_to_port(ds, port);

		priv->ports[port].isolated = !!(flags.val & BR_ISOLATED);
		mutex_lock(&priv->reg_mutex);
		en7528_update_port_member(priv, port,
					  dsa_port_bridge_dev_get(dp), true);
		mutex_unlock(&priv->reg_mutex);
	}
	return 0;
}

static int en7528_port_fdb_add(struct dsa_switch *ds, int port,
			       const unsigned char *addr, u16 vid,
			       struct dsa_db db)
{
	struct en7528_priv *priv = ds->priv;
	int ret;

	mutex_lock(&priv->reg_mutex);
	en7528_fdb_write(priv, vid, BIT(port), addr, -1, STATIC_ENT);
	ret = en7528_fdb_cmd(priv, MT7530_FDB_WRITE, NULL);
	mutex_unlock(&priv->reg_mutex);
	return ret;
}

static int en7528_port_fdb_del(struct dsa_switch *ds, int port,
			       const unsigned char *addr, u16 vid,
			       struct dsa_db db)
{
	struct en7528_priv *priv = ds->priv;
	int ret;

	mutex_lock(&priv->reg_mutex);
	en7528_fdb_write(priv, vid, BIT(port), addr, -1, STATIC_EMP);
	ret = en7528_fdb_cmd(priv, MT7530_FDB_WRITE, NULL);
	mutex_unlock(&priv->reg_mutex);
	return ret;
}

static int en7528_port_fdb_dump(struct dsa_switch *ds, int port,
				dsa_fdb_dump_cb_t *cb, void *data)
{
	struct en7528_priv *priv = ds->priv;
	struct mt7530_fdb _fdb = {};
	int cnt = MT7530_NUM_FDB_RECORDS;
	int ret;
	u32 rsp = 0;

	mutex_lock(&priv->reg_mutex);
	ret = en7528_fdb_cmd(priv, MT7530_FDB_START, &rsp);
	if (ret < 0)
		goto out;

	do {
		if (rsp & ATC_SRCH_HIT) {
			en7528_fdb_read(priv, &_fdb);
			if (_fdb.port_mask & BIT(port)) {
				ret = cb(_fdb.mac, _fdb.vid, _fdb.noarp, data);
				if (ret < 0)
					break;
			}
		}
	} while (--cnt &&
		 !(rsp & ATC_SRCH_END) &&
		 !en7528_fdb_cmd(priv, MT7530_FDB_NEXT, &rsp));
out:
	mutex_unlock(&priv->reg_mutex);
	return 0;
}

static int en7528_port_vlan_filtering(struct dsa_switch *ds, int port,
				      bool vlan_filtering,
				      struct netlink_ext_ack *extack)
{
	struct en7528_priv *priv = ds->priv;
	struct dsa_port *cpu_dp = dsa_to_port(ds, port)->cpu_dp;

	if (vlan_filtering) {
		/* User port: security mode + VLAN user attribute */
		if (dsa_is_user_port(ds, port)) {
			en7528_rmw(priv, MT7530_PCR_P(port),
				   PCR_PORT_VLAN_MASK,
				   MT7530_PORT_SECURITY_MODE);
			en7528_rmw(priv, MT7530_PPBV1_P(port),
				   G0_PORT_VID_MASK,
				   G0_PORT_VID(priv->ports[port].pvid));
			if (!priv->ports[port].pvid)
				en7528_rmw(priv, MT7530_PVC_P(port),
					   ACC_FRM_MASK,
					   MT7530_VLAN_ACC_TAGGED);
			en7528_rmw(priv, MT7530_PVC_P(port),
				   VLAN_ATTR_MASK | PVC_EG_TAG_MASK,
				   VLAN_ATTR(MT7530_VLAN_USER) |
				   PVC_EG_TAG(MT7530_VLAN_EG_DISABLED));
		} else {
			en7528_rmw(priv, MT7530_PVC_P(port), VLAN_ATTR_MASK,
				   VLAN_ATTR(MT7530_VLAN_USER));
		}
		/* Also set CPU port to VLAN-aware */
		en7528_rmw(priv, MT7530_PVC_P(cpu_dp->index), VLAN_ATTR_MASK,
			   VLAN_ATTR(MT7530_VLAN_USER));
	} else {
		if (dsa_port_bridge_dev_get(dsa_to_port(ds, port)))
			en7528_rmw(priv, MT7530_PCR_P(port),
				   PCR_PORT_VLAN_MASK,
				   MT7530_PORT_FALLBACK_MODE);
		en7528_rmw(priv, MT7530_PVC_P(port),
			   VLAN_ATTR_MASK | PVC_EG_TAG_MASK | ACC_FRM_MASK,
			   VLAN_ATTR(MT7530_VLAN_TRANSPARENT) |
			   PVC_EG_TAG(MT7530_VLAN_EG_CONSISTENT) |
			   MT7530_VLAN_ACC_ALL);
		en7528_rmw(priv, MT7530_PPBV1_P(port), G0_PORT_VID_MASK,
			   G0_PORT_VID_DEF);
	}
	return 0;
}

static int en7528_port_vlan_add(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan,
				struct netlink_ext_ack *extack)
{
	struct en7528_priv *priv = ds->priv;
	bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;
	u8 new_members;
	u32 val;

	mutex_lock(&priv->reg_mutex);

	/* Read existing VLAN entry */
	en7528_vlan_cmd(priv, MT7530_VTCR_RD_VID, vlan->vid);
	val = en7528_read(priv, MT7530_VAWD1);
	new_members = ((val >> PORT_MEM_SHFT) & PORT_MEM_MASK) | BIT(port);

	val = IVL_MAC | VTAG_EN | PORT_MEM(new_members) | FID(FID_BRIDGED) |
	      VLAN_VALID;
	en7528_write(priv, MT7530_VAWD1, val);

	if (dsa_is_cpu_port(ds, port))
		val = MT7530_VLAN_EGRESS_STACK;
	else if (untagged)
		val = MT7530_VLAN_EGRESS_UNTAG;
	else
		val = MT7530_VLAN_EGRESS_TAG;
	en7528_rmw(priv, MT7530_VAWD2, ETAG_CTRL_P_MASK(port),
		   ETAG_CTRL_P(port, val));

	en7528_vlan_cmd(priv, MT7530_VTCR_WR_VID, vlan->vid);

	if (pvid) {
		priv->ports[port].pvid = vlan->vid;
		en7528_rmw(priv, MT7530_PVC_P(port), ACC_FRM_MASK,
			   MT7530_VLAN_ACC_ALL);
		if (dsa_port_is_vlan_filtering(dsa_to_port(ds, port)))
			en7528_rmw(priv, MT7530_PPBV1_P(port),
				   G0_PORT_VID_MASK,
				   G0_PORT_VID(vlan->vid));
	}

	mutex_unlock(&priv->reg_mutex);
	return 0;
}

static int en7528_port_vlan_del(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan)
{
	struct en7528_priv *priv = ds->priv;
	u8 new_members;
	u32 val;

	mutex_lock(&priv->reg_mutex);

	en7528_vlan_cmd(priv, MT7530_VTCR_RD_VID, vlan->vid);
	val = en7528_read(priv, MT7530_VAWD1);
	if (!(val & VLAN_VALID))
		goto out;

	new_members = ((val >> PORT_MEM_SHFT) & PORT_MEM_MASK) & ~BIT(port);
	if (new_members) {
		val = IVL_MAC | VTAG_EN | PORT_MEM(new_members) | VLAN_VALID;
		en7528_write(priv, MT7530_VAWD1, val);
	} else {
		en7528_write(priv, MT7530_VAWD1, 0);
		en7528_write(priv, MT7530_VAWD2, 0);
	}
	en7528_vlan_cmd(priv, MT7530_VTCR_WR_VID, vlan->vid);

	if (priv->ports[port].pvid == vlan->vid) {
		priv->ports[port].pvid = G0_PORT_VID_DEF;
		if (dsa_port_is_vlan_filtering(dsa_to_port(ds, port)))
			en7528_rmw(priv, MT7530_PVC_P(port), ACC_FRM_MASK,
				   MT7530_VLAN_ACC_TAGGED);
		en7528_rmw(priv, MT7530_PPBV1_P(port), G0_PORT_VID_MASK,
			   G0_PORT_VID_DEF);
	}
out:
	mutex_unlock(&priv->reg_mutex);
	return 0;
}

static int en7528_port_mirror_add(struct dsa_switch *ds, int port,
				  struct dsa_mall_mirror_tc_entry *mirror,
				  bool ingress, struct netlink_ext_ack *extack)
{
	struct en7528_priv *priv = ds->priv;
	u32 val;

	if ((ingress ? priv->mirror_rx : priv->mirror_tx) & BIT(port))
		return -EEXIST;

	val = en7528_read(priv, MT7531_CFC);
	if (val & MT7531_MIRROR_EN) {
		int cur = MT7531_MIRROR_PORT_GET(val);
		if (cur != mirror->to_local_port)
			return -EEXIST;
	}

	val |= MT7531_MIRROR_EN;
	val &= ~MT7531_MIRROR_PORT_MASK;
	val |= MT7531_MIRROR_PORT_SET(mirror->to_local_port);
	en7528_write(priv, MT7531_CFC, val);

	val = en7528_read(priv, MT7530_PCR_P(port));
	if (ingress) {
		val |= PORT_RX_MIR;
		priv->mirror_rx |= BIT(port);
	} else {
		val |= PORT_TX_MIR;
		priv->mirror_tx |= BIT(port);
	}
	en7528_write(priv, MT7530_PCR_P(port), val);
	return 0;
}

static void en7528_port_mirror_del(struct dsa_switch *ds, int port,
				   struct dsa_mall_mirror_tc_entry *mirror)
{
	struct en7528_priv *priv = ds->priv;
	u32 val;

	val = en7528_read(priv, MT7530_PCR_P(port));
	if (mirror->ingress) {
		val &= ~PORT_RX_MIR;
		priv->mirror_rx &= ~BIT(port);
	} else {
		val &= ~PORT_TX_MIR;
		priv->mirror_tx &= ~BIT(port);
	}
	en7528_write(priv, MT7530_PCR_P(port), val);

	if (!priv->mirror_rx && !priv->mirror_tx) {
		val = en7528_read(priv, MT7531_CFC);
		val &= ~MT7531_MIRROR_EN;
		en7528_write(priv, MT7531_CFC, val);
	}
}

/* ------------------------------------------------------------------ */
/*  Phylink                                                            */
/* ------------------------------------------------------------------ */
static void en7528_phylink_get_caps(struct dsa_switch *ds, int port,
				    struct phylink_config *config)
{
	config->mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE;

	switch (port) {
	case 0 ... 4:
		__set_bit(PHY_INTERFACE_MODE_INTERNAL,
			  config->supported_interfaces);
		config->mac_capabilities |= MAC_10 | MAC_100 | MAC_1000FD;
		break;
	case 6:
		__set_bit(PHY_INTERFACE_MODE_INTERNAL,
			  config->supported_interfaces);
		config->mac_capabilities |= MAC_10 | MAC_100 | MAC_1000FD;
		break;
	}
}

static void en7528_pcs_get_state(struct phylink_pcs *pcs,
				 struct phylink_link_state *state)
{
	struct en7528_priv *priv = (void *)pcs_to_mt753x_pcs(pcs)->priv;
	int port = pcs_to_mt753x_pcs(pcs)->port;
	u32 pmsr;

	pmsr = en7528_read(priv, MT7530_PMSR_P(port));
	state->link = !!(pmsr & PMSR_LINK);
	state->an_complete = state->link;
	state->duplex = !!(pmsr & PMSR_DPX);

	switch (pmsr & PMSR_SPEED_MASK) {
	case PMSR_SPEED_10:
		state->speed = SPEED_10;
		break;
	case PMSR_SPEED_100:
		state->speed = SPEED_100;
		break;
	case PMSR_SPEED_1000:
		state->speed = SPEED_1000;
		break;
	default:
		state->speed = SPEED_UNKNOWN;
		break;
	}
	state->pause &= ~(MLO_PAUSE_RX | MLO_PAUSE_TX);
	if (pmsr & PMSR_RX_FC)
		state->pause |= MLO_PAUSE_RX;
	if (pmsr & PMSR_TX_FC)
		state->pause |= MLO_PAUSE_TX;
}

static int en7528_pcs_validate(struct phylink_pcs *pcs,
			       unsigned long *supported,
			       const struct phylink_link_state *state)
{
	return 0;
}

static int en7528_pcs_config(struct phylink_pcs *pcs, unsigned int neg_mode,
			     phy_interface_t interface,
			     const unsigned long *advertising,
			     bool permit_pause_to_mac)
{
	return 0;
}

static void en7528_pcs_an_restart(struct phylink_pcs *pcs)
{
}

static const struct phylink_pcs_ops en7528_pcs_ops = {
	.pcs_validate	= en7528_pcs_validate,
	.pcs_get_state	= en7528_pcs_get_state,
	.pcs_config	= en7528_pcs_config,
	.pcs_an_restart	= en7528_pcs_an_restart,
};

static struct phylink_pcs *
en7528_phylink_mac_select_pcs(struct phylink_config *config,
			      phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct en7528_priv *priv = dp->ds->priv;

	return &priv->pcs[dp->index].pcs;
}

static void en7528_phylink_mac_config(struct phylink_config *config,
				      unsigned int mode,
				      const struct phylink_link_state *state)
{
	/* EN7528 ports are all internal, no MAC-level configuration needed */
}

static void en7528_phylink_mac_link_down(struct phylink_config *config,
					 unsigned int mode,
					 phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct en7528_priv *priv = dp->ds->priv;

	en7528_clear(priv, MT753X_PMCR_P(dp->index), PMCR_LINK_SETTINGS_MASK);
}

static void en7528_phylink_mac_link_up(struct phylink_config *config,
				       struct phy_device *phydev,
				       unsigned int mode,
				       phy_interface_t interface,
				       int speed, int duplex,
				       bool tx_pause, bool rx_pause)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct en7528_priv *priv = dp->ds->priv;
	u32 mcr;

	mcr = PMCR_MAC_RX_EN | PMCR_MAC_TX_EN | PMCR_FORCE_LNK;

	switch (speed) {
	case SPEED_1000:
		mcr |= PMCR_FORCE_SPEED_1000;
		break;
	case SPEED_100:
		mcr |= PMCR_FORCE_SPEED_100;
		break;
	}
	if (duplex == DUPLEX_FULL) {
		mcr |= PMCR_FORCE_FDX;
		if (tx_pause)
			mcr |= PMCR_FORCE_TX_FC_EN;
		if (rx_pause)
			mcr |= PMCR_FORCE_RX_FC_EN;
	}

	if (mode == MLO_AN_PHY && phydev && phy_init_eee(phydev, false) >= 0) {
		if (speed == SPEED_1000)
			mcr |= PMCR_FORCE_EEE1G;
		else if (speed == SPEED_100)
			mcr |= PMCR_FORCE_EEE100;
	}

	en7528_set(priv, MT753X_PMCR_P(dp->index), mcr);
}

static const struct phylink_mac_ops en7528_phylink_mac_ops = {
	.mac_select_pcs	= en7528_phylink_mac_select_pcs,
	.mac_config	= en7528_phylink_mac_config,
	.mac_link_down	= en7528_phylink_mac_link_down,
	.mac_link_up	= en7528_phylink_mac_link_up,
};

/* ------------------------------------------------------------------ */
/*  IRQ handling (MT7988/EN7581 style — direct register write)         */
/* ------------------------------------------------------------------ */
static irqreturn_t en7528_irq_thread_fn(int irq, void *dev_id)
{
	struct en7528_priv *priv = dev_id;
	bool handled = false;
	u32 val;

	val = en7528_read(priv, MT7530_SYS_INT_STS);
	en7528_write(priv, MT7530_SYS_INT_STS, val);

	for (int p = 0; p < MT7530_NUM_PHYS; p++) {
		if (BIT(p) & val) {
			unsigned int virq = irq_find_mapping(priv->irq_domain, p);
			handle_nested_irq(virq);
			handled = true;
		}
	}
	return IRQ_RETVAL(handled);
}

static void en7528_irq_mask(struct irq_data *d)
{
	struct en7528_priv *priv = irq_data_get_irq_chip_data(d);

	priv->irq_enable &= ~BIT(d->hwirq);
	en7528_write(priv, MT7530_SYS_INT_EN, priv->irq_enable);
}

static void en7528_irq_unmask(struct irq_data *d)
{
	struct en7528_priv *priv = irq_data_get_irq_chip_data(d);

	priv->irq_enable |= BIT(d->hwirq);
	en7528_write(priv, MT7530_SYS_INT_EN, priv->irq_enable);
}

static struct irq_chip en7528_irq_chip = {
	.name		= "en7528-dsa",
	.irq_mask	= en7528_irq_mask,
	.irq_unmask	= en7528_irq_unmask,
};

static int en7528_irq_map(struct irq_domain *domain, unsigned int irq,
			  irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, domain->host_data);
	irq_set_chip_and_handler(irq, &en7528_irq_chip, handle_simple_irq);
	irq_set_nested_thread(irq, true);
	irq_set_noprobe(irq);
	return 0;
}

static const struct irq_domain_ops en7528_irq_domain_ops = {
	.map	= en7528_irq_map,
	.xlate	= irq_domain_xlate_onecell,
};

static int en7528_setup_irq(struct en7528_priv *priv)
{
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	int ret;

	if (!of_property_read_bool(np, "interrupt-controller")) {
		dev_info(dev, "no interrupt support\n");
		return 0;
	}

	priv->irq = of_irq_get(np, 0);
	if (priv->irq <= 0)
		return priv->irq ? : -EINVAL;

	priv->irq_domain = irq_domain_add_linear(np, MT7530_NUM_PHYS,
						  &en7528_irq_domain_ops,
						  priv);
	if (!priv->irq_domain)
		return -ENOMEM;

	ret = request_threaded_irq(priv->irq, NULL, en7528_irq_thread_fn,
				   IRQF_ONESHOT, "en7528-dsa", priv);
	if (ret) {
		irq_domain_remove(priv->irq_domain);
		return ret;
	}
	return 0;
}

static void en7528_free_irq(struct en7528_priv *priv)
{
	if (!priv->irq)
		return;

	for (int p = 0; p < MT7530_NUM_PHYS; p++) {
		unsigned int virq = irq_find_mapping(priv->irq_domain, p);
		if (virq)
			irq_dispose_mapping(virq);
	}
	free_irq(priv->irq, priv);
	irq_domain_remove(priv->irq_domain);
}

/* ------------------------------------------------------------------ */
/*  MDIO bus setup                                                     */
/* ------------------------------------------------------------------ */
static int en7528_setup_mdio(struct en7528_priv *priv)
{
	struct device_node *mnp, *np = priv->dev->of_node;
	struct dsa_switch *ds = priv->ds;
	struct mii_bus *bus;
	static int idx;
	int ret = 0;

	mnp = of_get_child_by_name(np, "mdio");

	if (mnp && !of_device_is_available(mnp))
		goto out;

	bus = devm_mdiobus_alloc(priv->dev);
	if (!bus) {
		ret = -ENOMEM;
		goto out;
	}

	if (!mnp)
		ds->user_mii_bus = bus;

	bus->priv = priv;
	bus->name = "en7528-dsa-mii";
	snprintf(bus->id, MII_BUS_ID_SIZE, "en7528-dsa-%d", idx++);
	bus->read = en7528_mdio_read_c22;
	bus->write = en7528_mdio_write_c22;
	bus->read_c45 = en7528_mdio_read_c45;
	bus->write_c45 = en7528_mdio_write_c45;
	bus->parent = priv->dev;
	bus->phy_mask = ~ds->phys_mii_mask;

	if (priv->irq && !mnp) {
		for (int p = 0; p < MT7530_NUM_PHYS; p++) {
			if (BIT(p) & ds->phys_mii_mask)
				ds->user_mii_bus->irq[p] =
					irq_create_mapping(priv->irq_domain, p);
		}
	}

	ret = devm_of_mdiobus_register(priv->dev, bus, mnp);
	if (ret)
		dev_err(priv->dev, "failed to register MDIO bus: %d\n", ret);
out:
	of_node_put(mnp);
	return ret;
}

/* ------------------------------------------------------------------ */
/*  Switch setup (EN7528-specific, based on MT7988/EN7581 path)        */
/* ------------------------------------------------------------------ */
static int en7528_sw_setup(struct dsa_switch *ds)
{
	struct en7528_priv *priv = ds->priv;
	int ret;

	ds->assisted_learning_on_cpu_port = true;
	ds->mtu_enforcement_ingress = true;

	/* Reset the switch */
	reset_control_assert(priv->rstc);
	usleep_range(20, 50);
	reset_control_deassert(priv->rstc);
	usleep_range(20, 50);

	/* Reset the switch PHYs */
	en7528_write(priv, MT7530_SYS_CTRL, SYS_CTRL_PHY_RST);

	en7528_trap_frames(priv);
	en7528_mib_reset(priv);

	/* Disable flooding on all ports initially */
	en7528_clear(priv, MT753X_MFC, BC_FFP_MASK | UNM_FFP_MASK |
		     UNU_FFP_MASK);

	for (int i = 0; i < ds->num_ports; i++) {
		/* Force link down on all ports until phylink brings them up */
		en7528_rmw(priv, MT753X_PMCR_P(i),
			   PMCR_LINK_SETTINGS_MASK | EN7528_FORCE_MODE,
			   EN7528_FORCE_MODE);

		/* Disable forwarding */
		en7528_rmw(priv, MT7530_PCR_P(i), PCR_MATRIX_MASK,
			   PCR_MATRIX_CLR);

		/* Disable learning */
		en7528_set(priv, MT7530_PSC_P(i), SA_DIS);

		en7528_set(priv, MT7531_DBG_CNT(i), MT7531_DIS_CLR);

		if (dsa_is_cpu_port(ds, i)) {
			en7528_cpu_port_enable(priv, ds, i);
		} else {
			en7528_port_disable(ds, i);
			en7528_rmw(priv, MT7530_PPBV1_P(i), G0_PORT_VID_MASK,
				   G0_PORT_VID_DEF);
		}

		/* Consistent egress tag */
		en7528_rmw(priv, MT7530_PVC_P(i), PVC_EG_TAG_MASK,
			   PVC_EG_TAG(MT7530_VLAN_EG_CONSISTENT));
	}

	/* Allow mirroring frames received on the local port */
	en7528_set(priv, MT753X_AGC, LOCAL_EN);

	/* Flush the FDB table */
	ret = en7528_fdb_cmd(priv, MT7530_FDB_FLUSH, NULL);
	if (ret < 0)
		return ret;

	/* Setup VLAN ID 0 for VLAN-unaware bridges */
	ret = en7528_setup_vlan0(priv);
	if (ret)
		return ret;

	/* Setup IRQ */
	ret = en7528_setup_irq(priv);
	if (ret)
		return ret;

	/* Setup MDIO bus for internal PHYs */
	ret = en7528_setup_mdio(priv);
	if (ret && priv->irq)
		en7528_free_irq(priv);
	if (ret)
		return ret;

	/* Initialise PCS instances */
	for (int i = 0; i < ds->num_ports; i++) {
		priv->pcs[i].pcs.ops = &en7528_pcs_ops;
		priv->pcs[i].pcs.neg_mode = true;
		priv->pcs[i].priv = (struct mt7530_priv *)priv;
		priv->pcs[i].port = i;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/*  EEE                                                                */
/* ------------------------------------------------------------------ */
static int en7528_get_mac_eee(struct dsa_switch *ds, int port,
			      struct ethtool_keee *e)
{
	struct en7528_priv *priv = ds->priv;
	u32 eeecr = en7528_read(priv, MT753X_PMEEECR_P(port));

	e->tx_lpi_enabled = !(eeecr & LPI_MODE_EN);
	e->tx_lpi_timer = LPI_THRESH_GET(eeecr);
	return 0;
}

static int en7528_set_mac_eee(struct dsa_switch *ds, int port,
			      struct ethtool_keee *e)
{
	struct en7528_priv *priv = ds->priv;
	u32 set, mask = LPI_THRESH_MASK | LPI_MODE_EN;

	if (e->tx_lpi_timer > 0xFFF)
		return -EINVAL;

	set = LPI_THRESH_SET(e->tx_lpi_timer);
	if (!e->tx_lpi_enabled)
		set |= LPI_MODE_EN;
	en7528_rmw(priv, MT753X_PMEEECR_P(port), mask, set);
	return 0;
}

/* ------------------------------------------------------------------ */
/*  DSA switch ops table                                               */
/* ------------------------------------------------------------------ */
static const struct dsa_switch_ops en7528_switch_ops = {
	.get_tag_protocol		= en7528_get_tag_protocol,
	.setup				= en7528_sw_setup,
	.preferred_default_local_cpu_port = en7528_preferred_default_local_cpu_port,
	.get_strings			= en7528_get_strings,
	.get_ethtool_stats		= en7528_get_ethtool_stats,
	.get_sset_count			= en7528_get_sset_count,
	.set_ageing_time		= en7528_set_ageing_time,
	.port_enable			= en7528_port_enable,
	.port_disable			= en7528_port_disable,
	.port_change_mtu		= en7528_port_change_mtu,
	.port_max_mtu			= en7528_port_max_mtu,
	.port_stp_state_set		= en7528_stp_state_set,
	.port_pre_bridge_flags		= en7528_port_pre_bridge_flags,
	.port_bridge_flags		= en7528_port_bridge_flags,
	.port_bridge_join		= en7528_port_bridge_join,
	.port_bridge_leave		= en7528_port_bridge_leave,
	.port_fdb_add			= en7528_port_fdb_add,
	.port_fdb_del			= en7528_port_fdb_del,
	.port_fdb_dump			= en7528_port_fdb_dump,
	.port_vlan_filtering		= en7528_port_vlan_filtering,
	.port_vlan_add			= en7528_port_vlan_add,
	.port_vlan_del			= en7528_port_vlan_del,
	.port_mirror_add		= en7528_port_mirror_add,
	.port_mirror_del		= en7528_port_mirror_del,
	.phylink_get_caps		= en7528_phylink_get_caps,
	.get_mac_eee			= en7528_get_mac_eee,
	.set_mac_eee			= en7528_set_mac_eee,
};

/* ------------------------------------------------------------------ */
/*  Platform driver                                                    */
/* ------------------------------------------------------------------ */
static int en7528_dsa_probe(struct platform_device *pdev)
{
	struct regmap_config *sw_regmap_config;
	struct en7528_priv *priv;
	void __iomem *base_addr;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	mutex_init(&priv->reg_mutex);

	priv->ds = devm_kzalloc(&pdev->dev, sizeof(*priv->ds), GFP_KERNEL);
	if (!priv->ds)
		return -ENOMEM;

	priv->ds->dev = &pdev->dev;
	priv->ds->num_ports = MT7530_NUM_PORTS;
	priv->ds->priv = priv;
	priv->ds->ops = &en7528_switch_ops;
	priv->ds->phylink_mac_ops = &en7528_phylink_mac_ops;
	dev_set_drvdata(&pdev->dev, priv);

	priv->rstc = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(priv->rstc))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->rstc),
				     "failed to get reset control\n");

	base_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base_addr))
		return dev_err_probe(&pdev->dev, PTR_ERR(base_addr),
				     "failed to map switch registers\n");

	sw_regmap_config = devm_kzalloc(&pdev->dev,
					sizeof(*sw_regmap_config), GFP_KERNEL);
	if (!sw_regmap_config)
		return -ENOMEM;

	sw_regmap_config->name = "en7528-switch";
	sw_regmap_config->reg_bits = 16;
	sw_regmap_config->val_bits = 32;
	sw_regmap_config->reg_stride = 4;
	sw_regmap_config->max_register = MT7530_CREV;

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base_addr,
					     sw_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->regmap),
				     "failed to init regmap\n");

	ret = dsa_register_switch(priv->ds);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register DSA switch\n");

	dev_info(&pdev->dev, "EN7528 DSA switch registered (PoC)\n");
	return 0;
}

static void en7528_dsa_remove(struct platform_device *pdev)
{
	struct en7528_priv *priv = platform_get_drvdata(pdev);

	if (!priv)
		return;

	en7528_free_irq(priv);
	dsa_unregister_switch(priv->ds);
	mutex_destroy(&priv->reg_mutex);
}

static void en7528_dsa_shutdown(struct platform_device *pdev)
{
	struct en7528_priv *priv = platform_get_drvdata(pdev);

	if (!priv)
		return;

	dsa_switch_shutdown(priv->ds);
	dev_set_drvdata(&pdev->dev, NULL);
}

static const struct of_device_id en7528_dsa_of_match[] = {
	{ .compatible = "econet,en7528-switch" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, en7528_dsa_of_match);

static struct platform_driver en7528_dsa_driver = {
	.probe		= en7528_dsa_probe,
	.remove		= en7528_dsa_remove,
	.shutdown	= en7528_dsa_shutdown,
	.driver = {
		.name		= "econet-dsa",
		.of_match_table	= en7528_dsa_of_match,
	},
};
module_platform_driver(en7528_dsa_driver);

MODULE_AUTHOR("nazo-x1 <nazo_e@163.com>");
MODULE_DESCRIPTION("EcoNet EN7528 DSA Switch Driver (self-contained PoC)");
MODULE_LICENSE("GPL");
