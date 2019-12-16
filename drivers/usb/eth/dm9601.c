/*
 * Davicom DM96xx USB 10/100Mbps ethernet devices
 *
 * Peter Korsgaard <jacmet@sunsite.dk>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <common.h>
#include <dm.h>
#include <usb.h>
#include <malloc.h>
#include <memalign.h>
#include <errno.h>
#include <linux/mii.h>
#include "usb_ether.h"

#define DEBUG
#ifdef DEBUG
#undef debug
#define debug(fmt, args...) debug_cond(true, fmt, ##args)
#endif


/* datasheet:
 http://ptm2.cc.utu.fi/ftp/network/cards/DM9601/From_NET/DM9601-DS-P01-930914.pdf
*/

/* control requests */
#define DM_READ_REGS	0x00
#define DM_WRITE_REGS	0x01
#define DM_READ_MEMS	0x02
#define DM_WRITE_REG	0x03
#define DM_WRITE_MEMS	0x05
#define DM_WRITE_MEM	0x07

/* registers */
#define DM_NET_CTRL	0x00
#define DM_RX_CTRL	0x05
#define DM_SHARED_CTRL	0x0b
#define DM_SHARED_ADDR	0x0c
#define DM_SHARED_DATA	0x0d	/* low + high */
#define DM_PHY_ADDR	0x10	    /* 6 bytes */
#define DM_MCAST_ADDR	0x16	/* 8 bytes */
#define DM_GPR_CTRL	0x1e
#define DM_GPR_DATA	0x1f
#define DM_CHIP_ID	0x2c
#define DM_MODE_CTRL	0x91	/* only on dm9620 */

/* chip id values */
#define ID_DM9601	0
#define ID_DM9620	1

#define DM_MAX_MCAST	64
#define DM_MCAST_SIZE	8
#define DM_EEPROM_LEN	256
#define DM_TX_OVERHEAD	2	    /* 2 byte header */
#define DM_RX_OVERHEAD	7	    /* 3 byte header + 4 byte crc tail */
#define DM_TIMEOUT	1000

/* local defines */
#define USB_CTRL_SET_TIMEOUT 5000
#define USB_CTRL_GET_TIMEOUT 5000
#define USB_BULK_SEND_TIMEOUT 5000
#define USB_BULK_RECV_TIMEOUT 5000

#define DM9601_RX_URB_SIZE 2048
#define PHY_CONNECT_TIMEOUT 5000

/* driver private */
struct dm9601_private {
    int flags;
#ifdef CONFIG_DM_ETH
    struct ueth_data ueth;
#endif
};


static int dm_read(struct ueth_data *dev, u8 reg, u16 length, void *data)
{
	int err;
    struct usb_device *usb_dev = dev->pusb_dev;

	err = usb_control_msg(
                          usb_dev,
                          usb_rcvctrlpipe(usb_dev, 0),
                          DM_READ_REGS,
                          USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                          0,
                          reg,
                          data,
                          length,
                          USB_CTRL_SET_TIMEOUT);
	if(err != length && err >= 0)
		err = -EINVAL;

	return err;
}

static int dm_read_reg(struct ueth_data *dev, u8 reg, u8 *value)
{
	return dm_read(dev, reg, 1, value);
}

static int dm_write(struct ueth_data *dev, u8 reg, u16 length, void *data)
{
	int err;
    struct usb_device *usb_dev = dev->pusb_dev;

    err = usb_control_msg(
                          usb_dev,
                          usb_sndctrlpipe(usb_dev, 0),
                          DM_WRITE_REGS,
                          USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                          0, reg, data, length,
                          USB_CTRL_SET_TIMEOUT);
    if (err >= 0 && err < length)
        err = -EINVAL;

    return err;
}

static int dm_write_reg(struct ueth_data *dev, u8 reg, u8 value)
{
    struct usb_device *usb_dev = dev->pusb_dev;

    return usb_control_msg(
                           usb_dev,
                           usb_sndctrlpipe(usb_dev, 0),
                           DM_WRITE_REG,
                           USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                           value,
                           reg,
                           NULL,
                           0,
                           USB_CTRL_SET_TIMEOUT
                          );
}

#if 0
static void dm_write_async(struct usbnet *dev, u8 reg, u16 length, void *data)
{
	usbnet_write_cmd_async(dev, DM_WRITE_REGS,
			       USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			       0, reg, data, length);
}

static void dm_write_reg_async(struct usbnet *dev, u8 reg, u8 value)
{
	usbnet_write_cmd_async(dev, DM_WRITE_REG,
			       USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			       value, reg, NULL, 0);
}
#endif

static int dm_read_shared_word(struct ueth_data *dev, int phy, u8 reg, __le16 *value)
{
	int ret, i;

	/* mutex_lock(&dev->phy_mutex); */

	dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	dm_write_reg(dev, DM_SHARED_CTRL, phy ? 0xc : 0x4);

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp = 0;

		udelay(1);
		ret = dm_read_reg(dev, DM_SHARED_CTRL, &tmp);
		if (ret < 0)
			goto out;

		/* ready */
		if ((tmp & 1) == 0)
			break;
	}

	if (i == DM_TIMEOUT) {
		printf("%s read timed out!\n", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	dm_write_reg(dev, DM_SHARED_CTRL, 0x0);
	ret = dm_read(dev, DM_SHARED_DATA, 2, value);

	debug("read shared %d 0x%02x returned 0x%04x, %d\n",
		   phy, reg, *value, ret);

 out:
	/* mutex_unlock(&dev->phy_mutex); */
	return ret;
}

static int dm_write_shared_word(struct ueth_data *dev, int phy, u8 reg, __le16 value)
{
	int ret, i;

	/* mutex_lock(&dev->phy_mutex); */

	ret = dm_write(dev, DM_SHARED_DATA, 2, &value);
	if (ret < 0)
		goto out;

	dm_write_reg(dev, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	dm_write_reg(dev, DM_SHARED_CTRL, phy ? 0x1a : 0x12);

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp = 0;

		udelay(1);
		ret = dm_read_reg(dev, DM_SHARED_CTRL, &tmp);
		if (ret < 0)
			goto out;

		/* ready */
		if ((tmp & 1) == 0)
			break;
	}

	if (i == DM_TIMEOUT) {
		printf("%s write timed out!\n", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	dm_write_reg(dev, DM_SHARED_CTRL, 0x0);

out:
	/* mutex_unlock(&dev->phy_mutex); */
	return ret;
}

#if 0
static int dm_read_eeprom_word(struct usbnet *dev, u8 offset, void *value)
{
	return dm_read_shared_word(dev, 0, offset, value);
}



static int dm9601_get_eeprom_len(struct net_device *dev)
{
	return DM_EEPROM_LEN;
}

static int dm9601_get_eeprom(struct net_device *net,
			     struct ethtool_eeprom *eeprom, u8 * data)
{
	struct usbnet *dev = netdev_priv(net);
	__le16 *ebuf = (__le16 *) data;
	int i;

	/* access is 16bit */
	if ((eeprom->offset % 2) || (eeprom->len % 2))
		return -EINVAL;

	for (i = 0; i < eeprom->len / 2; i++) {
		if (dm_read_eeprom_word(dev, eeprom->offset / 2 + i,
					&ebuf[i]) < 0)
			return -EINVAL;
	}
	return 0;
}
#endif

static int dm9601_mdio_read(struct ueth_data *dev, int phy_id, int loc)
{
    /* TODO: need ALLOC_CACHE_ALIGN_BUFFER ? */
	__le16 res;

	if (phy_id) {
		printf("Only internal phy supported\n");
		return 0;
	}

	dm_read_shared_word(dev, 1, loc, &res);

    debug(
          "dm9601_mdio_read() phy_id=0x%02x, loc=0x%02x, returns=0x%04x\n",
          phy_id, loc, le16_to_cpu(res));

	return le16_to_cpu(res);
}

static void dm9601_mdio_write(struct ueth_data *dev, int phy_id, int loc,
			      int val)
{
	__le16 res = cpu_to_le16(val);

	if (phy_id) {
		printf("Only internal phy supported\n");
		return;
	}

	debug("dm9601_mdio_write() phy_id=0x%02x, loc=0x%02x, val=0x%04x\n",
		   phy_id, loc, val);

	dm_write_shared_word(dev, 1, loc, res);
}

#if 0
static void dm9601_get_drvinfo(struct net_device *net,
			       struct ethtool_drvinfo *info)
{
	/* Inherit standard device info */
	usbnet_get_drvinfo(net, info);
}

static u32 dm9601_get_link(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	return mii_link_ok(&dev->mii);
}

static int dm9601_ioctl(struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}

static const struct ethtool_ops dm9601_ethtool_ops = {
	.get_drvinfo	= dm9601_get_drvinfo,
	.get_link	= dm9601_get_link,
	.get_msglevel	= usbnet_get_msglevel,
	.set_msglevel	= usbnet_set_msglevel,
	.get_eeprom_len	= dm9601_get_eeprom_len,
	.get_eeprom	= dm9601_get_eeprom,
	.nway_reset	= usbnet_nway_reset,
	.get_link_ksettings	= usbnet_get_link_ksettings,
	.set_link_ksettings	= usbnet_set_link_ksettings,
};
#endif

static void dm9601_set_multicast(struct ueth_data *dev)
{
	/* We use the 20 byte dev->data for our 8 byte filter buffer
	 * to avoid allocating memory that is tricky to free later */
    ALLOC_CACHE_ALIGN_BUFFER(unsigned char, hashes, DM_MCAST_SIZE);
	/* u8 *hashes = (u8 *) & dev->data; */
	u8 rx_ctl = 0x31;

	memset(hashes, 0x00, DM_MCAST_SIZE);
	hashes[DM_MCAST_SIZE - 1] |= 0x80;	/* broadcast address */

#if 0
	if (net->flags & IFF_PROMISC) {
		rx_ctl |= 0x02;
	} else if (net->flags & IFF_ALLMULTI ||
		   netdev_mc_count(net) > DM_MAX_MCAST) {
		rx_ctl |= 0x08;
	} else if (!netdev_mc_empty(net)) {
		struct netdev_hw_addr *ha;

		netdev_for_each_mc_addr(ha, net) {
			u32 crc = ether_crc(ETH_ALEN, ha->addr) >> 26;
			hashes[crc >> 3] |= 1 << (crc & 0x7);
		}
	}
#endif

	dm_write(dev, DM_MCAST_ADDR, DM_MCAST_SIZE, hashes);
	dm_write_reg(dev, DM_RX_CTRL, rx_ctl);
}

static int dm9601_read_mac_address(struct ueth_data *dev, uint8_t *enetaddr)
{
    ALLOC_CACHE_ALIGN_BUFFER(unsigned char, buf, ETH_ALEN);

    /* read MAC */
    if (dm_read(dev, DM_PHY_ADDR, ETH_ALEN, buf) < 0) {
        printf("dm9601: Error reading MAC address\n");
        return -ENODEV;

    }

    memcpy(enetaddr, buf, ETH_ALEN);

    return 0;
}


static void __dm9601_set_mac_address(struct ueth_data *dev, void *addr)
{
	dm_write(dev, DM_PHY_ADDR, ETH_ALEN, addr);
}

static int dm9601_set_mac_address(struct ueth_data *dev, uint8_t *enetaddr)
{
    ALLOC_CACHE_ALIGN_BUFFER(unsigned char, addr, ETH_ALEN);

	if (!is_valid_ethaddr(enetaddr)) {
		printf("not setting invalid mac address %pM\n", enetaddr);
		return -EINVAL;
	}

	memcpy(addr, enetaddr, ETH_ALEN);
	__dm9601_set_mac_address(dev, addr);

#if 1
    dm9601_read_mac_address(dev, addr);
    debug("---->after write: <%02x:%02x:%02x:%02x:%02x:%02x>\n",
          addr[0], addr[1], addr[2],
          addr[3], addr[4], addr[5]);
#endif

	return 0;
}

/*
 * mii_nway_restart - restart NWay (autonegotiation) for this interface
 *
 * Returns 0 on success, negative on error.
 */
static int mii_nway_restart(struct ueth_data *dev)
{
    int bmcr;
    int r = -1;

    /* if autoneg is off, it's an error */
    bmcr = dm9601_mdio_read(dev, dev->phy_id, MII_BMCR);
    debug("%s: bmcr: 0x%x\n", __func__, bmcr);
    if (bmcr & BMCR_ANENABLE) {
        bmcr |= BMCR_ANRESTART;
        dm9601_mdio_write(dev, dev->phy_id, MII_BMCR, bmcr);
        r = 0;
    }

    return r;
}

#if 0
static const struct net_device_ops dm9601_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= usbnet_start_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_change_mtu		= usbnet_change_mtu,
	.ndo_get_stats64	= usbnet_get_stats64,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl 		= dm9601_ioctl,
	.ndo_set_rx_mode	= dm9601_set_multicast,
	.ndo_set_mac_address	= dm9601_set_mac_address,
};
#endif

static int dm9601_init(struct ueth_data *dev)
{
    int timeout = 0;
    int link_detected;

	debug("\n----> %s()\n", __func__);

    /* mii_nway_restart(dev); */
    /*dm_set_autoneg(dev);*/

    /* dm9601_link_reset(dev); */

#define TIMEOUT_RESOLUTION 50   /* ms */
    do {
        link_detected = dm9601_mdio_read(dev, dev->phy_id, MII_BMSR) &
            BMSR_LSTATUS;
        if (!link_detected) {
            if (timeout == 0)
                printf("Waiting for Ethernet connection... ");

            udelay(TIMEOUT_RESOLUTION * 1000);
            timeout += TIMEOUT_RESOLUTION;
        }
    } while (!link_detected && timeout < PHY_CONNECT_TIMEOUT);

    if (link_detected) {
        if (timeout != 0)
            printf("done.\n");

    } else {
        printf("unable to connect.\n");
        goto out_err;

    }

    mdelay(25);
    return 0;

out_err:
    printf("dm9601: Error: unable to init device.\n");
    return -1;

}
static int dm9601_bind(struct udevice *udev)
{
	int ret = 0;
	u8 mac[ETH_ALEN], id;

    struct eth_pdata *pdata = dev_get_platdata(udev);
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *dev = &priv->ueth;

#if 0
	ret = usbnet_get_endpoints(dev, intf);
	if (ret)
		goto out;

	dev->net->netdev_ops = &dm9601_netdev_ops;
	dev->net->ethtool_ops = &dm9601_ethtool_ops;
	dev->net->hard_header_len += DM_TX_OVERHEAD;
	dev->hard_mtu = dev->net->mtu + dev->net->hard_header_len;

	/* dm9620/21a require room for 4 byte padding, even in dm9601
	 * mode, so we need +1 to be able to receive full size
	 * ethernet frames.
	 */
	dev->rx_urb_size = dev->net->mtu + ETH_HLEN + DM_RX_OVERHEAD + 1;

	dev->mii.dev = dev->net;
	dev->mii.mdio_read = dm9601_mdio_read;
	dev->mii.mdio_write = dm9601_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0x1f;
#endif

	/* reset */
	dm_write_reg(dev, DM_NET_CTRL, 1);
	udelay(20);

	/* read MAC */
	if (dm_read(dev, DM_PHY_ADDR, ETH_ALEN, mac) < 0) {
		printf("Error reading MAC address\n");
		ret = -ENODEV;
		goto out;
	}

	/*
	 * Overwrite the auto-generated address only with good ones.
	 */
	if (is_valid_ethaddr(mac))
		memcpy(pdata->enetaddr, mac, ETH_ALEN);
	else {
		printf("dm9601: No valid MAC address in EEPROM, using %pM\n", mac);
		/* __dm9601_set_mac_address(mac); */
	}

	if (dm_read_reg(dev, DM_CHIP_ID, &id) < 0) {
		printf("Error reading chip ID\n");
		ret = -ENODEV;
		goto out;
	}

    debug("id: %s\n", ID_DM9601 ? "DM9601" : "DM9621");

	/* put dm9620 devices in dm9601 mode */
	if (id == ID_DM9620) {
		u8 mode;

		if (dm_read_reg(dev, DM_MODE_CTRL, &mode) < 0) {
			printf("Error reading MODE_CTRL\n");
			ret = -ENODEV;
			goto out;
		}
		dm_write_reg(dev, DM_MODE_CTRL, mode & 0x7f);
	}

	/* power up phy */
	dm_write_reg(dev, DM_GPR_CTRL, 1);
	dm_write_reg(dev, DM_GPR_DATA, 0);

	/* receive broadcast packets */
	dm9601_set_multicast(dev);

	dm9601_mdio_write(dev, dev->phy_id, MII_BMCR, BMCR_RESET);
	dm9601_mdio_write(dev, dev->phy_id, MII_ADVERTISE,
			  ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
	mii_nway_restart(dev);

out:
	return ret;
}

#if 0
static int dm9601_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	u8 status;
	int len;

	/* format:
	   b1: rx status
	   b2: packet length (incl crc) low
	   b3: packet length (incl crc) high
	   b4..n-4: packet data
	   bn-3..bn: ethernet crc
	 */

	if (unlikely(skb->len < DM_RX_OVERHEAD)) {
		dev_err(&dev->udev->dev, "unexpected tiny rx frame\n");
		return 0;
	}

	status = skb->data[0];
	len = (skb->data[1] | (skb->data[2] << 8)) - 4;

	if (unlikely(status & 0xbf)) {
		if (status & 0x01) dev->net->stats.rx_fifo_errors++;
		if (status & 0x02) dev->net->stats.rx_crc_errors++;
		if (status & 0x04) dev->net->stats.rx_frame_errors++;
		if (status & 0x20) dev->net->stats.rx_missed_errors++;
		if (status & 0x90) dev->net->stats.rx_length_errors++;
		return 0;
	}

	skb_pull(skb, 3);
	skb_trim(skb, len);

	return 1;
}

static struct sk_buff *dm9601_tx_fixup(struct usbnet *dev, struct sk_buff *skb,
				       gfp_t flags)
{
	int len, pad;

	/* format:
	   b1: packet length low
	   b2: packet length high
	   b3..n: packet data
	*/

	len = skb->len + DM_TX_OVERHEAD;

	/* workaround for dm962x errata with tx fifo getting out of
	 * sync if a USB bulk transfer retry happens right after a
	 * packet with odd / maxpacket length by adding up to 3 bytes
	 * padding.
	 */
	while ((len & 1) || !(len % dev->maxpacket))
		len++;

	len -= DM_TX_OVERHEAD; /* hw header doesn't count as part of length */
	pad = len - skb->len;

	if (skb_headroom(skb) < DM_TX_OVERHEAD || skb_tailroom(skb) < pad) {
		struct sk_buff *skb2;

		skb2 = skb_copy_expand(skb, DM_TX_OVERHEAD, pad, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

	__skb_push(skb, DM_TX_OVERHEAD);

	if (pad) {
		memset(skb->data + skb->len, 0, pad);
		__skb_put(skb, pad);
	}

	skb->data[0] = len;
	skb->data[1] = len >> 8;

	return skb;
}

static void dm9601_status(struct usbnet *dev, struct urb *urb)
{
	int link;
	u8 *buf;

	/* format:
	   b0: net status
	   b1: tx status 1
	   b2: tx status 2
	   b3: rx status
	   b4: rx overflow
	   b5: rx count
	   b6: tx count
	   b7: gpr
	*/

	if (urb->actual_length < 8)
		return;

	buf = urb->transfer_buffer;

	link = !!(buf[0] & 0x40);
	if (netif_carrier_ok(dev->net) != link) {
		usbnet_link_change(dev, link, 1);
		netdev_dbg(dev->net, "Link Status is: %d\n", link);
	}
}

static int dm9601_link_reset(struct usbnet *dev)
{
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };

	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);

	netdev_dbg(dev->net, "link_reset() speed: %u duplex: %d\n",
		   ethtool_cmd_speed(&ecmd), ecmd.duplex);

	return 0;
}
#endif

/* FIXME: those not valid */
static const unsigned long dm9601_info;

#ifdef CONFIG_DM_ETH
static int dm9601_eth_start(struct udevice *udev)
{
    /* struct eth_pdata *pdata = dev_get_platdata(udev); */
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *dev = &priv->ueth;

	debug("\n----> %s()\n", __func__);
    return dm9601_init(dev);
}

void dm9601_eth_stop(struct udevice *udev)
{
	debug("\n----> %s()\n", __func__);
}

int dm9601_eth_send(struct udevice *udev, void *packet, int length)
{
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *dev = &priv->ueth;

	debug("\n----> %s()\n", __func__);

    /* return dm9601_send(dev, packet, length); */
    return 0;
}

int dm9601_eth_recv(struct udevice *udev, int flags, uchar **packetp)
{
	debug("\n----> %s()\n", __func__);
    return 0;
}

static int dm9601_free_pkt(struct udevice *udev, uchar *packet, int packet_len)
{
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *dev = &priv->ueth;

	debug("\n----> %s()\n", __func__);

    /* TODO: check this */
    packet_len = ALIGN(packet_len, 4);
    usb_ether_advance_rxbuf(dev, sizeof(u32) + packet_len);

    return 0;
}

int dm9601_write_hwaddr(struct udevice *dev)
{
    struct eth_pdata *pdata = dev_get_platdata(dev);
    struct dm9601_private *priv = dev_get_priv(dev);

    debug("---->set mac addr <%02x:%02x:%02x:%02x:%02x:%02x>\n",
          pdata->enetaddr[0], pdata->enetaddr[1], pdata->enetaddr[2],
          pdata->enetaddr[3], pdata->enetaddr[4], pdata->enetaddr[5]);
    return dm9601_set_mac_address(&priv->ueth, pdata->enetaddr);
}

static int dm9601_eth_probe(struct udevice *udev)
{
    struct eth_pdata *pdata = dev_get_platdata(udev);
    struct dm9601_private *priv = dev_get_priv(udev);
    struct ueth_data *dev = &priv->ueth;
    int ret;

	debug("\n----> %s()\n", __func__);

    priv->flags = udev->driver_data;
    ret = usb_ether_register(udev, dev, DM9601_RX_URB_SIZE);
    if (ret) {
        printf("usb ether register failed! ret = %d\n", ret);
        return ret;
    }

    /* Do a reset in order to get the MAC address from HW */
    if(dm9601_bind(udev)) {
        printf("basic init failed!\n");
        goto err;
    }

    /* Get the MAC address */
    dm9601_read_mac_address(dev, pdata->enetaddr);
    debug("---->get mac addr: <%02x:%02x:%02x:%02x:%02x:%02x>\n",
          pdata->enetaddr[0], pdata->enetaddr[1], pdata->enetaddr[2],
          pdata->enetaddr[3], pdata->enetaddr[4], pdata->enetaddr[5]);

    return 0;

err:
    return usb_ether_deregister(dev);
}

static const struct eth_ops dm9601_eth_ops = {
    .start          = dm9601_eth_start,
    .send           = dm9601_eth_send,
    .recv           = dm9601_eth_recv,
    .free_pkt       = dm9601_free_pkt,
    .stop           = dm9601_eth_stop,
    .write_hwaddr   = dm9601_write_hwaddr,
};

U_BOOT_DRIVER(dm9601_eth) = {
    .name = "dm9601_eth",
    .id = UCLASS_ETH,
    .probe = dm9601_eth_probe,
    .ops = &dm9601_eth_ops,
    .priv_auto_alloc_size = sizeof(struct dm9601_private),
    .platdata_auto_alloc_size = sizeof(struct eth_pdata),
};

static const struct usb_device_id dm9601_eth_id_table[] = {
    /* Corega FEther USB-TXC */
	{ USB_DEVICE(0x07aa, 0x9601), .driver_info = (unsigned long)&dm9601_info, },
    /* Davicom USB-100 */
	{ USB_DEVICE(0x0a46, 0x9601), .driver_info = (unsigned long)&dm9601_info, },
    /* ZT6688 USB NIC */
	{ USB_DEVICE(0x0a46, 0x6688), .driver_info = (unsigned long)&dm9601_info, },
    /* ShanTou ST268 USB NIC */
	{ USB_DEVICE(0x0a46, 0x0268), .driver_info = (unsigned long)&dm9601_info, },
    /* ADMtek ADM8515 USB NIC */
	{ USB_DEVICE(0x0a46, 0x8515), .driver_info = (unsigned long)&dm9601_info, },
    /* Hirose USB-100 */
	{ USB_DEVICE(0x0a47, 0x9601), .driver_info = (unsigned long)&dm9601_info, },
	/* DM9601 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0fe6, 0x8101), .driver_info = (unsigned long)&dm9601_info, },
	/* DM9601 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0fe6, 0x9700), .driver_info = (unsigned long)&dm9601_info, },
	/* DM9000E */
	{ USB_DEVICE(0x0a46, 0x9000), .driver_info = (unsigned long)&dm9601_info, },
	/* DM9620 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x9620), .driver_info = (unsigned long)&dm9601_info, },
	/* DM9621A USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x9621), .driver_info = (unsigned long)&dm9601_info, },
	/* DM9622 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x9622), .driver_info = (unsigned long)&dm9601_info, },
	/* DM962OA USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x0269), .driver_info = (unsigned long)&dm9601_info, },
	/* DM9621A USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x1269), .driver_info = (unsigned long)&dm9601_info, },
	{},			// END
};

U_BOOT_USB_DEVICE(dm9601_eth, dm9601_eth_id_table);
#endif

