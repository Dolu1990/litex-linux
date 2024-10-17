// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * spinal_sgeth Ethernet
 *
 * Charles Papon <charles.papon.90@gmail.com>
 *
 */

#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>

/*
            mac0: mac@f1000000 {
                compatible = "spinal,sgeth";
                reg = <0xf1000000 0x100>,
                      <0xf1000100 0x100>,
                      <0xf1000200 0x100>;
                reg-names = "mac", "tx-dma", "rx-dma";
                interrupts = <40 41>;
				interrupt-names = "tx-dma", "rx-dma";
                status = "okay";
            };


TODO tx need to ensure there is enough descriptor free, else need to throttle
TODO napi_alloc_skb ?
TODO check hardware TSO mtu size
netdev_fix_features
ethtool -k eth0

tx 170 -> hw crc 250 -> tso 350
 */

//#define DEBUG

#define DRV_NAME	"spinal-sgeth"
#define TX_DMA_CONTROL 0x00
#define TX_DMA_IRQ 0x04
#define TX_DMA_NEXT 0x10

#define TX_DMA_STATUS_BUSY 1
#define TX_DMA_STATUS_START 1
#define TX_DMA_STATUS_GOING_IDLE_BUSY (1 << 4)
#define TX_DMA_STATUS_GOING_IDLE_VALUE (1 << 5)
#define TX_DMA_STATUS_IRQ_ENABLE (1 << 8)
#define TX_DMA_STATUS_IRQ_DISABLE (1 << 9)

#define TX_DMA_DESC_STATUS_COMPLETED (1 << 31)

#define TX_DMA_DESC_CONTROL_IRQ (1 << 31)
#define TX_DMA_DESC_CONTROL_LAST (1 << 30)

#define TX_DMA_IRQ_IDLE_ENABLE 0x1
#define TX_DMA_IRQ_DELAY_ENABLE 0x2
#define TX_DMA_IRQ_COUNTER_ENABLE 0x4
#define TX_DMA_IRQ_DELAY_LIMIT_AT 16
#define TX_DMA_IRQ_COUNTER_TARGET_AT 24

#define SGETH_HDR_SIZE			14      /* size of Ethernet header */
#define SGETH_TRL_SIZE			4       /* size of Ethernet trailer (FCS) */
#define SGETH_JUMBO_MTU			1500//9000
#define SGETH_MAX_JUMBO_FRAME_SIZE	(SGETH_JUMBO_MTU + SGETH_HDR_SIZE + SGETH_TRL_SIZE)



// Should be aligned to 32 bytes !
struct sgeth_tx_descriptor {
   u32 status;
   u32 control;
   u64 next;
   u64 from;
   u8 user[8];
};

inline static void spinal_sgeth_tx_dma_start(void __iomem * dma){
    writel(TX_DMA_STATUS_START, dma);
}
inline static int spinal_sgeth_tx_dma_busy(void __iomem * dma){
    return readl(dma) & TX_DMA_STATUS_BUSY;
}
inline static void spinal_sgeth_tx_dma_set_next(void __iomem * dma, u64 next){
	writel_relaxed(next, dma + TX_DMA_NEXT);
	writel_relaxed(next >> 32, dma + TX_DMA_NEXT + 4);
}
inline static int spinal_sgeth_tx_dma_is_going_idle(void __iomem * dma){
	writel(TX_DMA_STATUS_GOING_IDLE_BUSY, dma + TX_DMA_CONTROL); //Not relaxed, as we need to ensure possible status updates are synced
	while(1){
		u32 status = readl_relaxed(dma + TX_DMA_CONTROL);
		if((status & TX_DMA_STATUS_GOING_IDLE_BUSY) == 0) return status & TX_DMA_STATUS_GOING_IDLE_VALUE;
	}
}
inline static void spinal_sgeth_tx_dma_irq_enable(void __iomem * dma){
    return writel_relaxed(TX_DMA_STATUS_IRQ_ENABLE, dma + TX_DMA_CONTROL);
}
inline static void spinal_sgeth_tx_dma_irq_disable(void __iomem * dma){
    return writel_relaxed(TX_DMA_STATUS_IRQ_DISABLE, dma + TX_DMA_CONTROL);
}




#define RX_DMA_CONTROL 0x00
#define RX_DMA_IRQ 0x4
#define RX_DMA_NEXT 0x10

#define RX_DMA_STATUS_BUSY 1
#define RX_DMA_STATUS_START 1
#define RX_DMA_STATUS_IRQ_ENABLE (1 << 8)
#define RX_DMA_STATUS_IRQ_DISABLE (1 << 9)

#define RX_DMA_IRQ_IDLE_ENABLE 0x1
#define RX_DMA_IRQ_DELAY_ENABLE 0x2
#define RX_DMA_IRQ_COUNTER_ENABLE 0x4
#define RX_DMA_IRQ_DELAY_LIMIT_AT 16
#define RX_DMA_IRQ_COUNTER_TARGET_AT 24

#define RX_DMA_DESC_STATUS_COMPLETED (1 << 31)
#define RX_DMA_DESC_STATUS_LAST (1 << 30)
#define RX_DMA_DESC_STATUS_ERROR (1 << 29)
#define RX_DMA_DESC_STATUS_BYTES 0x7FFFFFF

#define RX_DMA_DESC_CONTROL_IRQ_ALL (1 << 31)
#define RX_DMA_DESC_CONTROL_IRQ_LAST (1 << 30)


// Should be aligned to 32 bytes !
struct sgeth_rx_descriptor {
   u32 status;
   u32 control;
   u64 next;
   u64 to;
   u8 user[8];
};

inline static void spinal_sgeth_rx_dma_start(void __iomem * dma){
    writel(RX_DMA_STATUS_START, dma);
}
inline static int spinal_sgeth_rx_dma_busy(void __iomem * dma){
    return readl(dma) & RX_DMA_STATUS_BUSY;
}
inline static void spinal_sgeth_rx_dma_set_next(void __iomem * dma, u64 next){
	writel_relaxed(next, dma + RX_DMA_NEXT);
    writel_relaxed(next >> 32, dma + RX_DMA_NEXT + 4);
}
inline static void spinal_sgeth_rx_dma_irq_enable(void __iomem * dma){
    return writel_relaxed(RX_DMA_STATUS_IRQ_ENABLE, dma + RX_DMA_CONTROL);
}
inline static void spinal_sgeth_rx_dma_irq_disable(void __iomem * dma){
    return writel_relaxed(RX_DMA_STATUS_IRQ_DISABLE, dma + RX_DMA_CONTROL);
}

struct spinal_sgeth {
	struct net_device *ndev;
	struct device *dev;

	/* mac */
	void __iomem *mac;

	/* Tx */
	void __iomem *tx_dma;
	spinlock_t tx_lock;
	int tx_desc_count;
	int tx_desc_alloc, tx_desc_sent, tx_desc_free;
	volatile struct sgeth_tx_descriptor *tx_desc_virt;
	dma_addr_t tx_desc_phys;
	int tx_irq;
	int tx_irq_config;
	struct sk_buff **tx_skb;

	/* Rx */
	void __iomem *rx_dma;
	int rx_desc_count;
	int rx_desc_ptr;
	volatile struct sgeth_rx_descriptor *rx_desc_virt;
	dma_addr_t rx_desc_phys;
	struct sk_buff **rx_skb;
	int rx_irq;
	spinlock_t rx_lock;
	int rx_irq_first;
	u32 rx_irq_config;
	struct napi_struct rx_napi;


	struct timer_list poll_timer;
};

inline static int spinal_sgeth_get_tx_available(struct spinal_sgeth *priv){
	return priv->tx_desc_count - (priv->tx_desc_alloc - priv->tx_desc_free);
}

static irqreturn_t spinal_sgeth_tx_irq(int irq, void *_ndev)
{
	struct net_device *ndev = _ndev;
	struct spinal_sgeth *priv = netdev_priv(ndev);
	unsigned long flags;

	spin_lock_irqsave(&priv->tx_lock, flags);
	smp_mb();

	int desc_id = priv->tx_desc_free;

	writel_relaxed(priv->tx_irq_config, priv->tx_dma + TX_DMA_IRQ);

	while(desc_id != priv->tx_desc_sent) {
		int desc_id_masked = desc_id & (priv->tx_desc_count-1);
		struct sgeth_tx_descriptor *desc = &priv->tx_desc_virt[desc_id_masked];
		struct sk_buff *skb;

		writel_relaxed(priv->tx_irq_config, priv->tx_dma + TX_DMA_IRQ);

		if (!(desc->status & TX_DMA_DESC_STATUS_COMPLETED))
			break;

		skb = priv->tx_skb[desc_id_masked];
		if(skb){
			dev_kfree_skb_any(skb);
			priv->tx_skb[desc_id_masked] = NULL;
		}
		desc_id += 1;
	}

	priv->tx_desc_free = desc_id;
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	if (netif_queue_stopped(ndev)) {
		smp_mb();
		if (spinal_sgeth_get_tx_available(priv) >= MAX_SKB_FRAGS + 1) {
			netif_wake_queue(ndev);
		}
	}
	return IRQ_HANDLED;
}


static inline int spinal_sgeth_rx_refill(struct spinal_sgeth *priv, int index){
	dma_addr_t skb_dma_addr;
	struct sk_buff *skb = napi_alloc_skb(&priv->rx_napi, SGETH_MAX_JUMBO_FRAME_SIZE);
	if (!skb){
		netdev_err(priv->ndev, "Can't allocate enough RX SKB\n");
		return -ENOMEM;
	}

	priv->rx_skb[index] = skb;
	/* returns physical address of skb->data */
	skb_dma_addr = dma_map_single(priv->dev->parent, skb->data,
						SGETH_MAX_JUMBO_FRAME_SIZE,
				      DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->dev->parent, skb_dma_addr)){
		netdev_err(priv->ndev, "Can't map RX SKB\n");
		return -ENOMEM;
	}

	priv->rx_desc_virt[index].to = skb_dma_addr;
	priv->rx_desc_virt[index].status = 0;
	return 0;
}

static int spinal_sgeth_rx_refresh(struct spinal_sgeth *priv, int budget)
{
	int err;
	unsigned long flags;
	int works = 0;

	spin_lock_irqsave(&priv->rx_lock, flags);

	while(works != budget) {
		int desc_ptr = priv->rx_desc_ptr;
		struct sgeth_rx_descriptor *desc = &priv->rx_desc_virt[desc_ptr];
		struct sk_buff *skb;
		int length;

		writel_relaxed(priv->rx_irq_config, priv->rx_dma + RX_DMA_IRQ);

		mb();

		/* Loop over all completed buffer descriptors */
		if (!(desc->status & RX_DMA_DESC_STATUS_COMPLETED)){
			break;
		}

		skb = priv->rx_skb[desc_ptr];
		if(skb){
			dma_unmap_single(priv->dev->parent, desc->to,
					SGETH_MAX_JUMBO_FRAME_SIZE, DMA_FROM_DEVICE);

			length = (desc->status & 0x3FFF) - SGETH_TRL_SIZE;

//			if (netif_msg_pktdata(priv)) {
//			netdev_info(priv->ndev, "frame received %d bytes\n",
//					length);
//			print_hex_dump(KERN_ERR, "RX: ", DUMP_PREFIX_OFFSET,
//					   16, 1, skb->data, length, 1);
//			}

			skb_put(skb, length);
			skb->protocol = eth_type_trans(skb, priv->ndev);
			skb->ip_summed = CHECKSUM_UNNECESSARY;

////			if (!skb_defer_rx_timestamp(skb)){
			napi_gro_receive(&priv->rx_napi, skb);
//			}
			/* The skb buffer is now owned by network stack above */
			priv->rx_skb[desc_ptr] = NULL;

			priv->ndev->stats.rx_packets++;
			priv->ndev->stats.rx_bytes += length;

			works += 1;
		}

		if(err = spinal_sgeth_rx_refill(priv, desc_ptr)) {
			dev_err_ratelimited(priv->dev, "spinal_sgeth_rx_refill failed :(");
			return works;
		}

		desc_ptr += 1;
		if(desc_ptr == priv->rx_desc_count) desc_ptr = 0;
		priv->rx_desc_ptr = desc_ptr;
	}


	spin_unlock_irqrestore(&priv->rx_lock, flags);

	return works;
}



inline static void spinal_sgeth_rx_ensure_started(struct spinal_sgeth *priv) {
	if(!spinal_sgeth_rx_dma_busy(priv->rx_dma)){
		spinal_sgeth_rx_dma_set_next(priv->rx_dma, priv->rx_desc_phys + priv->rx_desc_ptr * sizeof(struct sgeth_rx_descriptor));
		mb();
		spinal_sgeth_rx_dma_start(priv->rx_dma);
		if(!priv->rx_irq_first) dev_err_ratelimited(priv->dev, "DMA RX ran out of descriptors :(");
	}
	priv->rx_irq_first = 0;
}

static irqreturn_t spinal_sgeth_rx_irq(int irq, void *_ndev)
{
	struct net_device *ndev = _ndev;
	struct spinal_sgeth *priv = netdev_priv(ndev);
	spinal_sgeth_rx_dma_irq_disable(priv->rx_dma);
	writel_relaxed(priv->rx_irq_config, priv->rx_dma + RX_DMA_IRQ);
	napi_schedule(&priv->rx_napi);
	return IRQ_HANDLED;
}

static int spinal_sgeth_poll(struct napi_struct *napi, int budget){
	struct spinal_sgeth *priv = container_of(napi, struct spinal_sgeth, rx_napi);

	int works = spinal_sgeth_rx_refresh(priv, budget);
	spinal_sgeth_rx_ensure_started(priv);

	if (works < budget) {
		napi_complete_done(napi, works);
		spinal_sgeth_rx_dma_irq_enable(priv->rx_dma);
	}
	return works;
}

static int spinal_sgeth_open(struct net_device *ndev)
{
	struct spinal_sgeth *priv = netdev_priv(ndev);
	struct sk_buff *skb;
	dma_addr_t skb_dma_addr;
	int err;

	netdev_info(ndev, "spinal_sgeth_open\n");

	/* Enable IRQs */
//	err = request_irq(ndev->irq, spinal_sgeth_interrupt, 0, ndev->name, ndev);
//	if (err) {
//		netdev_err(ndev, "failed to request irq %d\n", ndev->irq);
//		return err;
//	}

	priv->tx_desc_count = 256; //Need to be more than MAX_SKB_FRAGS + 1
	priv->tx_desc_alloc = 0;
	priv->tx_desc_sent = 0;
	priv->tx_desc_free = 0;
	priv->tx_desc_virt = dma_alloc_coherent(ndev->dev.parent,
					 sizeof(struct sgeth_tx_descriptor) * priv->tx_desc_count,
					 &priv->tx_desc_phys, GFP_KERNEL);

	netdev_info(ndev, "tx_desc_phys %p %lx\n", priv->tx_desc_virt, priv->tx_desc_phys);

	if (!priv->tx_desc_virt){
		netdev_err(ndev, "Can't allocate DMA space\n");
		return -ENOMEM;
	}
	for(int i = 0;i < priv->tx_desc_count;i++){
		int i_next = (i+1) % priv->tx_desc_count;
		priv->tx_desc_virt[i].status = TX_DMA_DESC_STATUS_COMPLETED;
		priv->tx_desc_virt[i].next = priv->tx_desc_phys + i_next * sizeof(struct sgeth_tx_descriptor);
	}
	priv->tx_skb = devm_kcalloc(&ndev->dev, priv->tx_desc_count,
				  sizeof(*priv->tx_skb), GFP_KERNEL);
	if (!priv->tx_skb){
		return -ENOMEM;
	}

	priv->rx_desc_count = 256;
	priv->rx_desc_ptr = 0;
	priv->rx_desc_virt = dma_alloc_coherent(ndev->dev.parent,
					 sizeof(struct sgeth_rx_descriptor) * priv->rx_desc_count,
					 &priv->rx_desc_phys, GFP_KERNEL);

	if (!priv->rx_desc_virt){
		netdev_err(ndev, "Can't allocate DMA space\n");
		return -ENOMEM;
	}
	for(int i = 0;i < priv->rx_desc_count;i++){
		int i_next = (i+1) % priv->rx_desc_count;
		priv->rx_desc_virt[i].status = RX_DMA_DESC_STATUS_COMPLETED;
		priv->rx_desc_virt[i].next = priv->rx_desc_phys + i_next * sizeof(struct sgeth_rx_descriptor);
		priv->rx_desc_virt[i].control = SGETH_MAX_JUMBO_FRAME_SIZE | RX_DMA_DESC_CONTROL_IRQ_ALL;
	}

	priv->rx_skb = devm_kcalloc(&ndev->dev, priv->rx_desc_count,
				  sizeof(*priv->rx_skb), GFP_KERNEL);
	if (!priv->rx_skb){
		return -ENOMEM;
	}

	netif_napi_add(ndev, &priv->rx_napi, spinal_sgeth_poll);
	napi_enable(&priv->rx_napi);

	err = request_irq(priv->tx_irq, spinal_sgeth_tx_irq, 0, ndev->name, ndev);
	if (err)
		goto err_tx_irq;
	err = request_irq(priv->rx_irq, spinal_sgeth_rx_irq, 0, ndev->name, ndev);
	if (err)
		goto err_rx_irq;



	netif_carrier_on(ndev);
	netif_start_queue(ndev);

	priv->rx_irq_first = 1;
	priv->rx_irq_config  = RX_DMA_IRQ_IDLE_ENABLE;
	priv->rx_irq_config |= RX_DMA_IRQ_DELAY_ENABLE | (50 << RX_DMA_IRQ_DELAY_LIMIT_AT);
	priv->rx_irq_config |= RX_DMA_IRQ_COUNTER_ENABLE | ((priv->rx_desc_count+3)/4 << RX_DMA_IRQ_COUNTER_TARGET_AT);
	writel_relaxed(priv->rx_irq_config, priv->rx_dma + RX_DMA_IRQ);
	spinal_sgeth_rx_dma_irq_enable(priv->rx_dma);

	priv->tx_irq_config |= TX_DMA_IRQ_DELAY_ENABLE | (200 << TX_DMA_IRQ_DELAY_LIMIT_AT);
	priv->tx_irq_config |= TX_DMA_IRQ_COUNTER_ENABLE | ((priv->tx_desc_count+3)/4 << TX_DMA_IRQ_COUNTER_TARGET_AT);
//	priv->tx_irq_config = 0;
	writel_relaxed(priv->tx_irq_config, priv->tx_dma + TX_DMA_IRQ);
	spinal_sgeth_tx_dma_irq_enable(priv->tx_dma);




	return 0;

 err_rx_irq:
	free_irq(priv->tx_irq, ndev);
 err_tx_irq:
	netdev_err(ndev, "request_irq() failed\n");
	return err;
}

static int spinal_sgeth_stop(struct net_device *ndev)
{
	struct spinal_sgeth *priv = netdev_priv(ndev);

	netdev_info(ndev, "spinal_sgeth_stop\n");

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);


//	free_irq(ndev->irq, ndev);

	return 0;
}




static netdev_tx_t spinal_sgeth_start_xmit(struct sk_buff *skb,
				      struct net_device *ndev)
{
	struct spinal_sgeth *priv = netdev_priv(ndev);
//	netdev_info(ndev, "spinal_sgeth_start_xmit %d %d\n", skb->len, skb_shinfo(skb)->nr_frags);
	unsigned long num_frag;
	skb_frag_t *frag;
	int desc_id, desc_id_masked, first_id;
	dma_addr_t skb_dma_addr;
	volatile struct sgeth_tx_descriptor * desc;

	frag = &skb_shinfo(skb)->frags[0];
	num_frag = skb_shinfo(skb)->nr_frags;
	desc_id = priv->tx_desc_alloc;
	first_id = desc_id;
	desc_id_masked = desc_id & (priv->tx_desc_count-1);
//	netdev_info(ndev, "skb %d ", skb_frag_size(frag));

//	netdev_info(priv->ndev, "frame transmited %d bytes %d frag\n",
//			skb_headlen(skb), num_frag);
//	print_hex_dump(KERN_ERR, "TX: ", DUMP_PREFIX_OFFSET,
//			   16, 1, skb->data, skb_headlen(skb), 1);

//	if (skb->ip_summed == CHECKSUM_PARTIAL) {
//		unsigned int csum_start_off = skb_checksum_start_offset(skb);
//		unsigned int csum_index_off = csum_start_off + skb->csum_offset;
//		netdev_info(ndev, "CHECKSUM_PARTIAL ??\n");
//	}

	if (spinal_sgeth_get_tx_available(priv) < num_frag + 1) {
		netdev_err(ndev, "NETDEV_TX_BUSY ????\n");
		return NETDEV_TX_BUSY;
	}

	priv->tx_desc_alloc += num_frag + 1;
	if (spinal_sgeth_get_tx_available(priv) < MAX_SKB_FRAGS + 1){
		smp_mb();
		netif_stop_queue(ndev);
	}


	skb_dma_addr = dma_map_single(ndev->dev.parent, skb->data,
				      skb_headlen(skb), DMA_TO_DEVICE);
	if (WARN_ON_ONCE(dma_mapping_error(ndev->dev.parent, skb_dma_addr))) {
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		priv->tx_desc_alloc -= num_frag + 1;
		return NETDEV_TX_OK;
	}

	desc = priv->tx_desc_virt + desc_id_masked;
	desc->from = skb_dma_addr;
	desc->control = skb_headlen(skb) | TX_DMA_DESC_CONTROL_IRQ | (num_frag == 0 ? TX_DMA_DESC_CONTROL_LAST : 0);
	desc_id += 1;
	desc_id_masked = desc_id & (priv->tx_desc_count-1);
	for (int ii = 0; ii < num_frag; ii++) {
//		netdev_info(priv->ndev, "-  %d bytes\n", skb_frag_size(frag));
//		print_hex_dump(KERN_ERR, "TX: ", DUMP_PREFIX_OFFSET,
//				   16, 1, skb_frag_address(frag), skb_frag_size(frag), 1);

		if(skb_frag_size(frag) < 1){
			netdev_info(priv->ndev, "Zero byte fragment ????\n");
		}

		desc = priv->tx_desc_virt + desc_id_masked;
		skb_dma_addr = dma_map_single(ndev->dev.parent,
					      skb_frag_address(frag),
					      skb_frag_size(frag),
					      DMA_TO_DEVICE);
		if (dma_mapping_error(ndev->dev.parent, skb_dma_addr)) {
			netdev_err(ndev, "??? X\n");
//			if (--lp->tx_bd_tail < 0)
//				lp->tx_bd_tail = lp->tx_bd_num - 1;
//			cur_p = &lp->tx_bd_v[lp->tx_bd_tail];
//			while (--ii >= 0) {
//				--frag;
//				dma_unmap_single(ndev->dev.parent,
//						 be32_to_cpu(cur_p->phys),
//						 skb_frag_size(frag),
//						 DMA_TO_DEVICE);
//				if (--lp->tx_bd_tail < 0)
//					lp->tx_bd_tail = lp->tx_bd_num - 1;
//				cur_p = &lp->tx_bd_v[lp->tx_bd_tail];
//			}
//			dma_unmap_single(ndev->dev.parent,
//					 be32_to_cpu(cur_p->phys),
//					 skb_headlen(skb), DMA_TO_DEVICE);
//			dev_kfree_skb_any(skb);
//			ndev->stats.tx_dropped++;
			priv->tx_desc_alloc -= num_frag + 1;
			return NETDEV_TX_OK;
		}
		desc->from = skb_dma_addr;
		desc->control = skb_frag_size(frag) | TX_DMA_DESC_CONTROL_IRQ | (ii == num_frag-1 ? TX_DMA_DESC_CONTROL_LAST : 0);
		frag++;
//		netdev_info(ndev, "skb %d ", skb_frag_size(frag));
		desc_id += 1;
		desc_id_masked = desc_id & (priv->tx_desc_count-1);
	}

	unsigned long flags;
	spin_lock_irqsave(&priv->tx_lock, flags);

	int desc_last_id = (desc_id-1) & (priv->tx_desc_count-1) ;
	priv->tx_skb[desc_last_id] = skb;


	int from = desc_id;
	int to = first_id;
	while(from != to){
		from -= 1;
		priv->tx_desc_virt[from & (priv->tx_desc_count-1)].status = 0;
	}

	priv->tx_desc_sent += num_frag + 1;

	smp_mb();
	spin_unlock_irqrestore(&priv->tx_lock, flags);


	if(spinal_sgeth_tx_dma_is_going_idle(priv->tx_dma)){
		while(spinal_sgeth_tx_dma_busy(priv->tx_dma));

		int first_id_masked = first_id & (priv->tx_desc_count-1);
		spinal_sgeth_tx_dma_set_next(priv->tx_dma, priv->tx_desc_phys + first_id_masked * sizeof(struct sgeth_tx_descriptor));
		spinal_sgeth_tx_dma_start(priv->tx_dma);
	}


//	while(spinal_sgeth_tx_dma_busy(priv->tx_dma));
//	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}



//
//static netdev_tx_t spinal_sgeth_start_xmit(struct sk_buff *skb,
//				      struct net_device *ndev)
//{
//	struct spinal_sgeth *priv = netdev_priv(ndev);
////	netdev_info(ndev, "spinal_sgeth_start_xmit %d %d\n", skb->len, skb_shinfo(skb)->nr_frags);
//	unsigned long num_frag;
//	skb_frag_t *frag;
//	int desc_id;
//	dma_addr_t skb_dma_addr;
//	volatile struct sgeth_tx_descriptor * desc;
//
//	frag = &skb_shinfo(skb)->frags[0];
//	num_frag = skb_shinfo(skb)->nr_frags;
//	desc_id = priv->tx_desc_alloc;
////	netdev_info(ndev, "skb %d ", skb_frag_size(frag));
//
////	netdev_info(priv->ndev, "frame transmited %d bytes %d frag\n",
////			skb_headlen(skb), num_frag);
////	print_hex_dump(KERN_ERR, "TX: ", DUMP_PREFIX_OFFSET,
////			   16, 1, skb->data, skb_headlen(skb), 1);
//
//	if (skb->ip_summed == CHECKSUM_PARTIAL) {
//		unsigned int csum_start_off = skb_checksum_start_offset(skb);
//		unsigned int csum_index_off = csum_start_off + skb->csum_offset;
//		netdev_info(ndev, "CHECKSUM_PARTIAL ??\n");
//	}
//
//	skb_dma_addr = dma_map_single(ndev->dev.parent, skb->data,
//				      skb_headlen(skb), DMA_TO_DEVICE);
//	if (WARN_ON_ONCE(dma_mapping_error(ndev->dev.parent, skb_dma_addr))) {
//		dev_kfree_skb_any(skb);
//		ndev->stats.tx_dropped++;
//		return NETDEV_TX_OK;
//	}
//
//	desc = priv->tx_desc_virt + desc_id;
//	desc->from = skb_dma_addr;
//	desc->control = skb_headlen(skb) | (num_frag == 0 ? TX_DMA_DESC_CONTROL_LAST : 0);
//	desc->status = 0;
//	desc_id += 1;
//	if(desc_id == priv->tx_desc_count) desc_id = 0;
//
//	for (int ii = 0; ii < num_frag; ii++) {
//		desc = priv->tx_desc_virt + desc_id;
//		skb_dma_addr = dma_map_single(ndev->dev.parent,
//					      skb_frag_address(frag),
//					      skb_frag_size(frag),
//					      DMA_TO_DEVICE);
//		if (dma_mapping_error(ndev->dev.parent, skb_dma_addr)) {
//			netdev_err_once(ndev, "??? X\n");
////			if (--lp->tx_bd_tail < 0)
////				lp->tx_bd_tail = lp->tx_bd_num - 1;
////			cur_p = &lp->tx_bd_v[lp->tx_bd_tail];
////			while (--ii >= 0) {
////				--frag;
////				dma_unmap_single(ndev->dev.parent,
////						 be32_to_cpu(cur_p->phys),
////						 skb_frag_size(frag),
////						 DMA_TO_DEVICE);
////				if (--lp->tx_bd_tail < 0)
////					lp->tx_bd_tail = lp->tx_bd_num - 1;
////				cur_p = &lp->tx_bd_v[lp->tx_bd_tail];
////			}
////			dma_unmap_single(ndev->dev.parent,
////					 be32_to_cpu(cur_p->phys),
////					 skb_headlen(skb), DMA_TO_DEVICE);
////			dev_kfree_skb_any(skb);
////			ndev->stats.tx_dropped++;
//			return NETDEV_TX_OK;
//		}
//		desc->from = skb_dma_addr;
//		desc->control = skb_frag_size(frag) | (ii == num_frag-1 ? TX_DMA_DESC_CONTROL_LAST : 0);
//		desc->status = 0;
//		frag++;
////		netdev_info(ndev, "skb %d ", skb_frag_size(frag));
//		desc_id += 1;
//		if(desc_id == priv->tx_desc_count) desc_id = 0;
//	}
//
//	spinal_sgeth_tx_dma_set_next(priv->tx_dma, priv->tx_desc_phys + priv->tx_desc_alloc * sizeof(struct sgeth_tx_descriptor));
//	spinal_sgeth_tx_dma_start(priv->tx_dma);
//	while(spinal_sgeth_tx_dma_busy(priv->tx_dma));
////	netdev_info(ndev, "sent %x", priv->tx_desc_virt[priv->tx_desc_alloc].status);
//
//	priv->tx_desc_alloc = desc_id;
//
//	dev_kfree_skb_any(skb); //TODO maybe not any but irq once moved there
//
//	return NETDEV_TX_OK;
//
////	void __iomem *txbuffer;
////
////	if (!litex_read8(priv->base + spinal_sgeth_READER_READY)) {
////		if (net_ratelimit())
////			netdev_err(ndev, "spinal_sgeth_READER_READY not ready\n");
////
////		netif_stop_queue(ndev);
////
////		return NETDEV_TX_BUSY;
////	}
////
////	/* Reject oversize packets */
////	if (unlikely(skb->len > priv->slot_size)) {
////		if (net_ratelimit())
////			netdev_err(ndev, "tx packet too big\n");
////
////		dev_kfree_skb_any(skb);
////		ndev->stats.tx_dropped++;
////		ndev->stats.tx_errors++;
////
////		return NETDEV_TX_OK;
////	}
////
////	txbuffer = priv->tx_base + priv->tx_slot * priv->slot_size;
////	memcpy_toio(txbuffer, skb->data, skb->len);
////	litex_write8(priv->base + spinal_sgeth_READER_SLOT, priv->tx_slot);
////	litex_write16(priv->base + spinal_sgeth_READER_LENGTH, skb->len);
////	litex_write8(priv->base + spinal_sgeth_READER_START, 1);
////
////	dev_sw_netstats_tx_add(ndev, 1, skb->len);
////
////	priv->tx_slot = (priv->tx_slot + 1) % priv->num_tx_slots;
////	dev_kfree_skb_any(skb); //TODO maybe not any but irq
////
////	return NETDEV_TX_OK;
//}

static void spinal_sgeth_get_stats64(struct net_device *ndev, struct rtnl_link_stats64 *stats)
{
	netdev_stats_to_stats64(stats, &ndev->stats);
	dev_fetch_sw_netstats(stats, ndev->tstats);
}

static const struct net_device_ops spinal_sgeth_netdev_ops = {
	.ndo_open		= spinal_sgeth_open,
	.ndo_stop		= spinal_sgeth_stop,
	.ndo_get_stats64	= spinal_sgeth_get_stats64,
	.ndo_start_xmit         = spinal_sgeth_start_xmit,
};


static void sgeth_timer(struct timer_list *t)
{
	struct spinal_sgeth *priv = from_timer(priv, t, poll_timer);
	static int trigerred = 0;
	if(priv->tx_desc_virt){
		printk("RPT: %d %d %d %d %x\n", priv->tx_desc_alloc, priv->tx_desc_sent, priv->tx_desc_free, spinal_sgeth_tx_dma_busy(priv->tx_dma), priv->tx_desc_virt[priv->tx_desc_free & (priv->tx_desc_count-1)].status);
		int id = priv->tx_desc_free & (priv->tx_desc_count-1);
		int desc_done = priv->tx_desc_virt[id].status & TX_DMA_DESC_STATUS_COMPLETED;

		if(!desc_done && spinal_sgeth_tx_dma_is_going_idle(priv->tx_dma)){
	//		printk("G\n");
			trigerred = trigerred + 1;
			if(trigerred == 8){
				printk("Try restart\n");
				spinal_sgeth_tx_dma_set_next(priv->tx_dma, priv->tx_desc_phys + id * sizeof(struct sgeth_tx_descriptor));
				spinal_sgeth_tx_dma_start(priv->tx_dma);
			}
		} else {
			trigerred = 0;
		}


	}
	mod_timer(&priv->poll_timer, jiffies + msecs_to_jiffies(1000));
}


static int spinal_sgeth_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	void __iomem *buf_base;
	struct spinal_sgeth *priv;
	int err;
	ndev = devm_alloc_etherdev(&pdev->dev, sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, &pdev->dev);
	platform_set_drvdata(pdev, ndev);

	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	priv->dev = &pdev->dev;
	spin_lock_init(&priv->rx_lock);
	spin_lock_init(&priv->tx_lock);

	ndev->tstats = devm_netdev_alloc_pcpu_stats(&pdev->dev,
						      struct pcpu_sw_netstats);
	if (!ndev->tstats)
		return -ENOMEM;

	priv->rx_irq = platform_get_irq_byname(pdev, "rx-dma");
	if (priv->rx_irq < 0) {
		dev_err(&pdev->dev, "Failed to get rx-dma irq\n");
		return priv->rx_irq;
	}
	priv->tx_irq = platform_get_irq_byname(pdev, "tx-dma");
	if (priv->tx_irq < 0) {
		dev_err(&pdev->dev, "Failed to get tx-dma irq\n");
		return priv->tx_irq;
	}

	priv->mac = devm_platform_ioremap_resource_byname(pdev, "mac");
	if (IS_ERR(priv->mac)) {
		dev_err(&pdev->dev, "Missing mac reg\n");
		return PTR_ERR(priv->mac);
	}

	priv->rx_dma = devm_platform_ioremap_resource_byname(pdev, "rx-dma");
	if (IS_ERR(priv->rx_dma)) {
		dev_err(&pdev->dev, "rx-dma\n");
		return PTR_ERR(priv->rx_dma);
	}

	priv->tx_dma = devm_platform_ioremap_resource_byname(pdev, "tx-dma");
	if (IS_ERR(priv->tx_dma)) {
		dev_err(&pdev->dev, "tx-dma\n");
		return PTR_ERR(priv->tx_dma);
	}

	err = of_get_ethdev_address(pdev->dev.of_node, ndev);
	if (err)
		eth_hw_addr_random(ndev);

//	ndev->mtu = 9000;
	ndev->netdev_ops = &spinal_sgeth_netdev_ops;
	ndev->features  = NETIF_F_SG;
	ndev->features |= NETIF_F_HIGHDMA;
	ndev->features |= NETIF_F_GSO;
	ndev->features |= NETIF_F_IP_CSUM;
	ndev->features |= NETIF_F_TSO;


//	ndev->features |= (NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM | NETIF_F_RXCSUM) | NETIF_F_GRO;
//	ndev->features |= NETIF_F_GSO_FRAGLIST;
//	ndev->features |= NETIF_F_FRAGLIST;

#if 0
	ndev->features |= NETIF_F_IP_CSUM; /* Can checksum TCP/UDP over IPv4. */
	ndev->features |= NETIF_F_HW_CSUM; /* Can checksum all the packets. */
	ndev->features |= NETIF_F_IPV6_CSUM; /* Can checksum IPV6 TCP/UDP */
	ndev->features |= NETIF_F_HW_VLAN_CTAG_TX; /* Transmit VLAN hw accel */
	ndev->features |= NETIF_F_HW_VLAN_CTAG_RX; /* Receive VLAN hw acceleration */
	ndev->features |= NETIF_F_HW_VLAN_CTAG_FILTER; /* Receive VLAN filtering */
	ndev->features |= NETIF_F_VLAN_CHALLENGED; /* cannot handle VLAN pkts */
	ndev->features |= NETIF_F_GSO; /* Enable software GSO. */
	ndev->features |= NETIF_F_MULTI_QUEUE; /* Has multiple TX/RX queues */
	ndev->features |= NETIF_F_LRO; /* large receive offload */
#endif


	err = register_netdev(ndev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register ndev %d\n", err);
		return err;
	}

	netdev_info(ndev, "Hello world \n");
	netdev_dbg(ndev, "Brawwwww\n");

//	timer_setup(&priv->poll_timer, sgeth_timer, 0);
//	mod_timer(&priv->poll_timer, jiffies + msecs_to_jiffies(50));

	return 0;
}

static void spinal_sgeth_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	unregister_netdev(ndev);
}

static const struct of_device_id spinal_sgeth_of_match[] = {
	{ .compatible = "spinal,sgeth" },
	{ }
};
MODULE_DEVICE_TABLE(of, spinal_sgeth_of_match);

static struct platform_driver spinal_sgeth_driver = {
	.probe = spinal_sgeth_probe,
	.remove_new = spinal_sgeth_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = spinal_sgeth_of_match,
	},
};
module_platform_driver(spinal_sgeth_driver);

MODULE_AUTHOR("Charles Papon <charles.papon.90@gmail.com>");
MODULE_DESCRIPTION("Spinal sgeth Ethernet driver");
MODULE_LICENSE("GPL");
