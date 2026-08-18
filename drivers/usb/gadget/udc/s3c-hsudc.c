// SPDX-License-Identifier: GPL-2.0
/* linux/drivers/usb/gadget/s3c-hsudc.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S3C24XX USB 2.0 High-speed USB controller gadget driver
 *
 * The S3C24XX USB 2.0 high-speed USB controller supports upto 9 endpoints.
 * Each endpoint can be configured as either in or out endpoint. Endpoints
 * can be configured for Bulk or Interrupt transfer mode.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
//#include <linux/usb/otg.h>
#include <linux/prefetch.h>
//#include <linux/platform_data/s3c-hsudc.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/usb/composite.h>



#define S3C_HSUDC_REG(x)	(x)

/* Non-Indexed Registers */
#define S3C_IR				S3C_HSUDC_REG(0x00) /* Index Register */
#define S3C_EIR				S3C_HSUDC_REG(0x04) /* EP Intr Status */
#define S3C_EIR_EP0			(1<<0)
#define S3C_EIER			S3C_HSUDC_REG(0x08) /* EP Intr Enable */
#define S3C_FAR				S3C_HSUDC_REG(0x0c) /* Gadget Address */
#define S3C_FNR				S3C_HSUDC_REG(0x10) /* Frame Number */
#define S3C_EDR				S3C_HSUDC_REG(0x14) /* EP Direction */
#define S3C_TR				S3C_HSUDC_REG(0x18) /* Test Register */
#define S3C_SSR				S3C_HSUDC_REG(0x1c) /* System Status */
#define S3C_SSR_DTZIEN_EN		(0xff8f)
#define S3C_SSR_ERR			(0xff80)
#define S3C_SSR_VBUSON			(1 << 8)
#define S3C_SSR_HSP			(1 << 4)
#define S3C_SSR_SDE			(1 << 3)
#define S3C_SSR_RESUME			(1 << 2)
#define S3C_SSR_SUSPEND			(1 << 1)
#define S3C_SSR_RESET			(1 << 0)
#define S3C_SCR				S3C_HSUDC_REG(0x20) /* System Control */
#define S3C_SCR_DTZIEN_EN		(1 << 14)
#define S3C_SCR_DIEN_EN			(1 << 12)
#define S3C_SCR_RRD_EN			(1 << 5)
#define S3C_SCR_SUS_EN			(1 << 1)
#define S3C_SCR_RST_EN			(1 << 0)
#define S3C_EP0SR			S3C_HSUDC_REG(0x24) /* EP0 Status */
#define S3C_EP0SR_EP0_LWO		(1 << 6)
#define S3C_EP0SR_STALL			(1 << 4)
#define S3C_EP0SR_TX_SUCCESS		(1 << 1)
#define S3C_EP0SR_RX_SUCCESS		(1 << 0)//auto clear,no need for manual clear
#define S3C_EP0CR_TZLS			(1 << 0) /* Tx Zero Length Set */
#define S3C_EP0CR			S3C_HSUDC_REG(0x28) /* EP0 Control */
#define S3C_BR(_x)			S3C_HSUDC_REG(0x60 + (_x * 4))

/* Indexed Registers */
#define S3C_ESR				S3C_HSUDC_REG(0x2c) /* EPn Status */
#define S3C_ESR_DTCZ        	(1 << 9) /* DMA Total Count Zero */
#define S3C_ESR_DOM				(1 << 7)
#define S3C_ESR_FLUSH			(1 << 6)
#define S3C_ESR_STALL			(1 << 5)
#define S3C_ESR_LWO				(1 << 4)
#define S3C_ESR_PSIF_ONE		(1 << 2)
#define S3C_ESR_PSIF_TWO		(2 << 2)
#define S3C_ESR_TX_SUCCESS		(1 << 1)
#define S3C_ESR_RX_SUCCESS		(1 << 0)
#define S3C_ECR				S3C_HSUDC_REG(0x30) /* EPn Control */
#define S3C_ECR_DUEN			(1 << 7)
#define S3C_ECR_FLUSH			(1 << 6)
#define S3C_ECR_STALL			(1 << 1)
#define S3C_ECR_IEMS			(1 << 0)
#define S3C_BRCR			S3C_HSUDC_REG(0x34) /* Read Count */
#define S3C_BWCR			S3C_HSUDC_REG(0x38) /* Write Count */
#define S3C_MPR				S3C_HSUDC_REG(0x3c) /* Max Pkt Size */
#define S3C_DCR             S3C_HSUDC_REG(0x40) /* DMA Control */
#define S3C_DCR_FMDE        (1 << 4) /* Burst Mode Enable */
#define S3C_DCR_TDR         (1 << 2) /* TX DMA Run */
#define S3C_DCR_DEN         (1 << 0) /* DMA Enable */

#define S3C_DTCR            S3C_HSUDC_REG(0x44) /* DMA Transfer Count (Packet Size) */
#define S3C_DFCR            S3C_HSUDC_REG(0x48) /* DMA FIFO Control */
#define S3C_DTTCR1          S3C_HSUDC_REG(0x4c) /* Total Transfer Count Low */
#define S3C_DTTCR2          S3C_HSUDC_REG(0x50) /* Total Transfer Count High */

#define S3C_DICR            S3C_HSUDC_REG(0x84) /* DMA Interface Control */
#define S3C_DICR_BURST_16   (3 << 0)

#define S3C_MBAR            S3C_HSUDC_REG(0x88) /* Memory Base Address */

#define S3C_FCON            S3C_HSUDC_REG(0x100)
#define S3C_FCON_DMAEN		(1 << 8)


#define DMA_ADDR_INVALID        (~(dma_addr_t)0)

#define WAIT_FOR_SETUP			(0)
#define DATA_STATE_XMIT			(1)
#define DATA_STATE_RECV			(2)


static const char * const s3c_hsudc_supply_names[] = {
	"vdda",		/* analog phy supply, 3.3V */
	"vddi",		/* digital phy supply, 1.2V */
	"vddosc",	/* oscillator supply, 1.8V - 3.3V */
};
enum s3c_ep0_state {
	EP0_IDLE,
	EP0_STAGE_SETUP,    /* 收到 SETUP 包 */
	EP0_STAGE_TX,       /* IN 数据阶段 (Device -> Host) */
	EP0_STAGE_RX,       /* OUT 数据阶段 (Host -> Device) */
	EP0_STAGE_STATUSIN, /* OUT 传输后的 Status IN (ZLP) */
	EP0_STAGE_STATUSOUT,/* IN 传输后的 Status OUT (ZLP) */
	EP0_STAGE_ACKWAIT,  /* 等待 Host 的 ZLP (无数据阶段) */
};
static const char *decode_ep0stage(enum s3c_ep0_state state)
{
	switch (state) {
	case EP0_IDLE: return "IDLE";
	case EP0_STAGE_SETUP: return "SETUP";
	case EP0_STAGE_TX: return "TX";
	case EP0_STAGE_RX: return "RX";
	case EP0_STAGE_STATUSIN: return "STATUS_IN";
	case EP0_STAGE_STATUSOUT: return "STATUS_OUT";
	case EP0_STAGE_ACKWAIT: return "ACKWAIT";
	default: return "?";
	}
}
/**
 * struct s3c_hsudc_ep - Endpoint representation used by driver.
 * @ep: USB gadget layer representation of device endpoint.
 * @name: Endpoint name (as required by ep autoconfiguration).
 * @dev: Reference to the device controller to which this EP belongs.
 * @desc: Endpoint descriptor obtained from the gadget driver.
 * @queue: Transfer request queue for the endpoint.
 * @stopped: Maintains state of endpoint, set if EP is halted.
 * @bEndpointAddress: EP address (including direction bit).
 * @fifo: Base address of EP FIFO.
 */
struct s3c_hsudc_ep {
	struct usb_ep ep;
	char name[20];
	struct s3c_hsudc *dev;
	struct list_head queue;
	u8 stopped;
	u8 wedge;
	u8 bEndpointAddress;
	void __iomem *fifo;
	bool dma_running; 
};

/**
 * struct s3c_hsudc_req - Driver encapsulation of USB gadget transfer request.
 * @req: Reference to USB gadget transfer request.
 * @queue: Used for inserting this request to the endpoint request queue.
 */
struct s3c_hsudc_req {
	struct usb_request req;
	struct list_head queue;
	bool mapped; 
};

/**
 * struct s3c_hsudc - Driver's abstraction of the device controller.
 * @gadget: Instance of usb_gadget which is referenced by gadget driver.
 * @driver: Reference to currently active gadget driver.
 * @dev: The device reference used by probe function.
 * @lock: Lock to synchronize the usage of Endpoints (EP's are indexed).
 * @regs: Remapped base address of controller's register space.
 * irq: IRQ number used by the controller.
 * uclk: Reference to the controller clock.
 * ep0state: Current state of EP0.
 * ep: List of endpoints supported by the controller.
 */
struct s3c_hsudc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct device *dev;
	
	int epnum;
	struct phy *phy;
	
	struct regulator_bulk_data supplies[ARRAY_SIZE(s3c_hsudc_supply_names)];
	spinlock_t lock;
	void __iomem *regs;
	int irq;
	struct clk *uclk;
	u8 dev_addr;
    bool set_addr_pending;
	enum s3c_ep0_state ep0state;
	struct s3c_hsudc_ep ep[];
};

#define ep_maxpacket(_ep)	((_ep)->ep.maxpacket)
#define ep_is_in(_ep)		((_ep)->bEndpointAddress & USB_DIR_IN)
#define ep_index(_ep)		((_ep)->bEndpointAddress & \
					USB_ENDPOINT_NUMBER_MASK)

static const char driver_name[] = "s3c-udc";
static const char ep0name[] = "ep0-control";
/* 调试开关：只在调试时打开，量产时关闭 */
//#define DEBUG_EP0_TRACE

#ifdef DEBUG_EP0_TRACE
#define ep0_dbg(fmt, args...) printk(KERN_ERR "[EP0][%s] " fmt "\n", __func__, ## args)
#else
#define ep0_dbg(fmt, args...) do {} while (0)
#endif

/* 辅助函数：打印状态名 */
void s3c_hsudc_dump_registers(struct s3c_hsudc *hsudc)
{
	unsigned long flags;
	int i;
	u32 esr, ecr, brcr;
    u32 sys_status = readl(hsudc->regs + S3C_SSR);
    u32 irq_status = readl(hsudc->regs + S3C_EIR);
    u32 irq_mask = readl(hsudc->regs + S3C_EIER);

	spin_lock_irqsave(&hsudc->lock, flags);

	printk(KERN_ERR "=== S3C HSUDC DEBUG DUMP ===\n");
    printk(KERN_ERR "SSR: %08x | EIR: %08x | EIER: %08x\n", sys_status, irq_status, irq_mask);

	for (i = 0; i < hsudc->epnum; i++) {
		struct s3c_hsudc_ep *hsep = &hsudc->ep[i];
		u32 offset = (i == 0) ? S3C_EP0SR : S3C_ESR;
        u32 ctrl_offset = (i == 0) ? S3C_EP0CR : S3C_ECR;

		// 切换索引以读取正确的寄存器
		writel(i, hsudc->regs + S3C_IR);
		esr = readl(hsudc->regs + offset);
        ecr = readl(hsudc->regs + ctrl_offset);
        brcr = readl(hsudc->regs + S3C_BRCR);

		printk(KERN_ERR "EP%d (%s): ESR=%08x ECR=%08x BRCR=%08x QLen=%d stopped=%d\n",
			i, hsep->ep.name, esr, ecr, brcr, 
            list_empty(&hsep->queue) ? 0 : 1, // 简化的队列检查
            hsep->stopped);
        
        if (i != 0) { // 分析 Bulk 端点
            if (esr & S3C_ESR_RX_SUCCESS) printk(KERN_ERR "  -> HAS DATA in FIFO!\n");
            if (esr & S3C_ESR_PSIF_TWO)   printk(KERN_ERR "  -> FIFO FULL!\n");
            if (list_empty(&hsep->queue)) printk(KERN_ERR "  -> NO REQUEST queued by gadget!\n");
        }
	}
	spin_unlock_irqrestore(&hsudc->lock, flags);
}
/* 控制测试只运行一次 */
static int g_flush_test_done = 0;

/* 
 * 暴力测试 FLUSH 行为 
 * 调用时必须持有 hsudc->lock
 */
static void debug_test_flush_behavior(struct s3c_hsudc *hsudc, int ep_num)
{
    u32 ecr, esr, psif;
    int i;
    int loop_limit = 1000;

    printk(KERN_ERR "\n====== FLUSH TEST START [EP%d] ======\n", ep_num);

    // 1. 读取初始状态
    // 切换 Index 到当前端点
    writel(ep_num, hsudc->regs + S3C_IR); 
    
    esr = readl(hsudc->regs + S3C_ESR);
    psif = (esr >> 2) & 0x3; // PSIF bits [3:2]

    printk(KERN_ERR "Step 1: Before Flush\n");
    printk(KERN_ERR "   -> ESR = 0x%08x\n", esr);
    printk(KERN_ERR "   -> PSIF(Packets in FIFO) = %d (Should be > 0)\n", psif);

    if (psif == 0) {
        printk(KERN_ERR "   -> TEST ABORTED: FIFO is empty! Cannot test flush.\n");
        return;
    }

    // 2. 执行 FLUSH 置位
    printk(KERN_ERR "Step 2: Set FLUSH bit\n");
    ecr = readl(hsudc->regs + S3C_ECR);
    writel(ecr | S3C_ECR_FLUSH, hsudc->regs + S3C_ECR);

    // 3. 立即回读 ECR，看是否自动清除
    ecr = readl(hsudc->regs + S3C_ECR);
    printk(KERN_ERR "   -> Readback ECR = 0x%08x\n", ecr);
    printk(KERN_ERR "   -> FLUSH bit is: %s\n", (ecr & S3C_ECR_FLUSH) ? "1 (Manual Clear Needed?)" : "0 (Auto Cleared)");

    // 4. 等待 FFS (FIFO Flushed) 标志位置位
    printk(KERN_ERR "Step 3: Wait for FFS in ESR\n");
    for (i = 0; i < loop_limit; i++) {
        esr = readl(hsudc->regs + S3C_ESR);
        if (esr & S3C_ESR_FLUSH) { // 假设 S3C_ESR_FLUSH 是 FFS 位 (bit 6)
            printk(KERN_ERR "   -> FFS set detected at loop %d! ESR=0x%08x\n", i, esr);
            break;
        }
        // 在中断里不能 msleep，只能空转或 udelay
        udelay(1); 
    }
    if (i == loop_limit) {
        printk(KERN_ERR "   -> TIMEOUT: FFS bit never appeared!\n");
    }

    // 5. 如果 ECR 里的 FLUSH 还是 1，手动清除它
    ecr = readl(hsudc->regs + S3C_ECR);
    if (ecr & S3C_ECR_FLUSH) {
        printk(KERN_ERR "Step 4: Manually Clearing FLUSH bit in ECR\n");
        writel(ecr & ~S3C_ECR_FLUSH, hsudc->regs + S3C_ECR);
    } else {
        printk(KERN_ERR "Step 4: Skipped (FLUSH bit already 0)\n");
    }

    // 6. 再次检查 FIFO 状态
    esr = readl(hsudc->regs + S3C_ESR);
    psif = (esr >> 2) & 0x3;
    
    printk(KERN_ERR "Step 5: Final Result\n");
    printk(KERN_ERR "   -> ESR = 0x%08x\n", esr);
    printk(KERN_ERR "   -> Packets remaining: %d (Expected 0)\n", psif);
    
    if (psif == 0)
        printk(KERN_ERR "====== TEST SUCCESS: FIFO Cleared ======\n");
    else
        printk(KERN_ERR "====== TEST FAILED: FIFO Not Empty ======\n");
}

static inline struct s3c_hsudc_req *our_req(struct usb_request *req)
{
	return container_of(req, struct s3c_hsudc_req, req);
}

static inline struct s3c_hsudc_ep *our_ep(struct usb_ep *ep)
{
	return container_of(ep, struct s3c_hsudc_ep, ep);
}

static inline struct s3c_hsudc *to_hsudc(struct usb_gadget *gadget)
{
	return container_of(gadget, struct s3c_hsudc, gadget);
}

static inline void set_index(struct s3c_hsudc *hsudc, int ep_addr)
{
	ep_addr &= USB_ENDPOINT_NUMBER_MASK;
	writel(ep_addr, hsudc->regs + S3C_IR);
}

static inline void __orr32(void __iomem *ptr, u32 val)
{
	writel(readl(ptr) | val, ptr);
}
static void s3c_hsudc_ep0_stall(struct s3c_hsudc *hsudc)
{
	u32 ecr;
	set_index(hsudc, 0);
	ecr = readl(hsudc->regs + S3C_EP0CR);
	ecr |= S3C_ECR_STALL;
	writel(ecr, hsudc->regs + S3C_EP0CR);
	hsudc->ep0state = EP0_IDLE;
	ep0_dbg("EP0 Stalled");
}
/* 
 * 硬件 Flush 逻辑，基于实验结果：
 * 1. 写 ECR.FLUSH
 * 2. 等 ESR.FFS 置位
 * 3. 写 ESR.FFS 清除标志
 */
static void s3c_hsudc_flush_fifo(struct s3c_hsudc_ep *hsep)
{
	struct s3c_hsudc *hsudc = hsep->dev;
	u32 ecr, esr;
	int count;

	// 1. 选中端点
	set_index(hsudc, ep_index(hsep));

	// 2. 触发 Flush
	ecr = readl(hsudc->regs + S3C_ECR);
	writel(ecr | S3C_ECR_FLUSH, hsudc->regs + S3C_ECR);

	// 3. 等待硬件完成 (FFS bit)
	// 实验显示 loop 0 就完成了，给 100 次防身足够
	for (count = 0; count < 100; count++) {
		esr = readl(hsudc->regs + S3C_ESR);
		if (esr & S3C_ESR_FLUSH)
			break;
		udelay(1);
	}
    
	if (count == 100)
		dev_err(hsudc->dev, "Timeout flushing EP%d\n", ep_index(hsep));

	// 4. 清除 FFS 标志位 (Write-1-to-Clear)
	// 这一步至关重要，否则中断线可能一直拉高
	writel(S3C_ESR_FLUSH, hsudc->regs + S3C_ESR);
}
/**
 * s3c_hsudc_complete_request - Complete a transfer request.
 * @hsep: Endpoint to which the request belongs.
 * @hsreq: Transfer request to be completed.
 * @status: Transfer completion status for the transfer request.
 */
/**
 * s3c_hsudc_complete_request - Complete a transfer request.
 * 
 * MUSB 风格: Unlock -> Giveback -> Lock
 */
static void s3c_hsudc_complete_request(struct s3c_hsudc_ep *hsep,
				struct s3c_hsudc_req *hsreq, int status)
{
	struct s3c_hsudc *hsudc = hsep->dev;

	list_del_init(&hsreq->queue);
	hsreq->req.status = status;

	/* 
	 * CRITICAL: MUSB Logic
	 * 在调用上层回调之前释放锁，防止上层在回调中再次调用 queue/dequeue 导致死锁
	 */
	spin_unlock(&hsudc->lock);
	
	if (hsreq->req.complete)
		usb_gadget_giveback_request(&hsep->ep, &hsreq->req);
	
	spin_lock(&hsudc->lock);
}

static void s3c_hsudc_unmap_dma(struct s3c_hsudc_ep *hsep, struct s3c_hsudc_req *hsreq)
{
    struct s3c_hsudc *hsudc = hsep->dev;

    /* 只有当我们自己 map 过的时候才 unmap */
    if (hsreq->mapped) {
        dma_unmap_single(hsudc->dev, hsreq->req.dma, hsreq->req.length, DMA_TO_DEVICE);
        hsreq->req.dma = DMA_ADDR_INVALID;
        hsreq->mapped = false;
    }
}


/**
 * s3c_hsudc_nuke_ep - Terminate all requests queued for a endpoint.
 * @hsep: Endpoint for which queued requests have to be terminated.
 * @status: Transfer completion status for the transfer request.
 */
static void s3c_hsudc_nuke_ep(struct s3c_hsudc_ep *hsep, int status)
{
	struct s3c_hsudc_req *hsreq;
	struct s3c_hsudc *hsudc = hsep->dev;

	/* 1. 如果 DMA 还在运行，立即停止 */
	if (hsep->dma_running) {
		writel(0, hsudc->regs + S3C_DCR);
		hsep->dma_running = false;
	}

	while (!list_empty(&hsep->queue)) {
		hsreq = list_entry(hsep->queue.next,
				struct s3c_hsudc_req, queue);
				
		/* 2. 在 complete 之前，必须解除 DMA 映射，防止内存泄漏 */
		s3c_hsudc_unmap_dma(hsep, hsreq);
		
		s3c_hsudc_complete_request(hsep, hsreq, status);
	}
}



/**
 * s3c_hsudc_stop_activity - Stop activity on all endpoints.
 * @hsudc: Device controller for which EP activity is to be stopped.
 *
 * All the endpoints are stopped and any pending transfer requests if any on
 * the endpoint are terminated.
 */
static void s3c_hsudc_stop_activity(struct s3c_hsudc *hsudc)
{
	struct s3c_hsudc_ep *hsep;
	int epnum;

	hsudc->gadget.speed = USB_SPEED_UNKNOWN;

	for (epnum = 0; epnum < hsudc->epnum; epnum++) {
		hsep = &hsudc->ep[epnum];
		hsep->stopped = 1;
		s3c_hsudc_nuke_ep(hsep, -ESHUTDOWN);
	}
}

static int s3c_hsudc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct s3c_hsudc *hsudc = to_hsudc(gadget);
	unsigned long flags;

	spin_lock_irqsave(&hsudc->lock, flags);
	if (!is_on) {
		/* 在驱动 unbind 前安全地终止所有活动并归还合法请求 */
		s3c_hsudc_stop_activity(hsudc);
	}
	spin_unlock_irqrestore(&hsudc->lock, flags);

	return 0;
}

/**
 * s3c_hsudc_read_setup_pkt - Read the received setup packet from EP0 fifo.
 * @hsudc: Device controller from which setup packet is to be read.
 * @buf: The buffer into which the setup packet is read.
 *
 * The setup packet received in the EP0 fifo is read and stored into a
 * given buffer address.
 */

static void s3c_hsudc_read_setup_pkt(struct s3c_hsudc *hsudc, u16 *buf)
{
	int count;

	count = readl(hsudc->regs + S3C_BRCR) &0xFFFF;
	// printk(KERN_ERR "S3C_UDC: Setup BRCR Count = %d\n", count);
	while (count--)
		*buf++ = (u16)readl(hsudc->regs + S3C_BR(0));

	writel(S3C_EP0SR_RX_SUCCESS, hsudc->regs + S3C_EP0SR);
}

/**
 * s3c_hsudc_write_fifo - Write next chunk of transfer data to EP fifo.
 * @hsep: Endpoint to which the data is to be written.
 * @hsreq: Transfer request from which the next chunk of data is written.
 *
 * Write the next chunk of data from a transfer request to the endpoint FIFO.
 * If the transfer request completes, 1 is returned, otherwise 0 is returned.
 */
static int s3c_hsudc_write_fifo(struct s3c_hsudc_ep *hsep,
				struct s3c_hsudc_req *hsreq)
{
	u16 *buf;
	u32 max = ep_maxpacket(hsep);
	u32 count, length;
	bool is_last;
	void __iomem *fifo = hsep->fifo;

	buf = hsreq->req.buf + hsreq->req.actual;
	prefetch(buf);

	length = hsreq->req.length - hsreq->req.actual;
	length = min(length, max);
	hsreq->req.actual += length;
	//printk(KERN_INFO "s3c-udc: write_fifo ep%d len=%d total=%d\n",         ep_index(hsep), length, hsreq->req.length);
	writel(length, hsep->dev->regs + S3C_BWCR);
	for (count = 0; count < length; count += 2)
		writel(*buf++, fifo);

	if (count != max) {
		is_last = true;
	} else {
		if (hsreq->req.length != hsreq->req.actual || hsreq->req.zero)
			is_last = false;
		else
			is_last = true;
	}

	if (is_last) {
		s3c_hsudc_complete_request(hsep, hsreq, 0);
		return 1;
	}

	return 0;
}

/**
 * s3c_hsudc_read_fifo - Read the next chunk of data from EP fifo.
 * @hsep: Endpoint from which the data is to be read.
 * @hsreq: Transfer request to which the next chunk of data read is written.
 *
 * Read the next chunk of data from the endpoint FIFO and a write it to the
 * transfer request buffer. If the transfer request completes, 1 is returned,
 * otherwise 0 is returned.
 */
static int s3c_hsudc_read_fifo(struct s3c_hsudc_ep *hsep,
				struct s3c_hsudc_req *hsreq)
{
	struct s3c_hsudc *hsudc = hsep->dev;
	u32 offset = (ep_index(hsep)) ? S3C_ESR : S3C_EP0SR;
	u32 csr;
	u32 rcnt;     /* 寄存器里的 Word 计数 */
	u32 bytes;    /* 实际字节数 */
	u16 *buf;
	int i;
	void __iomem *fifo = hsep->fifo;

	csr = readl(hsudc->regs + offset);
	if (!(csr & S3C_ESR_RX_SUCCESS))
		return -EINVAL;

	/* 获取 FIFO 中的数据量 */
	rcnt = readl(hsudc->regs + S3C_BRCR) & 0xFFFF; // Mask important!
	
	/* 
	 * 计算实际字节数：
	 * S3C2416 手册：BRCR 是 16位(Word) 计数。
	 * 如果 LWO (Last Word Odd) 置位，说明最后一个 Word 只有低字节有效 (-1 byte)。
	 */
	bytes = rcnt * 2;
	if (csr & S3C_ESR_LWO)
		bytes -= 1;

	/* 调试日志：打开这个可以看数据流，但太多会卡死，仅调试用 */
	//printk(KERN_DEBUG "EP%d RX: rcnt=%d LWO=%d Bytes=%d\n", ep_index(hsep), rcnt, (csr & S3C_ESR_LWO)?1:0, bytes);

	/* 检查 buffer 是否够大 */
	if (hsreq->req.actual + bytes > hsreq->req.length) {
		/* 即使溢出也要读空 FIFO，否则堵塞 */
		while (rcnt--) readl(fifo);
		return -EOVERFLOW;
	}

	buf = (u16 *)(hsreq->req.buf + hsreq->req.actual);
	
	/* 读数据 (以 Word 为单位) */
	for (i = 0; i < rcnt; i++) {
		*buf++ = (u16)readl(fifo);
	}

	hsreq->req.actual += bytes;

	
	/* Data endpoint RX_SUCCESS clears when the FIFO has been drained. */
	/* 判断请求是否完成：
	 * 1. 收到短包 (Short Packet)：长度 < MaxPacket
	 * 2. 缓冲区填满了
	 */
	if ((bytes < hsep->ep.maxpacket) || (hsreq->req.actual == hsreq->req.length))
		return 1; /* 完成 */

	return 0; /* 还没完成，等待更多数据 */
}

static void s3c_hsudc_start_dma_tx(struct s3c_hsudc_ep *hsep, struct s3c_hsudc_req *hsreq)
{
    struct s3c_hsudc *hsudc = hsep->dev;
    u32 len = hsreq->req.length;
    dma_addr_t dma_addr;
    int ret;

    /* 1. 处理 DMA 映射 (解决 Cache 一致性和物理地址问题) */
    if (hsreq->req.dma == DMA_ADDR_INVALID) {
        dma_addr = dma_map_single(hsudc->dev, hsreq->req.buf, len, DMA_TO_DEVICE);
        if (dma_mapping_error(hsudc->dev, dma_addr)) {
            dev_err(hsudc->dev, "dma mapping failed\n");
            /* 映射失败回退到 PIO 可以在上层处理，这里简单处理直接返回 */
            return;
        }
        hsreq->req.dma = dma_addr;
        hsreq->mapped = true;
    } else {
        dma_addr = hsreq->req.dma;
        hsreq->mapped = false; /* 上层已经 map 过了，我们不用管 */
        /* 如果上层 map 了，通常也同步了 cache */
        dma_sync_single_for_device(hsudc->dev, dma_addr, len, DMA_TO_DEVICE);
    }

    /* 2. 配置寄存器 - 按照 Request 级别配置 */
    
    /* 停止 DMA (清除 DEN) */
    writel(readl(hsudc->regs + S3C_DCR) & ~S3C_DCR_DEN, hsudc->regs + S3C_DCR);

    /* 设置物理基地址 */
    writel(dma_addr, hsudc->regs + S3C_MBAR);

    /* 设置总传输长度 (Request 长度) */
    /* 硬件会自动切分成 MaxPacket 大小发送 */
    writel(len&0xffff, hsudc->regs + S3C_DTTCR1);
    writel(len>>16, hsudc->regs + S3C_DTTCR2); 

    /* 设置单包大小 (通常是 512) */
    writel(hsep->ep.maxpacket, hsudc->regs + S3C_DTCR);

    /* 设置 DMA FIFO 阈值 (参考裸机代码) */
    writel(hsep->ep.maxpacket, hsudc->regs + S3C_DFCR);

    /* 设置 Burst 模式 */
    writel(S3C_DICR_BURST_16, hsudc->regs + S3C_DICR);

    /* 3. 启动 DMA */
    /* DEN: Enable, FMDE: Burst Mode, TDR: TX Direction */
    writel(S3C_DCR_DEN | S3C_DCR_FMDE | S3C_DCR_TDR, hsudc->regs + S3C_DCR);
	//dev_info(hsudc->dev, "Starting a dma len %d\n",len);
    hsep->dma_running = true;
}
static void s3c_hsudc_process_tx_queue(struct s3c_hsudc_ep *hsep)
{
    struct s3c_hsudc *hsudc = hsep->dev;
    struct s3c_hsudc_req *hsreq;
    
    set_index(hsudc, ep_index(hsep));

    /* 如果 DMA 已经在跑了，绝对不能动，直接返回 */
    if (hsep->dma_running)
        return;

    while (!list_empty(&hsep->queue)) {
        hsreq = list_entry(hsep->queue.next, struct s3c_hsudc_req, queue);

        /* --- 决策逻辑 --- */
        /* 条件：非 EP0，且长度 >= 64 (或其他阈值)，且没有已经发了一半的数据 */
        if (ep_index(hsep) != 0 && hsreq->req.length >= 1<<16 && hsreq->req.actual == 0) {
            //相当于不触发了，太傻逼了，dma性能比不过PIO,FUCK YOU
            /* 启动 DMA，一次性发完整个 Request */
            s3c_hsudc_start_dma_tx(hsep, hsreq);
            if(hsep->dma_running) break;

        } 
		uint32_t csr = readl(hsudc->regs + S3C_ESR);
		
		/* 如果 FIFO 已经有两个包(PSIF=10b/2)，则停止写入 */
		if ((csr & S3C_ESR_PSIF_TWO) == S3C_ESR_PSIF_TWO)
			break;
		if ((csr & S3C_ESR_DOM) != S3C_ESR_DOM && (csr & S3C_ESR_PSIF_ONE) == S3C_ESR_PSIF_ONE)
			break;

		/* 3. 写入一个包的数据 */
		/* s3c_hsudc_write_fifo 返回 1 表示该 Request 全部发完 */
		if (s3c_hsudc_write_fifo(hsep, hsreq) == 1) {
			/* Request 完成，write_fifo 内部已经做了 complete 回调 */
			/* 循环继续，处理链表中的下一个 Request */
			continue;
		} else {
			/* Request 还没发完（数据量 > MaxPacket），但当前包已写入 */
			/* 循环继续，检查是否还能再塞一个包进 FIFO (双缓冲) */
			continue;
		}
	}
}
/**
 * s3c_hsudc_epin_intr - Handle in-endpoint interrupt.
 * @hsudc - Device controller for which the interrupt is to be handled.
 * @ep_idx - Endpoint number on which an interrupt is pending.
 *
 * Handles interrupt for a in-endpoint. The interrupts that are handled are
 * stall and data transmit complete interrupt.
 */
static void s3c_hsudc_epin_intr(struct s3c_hsudc *hsudc, u32 ep_idx)
{
	struct s3c_hsudc_ep *hsep = &hsudc->ep[ep_idx];
	u32 csr;

	csr = readl(hsudc->regs + S3C_ESR);
	if (csr & S3C_ESR_FLUSH) {
			writel(S3C_ESR_FLUSH, hsudc->regs + S3C_ESR);
            // Flush 后 FIFO 空了，继续循环可能会读到空状态，从而自然退出
	}
	/* 处理 Stall */
	if (csr & S3C_ESR_STALL) {
		writel(S3C_ESR_STALL, hsudc->regs + S3C_ESR);
		return;
	}


	if (csr & S3C_ESR_DTCZ) {
        writel(S3C_ESR_DTCZ, hsudc->regs + S3C_ESR); /* 清除中断 */
		writel(S3C_ESR_TX_SUCCESS, hsudc->regs + S3C_ESR);//DTCZ同时会产生中断
        if (hsep->dma_running) {

            //writel(0, hsudc->regs + S3C_DCR);
            hsep->dma_running = false;

            /* 2. 获取当前 Request 并完成它 */
            if (!list_empty(&hsep->queue)) {
                struct s3c_hsudc_req *hsreq = list_entry(hsep->queue.next, struct s3c_hsudc_req, queue);
                
                /* 处理解映射 */
                s3c_hsudc_unmap_dma(hsep, hsreq);

                /* 更新长度：DMA 模式下 DTTCR 归零意味着全部发完了 */
                hsreq->req.actual = hsreq->req.length;
                
                /* 回调上层 */
                s3c_hsudc_complete_request(hsep, hsreq, 0);
            }
            
            /* 3. 继续处理队列中下一个请求 */
            s3c_hsudc_process_tx_queue(hsep);
            return;
        }
    }
	if (csr & S3C_ESR_TX_SUCCESS) {
		writel(S3C_ESR_TX_SUCCESS, hsudc->regs + S3C_ESR);
		if (hsep->dma_running)
            return;
	}

	if (!list_empty(&hsep->queue) && !hsep->dma_running) {
        s3c_hsudc_process_tx_queue(hsep);
    }
}
/**
 * s3c_hsudc_epout_intr - Handle out-endpoint interrupt.
 * @hsudc - Device controller for which the interrupt is to be handled.
 * @ep_idx - Endpoint number on which an interrupt is pending.
 *
 * Handles interrupt for a out-endpoint. The interrupts that are handled are
 * stall, flush and data ready interrupt.
 */
static void s3c_hsudc_epout_intr(struct s3c_hsudc *hsudc, u32 ep_idx)
{
	struct s3c_hsudc_ep *hsep = &hsudc->ep[ep_idx];
	struct s3c_hsudc_req *hsreq;
	u32 csr;
	int ret;
	int loop_count = 0; /* 防止死循环 */

	csr = readl(hsudc->regs + S3C_ESR);
	//dev_info(hsudc->dev, "IRQ: ep%dout int,esr=0x%08x\n",ep_idx, csr);

	/* The endpoint FIFO is double buffered, so handle at most two packets. */
	while (loop_count++ < 2) {
		csr = readl(hsudc->regs + S3C_ESR);
		//dev_info(hsudc->dev, "IRQ: ep%dout int,esr=0x%08x\n",ep_idx, csr);
		if (csr & S3C_ESR_FLUSH) {
			writel(S3C_ESR_FLUSH, hsudc->regs + S3C_ESR);
            // Flush 后 FIFO 空了，继续循环可能会读到空状态，从而自然退出
		}
		/* 1. 错误处理 */
		if (csr & S3C_ESR_STALL) {
			writel(S3C_ESR_STALL, hsudc->regs + S3C_ESR);
			break;
		}

		/* 2. 检查是否有数据 */
		if (!(csr & S3C_ESR_RX_SUCCESS)) {
			break; /* FIFO 空了，退出 */
		}

		/* 3. 检查是否有 Request */
		if (list_empty(&hsep->queue)) {
			/*
			 * Leave the packet in the FIFO, but mask this level interrupt
			 * until a consumer queues a receive request.  Otherwise the
			 * unchanged RX_SUCCESS condition immediately retriggers forever.
			 */
			writel(readl(hsudc->regs + S3C_EIER) & ~BIT(ep_idx),
			       hsudc->regs + S3C_EIER);
			dev_warn_ratelimited(hsudc->dev,
				"EP%u OUT data with no request; interrupt masked\n",
				ep_idx);
			break; 
		}

		/* 4. 开始读取 */
		hsreq = list_entry(hsep->queue.next, struct s3c_hsudc_req, queue);
		
		ret = s3c_hsudc_read_fifo(hsep, hsreq);
		
		/* 处理结果 */
		if (ret == 1) {
			/* 这个 Request 填满了或者读到了短包 */
			s3c_hsudc_complete_request(hsep, hsreq, 0);
		} else if (ret < 0) {
			/* 出错 */
			s3c_hsudc_complete_request(hsep, hsreq, ret);
		}
		

		
		/* 循环继续，再次检查 ESR ... */
	}
}


/** s3c_hsudc_set_halt - Set or clear a endpoint halt.
 * @_ep: Endpoint on which halt has to be set or cleared.
 * @value: 1 for setting halt on endpoint, 0 to clear halt.
 *
 * Set or clear endpoint halt. If halt is set, the endpoint is stopped.
 * If halt is cleared, for in-endpoints, if there are any pending
 * transfer requests, transfers are started.
 */
/* Caller must hold hsudc->lock. */
static int __s3c_hsudc_set_halt(struct s3c_hsudc_ep *hsep, int value)
{
	struct s3c_hsudc *hsudc = hsep->dev;
	struct s3c_hsudc_req *hsreq;
	u32 ecr;
	u32 offset;

	if (value && ep_is_in(hsep) && !list_empty(&hsep->queue))
		return -EAGAIN;

	set_index(hsudc, ep_index(hsep));
	offset = (ep_index(hsep)) ? S3C_ECR : S3C_EP0CR;
	ecr = readl(hsudc->regs + offset);

	if (value) {
		ecr |= S3C_ECR_STALL;
		if (ep_index(hsep))
			ecr |= S3C_ECR_FLUSH;
		hsep->stopped = 1;
	} else {
		ecr &= ~S3C_ECR_STALL;
		hsep->stopped = hsep->wedge = 0;
	}
	writel(ecr, hsudc->regs + offset);

	if (ep_is_in(hsep) && !list_empty(&hsep->queue) && !value) {
		hsreq = list_entry(hsep->queue.next,
			struct s3c_hsudc_req, queue);
		if (hsreq)
			s3c_hsudc_write_fifo(hsep, hsreq);
	}

	return 0;
}

static int s3c_hsudc_set_halt(struct usb_ep *_ep, int value)
{
	struct s3c_hsudc_ep *hsep = our_ep(_ep);
	struct s3c_hsudc *hsudc = hsep->dev;
	unsigned long irqflags;
	int ret;

	spin_lock_irqsave(&hsudc->lock, irqflags);
	ret = __s3c_hsudc_set_halt(hsep, value);
	spin_unlock_irqrestore(&hsudc->lock, irqflags);

	return ret;
}

/** s3c_hsudc_set_wedge - Sets the halt feature with the clear requests ignored
 * @_ep: Endpoint on which wedge has to be set.
 *
 * Sets the halt feature with the clear requests ignored.
 */
static int s3c_hsudc_set_wedge(struct usb_ep *_ep)
{
	struct s3c_hsudc_ep *hsep = our_ep(_ep);

	if (!hsep)
		return -EINVAL;

	hsep->wedge = 1;
	return usb_ep_set_halt(_ep);
}

/** s3c_hsudc_handle_reqfeat - Handle set feature or clear feature requests.
 * @_ep: Device controller on which the set/clear feature needs to be handled.
 * @ctrl: Control request as received on the endpoint 0.
 *
 * Handle set feature or clear feature control requests on the control endpoint.
 */
static int s3c_hsudc_handle_reqfeat(struct s3c_hsudc *hsudc,
					struct usb_ctrlrequest *ctrl)
{
	struct s3c_hsudc_ep *hsep;
	bool set = (ctrl->bRequest == USB_REQ_SET_FEATURE);
	u8 ep_num = ctrl->wIndex & USB_ENDPOINT_NUMBER_MASK;

	if (ctrl->bRequestType == USB_RECIP_ENDPOINT) {
		hsep = &hsudc->ep[ep_num];
		switch (le16_to_cpu(ctrl->wValue)) {
		case USB_ENDPOINT_HALT:
			if (set || !hsep->wedge)
				__s3c_hsudc_set_halt(hsep, set);
			return 0;
		}
	}

	return -ENOENT;
}

/**
 * s3c_hsudc_process_req_status - Handle get status control request.
 * @hsudc: Device controller on which get status request has be handled.
 * @ctrl: Control request as received on the endpoint 0.
 *
 * Handle get status control request received on control endpoint.
 */


/* 辅助函数：发送 ZLP (Zero Length Packet) */
static void s3c_hsudc_ep0_send_zlp(struct s3c_hsudc *hsudc)
{
	/* 确保操作的是 EP0 */
	set_index(hsudc, 0);
	
	/* 
	 * 修正：直接往 BWCR 写 0。
	 * 这告诉硬件：“这一包数据长度为0，请发送”。
	 * 硬件发送完后会触发 TX_SUCCESS 中断。
	 */
	writel(0, hsudc->regs + S3C_BWCR);
}



/** s3c_hsudc_handle_ep0_intr - Handle endpoint 0 interrupt.
 * @hsudc: Device controller on which endpoint 0 interrupt has occurred.
 *
 * Handle endpoint 0 interrupt when it occurs. EP0 interrupt could occur
 * when a stall handshake is sent to host or data is sent/received on
 * endpoint 0.
 */
/* ============================================================
 *  2. 标准请求处理逻辑
 * ============================================================ */

static int service_zero_data_request(struct s3c_hsudc *hsudc,
				     struct usb_ctrlrequest *ctrl)
{
	int handled = -EINVAL;

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (ctrl->bRequest) {
		case USB_REQ_SET_ADDRESS:
			/* S3C2416 硬件自动处理地址设置，我们只需记录 */
			hsudc->dev_addr = (u8)(ctrl->wValue & 0x7f);
			ep0_dbg("SET_ADDR: %d (HW Auto)", hsudc->dev_addr);
			handled = 1; 
			break;

		case USB_REQ_SET_CONFIGURATION:
		case USB_REQ_SET_INTERFACE:
			/* 交给 Gadget Driver */
			handled = 0;
			break;
		}
	} else {
		handled = 0; /* Class/Vendor 请求 */
	}

	return handled;
}

/* --------------------------------------------------------------------------
 * 核心：EP0 中断处理 (仿写 musb_g_ep0_irq)
 * -------------------------------------------------------------------------- */
/* ============================================================
 *  3. EP0 中断处理 (核心状态机)
 * ============================================================ */
static void s3c_hsudc_handle_ep0_intr(struct s3c_hsudc *hsudc)
{
	struct s3c_hsudc_ep *hsep = &hsudc->ep[0];
	struct s3c_hsudc_req *req;
	u32 csr = readl(hsudc->regs + S3C_EP0SR);
	u32 brcr = readl(hsudc->regs + S3C_BRCR) & 0xFFFF;
	u32 ecr;
	int handled = 0;
	int ret = 0;

	ep0_dbg("IRQ: CSR=0x%x, Count=%d, State=%s", 
		csr, brcr, decode_ep0stage(hsudc->ep0state));

	/* --- CASE 1: STALL SENT --- */
	if (csr & S3C_EP0SR_STALL) {
		writel(S3C_EP0SR_STALL, hsudc->regs + S3C_EP0SR); /* W1C */
		
		/* 清除 Stall 位，准备下一次传输 */
		ecr = readl(hsudc->regs + S3C_EP0CR);
		writel(ecr & ~S3C_ECR_STALL, hsudc->regs + S3C_EP0CR);

		hsudc->ep0state = EP0_IDLE;
		return;
	}

	/* --- CASE 2: SETUP PACKET RECEIVED --- */
	/* 判断依据：RX_SUCCESS 且 (状态为IDLE 或 长度为8字节) */
	if (csr & S3C_EP0SR_RX_SUCCESS) {
		bool is_setup = false;

		if (brcr == 4) {
			/* 
			 * If we receive exactly 8 bytes during IDLE, it's a SETUP packet.
			 * If we receive exactly 8 bytes during TX or STATUS phases, it is 
			 * almost certainly the Host forcing a Setup Phase Abort.
			 */
			if (hsudc->ep0state == EP0_IDLE ||
			    hsudc->ep0state != EP0_STAGE_RX)
				is_setup = true;
			/* (If it's exactly 8 bytes during EP0_STAGE_RX, we let it fall 
			 * through to CASE 3 because it could be a valid 8-byte OUT data chunk). */
		} else if (hsudc->ep0state == EP0_IDLE) {
			/* 
			 * If we are IDLE and receive non-8-byte data, this is OHCI PHY noise/garbage.
			 * We MUST drain the FIFO and discard it to avoid confusing the driver.
			 */
			ep0_dbg("EP0: Garbage packet (brcr=%d) during IDLE! Dropping.",
				brcr);
			while (brcr--)
				(void)readl(hsudc->regs + S3C_BR(0));
			writel(S3C_EP0SR_RX_SUCCESS, hsudc->regs + S3C_EP0SR);

			/* Safely clear TX_SUCCESS just in case it fired together */
			if (csr & S3C_EP0SR_TX_SUCCESS)
				writel(S3C_EP0SR_TX_SUCCESS,
				       hsudc->regs + S3C_EP0SR);
			return;
		}

		if (is_setup) {
			struct usb_ctrlrequest ctrl;

			/* 硬件可能还未清除标志，这里手动处理 Setup 逻辑 */
			if (!list_empty(&hsep->queue))
				s3c_hsudc_nuke_ep(hsep, -ECONNRESET);

			s3c_hsudc_read_setup_pkt(hsudc, (u16 *)&ctrl);

			/* 解析 SETUP */
			if (ctrl.wLength == 0) {
				/* 无数据阶段 (No-Data Phase) */
				hsudc->ep0state = EP0_STAGE_ACKWAIT;

				handled =
					service_zero_data_request(hsudc, &ctrl);

				if (handled > 0) {
					/* 驱动内部已处理 (如 SetAddr)，直接进 Status IN */
					hsudc->ep0state = EP0_STAGE_STATUSIN;
					s3c_hsudc_ep0_send_zlp(hsudc);
				} else if (handled < 0) {
					s3c_hsudc_ep0_stall(hsudc);
				} else {
					/* 交给 Gadget Driver */
					spin_unlock(&hsudc->lock);
					ret = hsudc->driver->setup(
						&hsudc->gadget, &ctrl);
					spin_lock(&hsudc->lock);

					if (ret < 0) {
						s3c_hsudc_ep0_stall(hsudc);
					} else if (hsudc->ep0state ==
						   EP0_STAGE_ACKWAIT) {
						/* The gadget driver did not queue its own ZLP. */
						/* 成功，发送 ZLP 完成握手 */
						hsudc->ep0state =
							EP0_STAGE_STATUSIN;
						s3c_hsudc_ep0_send_zlp(hsudc);
					}
				}
			} else {
				/* 有数据阶段 (Data Phase) */
				if (ctrl.bRequestType & USB_DIR_IN) {
					hsudc->ep0state = EP0_STAGE_TX;
					spin_unlock(&hsudc->lock);
					ret = hsudc->driver->setup(
						&hsudc->gadget, &ctrl);
					spin_lock(&hsudc->lock);
					if (ret < 0)
						s3c_hsudc_ep0_stall(hsudc);
				} else {
					hsudc->ep0state = EP0_STAGE_RX;
					spin_unlock(&hsudc->lock);
					ret = hsudc->driver->setup(
						&hsudc->gadget, &ctrl);
					spin_lock(&hsudc->lock);
					if (ret < 0)
						s3c_hsudc_ep0_stall(hsudc);
				}
			}
			return; /* Setup 处理完毕 */
		}
	}

	/* --- CASE 3: STATE MACHINE (DATA/STATUS) --- */
	switch (hsudc->ep0state) {
	
	case EP0_STAGE_TX:
		/* IN 数据发送完成 */
		if (csr & S3C_EP0SR_TX_SUCCESS) {
			writel(S3C_EP0SR_TX_SUCCESS, hsudc->regs + S3C_EP0SR);
			
			if (!list_empty(&hsep->queue)) {
				req = list_entry(hsep->queue.next, struct s3c_hsudc_req, queue);
				if (s3c_hsudc_write_fifo(hsep, req) == 1) {
					/* 请求完成，等待 Status OUT */
					hsudc->ep0state = EP0_STAGE_STATUSOUT;
				}
			} else {
				hsudc->ep0state = EP0_STAGE_STATUSOUT;
			}
		}
		break;

	case EP0_STAGE_RX:
		/* OUT 数据接收完成 */
		if (csr & S3C_EP0SR_RX_SUCCESS) {
			if (!list_empty(&hsep->queue)) {
				req = list_entry(hsep->queue.next, struct s3c_hsudc_req, queue);
				ret = s3c_hsudc_read_fifo(hsep, req);
				if (ret == 1) {
					s3c_hsudc_complete_request(hsep, req, 0);
					/* 接收完成，发送 Status IN ZLP */
					hsudc->ep0state = EP0_STAGE_STATUSIN;
					s3c_hsudc_ep0_send_zlp(hsudc);
				} else if (ret < 0) {
					s3c_hsudc_complete_request(hsep, req, ret);
					s3c_hsudc_ep0_stall(hsudc);
				}
			} else {
				/* 没有请求 buffer，丢弃数据 */
				writel(S3C_EP0SR_RX_SUCCESS, hsudc->regs + S3C_EP0SR);
			}
		}
		break;

	case EP0_STAGE_STATUSIN:
		/* Status ZLP 发送完毕 */
		if (csr & S3C_EP0SR_TX_SUCCESS) {
			writel(S3C_EP0SR_TX_SUCCESS, hsudc->regs + S3C_EP0SR);
			hsudc->ep0state = EP0_IDLE;
		}
		break;

	case EP0_STAGE_STATUSOUT:
		/* 
		 * [FIX FOR ECM/NCM] 
		 * If we just finished sending the last data packet (split packet),
		 * the hardware triggers TX_SUCCESS *after* we have already moved
		 * to STATUS_OUT state. We MUST clear it here.
		 */
		if (csr & S3C_EP0SR_TX_SUCCESS) {
			writel(S3C_EP0SR_TX_SUCCESS, hsudc->regs + S3C_EP0SR);
		}

		/* Normal Status Phase Completion */
		if (csr & S3C_EP0SR_RX_SUCCESS) {
			writel(S3C_EP0SR_RX_SUCCESS, hsudc->regs + S3C_EP0SR);
			hsudc->ep0state = EP0_IDLE;
		}
		break;
		
	default:
		/* 异常清理 */
		if (csr & S3C_EP0SR_RX_SUCCESS) 
			writel(S3C_EP0SR_RX_SUCCESS, hsudc->regs + S3C_EP0SR);
		if (csr & S3C_EP0SR_TX_SUCCESS) 
			writel(S3C_EP0SR_TX_SUCCESS, hsudc->regs + S3C_EP0SR);
		break;
	}
}

/**
 * s3c_hsudc_ep_enable - Enable a endpoint.
 * @_ep: The endpoint to be enabled.
 * @desc: Endpoint descriptor.
 *
 * Enables a endpoint when called from the gadget driver. Endpoint stall if
 * any is cleared, transfer type is configured and endpoint interrupt is
 * enabled.
 */
static int s3c_hsudc_ep_enable(struct usb_ep *_ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct s3c_hsudc_ep *hsep;
	struct s3c_hsudc *hsudc;
	unsigned long flags;
	u32 ecr = 0;

	hsep = our_ep(_ep);
	if (!_ep || !desc || _ep->name == ep0name
		|| desc->bDescriptorType != USB_DT_ENDPOINT
		|| hsep->bEndpointAddress != desc->bEndpointAddress
		|| ep_maxpacket(hsep) < usb_endpoint_maxp(desc))
		return -EINVAL;

	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
		&& usb_endpoint_maxp(desc) != ep_maxpacket(hsep))
		|| !desc->wMaxPacketSize)
		return -ERANGE;

	hsudc = hsep->dev;
	if (!hsudc->driver || hsudc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	spin_lock_irqsave(&hsudc->lock, flags);

	set_index(hsudc, hsep->bEndpointAddress);
	ecr |= ((usb_endpoint_xfer_int(desc)) ? S3C_ECR_IEMS : S3C_ECR_DUEN);
	writel(ecr, hsudc->regs + S3C_ECR);
	hsep->stopped = hsep->wedge = 0;
	hsep->ep.desc = desc;
	hsep->ep.maxpacket = usb_endpoint_maxp(desc);

	writel(hsep->ep.maxpacket, hsudc->regs + S3C_MPR);
	dev_info(hsudc->dev, "EP%d: MPS=%d\n", 
                 ep_index(hsep),readl(hsudc->regs + S3C_MPR));
	if (ep_index(hsep) != 0 && !(readl(hsudc->regs + S3C_ESR) & S3C_ESR_DOM))
        dev_warn(hsudc->dev, "EP%d: MPS=%d not half of FIFO size, DOM=0\n", 
                 ep_index(hsep), hsep->ep.maxpacket);
	/* hsudc->lock is already held here; do not recurse into .set_halt. */
	__s3c_hsudc_set_halt(hsep, 0);
	__set_bit(ep_index(hsep), hsudc->regs + S3C_EIER);

	spin_unlock_irqrestore(&hsudc->lock, flags);
	return 0;
}

/**
 * s3c_hsudc_ep_disable - Disable a endpoint.
 * @_ep: The endpoint to be disabled.
 * @desc: Endpoint descriptor.
 *
 * Disables a endpoint when called from the gadget driver.
 */
static int s3c_hsudc_ep_disable(struct usb_ep *_ep)
{
	struct s3c_hsudc_ep *hsep = our_ep(_ep);
	struct s3c_hsudc *hsudc = hsep->dev;
	unsigned long flags;

	if (!_ep || !hsep->ep.desc)
		return -EINVAL;

	spin_lock_irqsave(&hsudc->lock, flags);

	set_index(hsudc, hsep->bEndpointAddress);
	__clear_bit(ep_index(hsep), hsudc->regs + S3C_EIER);

	s3c_hsudc_nuke_ep(hsep, -ESHUTDOWN);

	hsep->ep.desc = NULL;
	hsep->stopped = 1;

	spin_unlock_irqrestore(&hsudc->lock, flags);
	return 0;
}

/**
 * s3c_hsudc_alloc_request - Allocate a new request.
 * @_ep: Endpoint for which request is allocated (not used).
 * @gfp_flags: Flags used for the allocation.
 *
 * Allocates a single transfer request structure when called from gadget driver.
 */
static struct usb_request *s3c_hsudc_alloc_request(struct usb_ep *_ep,
						gfp_t gfp_flags)
{
	struct s3c_hsudc_req *hsreq;

	hsreq = kzalloc(sizeof(*hsreq), gfp_flags);
	if (!hsreq)
		return NULL;

	INIT_LIST_HEAD(&hsreq->queue);
	hsreq->req.dma = DMA_ADDR_INVALID; 
    hsreq->mapped = false;
	return &hsreq->req;
}

/**
 * s3c_hsudc_free_request - Deallocate a request.
 * @ep: Endpoint for which request is deallocated (not used).
 * @_req: Request to be deallocated.
 *
 * Allocates a single transfer request structure when called from gadget driver.
 */
static void s3c_hsudc_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct s3c_hsudc_req *hsreq;

	hsreq = our_req(_req);
	WARN_ON(!list_empty(&hsreq->queue));
	kfree(hsreq);
}

/**
 * s3c_hsudc_queue - Queue a transfer request for the endpoint.
 * @_ep: Endpoint for which the request is queued.
 * @_req: Request to be queued.
 * @gfp_flags: Not used.
 *
 * Start or enqueue a request for a endpoint when called from gadget driver.
 */
static int s3c_hsudc_queue(struct usb_ep *_ep, struct usb_request *_req,
			gfp_t gfp_flags)
{
	struct s3c_hsudc_req *hsreq;
	struct s3c_hsudc_ep *hsep;
	struct s3c_hsudc *hsudc;
	unsigned long flags;
	bool was_empty;

	hsreq = our_req(_req);
	if ((!_req || !_req->complete || !_req->buf ||
		!list_empty(&hsreq->queue)))
		return -EINVAL;

	hsep = our_ep(_ep);
	hsudc = hsep->dev;
	if (!hsudc->driver || hsudc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	spin_lock_irqsave(&hsudc->lock, flags);
	set_index(hsudc, hsep->bEndpointAddress);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* --- EP0 特殊处理 --- */
	/* EP0 维持原有逻辑，因为它依赖状态机状态而不是单纯的队列为空 */
	if (ep_index(hsep) == 0) {
		list_add_tail(&hsreq->queue, &hsep->queue);

		if (hsudc->ep0state == EP0_STAGE_TX) {
			s3c_hsudc_write_fifo(hsep, hsreq);
		}
		else if (hsudc->ep0state == EP0_STAGE_ACKWAIT && hsreq->req.length == 0) {
			hsudc->ep0state = EP0_STAGE_STATUSIN;
			s3c_hsudc_ep0_send_zlp(hsudc);
			s3c_hsudc_complete_request(hsep, hsreq, 0);
		}

		spin_unlock_irqrestore(&hsudc->lock, flags);
		return 0;
	}

	/* --- 非 EP0 端点处理 (Bulk/Intr) --- */

	/* 1. 在入队前检查队列是否为空 */
	was_empty = list_empty(&hsep->queue);

	/* 2. 将请求加入队列尾部 */
	list_add_tail(&hsreq->queue, &hsep->queue);

	/* Re-arm an OUT endpoint that was masked while it had no reader. */
	if (!ep_is_in(hsep) && !hsep->stopped)
		writel(readl(hsudc->regs + S3C_EIER) |
		       BIT(ep_index(hsep)), hsudc->regs + S3C_EIER);

	/*printk(KERN_DEBUG "EP%d %s Request queued: length=%d, buffer=%p, was_empty=%d\n",
	       ep_index(hsep), ep_is_in(hsep) ? "IN" : "OUT", 
	       _req->length, _req->buf, was_empty);*/

	/* 
	 * 3. 只有当队列原本为空时，才尝试立即启动传输。
	 *    如果队列不为空，说明硬件正在忙于处理前一个请求，
	 *    当前请求只需排队等待中断处理即可。
	 */
	if (was_empty && !hsep->stopped) {
		if (ep_is_in(hsep)) {
			/* IN (TX): 队列空了，立即尝试填 FIFO */
			s3c_hsudc_process_tx_queue(hsep);
		} else {
			/* OUT (RX): 队列空了，检查 FIFO 里是否已经有数据等着了 */
			u32 csr = readl(hsudc->regs + S3C_ESR);
			if (csr & S3C_ESR_RX_SUCCESS) {
				/* 有数据，立即读取到当前请求中 */
				int ret = s3c_hsudc_read_fifo(hsep, hsreq);
				
				if (ret == 1) {
					/* 请求已填满或短包结束 */
					s3c_hsudc_complete_request(hsep, hsreq, 0);
				} else if (ret < 0) {
					/* 读取出错 */
					s3c_hsudc_complete_request(hsep, hsreq, ret);
				}
				/* ret == 0 表示读了数据但还没满，保持排队等待后续数据 */
				//	TODO:如果fifo里面有两个也许可以连读
			}
		}
	}

	spin_unlock_irqrestore(&hsudc->lock, flags);
	return 0;
}

/**
 * s3c_hsudc_dequeue - Dequeue a transfer request from an endpoint.
 * @_ep: Endpoint from which the request is dequeued.
 * @_req: Request to be dequeued.
 *
 * Dequeue a request from a endpoint when called from gadget driver.
 */
static int s3c_hsudc_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct s3c_hsudc_ep *hsep = our_ep(_ep);
	struct s3c_hsudc *hsudc = hsep->dev;
	struct s3c_hsudc_req *hsreq = NULL, *iter;
	unsigned long flags;
	bool need_flush = false;

	hsep = our_ep(_ep);
	if (!_ep || hsep->ep.name == ep0name)
		return -EINVAL;

	spin_lock_irqsave(&hsudc->lock, flags);

	// 1. 查找请求
	list_for_each_entry(iter, &hsep->queue, queue) {
		if (&iter->req != _req)
			continue;
		hsreq = iter;
		break;
	}
	if (!hsreq) {
		spin_unlock_irqrestore(&hsudc->lock, flags);
		return -EINVAL;
	}

	// 2. ★核心判断★：如果这个请求是队列的第一个，说明硬件可能已经动了它
	if (hsep->queue.next == &hsreq->queue) {
		need_flush = true;
		if (hsep->dma_running) {
            writel(0, hsudc->regs + S3C_DCR);
            hsep->dma_running = false;
            s3c_hsudc_unmap_dma(hsep, hsreq);
        }
	}
	// 3. 选中端点 (complete_request 里不会选，所以这里要选，防止副作用)
	set_index(hsudc, hsep->bEndpointAddress);

	// 4. 移除请求并回调 (释放锁 -> 回调 -> 获取锁)
	s3c_hsudc_complete_request(hsep, hsreq, -ECONNRESET);

	// 5. ★执行硬件清理★
	// 如果刚刚取消的是正在跑的请求，FIFO 里肯定有残留数据(OUT)或废数据(IN)
	// 必须 Flush，否则下一个 Request 会读到垃圾
	if (need_flush) {
		s3c_hsudc_flush_fifo(hsep);
	}

	spin_unlock_irqrestore(&hsudc->lock, flags);
	return 0;
}

static const struct usb_ep_ops s3c_hsudc_ep_ops = {
	.enable = s3c_hsudc_ep_enable,
	.disable = s3c_hsudc_ep_disable,
	.alloc_request = s3c_hsudc_alloc_request,
	.free_request = s3c_hsudc_free_request,
	.queue = s3c_hsudc_queue,
	.dequeue = s3c_hsudc_dequeue,
	.set_halt = s3c_hsudc_set_halt,
	.set_wedge = s3c_hsudc_set_wedge,
};

/**
 * s3c_hsudc_initep - Initialize a endpoint to default state.
 * @hsudc - Reference to the device controller.
 * @hsep - Endpoint to be initialized.
 * @epnum - Address to be assigned to the endpoint.
 *
 * Initialize a endpoint with default configuration.
 */
static void s3c_hsudc_initep(struct s3c_hsudc *hsudc,
				struct s3c_hsudc_ep *hsep, int epnum)
{
	char *dir;

	if ((epnum % 2) == 0) {
		dir = "out";
	} else {
		dir = "in";
		hsep->bEndpointAddress = USB_DIR_IN;
	}

	hsep->bEndpointAddress |= epnum;
	if (epnum)
		snprintf(hsep->name, sizeof(hsep->name), "ep%d%s", epnum, dir);
	else
		snprintf(hsep->name, sizeof(hsep->name), "%s", ep0name);

	INIT_LIST_HEAD(&hsep->queue);
	INIT_LIST_HEAD(&hsep->ep.ep_list);
	if (epnum)
		list_add_tail(&hsep->ep.ep_list, &hsudc->gadget.ep_list);

	hsep->dev = hsudc;
	hsep->ep.name = hsep->name;
	usb_ep_set_maxpacket_limit(&hsep->ep, epnum ? (epnum>4 ? 1024:512) : 64);
	hsep->ep.ops = &s3c_hsudc_ep_ops;
	hsep->fifo = hsudc->regs + S3C_BR(epnum);
	hsep->ep.desc = NULL;
	hsep->stopped = 0;
	hsep->wedge = 0;

	if (epnum == 0) {
		hsep->ep.caps.type_control = true;
		hsep->ep.caps.dir_in = true;
		hsep->ep.caps.dir_out = true;
	} else {
		//hsep->ep.caps.type_iso = true;
		hsep->ep.caps.type_bulk = true;
		hsep->ep.caps.type_int = true;
	}

	if (epnum & 1)
		hsep->ep.caps.dir_in = true;
	else
		hsep->ep.caps.dir_out = true;//TODO:Hardware support dynamic direction

	set_index(hsudc, epnum);
	writel(hsep->ep.maxpacket, hsudc->regs + S3C_MPR);

}

/**
 * s3c_hsudc_setup_ep - Configure all endpoints to default state.
 * @hsudc: Reference to device controller.
 *
 * Configures all endpoints to default state.
 */
static void s3c_hsudc_setup_ep(struct s3c_hsudc *hsudc)
{
	int epnum;

	hsudc->ep0state = WAIT_FOR_SETUP;
	INIT_LIST_HEAD(&hsudc->gadget.ep_list);
	/* 
     * 关键修改：先初始化 EP5 - EP8 (大 FIFO)，再初始化 EP1 - EP4。
     * 这样 Gadget 匹配时会先拿到性能好的端点。
     */
    for (epnum = 0; epnum < hsudc->epnum; epnum++) {
        // EP0 单独处理，或者放在最后也行，这里保持原样处理 EP0
        if (epnum == 0)
             s3c_hsudc_initep(hsudc, &hsudc->ep[epnum], epnum);
    }
    
    // 先加 EP5-8
    for (epnum = 5; epnum < hsudc->epnum; epnum++)
        s3c_hsudc_initep(hsudc, &hsudc->ep[epnum], epnum);

    // 再加 EP1-4
    for (epnum = 1; epnum < 5; epnum++)
        s3c_hsudc_initep(hsudc, &hsudc->ep[epnum], epnum);
}

/**
 * s3c_hsudc_reconfig - Reconfigure the device controller to default state.
 * @hsudc: Reference to device controller.
 *
 * Reconfigures the device controller registers to a default state.
 */
static void s3c_hsudc_reconfig(struct s3c_hsudc *hsudc)
{
	writel(0xAA, hsudc->regs + S3C_EDR);
	writel(1, hsudc->regs + S3C_EIER);
	writel(0, hsudc->regs + S3C_TR);
	writel(S3C_SCR_DTZIEN_EN |S3C_SCR_DIEN_EN | S3C_SCR_RRD_EN | S3C_SCR_SUS_EN |
			S3C_SCR_RST_EN, hsudc->regs + S3C_SCR);
	writel(S3C_FCON_DMAEN, hsudc->regs + S3C_FCON);
	writel(0, hsudc->regs + S3C_EP0CR);
	hsudc->ep0state = EP0_IDLE;
	
	s3c_hsudc_setup_ep(hsudc);
}
/* 
 * 暴力抽干 FIFO
 * 参考 PXA27x 驱动：当硬件状态机卡死或 Flush 无效时，
 * 手动读空 FIFO 是唯一让硬件复位 RPS 标志的方法。
 */
static void s3c_hsudc_drain_fifo(struct s3c_hsudc_ep *hsep)
{
	struct s3c_hsudc *hsudc = hsep->dev;
	void __iomem *fifo = hsep->fifo;
	u32 esr, brcr;
	int loop_safety = 1000; // 防止死循环

	set_index(hsudc, ep_index(hsep));

	while (loop_safety--) {
		esr = readl(hsudc->regs + S3C_ESR);
		
		/* 如果 RPS (RX_SUCCESS) 没置位，说明 FIFO 确实空了 */
		if (!(esr & S3C_ESR_RX_SUCCESS))
			break;

		/* 读取字节数 */
		brcr = readl(hsudc->regs + S3C_BRCR) & 0xFFFF;
		
		/* 
		 * 如果硬件说有 RX_SUCCESS 但字节数是 0，这是一种异常状态。
		 * 这种情况下读 FIFO 也没用，只能尝试强制 Flush。
		 */
		if (brcr == 0) {
			u32 ecr = readl(hsudc->regs + S3C_ECR);
			writel(ecr | S3C_ECR_FLUSH, hsudc->regs + S3C_ECR);
			udelay(10); // 给点时间
			break;
		}

		/* 
		 * 核心动作：把数据读出来扔掉！
		 * 这会触发硬件的 "Auto Clear" 逻辑。
		 * 注意：BRCR 是 Word 计数还是 Byte 计数要看具体实现，
		 * 这里我们简化处理，只要 RPS 还在就一直读。
		 */
		int words_to_read = (brcr + 1) / 2; // 将字节转换为 Word (向上取整)
		while (words_to_read--) {
			(void)readl(fifo); // 读出来，丢弃
		}
	}
}
/**
 * s3c_hsudc_irq - Interrupt handler for device controller.
 * @irq: Not used.
 * @_dev: Reference to the device controller.
 *
 * Interrupt handler for the device controller. This handler handles controller
 * interrupts and endpoint interrupts.
 */
static irqreturn_t s3c_hsudc_irq(int irq, void *_dev)
{
	struct s3c_hsudc *hsudc = _dev;
	struct s3c_hsudc_ep *hsep;
	u32 ep_intr;
	u32 sys_status;
	u32 ep_idx;

	spin_lock(&hsudc->lock);
	sys_status = readl(hsudc->regs + S3C_SSR);
	ep_intr = readl(hsudc->regs + S3C_EIR) & 0x3FF;
	
	if (!ep_intr && !(sys_status & S3C_SSR_DTZIEN_EN)) {
		spin_unlock(&hsudc->lock);
		return IRQ_HANDLED;
	}

	if (sys_status) {
		if (sys_status & S3C_SSR_VBUSON)
			writel(S3C_SSR_VBUSON, hsudc->regs + S3C_SSR);

		if (sys_status & S3C_SSR_ERR) {
			dev_err(hsudc->dev, "IRQ: System Error! SSR=0x%04x\n", sys_status);
			writel(S3C_SSR_ERR, hsudc->regs + S3C_SSR);
			ep_intr = 0; 
		}
		
		if (sys_status & S3C_SSR_SDE) {
			writel(S3C_SSR_SDE, hsudc->regs + S3C_SSR);
			hsudc->gadget.speed = (sys_status & S3C_SSR_HSP) ?
				USB_SPEED_HIGH : USB_SPEED_FULL;
		}

		/* 
         * [关键修复 1] 处理 SUSPEND (拔线通常先触发这个)
         * 如果挂起了，PHY 时钟可能马上消失，不要再处理数据中断！
         */
		if (sys_status & S3C_SSR_SUSPEND) {
			writel(S3C_SSR_SUSPEND, hsudc->regs + S3C_SSR);
			if (hsudc->gadget.speed != USB_SPEED_UNKNOWN
				&& hsudc->driver && hsudc->driver->suspend)
				hsudc->driver->suspend(&hsudc->gadget);
            
            // 强制清除端点中断标志，防止后续访问 FIFO 导致死机
            ep_intr = 0; 
		}

		if (sys_status & S3C_SSR_RESUME) {
			writel(S3C_SSR_RESUME, hsudc->regs + S3C_SSR);
			if (hsudc->gadget.speed != USB_SPEED_UNKNOWN
				&& hsudc->driver && hsudc->driver->resume)
				hsudc->driver->resume(&hsudc->gadget);
		}

		if (sys_status & S3C_SSR_RESET) {
			pr_info("SSR_RESET: Notify Upper Layer Disconnect\n");
			writel(S3C_SSR_RESET, hsudc->regs + S3C_SSR);

			spin_unlock(&hsudc->lock);
			if (hsudc->driver && hsudc->driver->disconnect)
				hsudc->driver->disconnect(&hsudc->gadget);
			spin_lock(&hsudc->lock);

			for (ep_idx = 0; ep_idx < hsudc->epnum; ep_idx++) {
				hsep = &hsudc->ep[ep_idx];
				hsep->stopped = 1;
				s3c_hsudc_nuke_ep(hsep, -ECONNRESET);
			}

			s3c_hsudc_reconfig(hsudc);
			
			hsudc->dev_addr = 0; 
			hsudc->ep0state = WAIT_FOR_SETUP;
			hsudc->gadget.speed = USB_SPEED_UNKNOWN; 

            /* 
             * [关键修复 2] Reset 发生后，意味着连接断开。
             * 此时绝对不能处理后续的 ep_intr (数据传输)，
             * 因为 FIFO 可能已经不可访问。直接返回！
             */
            spin_unlock(&hsudc->lock);
            return IRQ_HANDLED;
		}
	}

    /* 
     * 如果发生了 Suspend 或 Error，ep_intr 已经在上面被清零了，
     * 下面的循环不会执行，从而保护了系统不挂死。
     */

	if (ep_intr & S3C_EIR_EP0) {
		writel(S3C_EIR_EP0, hsudc->regs + S3C_EIR);
		set_index(hsudc, 0);
		s3c_hsudc_handle_ep0_intr(hsudc);
	}

	ep_intr >>= 1;
	ep_idx = 1;
	while (ep_intr) {
		if (ep_intr & 1)  {
			hsep = &hsudc->ep[ep_idx];
			set_index(hsudc, ep_idx);
			writel(1 << ep_idx, hsudc->regs + S3C_EIR);
			if (ep_is_in(hsep))
				s3c_hsudc_epin_intr(hsudc, ep_idx);
			else
				s3c_hsudc_epout_intr(hsudc, ep_idx);
		}
		ep_intr >>= 1;
		ep_idx++;
	}

	spin_unlock(&hsudc->lock);
	return IRQ_HANDLED;
}

static int s3c_hsudc_start(struct usb_gadget *gadget,
		struct usb_gadget_driver *driver)
{
	struct s3c_hsudc *hsudc = to_hsudc(gadget);
	int ret;

	if (!driver || driver->max_speed < USB_SPEED_FULL || !driver->setup)
		return -EINVAL;
	if (!hsudc)
		return -ENODEV;
	if (hsudc->driver)
		return -EBUSY;

	hsudc->driver = driver;

	ret = regulator_bulk_enable(ARRAY_SIZE(hsudc->supplies), hsudc->supplies);
	if (ret != 0) {
		dev_err(hsudc->dev, "failed to enable supplies: %d\n", ret);
		goto err_supplies;
	}

	/* Initialize and power on Generic PHY */
	if (hsudc->phy) {
		ret = phy_init(hsudc->phy);
		if (ret) goto err_phy_init;
		
		ret = phy_power_on(hsudc->phy);
		if (ret) {
			phy_exit(hsudc->phy);
			goto err_phy_init;
		}
	}

	s3c_hsudc_reconfig(hsudc);


	enable_irq(hsudc->irq);

	pm_runtime_get_sync(hsudc->dev);
	return 0;

err_phy_init:
	regulator_bulk_disable(ARRAY_SIZE(hsudc->supplies), hsudc->supplies);
err_supplies:
	hsudc->driver = NULL;
	return ret;
}

static int s3c_hsudc_stop(struct usb_gadget *gadget)
{
	struct s3c_hsudc *hsudc = to_hsudc(gadget);
	unsigned long flags;
	int epnum;

	if (!hsudc)
		return -ENODEV;

	spin_lock_irqsave(&hsudc->lock, flags);
	hsudc->gadget.speed = USB_SPEED_UNKNOWN;

	for (epnum = 0; epnum < hsudc->epnum; epnum++) {
		struct s3c_hsudc_ep *hsep = &hsudc->ep[epnum];
		hsep->stopped = 1;
		hsep->dma_running = false;
		INIT_LIST_HEAD(&hsep->queue);
	}

	pm_runtime_put(hsudc->dev);
	spin_unlock_irqrestore(&hsudc->lock, flags);

	disable_irq(hsudc->irq);

	/* Turn off Generic PHY */
	if (hsudc->phy) {
		phy_power_off(hsudc->phy);
		phy_exit(hsudc->phy);
	}

	regulator_bulk_disable(ARRAY_SIZE(hsudc->supplies), hsudc->supplies);
	hsudc->driver = NULL;

	return 0;
}

static int s3c_hsudc_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
    /* The Generic PHY framework handles this inside the role-switch 
       or PM events on its own, so we safely return not-supported */
	return -EOPNOTSUPP;
}

static inline u32 s3c_hsudc_read_frameno(struct s3c_hsudc *hsudc)
{
	return readl(hsudc->regs + S3C_FNR) & 0x3FF;
}

static int s3c_hsudc_gadget_getframe(struct usb_gadget *gadget)
{
	return s3c_hsudc_read_frameno(to_hsudc(gadget));
}



static const struct usb_gadget_ops s3c_hsudc_gadget_ops = {
	.get_frame	= s3c_hsudc_gadget_getframe,
	.udc_start	= s3c_hsudc_start,
	.udc_stop	= s3c_hsudc_stop,
	.vbus_draw	= s3c_hsudc_vbus_draw,
	.pullup     = s3c_hsudc_pullup,
};
#ifdef CONFIG_PM_SLEEP
static int s3c_hsudc_pm_suspend(struct device *dev)
{
	struct s3c_hsudc *hsudc = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&hsudc->lock, flags);
	s3c_hsudc_stop_activity(hsudc);
	spin_unlock_irqrestore(&hsudc->lock, flags);

	if (hsudc->driver && hsudc->driver->disconnect)
		hsudc->driver->disconnect(&hsudc->gadget);

	spin_lock_irqsave(&hsudc->lock, flags);
	if (hsudc->driver && hsudc->driver->suspend)
		hsudc->driver->suspend(&hsudc->gadget);

	if (hsudc->phy) {
		phy_power_off(hsudc->phy);
		phy_exit(hsudc->phy);
	}
	clk_disable(hsudc->uclk);
	spin_unlock_irqrestore(&hsudc->lock, flags);

	return 0;
}

static int s3c_hsudc_pm_resume(struct device *dev)
{
	struct s3c_hsudc *hsudc = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&hsudc->lock, flags);
	clk_prepare_enable(hsudc->uclk);

	if (hsudc->phy) {
		phy_init(hsudc->phy);
		phy_power_on(hsudc->phy);
	}

	s3c_hsudc_reconfig(hsudc);
	
	if (hsudc->driver && hsudc->driver->resume)
		hsudc->driver->resume(&hsudc->gadget);

	spin_unlock_irqrestore(&hsudc->lock, flags);
	return 0;
}
#endif

// 定义新的 PM 操作结构体
static SIMPLE_DEV_PM_OPS(s3c_hsudc_pm_ops, s3c_hsudc_pm_suspend, s3c_hsudc_pm_resume);




static int s3c_hsudc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s3c_hsudc *hsudc;
	int ret, i;
	u32 epnum;

	/* 1. Get endpoint count from DT or default to 9 */
	if (of_property_read_u32(dev->of_node, "samsung,epnums", &epnum))
		epnum = 9;

	hsudc = devm_kzalloc(dev, struct_size(hsudc, ep, epnum), GFP_KERNEL);
	if (!hsudc)
		return -ENOMEM;

	hsudc->dev = dev;
	hsudc->epnum = epnum;
	platform_set_drvdata(pdev, hsudc);

	/* 2. Bind the new Generic PHY */
	hsudc->phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(hsudc->phy)) {
		ret = PTR_ERR(hsudc->phy);
		if (ret != -ENODEV && ret != -ENOSYS)
			return dev_err_probe(dev, ret, "failed to get phy\n");
		hsudc->phy = NULL; /* Make it optional just in case */
	}

	for (i = 0; i < ARRAY_SIZE(hsudc->supplies); i++)
		hsudc->supplies[i].supply = s3c_hsudc_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(hsudc->supplies), hsudc->supplies);
	if (ret != 0 && ret != -EPROBE_DEFER)
		dev_err(dev, "failed to request supplies: %d\n", ret);

	hsudc->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hsudc->regs))
		return PTR_ERR(hsudc->regs);

	spin_lock_init(&hsudc->lock);

	hsudc->gadget.max_speed = USB_SPEED_HIGH;
	hsudc->gadget.ops = &s3c_hsudc_gadget_ops;
	hsudc->gadget.name = dev_name(dev);
	hsudc->gadget.ep0 = &hsudc->ep[0].ep;
	hsudc->gadget.is_otg = 0;
	hsudc->gadget.is_a_peripheral = 0;
	hsudc->gadget.speed = USB_SPEED_UNKNOWN;

	s3c_hsudc_setup_ep(hsudc);

	hsudc->irq = platform_get_irq(pdev, 0);
	if (hsudc->irq < 0){
		dev_err(dev, "invalid IRQ: %d\n", hsudc->irq);
        return hsudc->irq ? hsudc->irq : -EINVAL;
    }

	ret = devm_request_irq(dev, hsudc->irq, s3c_hsudc_irq, 
                           IRQF_NO_AUTOEN, driver_name, hsudc);

	if (ret < 0) {
		dev_err(dev, "irq request failed\n");
		return ret;
	}

	hsudc->uclk = devm_clk_get(dev, "usb-device");
	if (IS_ERR(hsudc->uclk)) {
		dev_err(dev, "failed to find usb-device clock source\n");
		return PTR_ERR(hsudc->uclk);
	}
	clk_prepare_enable(hsudc->uclk);


	ret = usb_add_gadget_udc(dev, &hsudc->gadget);
	if (ret)
		goto err_add_udc;

	pm_runtime_enable(dev);
	return 0;

err_add_udc:
	clk_disable(hsudc->uclk);
	return ret;
}

/* 3. Add DT Match Table */
static const struct of_device_id s3c_hsudc_of_match[] = {
	{ .compatible = "samsung,s3c2416-hsudc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s3c_hsudc_of_match);

static struct platform_driver s3c_hsudc_driver = {
	.driver		= {
		.name	= "s3c-hsudc",
		.of_match_table = s3c_hsudc_of_match,
		.pm     = &s3c_hsudc_pm_ops,
	},
	.probe		= s3c_hsudc_probe,
};

module_platform_driver(s3c_hsudc_driver);

MODULE_DESCRIPTION("Samsung S3C24XX USB high-speed controller driver");
MODULE_AUTHOR("Thomas Abraham <thomas.ab@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:s3c-hsudc");
