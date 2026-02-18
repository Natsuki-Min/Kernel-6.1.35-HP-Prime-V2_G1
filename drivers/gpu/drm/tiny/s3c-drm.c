// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for Samsung SoC Framebuffer (S3C2443/S3C64XX)
 *
 * Converted from s3c-fb.c
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_panel.h>
#include <drm/drm_bridge.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fb_dma_helper.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <video/samsung_fimd.h>

/* --- Hardware Definitions (Preserved from s3c-fb) --- */

#define S3C_FB_MAX_WIN 5

#define OSD_BASE(win, variant) ((variant).osd + ((win) * (variant).osd_stride))
#define VIDOSD_A(win, variant) (OSD_BASE(win, variant) + 0x00)
#define VIDOSD_B(win, variant) (OSD_BASE(win, variant) + 0x04)
#define VIDOSD_C(win, variant) (OSD_BASE(win, variant) + 0x08)
#define VIDOSD_D(win, variant) (OSD_BASE(win, variant) + 0x0C)

struct s3c_drm_variant {
	unsigned int	is_2443:1;
	unsigned short	nr_windows;
	unsigned int	vidtcon;
	unsigned short	wincon;
	unsigned short	winmap;
	unsigned short	keycon;
	unsigned short	buf_start;
	unsigned short	buf_end;
	unsigned short	buf_size;
	unsigned short	osd;
	unsigned short	osd_stride;
	unsigned short	palette[S3C_FB_MAX_WIN];

	unsigned int	has_prtcon:1;
	unsigned int	has_shadowcon:1;
	unsigned int	has_blendcon:1;
	unsigned int	has_clksel:1;
	unsigned int	has_fixvclk:1;
};

/* --- DRM Device Structure --- */

struct s3c_drm_device {
	struct drm_device drm;
	struct device *dev;
	void __iomem *regs;
	
	struct clk *bus_clk;
	struct clk *lcd_clk;
	
	struct s3c_drm_variant variant;
	
    struct videomode vm;
    u32 default_vidcon0;
    u32 default_vidcon1;

	/* DRM Objects */
	struct drm_crtc crtc;
	struct drm_plane primary_plane;
	struct drm_encoder encoder;
	struct drm_connector connector;
};

#define to_s3c_drm(d) container_of(d, struct s3c_drm_device, drm)

/* --- Helpers (Adapted from s3c-fb) --- */

static int s3c_drm_calc_pixclk(struct s3c_drm_device *sdev, unsigned int pixclk_hz)
{
	unsigned long clk_rate;
	unsigned long long tmp;
	unsigned int result;

	if (sdev->variant.has_clksel)
		clk_rate = clk_get_rate(sdev->bus_clk);
	else
		clk_rate = clk_get_rate(sdev->lcd_clk);

	/* Avoid division by zero */
	if (pixclk_hz == 0)
		return 1;

	/* result = clk_rate / pixclk_hz. 
	   Note: The hardware usually expects clk_div = rate / pixclk - 1 */
	tmp = (unsigned long long)clk_rate;
	do_div(tmp, pixclk_hz);
	result = (unsigned int)tmp;

	return result;
}

#define OSD_BASE(win, variant) ((variant).osd + ((win) * (variant).osd_stride))
#define VIDOSD_A(win, variant) (OSD_BASE(win, variant) + 0x00)
#define VIDOSD_B(win, variant) (OSD_BASE(win, variant) + 0x04)

static void s3c_drm_enable_controller(struct s3c_drm_device *sdev, bool enable)
{
	//pr_info("[drm] s3c_drm_%s_controller\n",enable?"enable":"disable");
	u32 vidcon0 = readl(sdev->regs + VIDCON0);

	if (enable) {
		vidcon0 |= VIDCON0_ENVID | VIDCON0_ENVID_F;
	} else {
		/*if (vidcon0 & VIDCON0_ENVID) {
			vidcon0 |= VIDCON0_ENVID;
			vidcon0 &= ~VIDCON0_ENVID_F;
		}*/
		vidcon0 &= ~(VIDCON0_ENVID | VIDCON0_ENVID_F);
	}
	writel(vidcon0, sdev->regs + VIDCON0);
}

/* --- Plane Operations (Scanning / Memory) --- */

static const u32 s3c_drm_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
};

static int s3c_drm_plane_atomic_check(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *crtc_state;

	if (!new_plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, new_plane_state->crtc);

	return drm_atomic_helper_check_plane_state(new_plane_state, crtc_state,
						   DRM_PLANE_NO_SCALING,
						   DRM_PLANE_NO_SCALING,
						   false, true);
}

/* --- Helper: Word Alignment (Critical for S3C2416/2443) --- */
static int s3c_drm_align_word(unsigned int bpp, unsigned int pix)
{
	int pix_per_word;

	if (bpp > 16)
		return pix;

	pix_per_word = (8 * 32) / bpp;
	return ALIGN(pix, pix_per_word);
}

/* --- Updated Plane Update --- */
static void s3c_drm_plane_atomic_update(struct drm_plane *plane,
					struct drm_atomic_state *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct s3c_drm_device *sdev = to_s3c_drm(plane->dev);
	struct drm_gem_dma_object *dma_obj;
	void __iomem *regs = sdev->regs;
	unsigned int win_no = 0;
	u32 data;
	dma_addr_t start_addr, end_addr;
	u32 offsize, pagewidth;
	unsigned int bpp_val;
	
	if (!fb) return;

	dma_obj = drm_fb_dma_get_gem_obj(fb, 0);
	
	/* Determine BPP for alignment calculations */
	bpp_val = fb->format->cpp[0] * 8;

	/* 
	 * 1. Calculate DMA Addresses
	 * Use 16.16 fixed point math from state->src only if panning is required,
	 * otherwise strictly use offsets.
	 */
	start_addr = dma_obj->dma_addr + fb->offsets[0];
	
	/* Add X/Y offset for panning */
	start_addr += (new_state->src.y1 >> 16) * fb->pitches[0];
	start_addr += (new_state->src.x1 >> 16) * fb->format->cpp[0];

	/* 
	 * End Address Calculation:
	 * Must match the Visual Y resolution, NOT the virtual buffer size.
	 * If this is wrong, you get the fading white bar (DMA reading past buffer).
	 */
	end_addr = start_addr + (new_state->crtc_h * fb->pitches[0]);

	/* 
	 * 2. Calculate Buffer Sizes
	 * Page width = bytes per line in the VISIBLE area
	 * Offsize = bytes to skip to reach the next line (Virtual - Visible)
	 */
	pagewidth = new_state->crtc_w * fb->format->cpp[0];
	offsize = fb->pitches[0] - pagewidth;
	
	/* 3. Prepare WINCON Data */
	data = WINCONx_ENWIN; 

	switch (fb->format->format) {
	case DRM_FORMAT_RGB565:
		data |= WINCON0_BPPMODE_16BPP_565;
		data |= WINCONx_HAWSWP;
		data |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_RGB888:
		data |= WINCON0_BPPMODE_24BPP_888;
		data |= WINCONx_WSWP;
		data |= WINCONx_BURSTLEN_16WORD;
		break;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		data |= win_no?WINCON1_BPPMODE_25BPP_A1888:WINCON0_BPPMODE_24BPP_888; 
		data |= WINCONx_WSWP;
		data |= WINCONx_BURSTLEN_16WORD;
		break;
	default:
		data |= WINCON0_BPPMODE_24BPP_888;
		break;
	}

	/* --- Hardware Write Sequence --- */

	if (sdev->variant.has_shadowcon) {
		u32 shadow = readl(regs + SHADOWCON);
		shadow |= SHADOWCON_CHx_ENABLE(win_no);
		shadow |= SHADOWCON_WINx_PROTECT(win_no);
		writel(shadow, regs + SHADOWCON);
	}

	writel(start_addr, regs + sdev->variant.buf_start + (win_no * 8));
	writel(end_addr,   regs + sdev->variant.buf_end   + (win_no * 8));

	/* 
	 * Write Buffer Size:
	 * Ensure the OFFSIZE/PAGEWIDTH aligns with hardware expectations.
	 * Some 2443/64xx variants mirror these bits in upper/lower halves.
	 */
	{
		u32 size_reg = (offsize << 13) | pagewidth;
		/* Mirror for E fields if 2443/2416 usually requires it */
		size_reg |= (offsize << 13) | pagewidth; // Logic check: bits might overlap if not careful, but usually ok
		writel(size_reg, regs + sdev->variant.buf_size + (win_no * 4));
	}

	/* 
	 * OSD Position:
	 * IMPORTANT: The Bottom-Right X coordinate MUST be word-aligned based on BPP.
	 * If not aligned, the top of the screen loops back or tears.
	 */
	writel(VIDOSDxA_TOPLEFT_X(new_state->crtc_x) | 
	       VIDOSDxA_TOPLEFT_Y(new_state->crtc_y), 
	       regs + VIDOSD_A(win_no, sdev->variant));

	writel(VIDOSDxB_BOTRIGHT_X(s3c_drm_align_word(bpp_val, new_state->crtc_x + new_state->crtc_w - 1)) |
	       VIDOSDxB_BOTRIGHT_Y(new_state->crtc_y + new_state->crtc_h - 1), 
	       regs + VIDOSD_B(win_no, sdev->variant));

	/* OSD Size (Window 0 specific usage of VIDOSD_C) */
	if(win_no)
		writel(new_state->crtc_w * new_state->crtc_h, regs + VIDOSD_C(win_no, sdev->variant));

	writel(data, regs + sdev->variant.wincon + (win_no * 4));
	writel(0x0, regs + sdev->variant.winmap + (win_no * 4));
	if (sdev->variant.has_shadowcon) {
		u32 shadow = readl(regs + SHADOWCON);
		shadow &= ~SHADOWCON_WINx_PROTECT(win_no);
		writel(shadow, regs + SHADOWCON);
	}
}

static void s3c_drm_plane_atomic_disable(struct drm_plane *plane,
					 struct drm_atomic_state *state)
{
	//pr_info("[drm] s3c_drm_plane_atomic_disable\n");
	struct s3c_drm_device *sdev = to_s3c_drm(plane->dev);
	unsigned int win_no = 0;

	/* Disable window */
	writel(0, sdev->regs + sdev->variant.wincon + (win_no * 4));
	writel(WINxMAP_MAP | WINxMAP_MAP_COLOUR(0x0), 
           sdev->regs + sdev->variant.winmap + (win_no * 4));
}

static const struct drm_plane_helper_funcs s3c_plane_helper_funcs = {
	.atomic_check = s3c_drm_plane_atomic_check,
	.atomic_update = s3c_drm_plane_atomic_update,
	.atomic_disable = s3c_drm_plane_atomic_disable,
};

static const struct drm_plane_funcs s3c_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.destroy	= drm_plane_cleanup,
	.reset		= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

/* --- CRTC Operations (Timing / Global Enable) --- */
/* --- Refactored & Fixed: Global Register Init --- */
static void s3c_drm_init_global_regs(struct s3c_drm_device *sdev, struct drm_display_mode *mode)
{
	//pr_info("[drm] s3c_drm_init_global_regs enter\n");
    void __iomem *regs = sdev->regs;
    int clkdiv;
    u32 vidcon0, vidcon1, vidtcon0, vidtcon1, vidtcon2;
    
    /* Variables for Porch calculations */
    int v_fp, v_bp, v_sw;
    int h_fp, h_bp, h_sw;

    if (!mode || mode->clock == 0) return;

    /* 
     * 1. Calculate Timings 
     * DRM Model: Active -> Front Porch -> Sync -> Back Porch
     * FIMD Model: Sync -> Back Porch (VBPD) -> Active -> Front Porch (VFPD)
     *
     * Therefore:
     * FIMD VBPD = DRM Back Porch (vtotal - vsync_end)
     * FIMD VFPD = DRM Front Porch (vsync_start - vdisplay)
     */

    /* Vertical */
    v_fp = mode->vsync_start - mode->vdisplay;
    v_sw = mode->vsync_end - mode->vsync_start;
    v_bp = mode->vtotal - mode->vsync_end;

    /* Horizontal */
    h_fp = mode->hsync_start - mode->hdisplay;
    h_sw = mode->hsync_end - mode->hsync_start;
    h_bp = mode->htotal - mode->hsync_end;

    /* 2. Configure VIDCON0 */
    clkdiv = s3c_drm_calc_pixclk(sdev, mode->clock * 1000);
    vidcon0 = sdev->default_vidcon0; 
    vidcon0 &= ~(VIDCON0_CLKVAL_F_MASK | VIDCON0_CLKDIR);
	
	
    if (clkdiv > 1) {
        vidcon0 |= VIDCON0_CLKVAL_F(clkdiv - 1) | VIDCON0_CLKDIR;
    }

    if (sdev->variant.is_2443)
        vidcon0 |= (1 << 5);

    /* 3. Write VIDCON registers */
    writel(vidcon0, regs + VIDCON0);

	vidcon1 = sdev->default_vidcon1;
    /* 
     * S3C2416/2443 specific: 
     * Force VCLK to run even if data is not ready. 
     * This prevents the 10Hz flickering on resume.
     */

    writel(vidcon1, regs + VIDCON1);

	if (sdev->variant.has_fixvclk) {
		vidcon1 = readl(regs + VIDCON1);
		vidcon1 &= ~VIDCON1_VCLK_MASK;
		vidcon1 |= VIDCON1_VCLK_RUN;
		writel(vidcon1, regs + VIDCON1);
	}

	for (int i = 0; i < (sdev->variant.nr_windows - 1); i++) {
        void __iomem *key_regs = sdev->regs + sdev->variant.keycon;
        key_regs += (i * 8);
        writel(0xffffff, key_regs + WKEYCON0);
        writel(0xffffff, key_regs + WKEYCON1);
    }

    /* 
     * 4. Configure VIDTCON0 (Vertical) 
     * Fixed: Swapped VBPD and VFPD to match hardware expectation.
     */
    vidtcon0 = VIDTCON0_VBPD(v_bp - 1) | 
               VIDTCON0_VFPD(v_fp - 1) | 
               VIDTCON0_VSPW(v_sw - 1);
    
    writel(vidtcon0, regs + sdev->variant.vidtcon);

    /* 
     * 5. Configure VIDTCON1 (Horizontal)
     * Fixed: Swapped HBPD and HFPD to match hardware expectation.
     */
    vidtcon1 = VIDTCON1_HBPD(h_bp - 1) | 
               VIDTCON1_HFPD(h_fp - 1) | 
               VIDTCON1_HSPW(h_sw - 1);
    
    writel(vidtcon1, regs + sdev->variant.vidtcon + 4);

    /* 6. Configure VIDTCON2 (Resolution) */
    vidtcon2 = VIDTCON2_LINEVAL(mode->vdisplay - 1) |
               VIDTCON2_HOZVAL(mode->hdisplay - 1);
    
    if (!sdev->variant.is_2443) {
        vidtcon2 |= VIDTCON2_LINEVAL_E(mode->vdisplay - 1) |
                    VIDTCON2_HOZVAL_E(mode->hdisplay - 1);
    }
    writel(vidtcon2, regs + sdev->variant.vidtcon + 8);

    /* Debug print to confirm values match your expectation */
}

static void s3c_drm_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
    struct s3c_drm_device *sdev = to_s3c_drm(crtc->dev);
    s3c_drm_init_global_regs(sdev, &crtc->state->adjusted_mode);
}

static void s3c_drm_crtc_atomic_enable(struct drm_crtc *crtc,
				       struct drm_atomic_state *state)
{
	struct s3c_drm_device *sdev = to_s3c_drm(crtc->dev);

	pm_runtime_get_sync(sdev->dev);
	s3c_drm_enable_controller(sdev, true);
	drm_crtc_vblank_on(crtc);
}

static void s3c_drm_crtc_atomic_disable(struct drm_crtc *crtc,
					struct drm_atomic_state *state)
{
	struct s3c_drm_device *sdev = to_s3c_drm(crtc->dev);
    int i;
    u32 val;
    for (i = 0; i < sdev->variant.nr_windows; i++) {
        /* A. Enable Shadow Protection (if supported) */
        if (sdev->variant.has_shadowcon) {
            val = readl(sdev->regs + SHADOWCON);
            val |= SHADOWCON_WINx_PROTECT(i);
            writel(val, sdev->regs + SHADOWCON);
        }

        writel(0, sdev->regs + sdev->variant.wincon + (i * 4));
        writel(WINxMAP_MAP | WINxMAP_MAP_COLOUR(0x0), 
               sdev->regs + sdev->variant.winmap + (i * 4));

        if (sdev->variant.has_shadowcon) {
            val = readl(sdev->regs + SHADOWCON);
            val &= ~SHADOWCON_WINx_PROTECT(i);
            writel(val, sdev->regs + SHADOWCON);
        }
    }
	drm_crtc_vblank_off(crtc);
	s3c_drm_enable_controller(sdev, false);
	pm_runtime_put_sync(sdev->dev);
}

static void s3c_drm_crtc_atomic_flush(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct s3c_drm_device *sdev = to_s3c_drm(crtc->dev);
	unsigned long flags;

	spin_lock_irqsave(&sdev->drm.event_lock, flags);
	if (crtc->state->event) {
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, crtc->state->event);
		else
			drm_crtc_send_vblank_event(crtc, crtc->state->event);
	}
	crtc->state->event = NULL;
	spin_unlock_irqrestore(&sdev->drm.event_lock, flags);
}

static const struct drm_crtc_helper_funcs s3c_crtc_helper_funcs = {
	.mode_set_nofb	= s3c_drm_crtc_mode_set_nofb,
	.atomic_enable	= s3c_drm_crtc_atomic_enable,
	.atomic_disable	= s3c_drm_crtc_atomic_disable,
	.atomic_flush	= s3c_drm_crtc_atomic_flush,
};

/* --- Interrupt Handling --- */

static irqreturn_t s3c_drm_irq(int irq, void *dev_id)
{
	struct s3c_drm_device *sdev = dev_id;
	//u32 val = readl(sdev->regs + VIDINTCON1);

	//if (val & VIDINTCON1_INT_FRAME) {
		//writel(VIDINTCON1_INT_FRAME, sdev->regs + VIDINTCON1);
		drm_crtc_handle_vblank(&sdev->crtc);
		return IRQ_HANDLED;
	//}

	//return IRQ_NONE;
}

static int s3c_drm_enable_vblank(struct drm_crtc *crtc)
{
	struct s3c_drm_device *sdev = to_s3c_drm(crtc->dev);
	u32 val = readl(sdev->regs + VIDINTCON0);

	val |= VIDINTCON0_INT_ENABLE | VIDINTCON0_INT_FRAME;
	val &= ~VIDINTCON0_FRAMESEL0_MASK;
	val |= VIDINTCON0_FRAMESEL0_VSYNC;
	writel(val, sdev->regs + VIDINTCON0);

	return 0;
}

static void s3c_drm_disable_vblank(struct drm_crtc *crtc)
{
	struct s3c_drm_device *sdev = to_s3c_drm(crtc->dev);
	u32 val = readl(sdev->regs + VIDINTCON0);

	val &= ~(VIDINTCON0_INT_ENABLE | VIDINTCON0_INT_FRAME |VIDINTCON0_FRAMESEL0_VSYNC);
	writel(val, sdev->regs + VIDINTCON0);
}

static const struct drm_crtc_funcs s3c_crtc_funcs = {
	.reset		= drm_atomic_helper_crtc_reset,
	.destroy	= drm_crtc_cleanup,
	.set_config	= drm_atomic_helper_set_config,
	.page_flip	= drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.enable_vblank	= s3c_drm_enable_vblank,
	.disable_vblank	= s3c_drm_disable_vblank,
};

/* --- Encoder / Connector (Simple implementation) --- */

static const struct drm_encoder_funcs s3c_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int s3c_connector_get_modes(struct drm_connector *connector)
{
    struct s3c_drm_device *sdev = to_s3c_drm(connector->dev);
    struct drm_display_mode *mode;

    /* Create a DRM mode from the parsed videomode */
    mode = drm_mode_create(connector->dev);
    if (!mode)
        return 0;

    /* Convert generic videomode to DRM format */
    drm_display_mode_from_videomode(&sdev->vm, mode);

    /* Set mode type */
    mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
    
    /* Add to connector and return count */
    drm_mode_probed_add(connector, mode);
    return 1;
}

static const struct drm_connector_helper_funcs s3c_connector_helper_funcs = {
	.get_modes = s3c_connector_get_modes,
};

static const struct drm_connector_funcs s3c_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* --- Driver Setup --- */
#ifdef CONFIG_PM_SLEEP
static int s3c_drm_pm_suspend(struct device *dev)
{
	struct s3c_drm_device *sdev = dev_get_drvdata(dev);
	int ret;

	/* 1. DRM Helper: Disables CRTCs and saves atomic state */
	ret = drm_mode_config_helper_suspend(&sdev->drm);
	if (ret)
		return ret;

	/* 2. Disable Hardware (optional, helper likely did it via atomic_disable) */
	//s3c_drm_enable_controller(sdev, false);

	/* 3. Gate Clocks */
	if (sdev->lcd_clk)
		clk_disable_unprepare(sdev->lcd_clk);
	clk_disable_unprepare(sdev->bus_clk);

	return 0;
}

static int s3c_drm_pm_resume(struct device *dev)
{
	struct s3c_drm_device *sdev = dev_get_drvdata(dev);
	int ret;

	/* 1. Ungate Clocks */
	ret = clk_prepare_enable(sdev->bus_clk);
	if (ret)
		return ret;
	
	if (sdev->lcd_clk) {
		ret = clk_prepare_enable(sdev->lcd_clk);
		if (ret) {
			clk_disable_unprepare(sdev->bus_clk);
			return ret;
		}
	}

	for (int i = 0; i < sdev->variant.nr_windows; i++) {
        writel(0, sdev->regs + sdev->variant.wincon + (i * 4));
    }
	/* 
	 * 2. Restore Global Registers 
	 * We need to look up the mode that was active.
	 * If the CRT was enabled, the atomic helper resume will trigger a modeset,
	 * but it's safer to ensure basic timing registers are ready.
	 */
	if (sdev->crtc.state && sdev->crtc.state->active) {
		s3c_drm_init_global_regs(sdev, &sdev->crtc.state->adjusted_mode);
	}

	/* 3. DRM Helper: Restores atomic state (enables CRTCs/Planes) */
	return drm_mode_config_helper_resume(&sdev->drm);
}
#endif

static const struct dev_pm_ops s3c_drm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(s3c_drm_pm_suspend, s3c_drm_pm_resume)
};


DEFINE_DRM_GEM_DMA_FOPS(s3c_drm_fops);

static const struct drm_driver s3c_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name			= "s3c-drm",
	.desc			= "Samsung SoC DRM",
	.date			= "20260212",
	.major			= 1,
	.minor			= 0,
	.fops			= &s3c_drm_fops,
	.dumb_create		= drm_gem_dma_dumb_create,
};

static const struct drm_mode_config_funcs s3c_mode_config_funcs = {
	.fb_create		= drm_gem_fb_create,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

/* --- Hardware Variants --- */

static struct s3c_drm_variant s3c_fb_data_64xx = {
	.nr_windows	= 5,
	.vidtcon	= VIDTCON0,
	.wincon		= WINCON(0),
	.winmap		= WINxMAP(0),
	.keycon		= WKEYCON,
	.osd		= VIDOSD_BASE,
	.osd_stride	= 16,
	.buf_start	= VIDW_BUF_START(0),
	.buf_size	= VIDW_BUF_SIZE(0),
	.buf_end	= VIDW_BUF_END(0),
	.has_prtcon	= 1,
	.has_clksel	= 1,
	
	.has_shadowcon	= 1,
};

static struct s3c_drm_variant s3c_fb_data_s3c2443 = {
	.nr_windows	= 2,
	.is_2443	= 1,
	.vidtcon	= 0x08,
	.wincon		= 0x14,
	.winmap		= 0xd0,
	.keycon		= 0xb0,
	.osd		= 0x28,
	.osd_stride	= 12,
	.buf_start	= 0x64,
	.buf_size	= 0x94,
	.buf_end	= 0x7c,
	.has_clksel	= 1,
};

/* --- Probe / Remove --- */

static int s3c_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct s3c_drm_device *sdev;
	struct drm_device *drm;
	struct s3c_drm_variant *variant;
	struct display_timings *disp_timing;
	struct device_node *win_node;
	int ret, irq;
	u32 bpp = 32;

	/* 1. Identify Variant */
	variant = (struct s3c_drm_variant *)of_device_get_match_data(dev);
	if (!variant)
		return -EINVAL;

	/* 2. Allocate DRM Device */
	sdev = devm_drm_dev_alloc(dev, &s3c_drm_driver, struct s3c_drm_device, drm);
	if (IS_ERR(sdev))
		return PTR_ERR(sdev);
	
	drm = &sdev->drm;
	sdev->dev = dev;
	sdev->variant = *variant;
	platform_set_drvdata(pdev, sdev);

	/* 3. Resources (Regs, Clocks, IRQ) */
	sdev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sdev->regs))
		return PTR_ERR(sdev->regs);

	sdev->bus_clk = devm_clk_get(dev, "lcd");
	if (IS_ERR(sdev->bus_clk))
		return dev_err_probe(dev, PTR_ERR(sdev->bus_clk), "no bus clock\n");

	if (!sdev->variant.has_clksel) {
		sdev->lcd_clk = devm_clk_get(dev, "sclk_fimd");
		if (IS_ERR(sdev->lcd_clk))
			return dev_err_probe(dev, PTR_ERR(sdev->lcd_clk), "no lcd clock\n");
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* 4. Parse Device Tree */
	disp_timing = of_get_display_timings(np);
	if (!disp_timing) {
		dev_err(dev, "failed to get display timings\n");
		return -EINVAL;
	}

	ret = videomode_from_timings(disp_timing, &sdev->vm, 0);
	if (ret) {
		dev_err(dev, "failed to parse videomode\n");
		return ret;
	}

	of_property_read_u32(np, "samsung,vidcon0", &sdev->default_vidcon0);
	of_property_read_u32(np, "samsung,vidcon1", &sdev->default_vidcon1);

	for_each_child_of_node(np, win_node) {
		u32 reg;
		if (of_property_read_u32(win_node, "reg", &reg) == 0 && reg == 0) {
			of_property_read_u32(win_node, "samsung,bpp", &bpp);
			of_node_put(win_node);
			break;
		}
	}

	/* 5. Modesetting Init */
	ret = drmm_mode_config_init(drm);
	if (ret) return ret;

	drm->mode_config.min_width = 0;
	drm->mode_config.min_height = 0;
	drm->mode_config.max_width = 4096;
	drm->mode_config.max_height = 4096;
	drm->mode_config.funcs = &s3c_mode_config_funcs;

	/* --- FIX START: Initialize Plane BEFORE CRTC --- */

	/* 6. Initialize Plane (Window 0) */
	/* 
	 * Third argument is 'possible_crtcs'. 
	 * We pass 1 (binary 0001) to indicate this plane belongs to CRTC index 0.
	 */
	ret = drm_universal_plane_init(drm, &sdev->primary_plane, 1,
				       &s3c_plane_funcs,
				       s3c_drm_formats, ARRAY_SIZE(s3c_drm_formats),
				       NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret) return ret;
	
	drm_plane_helper_add(&sdev->primary_plane, &s3c_plane_helper_funcs);

	/* 7. Initialize CRTC (Controller) */
	/* Now we can safely pass &sdev->primary_plane */
	ret = drm_crtc_init_with_planes(drm, &sdev->crtc, 
					&sdev->primary_plane, NULL,
					&s3c_crtc_funcs, NULL);
	if (ret) return ret;

	drm_crtc_helper_add(&sdev->crtc, &s3c_crtc_helper_funcs);

	/* --- FIX END --- */
	ret = drm_vblank_init(drm, 1);
	if (ret) {
		dev_err(dev, "Failed to init vblank\n");
		return ret;
	}
	/* 8. Initialize Encoder & Connector */
	ret = drm_encoder_init(drm, &sdev->encoder, &s3c_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret) return ret;

	sdev->encoder.possible_crtcs = drm_crtc_mask(&sdev->crtc);

	ret = drm_connector_init(drm, &sdev->connector, &s3c_connector_funcs,
				 DRM_MODE_CONNECTOR_Unknown);
	if (ret) return ret;

	drm_connector_helper_add(&sdev->connector, &s3c_connector_helper_funcs);
	drm_connector_attach_encoder(&sdev->connector, &sdev->encoder);

	/* 9. Setup Interrupts and Clocks */
	ret = devm_request_irq(dev, irq, s3c_drm_irq, 0, "s3c-drm", sdev);
	if (ret) return ret;

	ret = clk_prepare_enable(sdev->bus_clk);
	if (ret) return ret;
	
	if (sdev->lcd_clk)
		clk_prepare_enable(sdev->lcd_clk);

	pm_runtime_enable(dev);

	drm_mode_config_reset(drm);

	/* 10. Register */
	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_clk;

	drm_fbdev_generic_setup(drm, bpp);

	return 0;

err_clk:
	if (sdev->lcd_clk) clk_disable_unprepare(sdev->lcd_clk);
	clk_disable_unprepare(sdev->bus_clk);
	return ret;
}

static int s3c_drm_remove(struct platform_device *pdev)
{
	struct s3c_drm_device *sdev = platform_get_drvdata(pdev);

	drm_dev_unplug(&sdev->drm);
	pm_runtime_disable(sdev->dev);
	
	if (sdev->lcd_clk)
		clk_disable_unprepare(sdev->lcd_clk);
	clk_disable_unprepare(sdev->bus_clk);

	return 0;
}

static const struct of_device_id s3c_drm_dt_ids[] = {
	{ .compatible = "samsung,s3c2443-drm", .data = &s3c_fb_data_s3c2443 },
	{ .compatible = "samsung,s3c6400-drm", .data = &s3c_fb_data_64xx },
	{},
};
MODULE_DEVICE_TABLE(of, s3c_drm_dt_ids);

static struct platform_driver s3c_drm_platform_driver = {
	.probe		= s3c_drm_probe,
	.remove		= s3c_drm_remove,
	.driver		= {
		.name	= "s3c-drm",
		.of_match_table = s3c_drm_dt_ids,
		.pm     = &s3c_drm_pm_ops,
	},
};

module_platform_driver(s3c_drm_platform_driver);

MODULE_AUTHOR("Converted to DRM");
MODULE_DESCRIPTION("Samsung SoC DRM Driver");
MODULE_LICENSE("GPL");
