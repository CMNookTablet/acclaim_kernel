/*
 * linux/include/asm-arm/arch-omap/display.h
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ASM_ARCH_OMAP_DISPLAY_H
#define __ASM_ARCH_OMAP_DISPLAY_H

#include <linux/list.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <asm/atomic.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>
#include <video/omapdss.h>

#define DISPC_IRQ_SYNC_LOST_2		(1 << 17)
#define DISPC_IRQ_ACBIAS_COUNT_STAT2	(1 << 21)
#define DISPC_IRQ_FRAMEDONE2		(1 << 22)
#define DISPC_IRQ_FRAMEDONE_WB		(1 << 23)
#define DISPC_IRQ_FRAMEDONE_DIG		(1 << 24)
#define DISPC_IRQ_WB_BUF_OVERFLOW	(1 << 25)


enum omap_writeback_capturemode {
	OMAP_WB_CAPTURE_ALL	= 0x0,
	OMAP_WB_CAPTURE_1	= 0x1,
	OMAP_WB_CAPTURE_1_OF_2	= 0x2,
	OMAP_WB_CAPTURE_1_OF_3	= 0x3,
	OMAP_WB_CAPTURE_1_OF_4	= 0x4,
	OMAP_WB_CAPTURE_1_OF_5	= 0x5,
	OMAP_WB_CAPTURE_1_OF_6	= 0x6,
	OMAP_WB_CAPTURE_1_OF_7	= 0x7,
};

enum device_n_buffer_type {
	OMAP_FLAG_IDEV = 1,	/* interlaced device */
	OMAP_FLAG_IBUF = 2,	/* sequentially interlaced buffer */
	OMAP_FLAG_ISWAP = 4,	/* bottom-top interlacing */

	PBUF_PDEV	= 0,
	PBUF_IDEV	= OMAP_FLAG_IDEV,
	PBUF_IDEV_SWAP	= OMAP_FLAG_IDEV | OMAP_FLAG_ISWAP,
	IBUF_IDEV	= OMAP_FLAG_IBUF | OMAP_FLAG_IDEV,
	IBUF_IDEV_SWAP	= OMAP_FLAG_IBUF | OMAP_FLAG_IDEV | OMAP_FLAG_ISWAP,
	IBUF_PDEV	= OMAP_FLAG_IBUF,
	IBUF_PDEV_SWAP	= OMAP_FLAG_IBUF | OMAP_FLAG_ISWAP,
};

/* Stereoscopic Panel types
 * row, column, overunder, sidebyside options
 * are with respect to native scan order
*/
enum s3d_disp_type {
	S3D_DISP_NONE = 0,
	S3D_DISP_FRAME_SEQ,
	S3D_DISP_ROW_IL,
	S3D_DISP_COL_IL,
	S3D_DISP_PIX_IL,
	S3D_DISP_CHECKB,
	S3D_DISP_OVERUNDER,
	S3D_DISP_SIDEBYSIDE,
};

/* Subsampling direction is based on native panel scan order.
*/
enum s3d_disp_sub_sampling {
	S3D_DISP_SUB_SAMPLE_NONE = 0,
	S3D_DISP_SUB_SAMPLE_V,
	S3D_DISP_SUB_SAMPLE_H,
};

/* Indicates if display expects left view first followed by right or viceversa
 * For row interlaved displays, defines first row view
 * For column interleaved displays, defines first column view
 * For checkerboard, defines first pixel view
 * For overunder, defines top view
 * For sidebyside, defines west view
*/
enum s3d_disp_order {
	S3D_DISP_ORDER_L = 0,
	S3D_DISP_ORDER_R = 1,
};

/* Indicates current view
 * Used mainly for displays that need to trigger a sync signal
*/
enum s3d_disp_view {
	S3D_DISP_VIEW_L = 0,
	S3D_DISP_VIEW_R,
};



/* DSI */
enum omap_dsi_index {
	DSI1 = 0,
	DSI2 = 1,
};

#if 0
void dsi_bus_lock(enum omap_dsi_index ix);
void dsi_bus_unlock(enum omap_dsi_index ix);
bool dsi_bus_is_locked(enum omap_dsi_index ix);
int dsi_vc_dcs_write(enum omap_dsi_index ix, int channel,
		u8 *data, int len);
int dsi_vc_dcs_write_0(enum omap_dsi_index ix, int channel,
		u8 dcs_cmd);
int dsi_vc_dcs_write_1(enum omap_dsi_index ix, int channel,
		u8 dcs_cmd, u8 param);
int dsi_vc_dcs_write_nosync(enum omap_dsi_index ix, int channel,
		u8 *data, int len);
int dsi_vc_dcs_read(enum omap_dsi_index ix, int channel,
		u8 dcs_cmd, u8 *buf, int buflen);
int dsi_vc_dcs_read_1(enum omap_dsi_index ix, int channel,
		u8 dcs_cmd, u8 *data);
int dsi_vc_dcs_read_2(enum omap_dsi_index ix, int channel,
		u8 dcs_cmd, u8 *data1, u8 *data2);
int dsi_vc_set_max_rx_packet_size(enum omap_dsi_index ix,
		int channel, u16 len);
int dsi_vc_send_null(enum omap_dsi_index ix, int channel);
int dsi_vc_send_bta_sync(enum omap_dsi_index ix, int channel);
#endif

/* Board specific data */
#define PWM2ON			0x03
#define PWM2OFF			0x04
#define TOGGLE3			0x92
#define HDMI_GPIO_60		60
#define HDMI_GPIO_41		41
#define DLP_4430_GPIO_40	40
#define DLP_4430_GPIO_44	44
#define DLP_4430_GPIO_45	45
#define DLP_4430_GPIO_59	59

#if 0
#define PROGRESSIVE		0
#define INTERLACED		1

struct omap_dss_board_info {
	int (*get_last_off_on_transaction_id)(struct device *dev);
	int num_devices;
	struct omap_dss_device **devices;
	struct omap_dss_device *default_device;
};
extern void omap_display_init(struct omap_dss_board_info *board_data);

struct omap_display_platform_data{
	char name[16];
	int hwmod_count;
	struct omap_dss_board_info *board_data;
	int (*device_enable)(struct platform_device *pdev);
	int (*device_shutdown)(struct platform_device *pdev);
	int (*device_idle)(struct platform_device *pdev);
};

/* Weight coef are set as value * 1000 (if coef = 1 it is set to 1000) */
struct omap_dss_color_weight_coef {
	int rr, rg, rb;
	int gr, gg, gb;
	int br, bg, bb;
};

struct omap_dss_yuv2rgb_conv {
	enum omap_dss_color_conv_type type;
	struct omap_dss_color_weight_coef *weight_coef;
	bool dirty;
};

struct omap_overlay_info {
	bool enabled;

	u32 paddr;
	void __iomem *vaddr;
	u16 screen_width;
	u16 width;
	u16 height;
	enum omap_color_mode color_mode;
	struct omap_dss_yuv2rgb_conv yuv2rgb_conv;
	u8 rotation;
	enum omap_dss_rotation_type rotation_type;
	bool mirror;

	u16 pos_x;
	u16 pos_y;
	u16 out_width;	/* if 0, out_width == width */
	u16 out_height;	/* if 0, out_height == height */
	u8 global_alpha;
	u16 min_x_decim, max_x_decim, min_y_decim, max_y_decim;
	enum omap_overlay_zorder zorder;
	u32 p_uv_addr;	/* for NV12 format */
	enum device_n_buffer_type field;
	u16 pic_height;	/* required for interlacing with cropping */
	bool out_wb; /* true when this overlay only feeds wb pipeline */
};

struct omap_overlay {
	struct kobject kobj;
	struct list_head list;

	/* static fields */
	const char *name;
	int id;
	enum omap_color_mode supported_modes;
	enum omap_overlay_caps caps;

	/* dynamic fields */
	struct omap_overlay_manager *manager;
	struct omap_overlay_info info;

	/* if true, info has been changed, but not applied() yet */
	bool info_dirty;

	/* if true, overlay resource is used by an instance of a driver*/
	bool in_use;
	struct mutex lock;

	int (*set_manager)(struct omap_overlay *ovl,
		struct omap_overlay_manager *mgr);
	int (*unset_manager)(struct omap_overlay *ovl);

	int (*set_overlay_info)(struct omap_overlay *ovl,
			struct omap_overlay_info *info);
	void (*get_overlay_info)(struct omap_overlay *ovl,
			struct omap_overlay_info *info);

	int (*wait_for_go)(struct omap_overlay *ovl);
};

struct omap_overlay_manager_info {
	u32 default_color;

	enum omap_dss_trans_key_type trans_key_type;
	u32 trans_key;
	bool trans_enabled;

	bool alpha_enabled;
	bool cpr_enable;
	struct omap_dss_color_weight_coef cpr_coefs;

};

struct omap_overlay_manager {
	struct kobject kobj;
	struct list_head list;

	/* static fields */
	const char *name;
	int id;
	enum omap_overlay_manager_caps caps;
	int num_overlays;
	struct omap_overlay **overlays;
	enum omap_display_type supported_displays;

	/* dynamic fields */
	struct omap_dss_device *device;
	struct omap_overlay_manager_info info;

	bool device_changed;
	/* if true, info has been changed but not applied() yet */
	bool info_dirty;

	int (*set_device)(struct omap_overlay_manager *mgr,
		struct omap_dss_device *dssdev);
	int (*unset_device)(struct omap_overlay_manager *mgr);

	int (*set_manager_info)(struct omap_overlay_manager *mgr,
			struct omap_overlay_manager_info *info);
	void (*get_manager_info)(struct omap_overlay_manager *mgr,
			struct omap_overlay_manager_info *info);

	int (*apply)(struct omap_overlay_manager *mgr);
	int (*wait_for_go)(struct omap_overlay_manager *mgr);
	int (*wait_for_vsync)(struct omap_overlay_manager *mgr);

	int (*enable)(struct omap_overlay_manager *mgr);
	int (*disable)(struct omap_overlay_manager *mgr);
};

enum omap_writeback_source_type {
	OMAP_WB_SOURCE_OVERLAY	= 0,
	OMAP_WB_SOURCE_MANAGER	= 1
};


struct omap_writeback_info {
	bool				enabled;
	bool				info_dirty;
	enum omap_writeback_source	source;
	enum omap_writeback_source_type source_type;
	unsigned long			width;
	unsigned long			height;
	unsigned long			out_width;
	unsigned long			out_height;
	enum omap_color_mode		dss_mode;
	enum omap_writeback_capturemode capturemode;
	unsigned long			paddr;
	unsigned long			puv_addr;
	u32				line_skip;
};

struct omap_writeback {
	struct kobject		kobj;
	struct list_head	list;
	bool			enabled;
	bool			info_dirty;
	bool			first_time;

	/* mutex to control access to wb data */
	struct mutex lock;
	struct omap_writeback_info info;

	bool (*check_wb)(struct omap_writeback *wb);

	int (*set_wb_info)(struct omap_writeback *wb,
		struct omap_writeback_info *info);
	void (*get_wb_info)(struct omap_writeback *wb,
		struct omap_writeback_info *info);
};

struct s3d_disp_info {
	enum s3d_disp_type type;
	enum s3d_disp_sub_sampling sub_samp;
	enum s3d_disp_order order;
	/* Gap between left and right views
	 * For over/under units are lines
	 * For sidebyside units are pixels
	  *For other types ignored*/
	unsigned int gap;
};

/* info for delayed update */
struct omap_dss_sched_update {
	u16 x, y, w, h;		/* update window */
	struct work_struct work;
	bool scheduled;		/* scheduled */
	bool waiting;		/* waiting to update */
};

struct omap_dss_device {
	struct device dev;

	enum omap_display_type type;

	union {
		struct {
			u8 data_lines;
		} dpi;

		struct {
			u8 channel;
			u8 data_lines;
		} rfbi;

		struct {
			u8 datapairs;
		} sdi;

		struct {
			u8 clk_lane;
			u8 clk_pol;
			u8 data1_lane;
			u8 data1_pol;
			u8 data2_lane;
			u8 data2_pol;

			struct {
				u16 regn;
				u16 regm;
				u16 regm_dispc;
				u16 regm_dsi;

				u16 lp_clk_div;

				u16 lck_div;
				u16 pck_div;
			} div;
		} dsi;

		struct {
			enum omap_dss_venc_type type;
			bool invert_polarity;
		} venc;
	} phy;

	struct {
		struct omap_video_timings timings;

		int acbi;	/* ac-bias pin transitions per interrupt */
		/* Unit: line clocks */
		int acb;	/* ac-bias pin frequency */

		enum omap_panel_config config;
		struct s3d_disp_info s3d_info;
		u32 width_in_mm;
		u32 height_in_mm;
	} panel;

	struct {
		u8 pixel_size;
		struct rfbi_timings rfbi_timings;
	} ctrl;

	int max_backlight_level;

	const char *name;

	/* used to match device to driver */
	const char *driver_name;

	void *data;

	struct omap_dss_driver *driver;

	/* helper variable for driver suspend/resume */
	bool activate_after_resume;

	enum omap_display_caps caps;

	struct omap_overlay_manager *manager;
	struct omap_writeback *wb_manager;

	enum omap_dss_display_state state;
	enum omap_channel channel;

	struct blocking_notifier_head notifier;

	/* support for scheduling subsequent update */
	struct omap_dss_sched_update sched_update;

	/* HDMI specific */
	void (*enable_device_detect)(struct omap_dss_device *dssdev, u8 enable);
	bool (*get_device_detect)(struct omap_dss_device *dssdev);
	int (*get_device_connected)(struct omap_dss_device *dssdev);

	/* platform specific  */
	int (*platform_enable)(struct omap_dss_device *dssdev);
	void (*platform_disable)(struct omap_dss_device *dssdev);
	int (*set_backlight)(struct omap_dss_device *dssdev, int level);
	int (*get_backlight)(struct omap_dss_device *dssdev);
};

struct omap_dss_driver {
	struct device_driver driver;

	int (*probe)(struct omap_dss_device *);
	void (*remove)(struct omap_dss_device *);

	int (*enable)(struct omap_dss_device *display);
	void (*disable)(struct omap_dss_device *display);
	int (*suspend)(struct omap_dss_device *display);
	int (*resume)(struct omap_dss_device *display);
	int (*run_test)(struct omap_dss_device *display, int test);

	int (*set_update_mode)(struct omap_dss_device *dssdev,
			enum omap_dss_update_mode);
	enum omap_dss_update_mode (*get_update_mode)(
			struct omap_dss_device *dssdev);

	int (*update)(struct omap_dss_device *dssdev,
			       u16 x, u16 y, u16 w, u16 h);
	int (*sched_update)(struct omap_dss_device *dssdev,
			       u16 x, u16 y, u16 w, u16 h);
	int (*sync)(struct omap_dss_device *dssdev);

	int (*enable_te)(struct omap_dss_device *dssdev, bool enable);
	int (*get_te)(struct omap_dss_device *dssdev);

	u8 (*get_rotate)(struct omap_dss_device *dssdev);
	int (*set_rotate)(struct omap_dss_device *dssdev, u8 rotate);

	bool (*get_mirror)(struct omap_dss_device *dssdev);
	int (*set_mirror)(struct omap_dss_device *dssdev, bool enable);

	int (*memory_read)(struct omap_dss_device *dssdev,
			void *buf, size_t size,
			u16 x, u16 y, u16 w, u16 h);

	void (*get_resolution)(struct omap_dss_device *dssdev,
			u16 *xres, u16 *yres);
//	void (*get_dimension)(struct omap_dss_device *dssdev,
//			u32 *width, u32 *height);
	int (*get_recommended_bpp)(struct omap_dss_device *dssdev);

	int (*check_timings)(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);
	void (*set_timings)(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);
	void (*get_timings)(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);

	int (*set_wss)(struct omap_dss_device *dssdev, u32 wss);
	u32 (*get_wss)(struct omap_dss_device *dssdev);

	void (*enable_device_detect)(struct omap_dss_device *dssdev, u8 enable);
	bool (*get_device_detect)(struct omap_dss_device *dssdev);
	int (*get_device_connected)(struct omap_dss_device *dssdev);

	/*
	 * used for sysfs control for panels that are not fully enabled
	 * when powered on
	 */
	bool (*smart_is_enabled)(struct omap_dss_device *dssdev);
	int (*smart_enable)(struct omap_dss_device *display);

	/*HDMI specific */
	void (*get_edid)(struct omap_dss_device *dssdev);
	void (*set_custom_edid_timing_code)(struct omap_dss_device *dssdev,
			int mode, int code);
	int (*hpd_enable)(struct omap_dss_device *dssdev);
	/* resets display.  returns status (of reenabling the display).*/
	int (*reset)(struct omap_dss_device *dssdev,
					enum omap_dss_reset_phase phase);

	/* S3D specific */
	/* Used for displays that can switch 3D mode on/off
	3D only displays should return non-zero value when trying to disable */
	int (*enable_s3d)(struct omap_dss_device *dssdev, bool enable);
	/* 3D only panels should return true always */
	bool (*get_s3d_enabled)(struct omap_dss_device *dssdev);
	/* Only used for frame sequential displays*/
	int (*set_s3d_view)(struct omap_dss_device *dssdev, enum s3d_disp_view view);
	/*Some displays may accept multiple 3D packing formats (like HDMI)
	 *hence we add capability to choose the most optimal one given a source
	 *Returns non-zero if the type was not supported*/
	int (*set_s3d_disp_type)(struct omap_dss_device *dssdev, struct s3d_disp_info *info);
};

struct pico_platform_data {
	u8 gpio_intr;
};

int omap_dss_register_driver(struct omap_dss_driver *);
void omap_dss_unregister_driver(struct omap_dss_driver *);

int omap_dss_register_device(struct omap_dss_device *);
void omap_dss_unregister_device(struct omap_dss_device *);

void omap_dss_get_device(struct omap_dss_device *dssdev);
void omap_dss_put_device(struct omap_dss_device *dssdev);
#define for_each_dss_dev(d) while ((d = omap_dss_get_next_device(d)) != NULL)
struct omap_dss_device *omap_dss_get_next_device(struct omap_dss_device *from);
struct omap_dss_device *omap_dss_find_device(void *data,
		int (*match)(struct omap_dss_device *dssdev, void *data));

int omap_dss_start_device(struct omap_dss_device *dssdev);
void omap_dss_stop_device(struct omap_dss_device *dssdev);

/* the event id of the event that occurred is passed in as the second arg
 * to the notifier function, and the dssdev is passed as the third.
 */
enum omap_dss_event {
	OMAP_DSS_SIZE_CHANGE
	/* possibly add additional events, like hot-plug connect/disconnect */
};

void omap_dss_notify(struct omap_dss_device *dssdev, enum omap_dss_event evt);
void omap_dss_add_notify(struct omap_dss_device *dssdev,
			struct notifier_block *nb);
void omap_dss_remove_notify(struct omap_dss_device *dssdev,
			struct notifier_block *nb);


int omap_dss_get_num_overlay_managers(void);
struct omap_overlay_manager *omap_dss_get_overlay_manager(int num);
bool dss_ovl_manually_updated(struct omap_overlay *ovl);
static inline bool dssdev_manually_updated(struct omap_dss_device *dev)
{
	return dev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE &&
		dev->driver->get_update_mode(dev) != OMAP_DSS_UPDATE_AUTO;
}

int omap_dss_get_num_overlays(void);
struct omap_overlay *omap_dss_get_overlay(int num);
struct omap_writeback *omap_dss_get_wb(int num);

void omapdss_default_get_resolution(struct omap_dss_device *dssdev,
			u16 *xres, u16 *yres);
int omapdss_default_get_recommended_bpp(struct omap_dss_device *dssdev);
bool dispc_go_busy(enum omap_channel channel);
void dispc_go(enum omap_channel channel);
bool dispc_is_vsync_fake(void);
typedef void (*omap_dispc_isr_t) (void *arg, u32 mask);
int omap_dispc_register_isr(omap_dispc_isr_t isr, void *arg, u32 mask);
int omap_dispc_unregister_isr(omap_dispc_isr_t isr, void *arg, u32 mask);

int omap_dispc_wait_for_irq_timeout(u32 irqmask, unsigned long timeout);
int omap_dispc_wait_for_irq_interruptible_timeout(u32 irqmask,
			unsigned long timeout);

#define to_dss_driver(x) container_of((x), struct omap_dss_driver, driver)
#define to_dss_device(x) container_of((x), struct omap_dss_device, dev)

int omapdss_display_enable(struct omap_dss_device *dssdev);
void omapdss_display_disable(struct omap_dss_device *dssdev);

void omapdss_dsi_vc_enable_hs(enum omap_dsi_index ix, int channel,
			bool enable);
int omapdss_dsi_enable_te(struct omap_dss_device *dssdev, bool enable);

int omap_dsi_sched_update_lock(struct omap_dss_device *dssdev,
				u16 x, u16 y, u16 w, u16 h, bool sched_only);
int omap_dsi_prepare_update(struct omap_dss_device *dssdev,
			u16 *x, u16 *y, u16 *w, u16 *h,
			bool enlarge_update_area);
int omap_dsi_update(struct omap_dss_device *dssdev,
			int channel,
			u16 x, u16 y, u16 w, u16 h,
			void (*callback)(int, void *), void *data);

int omapdss_dsi_display_enable(struct omap_dss_device *dssdev);
void omapdss_dsi_display_disable(struct omap_dss_device *dssdev);
bool omap_dsi_recovery_state(enum omap_dsi_index ix);

int omapdss_dpi_display_enable(struct omap_dss_device *dssdev);
void omapdss_dpi_display_disable(struct omap_dss_device *dssdev);
void dpi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);
int dpi_check_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings);

int omapdss_sdi_display_enable(struct omap_dss_device *dssdev);
void omapdss_sdi_display_disable(struct omap_dss_device *dssdev);

int omapdss_rfbi_display_enable(struct omap_dss_device *dssdev);
void omapdss_rfbi_display_disable(struct omap_dss_device *dssdev);
int omap_rfbi_prepare_update(struct omap_dss_device *dssdev,
			u16 *x, u16 *y, u16 *w, u16 *h);
int omap_rfbi_update(struct omap_dss_device *dssdev,
			u16 x, u16 y, u16 w, u16 h,
			void (*callback)(void *), void *data);

void change_base_address(int id, u32 p_uv_addr);
bool is_hdmi_interlaced(void);

#endif
#endif
