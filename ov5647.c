#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-mediabus.h>
#include <asm/unaligned.h>

// #define OV5647_REG_VALUE_08BIT		1
// #define OV5647_REG_VALUE_16BIT		2

#define PWDN_ACTIVE_DELAY_MS	20

/* Chip ID */
#define OV5647_REG_CHIP_ID_HIGH		0x300A
#define OV5647_REG_CHIP_ID_LOW	    0x300B
#define OV5647_CHIP_ID_HIGH			0x56
#define OV5647_CHIP_ID_LOW	        0x47

/* i2c(SCCB) slave addr*/
#define OV5647_SCCB_SLAVE_ID        0x6c

/* External clock frequency is 25.0M */
#define OV5647_XCLK_FREQ		25000000

/* Binning, Flip, Mirror*/
#define OV5647_REG_VER_BIN_FLIP_MIR	0x3820
#define OV5647_REG_HOR_BIN_FLIP_MIR	0x3821

/* Binning  Mode */
#define OV5647_VER_BINNING_EN		0x41
#define OV5647_VER_BINNING_DISABLE	0x40
#define OV5647_HOR_BINNING_EN		0x01
#define OV5647_HOR_BINNING_DISABLE	0x00

/* Flip and Mirror Mode*/
#define OV5647_VER_FLIP_EN  		0x46
#define OV5647_VER_FLIP_DISABLE	    0x40
#define OV5647_HOR_MIRROR_EN		0x06
#define OV5647_HOR_MIRROR_DISABLE	0x00

/* Image windowing */
#define OV5647_REG_X_ADDR_START_HIGH 	0x3800
#define OV5647_REGX_ADDR_START_LOW     	0x3801
#define OV5647_REG_Y_ADDR_START_HIGH    0x3802
#define OV5647_REG_Y_ADDR_START_LOW     0x3803
#define OV5647_REG_X_ADDR_END_HIGH      0x3804
#define OV5647_REG_X_ADDR_END_LOW       0x3805
#define OV5647_REG_Y_ADDR_END_HIGH      0x3806
#define OV5647_REG_Y_ADDR_END_LOW       0x3807

/* Test Pattern Control */
#define OV5647_REG_TEST_PATT_TRANS		0x503D
#define OV5647_EN_TEST_PATTERN			0x80
#define OV5647_EN_ROLL_BAR				0x40
#define OV5647_EN_TRANSPARENT_MODE		0x20

#define OV5647_REG_TEST_PATT_ROLL		0x503E
#define OV5647_TEST_PATTERN_COLOR_BAR 	0x00
#define OV5647_TEST_PATTERN_SQUARE	 	0x01
#define OV5647_TEST_PATTERN_RANDOM_DATA 0x02
#define OV5647_TEST_PATTERN_INPUT_DATA 	0x03

/* Exposure/gain Manual Ctrl*/
#define OV5647_REG_MANUAL_CTRL	0x3503
#define OV5647_MANUAL_EXPOSURE	0x1
#define OV5647_MANUAL_GAIN		0x2

/* Exposure control */
#define OV5647_REG_EXPOSURE2	0x3500
#define OV5647_REG_EXPOSURE1	0x3501
#define OV5647_REG_EXPOSURE0	0x3502
#define OV5647_EXPOSURE_MIN		4
#define OV5647_EXPOSURE_STEP	1
#define OV5647_EXPOSURE_DEFAULT	1000
#define OV5647_EXPOSURE_MAX		65535

/* Analog gain control */
#define OV564_REG_ANALOG_GAIN1		0x350A
#define OV564_REG_ANALOG_GAIN0		0x350B
#define OV564_ANA_GAIN_MIN			0
#define OV564_ANA_GAIN_MAX			1024
#define OV564_ANA_GAIN_STEP			1
#define OV564_ANA_GAIN_DEFAULT		0x0

/* OV5647 native and active pixel array size */
#define OV5647_NATIVE_WIDTH			2624U
#define OV5647_NATIVE_HEIGHT		1956U

#define OV5647_PIXEL_ARRAY_LEFT		16U
#define OV5647_PIXEL_ARRAY_TOP		6U
#define OV5647_PIXEL_ARRAY_WIDTH	2592U
#define OV5647_PIXEL_ARRAY_HEIGHT	1944U

/* MIPI Ctrl*/
#define OV5647_REG_MIPI_FRAME_OFF_NUMBER	0x4202
#define OV5647_REG_MIPI_CTRL14			0x4814
#define OV5647_REG_MIPI_AWB				0x5001
#define OV5647_REG_MIPI_CTRL00			0x4800

#define MIPI_CTRL00_CLOCK_LANE_GATE		BIT(5)
#define MIPI_CTRL00_LINE_SYNC_ENABLE	BIT(4)
#define MIPI_CTRL00_BUS_IDLE			BIT(2)
#define MIPI_CTRL00_CLOCK_LANE_DISABLE	BIT(0)

#define OV5647_DEFAULT_LINK_FREQ 297000000

/* regulator supplies */
static const char * const ov5647_supply_name[] = {
	"dovdd",
	"avdd",  
	"dvdd"
};

#define OV5647_NUM_SUPPLIES ARRAY_SIZE(ov5647_supply_name)

/*Average luminace */
struct ov5647_reg {
	uint16_t address;
	uint8_t val;
};

struct ov5647_reg_list {
	unsigned int num_of_regs;
	const struct ov5647_reg *regs;
};

enum binning_mode {
	BINNING_NONE,
	BINNING_VER,
	BINNING_HOR,
    BINNING_BOTH,
};

/* Mode : resolution and related config&values */
struct ov5647_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	uint64_t pixel_rate;

	/* V-timing */
	unsigned int vts_def;
	unsigned int hts_def;

	/* Default register values */
	struct ov5647_reg_list reg_list;

	/* binning mode based on format code */
	enum binning_mode binning;
};

struct ov5647 {
	struct v4l2_subdev sd;

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to IMX219 */
	uint32_t xclk_freq;

	struct gpio_desc *pwr_gpio;
	struct regulator_bulk_data supplies[OV5647_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct ov5647_mode *mode;

    /*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

    /* Streaming on/off */
	bool streaming;
};

static const struct ov5647_reg  sensor_oe_disable_regs[] = {
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
};

static const struct ov5647_reg  sensor_oe_enable_regs[] = {
	{0x3000, 0x0f},
	{0x3001, 0xff},
	{0x3002, 0xe4},
};

// modes 
// 2592 x 1944 15fps
static struct ov5647_reg ov5647_2592x1944_10bpp[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x69},
	{0x303c, 0x11},
	{0x3106, 0xf5},
	{0x3821, 0x00},
	{0x3820, 0x00},
	{0x3827, 0xec},
	{0x370c, 0x03},
	{0x3612, 0x5b},
	{0x3618, 0x04},
	{0x5000, 0x06},
	{0x5002, 0x41},
	{0x5003, 0x08},
	{0x5a00, 0x08},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3016, 0x08},
	{0x3017, 0xe0},
	{0x3018, 0x44},
	{0x301c, 0xf8},
	{0x301d, 0xf0},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3c01, 0x80},
	{0x3b07, 0x0c},
	{0x380c, 0x0b},
	{0x380d, 0x1c},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3708, 0x64},
	{0x3709, 0x12},
	{0x3808, 0x0a},
	{0x3809, 0x20},
	{0x380a, 0x07},
	{0x380b, 0x98},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0xa3},
	{0x3811, 0x10},
	{0x3813, 0x06},
	{0x3630, 0x2e},
	{0x3632, 0xe2},
	{0x3633, 0x23},
	{0x3634, 0x44},
	{0x3636, 0x06},
	{0x3620, 0x64},
	{0x3621, 0xe0},
	{0x3600, 0x37},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x3731, 0x02},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3f05, 0x02},
	{0x3f06, 0x10},
	{0x3f01, 0x0a},
	{0x3a08, 0x01},
	{0x3a09, 0x28},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0d, 0x08},
	{0x3a0e, 0x06},
	{0x3a0f, 0x58},
	{0x3a10, 0x50},
	{0x3a1b, 0x58},
	{0x3a1e, 0x50},
	{0x3a11, 0x60},
	{0x3a1f, 0x28},
	{0x4001, 0x02},
	{0x4004, 0x04},
	{0x4000, 0x09},
	{0x4837, 0x19},
	{0x4800, 0x24},
	{0x3503, 0x03},
	{0x0100, 0x01},
};

// 1080p 30fps
static struct ov5647_reg ov5647_1080p30_10bpp[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x62},
	{0x303c, 0x11},
	{0x3106, 0xf5},
	{0x3821, 0x00},
	{0x3820, 0x00},
	{0x3827, 0xec},
	{0x370c, 0x03},
	{0x3612, 0x5b},
	{0x3618, 0x04},
	{0x5000, 0x06},
	{0x5002, 0x41},
	{0x5003, 0x08},
	{0x5a00, 0x08},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3016, 0x08},
	{0x3017, 0xe0},
	{0x3018, 0x44},
	{0x301c, 0xf8},
	{0x301d, 0xf0},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3c01, 0x80},
	{0x3b07, 0x0c},
	{0x380c, 0x09},
	{0x380d, 0x70},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3708, 0x64},
	{0x3709, 0x12},
	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x3800, 0x01},
	{0x3801, 0x5c},
	{0x3802, 0x01},
	{0x3803, 0xb2},
	{0x3804, 0x08},
	{0x3805, 0xe3},
	{0x3806, 0x05},
	{0x3807, 0xf1},
	{0x3811, 0x04},
	{0x3813, 0x02},
	{0x3630, 0x2e},
	{0x3632, 0xe2},
	{0x3633, 0x23},
	{0x3634, 0x44},
	{0x3636, 0x06},
	{0x3620, 0x64},
	{0x3621, 0xe0},
	{0x3600, 0x37},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x3731, 0x02},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3f05, 0x02},
	{0x3f06, 0x10},
	{0x3f01, 0x0a},
	{0x3a08, 0x01},
	{0x3a09, 0x4b},
	{0x3a0a, 0x01},
	{0x3a0b, 0x13},
	{0x3a0d, 0x04},
	{0x3a0e, 0x03},
	{0x3a0f, 0x58},
	{0x3a10, 0x50},
	{0x3a1b, 0x58},
	{0x3a1e, 0x50},
	{0x3a11, 0x60},
	{0x3a1f, 0x28},
	{0x4001, 0x02},
	{0x4004, 0x04},
	{0x4000, 0x09},
	{0x4837, 0x19},
	{0x4800, 0x34},
	{0x3503, 0x03},
	{0x0100, 0x01},
};

static struct ov5647_reg ov5647_2x2binned_10bpp[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x62},
	{0x303c, 0x11},
	{0x3106, 0xf5},
	{0x3827, 0xec},
	{0x370c, 0x03},
	{0x3612, 0x59},
	{0x3618, 0x00},
	{0x5000, 0x06},
	{0x5002, 0x41},
	{0x5003, 0x08},
	{0x5a00, 0x08},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3016, 0x08},
	{0x3017, 0xe0},
	{0x3018, 0x44},
	{0x301c, 0xf8},
	{0x301d, 0xf0},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3c01, 0x80},
	{0x3b07, 0x0c},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0xa3},
	{0x3808, 0x05},
	{0x3809, 0x10},
	{0x380a, 0x03},
	{0x380b, 0xcc},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x3811, 0x0c},
	{0x3813, 0x06},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3630, 0x2e},
	{0x3632, 0xe2},
	{0x3633, 0x23},
	{0x3634, 0x44},
	{0x3636, 0x06},
	{0x3620, 0x64},
	{0x3621, 0xe0},
	{0x3600, 0x37},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x3731, 0x02},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3f05, 0x02},
	{0x3f06, 0x10},
	{0x3f01, 0x0a},
	{0x3a08, 0x01},
	{0x3a09, 0x28},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0d, 0x08},
	{0x3a0e, 0x06},
	{0x3a0f, 0x58},
	{0x3a10, 0x50},
	{0x3a1b, 0x58},
	{0x3a1e, 0x50},
	{0x3a11, 0x60},
	{0x3a1f, 0x28},
	{0x4001, 0x02},
	{0x4004, 0x04},
	{0x4000, 0x09},
	{0x4837, 0x16},
	{0x4800, 0x24},
	{0x3503, 0x03},
	{0x3820, 0x41},
	{0x3821, 0x01},
	{0x350a, 0x00},
	{0x350b, 0x10},
	{0x3500, 0x00},
	{0x3501, 0x1a},
	{0x3502, 0xf0},
	{0x3212, 0xa0},
	{0x0100, 0x01},
};

static struct ov5647_reg ov5647_640x480_10bpp[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x3035, 0x11},
	{0x3036, 0x46},
	{0x303c, 0x11},
	{0x3821, 0x01},
	{0x3820, 0x41},
	{0x370c, 0x03},
	{0x3612, 0x59},
	{0x3618, 0x00},
	{0x5000, 0x06},
	{0x5003, 0x08},
	{0x5a00, 0x08},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xff},
	{0x301d, 0xf0},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3c01, 0x80},
	{0x3b07, 0x0c},
	{0x380c, 0x07},
	{0x380d, 0x3c},
	{0x3814, 0x35},
	{0x3815, 0x35},
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x3800, 0x00},
	{0x3801, 0x10},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x2f},
	{0x3806, 0x07},
	{0x3807, 0x9f},
	{0x3630, 0x2e},
	{0x3632, 0xe2},
	{0x3633, 0x23},
	{0x3634, 0x44},
	{0x3620, 0x64},
	{0x3621, 0xe0},
	{0x3600, 0x37},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x3731, 0x02},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3f05, 0x02},
	{0x3f06, 0x10},
	{0x3f01, 0x0a},
	{0x3a08, 0x01},
	{0x3a09, 0x2e},
	{0x3a0a, 0x00},
	{0x3a0b, 0xfb},
	{0x3a0d, 0x02},
	{0x3a0e, 0x01},
	{0x3a0f, 0x58},
	{0x3a10, 0x50},
	{0x3a1b, 0x58},
	{0x3a1e, 0x50},
	{0x3a11, 0x60},
	{0x3a1f, 0x28},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x4000, 0x09},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3017, 0xe0},
	{0x301c, 0xfc},
	{0x3636, 0x06},
	{0x3016, 0x08},
	{0x3827, 0xec},
	{0x3018, 0x44},
	{0x3035, 0x21},
	{0x3106, 0xf5},
	{0x3034, 0x1a},
	{0x301c, 0xf8},
	{0x4800, 0x34},
	{0x3503, 0x03},
	{0x0100, 0x01},
};

/* Mode configs */
static const struct ov5647_mode supported_modes[] = {
	/* 2592x1944 full resolution full FOV 10-bit mode. */
	{
		.width 		= 2592,
		.height 	= 1944,
		.crop = {
			.left		= OV5647_PIXEL_ARRAY_LEFT,
			.top		= OV5647_PIXEL_ARRAY_TOP,
			.width		= OV5647_PIXEL_ARRAY_WIDTH,
			.height		= OV5647_PIXEL_ARRAY_HEIGHT
		},
		.pixel_rate	= 87500000,
		.hts_def		= 2844,
		.vts_def		= 0x7b0,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(ov5647_2592x1944_10bpp),
			.regs = ov5647_2592x1944_10bpp,
		},
		.binning = BINNING_NONE
	},
	/* 1080p30 10-bit mode. Full resolution centre-cropped down to 1080p. */
	{
		.width		= 1920,
		.height		= 1080,
		.crop = {
			.left		= 348 + OV5647_PIXEL_ARRAY_LEFT,
			.top		= 434 + OV5647_PIXEL_ARRAY_TOP,
			.width		= 1928,
			.height		= 1080,
		},
		.pixel_rate	= 81666700,
		.hts_def		= 2416,
		.vts_def		= 0x450,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(ov5647_1080p30_10bpp),
			.regs = ov5647_1080p30_10bpp,
		},
		.binning = BINNING_NONE
	}, 
	/* 2x2 binned full FOV 10-bit mode. */
	{
		.width		= 1296,
		.height		= 972,
		.crop = {
			.left		= OV5647_PIXEL_ARRAY_LEFT,
			.top		= OV5647_PIXEL_ARRAY_TOP,
			.width		= OV5647_PIXEL_ARRAY_WIDTH,
			.height		= OV5647_PIXEL_ARRAY_HEIGHT
		},
		.pixel_rate	= 81666700,
		.hts_def		= 1896,
		.vts_def		= 0x59b,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(ov5647_2x2binned_10bpp),
			.regs = ov5647_2x2binned_10bpp,
		},
		.binning = BINNING_BOTH
	},
	/* 10-bit VGA full FOV 60fps. 2x2 binned and subsampled down to VGA. */
	{
		.width		= 640,
		.height		= 480,
		.crop = {
			.left		= 16 + OV5647_PIXEL_ARRAY_LEFT,
			.top		= OV5647_PIXEL_ARRAY_TOP,
			.width		= 2560,
			.height		= 1920,
		},
		.pixel_rate	= 55000000,
		.hts_def		= 1852,
		.vts_def		= 0x1f8,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(ov5647_640x480_10bpp),
			.regs = ov5647_640x480_10bpp,
		},
		.binning = BINNING_NONE
	}
};

static int ov5647_read_reg_8bit(struct ov5647 *ov5647, uint16_t reg, uint8_t *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5647->sd);
	struct i2c_msg msgs[2];
	uint8_t addr_buf[2] = { reg >> 8, reg & 0xff };
	int ret;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = val;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;
	return 0;
}

static int ov5647_write_reg_8bit(struct ov5647 *ov5647, uint16_t reg, uint8_t val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5647->sd);
	uint8_t buf[3] = { reg >> 8, reg & 0xff, val};;

	if (i2c_master_send(client, buf, 3) != 3)
		printk("error in write reg 8 bit");
		return -EINVAL;

	return 0;
}

static int ov5647_write_regs(struct ov5647 *ov5647, const struct ov5647_reg *regs, u32 len) 
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5647->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		printk("attempting to write %d to reg 0x%4.4x.", regs[i].val, regs[i].address);
		ret = ov5647_write_reg_8bit(ov5647, regs[i].address, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

static inline struct ov5647 *to_ov5647(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct ov5647, sd);
}

/* Verify chip ID */
static int ov5647_identify_module(struct ov5647 *ov5647)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5647->sd);
	int ret;
	uint8_t val_high, val_low;

	printk("ov5647_identify_module::clinet addres :  %d", client->addr);
	ret = ov5647_read_reg_8bit(ov5647, (uint16_t) OV5647_REG_CHIP_ID_HIGH, &val_high);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x\n",
			OV5647_CHIP_ID_HIGH);
		return ret;
	}

    ret = ov5647_read_reg_8bit(ov5647, (uint16_t) OV5647_REG_CHIP_ID_LOW, &val_low);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x\n",
			OV5647_CHIP_ID_LOW);
		return ret;
	}

	if (val_high != OV5647_CHIP_ID_HIGH ) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			OV5647_CHIP_ID_HIGH, val_high);
		return -EIO;
	}

    if (val_low != OV5647_CHIP_ID_LOW ) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			OV5647_CHIP_ID_LOW, val_low);
		return -EIO;
	}
	return 0;
}

//----- TODO --------
static int ov5647_set_binning(struct ov5647 *ov5647)
{
	enum binning_mode binning = BINNING_NONE;
	// int ret = imx219_resolve_binning(imx219, &binning); // TODO implement this
	// if (ret < 0)
	// 	return ret;
    
	switch (binning) {
        case BINNING_NONE:
            return ov5647_write_reg_8bit(ov5647, 
                    OV5647_REG_VER_BIN_FLIP_MIR, OV5647_VER_BINNING_DISABLE) 
                + ov5647_write_reg_8bit(ov5647, 
                    OV5647_REG_HOR_BIN_FLIP_MIR, OV5647_HOR_BINNING_DISABLE);
        case BINNING_BOTH:
            return ov5647_write_reg_8bit(ov5647, 
                    OV5647_REG_VER_BIN_FLIP_MIR, OV5647_VER_BINNING_EN);  
                + ov5647_write_reg_8bit(ov5647, 
                    OV5647_REG_HOR_BIN_FLIP_MIR, OV5647_HOR_BINNING_EN);
        case BINNING_VER:
            return ov5647_write_reg_8bit(ov5647, 
                    OV5647_REG_VER_BIN_FLIP_MIR, OV5647_VER_BINNING_EN);  
        case BINNING_HOR:
            return ov5647_write_reg_8bit(ov5647, 
                    OV5647_REG_HOR_BIN_FLIP_MIR, OV5647_HOR_BINNING_EN);
	}
	return -EINVAL;
}

// static int ov5647_start_streaming(struct ov5647 *ov5647)
// {

// }

// static void ov5647_stop_streaming(struct ov5647 *ov5647)
// {
// 	// todo
// }

static int ov5647_set_stream(struct v4l2_subdev *sd, int enable) {
	struct ov5647 *ov5647 = to_ov5647(sd);
	int ret = 0;
	mutex_lock(&ov5647->mutex);
	if (ov5647->streaming == enable) {
		mutex_unlock(&ov5647->mutex);
		return 0;
	}

	printk(" this is the ov5647_set_stream, to be implemented");

	// if (enable) {
	// 	/*
	// 	 * Apply default & customized values
	// 	 * and then start streaming.
	// 	 */
	// 	if (ov5647_start_streaming(ov5647))
	// 		goto err_unlock;
	// } else {
	// 	ov5647_stop_streaming(ov5647);
	// }
	// ov5647->streaming = enable;

// err_unlock: // TODO enable later
	mutex_unlock(&ov5647->mutex);
	return ret;
}

//---------------------------

static int ov5647_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov5647 *ov5647 = to_ov5647(sd);
	struct v4l2_mbus_framefmt *try_fmt_img;
	struct v4l2_rect *try_crop;

	printk("ov5647_open");

	mutex_lock(&ov5647->mutex);

	/* Initialize try_fmt */
	try_fmt_img = v4l2_subdev_get_try_format(sd, fh->state, 0);
	try_fmt_img->width = supported_modes[0].width;
	try_fmt_img->height = supported_modes[0].height;
	try_fmt_img->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	try_fmt_img->colorspace = V4L2_COLORSPACE_RAW;
	try_fmt_img->field = V4L2_FIELD_NONE;

	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
	try_crop->top = OV5647_PIXEL_ARRAY_TOP;
	try_crop->left = OV5647_PIXEL_ARRAY_LEFT;
	try_crop->width = OV5647_PIXEL_ARRAY_WIDTH;
	try_crop->height = OV5647_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&ov5647->mutex);

	return 0;
}

static void free_controls(struct ov5647 *ov5647)
{
	v4l2_ctrl_handler_free(ov5647->sd.ctrl_handler);
	mutex_destroy(&ov5647->mutex);
}

static int check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}
	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != OV5647_DEFAULT_LINK_FREQ) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int get_regulators(struct ov5647 *ov5647)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5647->sd);
	unsigned int i;

	for (i = 0; i < OV5647_NUM_SUPPLIES; i++) {
		ov5647->supplies[i].supply = ov5647_supply_name[i];
	}

	printk("get_regulators::devm_regulator_bulk_get");
	return devm_regulator_bulk_get(&client->dev,
					OV5647_NUM_SUPPLIES,
					ov5647->supplies);
}

/* Power/clock management functions */
static int power_on(struct device *dev)
{
	struct ov5647 *ov5647 = dev_get_drvdata(dev);
	int ret;

	ret = regulator_bulk_enable(OV5647_NUM_SUPPLIES,
				    ov5647->supplies);
	if (ret) {
		printk( "%s: failed to enable regulators\n", __func__);
		return ret;
	}

	if (ov5647->pwr_gpio) {
		gpiod_set_value_cansleep(ov5647->pwr_gpio, 0);
		msleep(PWDN_ACTIVE_DELAY_MS);
	}

	ret = clk_prepare_enable(ov5647->xclk);
	if (ret) {
		printk("%s: failed to enable clock\n", __func__);
		goto reg_off;
	}

	ret = ov5647_write_regs(ov5647, sensor_oe_enable_regs,
				 ARRAY_SIZE(sensor_oe_enable_regs));
	if (ret < 0) {
		printk( "write sensor_oe_enable_regs error\n");
		goto reg_off;
	}

	return 0;

reg_off:
	gpiod_set_value_cansleep(ov5647->pwr_gpio, 1);
	regulator_bulk_disable(OV5647_NUM_SUPPLIES, ov5647->supplies);

	ret = ov5647_write_regs(ov5647, sensor_oe_disable_regs,
				 ARRAY_SIZE(sensor_oe_disable_regs));
	if (ret < 0) {
		printk( "write sensor_oe_disable_regs error\n");
	}
	clk_disable_unprepare(ov5647->xclk);

	return ret;
}

static int power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov5647 *ov5647 = to_ov5647(sd);

	gpiod_set_value_cansleep(ov5647->pwr_gpio, 1);
	regulator_bulk_disable(OV5647_NUM_SUPPLIES, ov5647->supplies);
	clk_disable_unprepare(ov5647->xclk);

	return 0;
}

static void set_default_format(struct ov5647 *ov5647)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &ov5647->fmt;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = supported_modes[0].width;
	fmt->height = supported_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int get_rate_factor(struct ov5647 *ov5647)
{
	switch (ov5647->mode->binning) {
		case BINNING_NONE:
		case BINNING_VER:
			return 1;
		case BINNING_HOR:
			return 2;
		case BINNING_BOTH:
			return 3;
	}
	return -EINVAL;
}

/* Initialize control handlers */
static int init_controls(struct ov5647 *ov5647)
{
	// struct i2c_client *client = v4l2_get_subdevdata(&ov5647->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	// unsigned int height = ov5647->mode->height;
	// struct v4l2_fwnode_device_properties props;
	// int exposure_max, exposure_def, hblank, rate_factor, pixel_rate;
	// int i;
	int ret;
	int rate_factor;

	ctrl_hdlr = &ov5647->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 12);
	if (ret)
		return ret;

	printk("init_controls:: mutex_init ");
	mutex_init(&ov5647->mutex);
	ctrl_hdlr->lock = &ov5647->mutex;

	rate_factor = get_rate_factor(ov5647);
	if (rate_factor < 0)
		return rate_factor;

	// TODO : complete init controls

	// TODO : move it to set_ctrl
	// switch (ctrl->id) {
	// 	case V4L2_CID_EXPOSURE:
	// 		ret = imx219_write_reg(imx219, ov5647_write_reg_8bit,
	// 			IMX219_REG_VALUE_16BIT,
	// 			ctrl->val / rate_factor);
	// 	default:
	// 		dev_info(&client->dev,
	// 			"ctrl(id:0x%x,val:0x%x) is not handled\n",
	// 			ctrl->id, ctrl->val);
	// 		ret = -EINVAL;
	// 		break;
	// }

	// pm_runtime_put(&client->dev);

	// return ret;

	ov5647->sd.ctrl_handler = ctrl_hdlr;
	return 0;

// error: //TODO enable
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&ov5647->mutex);

	return ret;

}

//-------------------------------------

static const struct v4l2_subdev_core_ops core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops video_ops = {
	.s_stream = ov5647_set_stream,
};

static const struct v4l2_subdev_pad_ops pad_ops = {
	// .enum_mbus_code = ov5647_enum_mbus_code,
	// .get_fmt = ov5647_get_pad_format,
	// .set_fmt = ov5647_set_pad_format,
	// .get_selection = ov5647_get_selection,
	// .enum_frame_size = ov5647_enum_frame_size,
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
	.video = &video_ops,
	.pad = &pad_ops,
};

static const struct v4l2_subdev_internal_ops ov5647_internal_ops = {
	.open = ov5647_open,
};

//-------------------------------------

static int ov5647_probe(struct i2c_client *client)
{
	struct device* dev;
	struct ov5647* ov5647;
	int ret;

	printk("ov5647_probe");
	ret = 0;
	dev = &client->dev;

	printk("ov5647_probe:: devm_kzalloc ");
	ov5647 = devm_kzalloc(dev, sizeof(*ov5647), GFP_KERNEL);
	if (!ov5647)
		return -ENOMEM;

	/* Initialize subdev */
	printk("ov5647_probe:: Initialize subdev ");
	v4l2_i2c_subdev_init(&ov5647->sd, client, &subdev_ops);
	ov5647->sd.internal_ops = &ov5647_internal_ops;
	ov5647->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	ov5647->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Check the hardware configuration in device tree */
	printk("ov5647_probe:: check_hwcfg ");
	if (check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
	printk("ov5647_probe:: devm_clk_get ");
	ov5647->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(ov5647->xclk)) {
		printk( "ov5647_probe::failed to get xclk\n");
		return PTR_ERR(ov5647->xclk);
	}
	/* Get clk rate freq (xclk_freq)*/
	printk("ov5647_probe:: clk_get_rate ");
	ov5647->xclk_freq = clk_get_rate(ov5647->xclk);
	if (ov5647->xclk_freq != OV5647_XCLK_FREQ) {
		printk("ov5647_probe::xclk frequency not supported: %d Hz\n",
			ov5647->xclk_freq);
		return -EINVAL;
	}

	/* Request optional pwr pin */
	printk("ov5647_probe:: devm_gpiod_get_optional ");
	ov5647->pwr_gpio = devm_gpiod_get_optional(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(ov5647->pwr_gpio)) {
		printk("ov5647_probe :: error init the gpio pwr ");
		return -EINVAL;
	}

	printk("ov5647_probe:: get_regulators 2");
	ret = get_regulators(ov5647); 
	if (ret) {
		printk("ov5647_probe::failed to get regulators\n");
		return ret;
	}

	/* Set default mode to max resolution */
	printk("ov5647_probe::Set default mode");
	ov5647->mode = &supported_modes[0];

	printk("ov5647_probe::init_controls");
	ret = init_controls(ov5647);
	if (ret)
		goto error_power_off;

	ret = power_on(dev);
	if (ret)
		return ret;

	// ret = ov5647_identify_module(ov5647);
	// if (ret)
	// 	goto error_power_off;


// 	/* sensor doesn't enter LP-11 state upon power up until and unless
// 	 * streaming is started, so upon power up switch the modes to:
// 	 * streaming -> standby
// 	 */
// 	ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_MIPI_CTRL00, MIPI_CTRL00_CLOCK_LANE_GATE 
// 									| MIPI_CTRL00_BUS_IDLE | MIPI_CTRL00_CLOCK_LANE_DISABLE);
// 	if (ret < 0)
// 		goto error_power_off;

// 	/* Initialize default format */
// 	set_default_format(ov5647);

// 	/* Initialize source pad */
// 	// TODO

// 	ret = v4l2_async_register_subdev_sensor(&ov5647->sd);
// 	if (ret < 0) {
// 		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
// 		goto error_media_entity;
// 	}

// 	/* Enable runtime PM and turn off the device */
// 	pm_runtime_set_active(dev);
// 	pm_runtime_enable(dev);
// 	pm_runtime_idle(dev);

// 	return 0;

// error_media_entity:
// 	media_entity_cleanup(&ov5647->sd.entity);

// // error_handler_free:
// 	free_controls(ov5647);

error_power_off:
// 	power_off(dev);

	return ret;
}


static int ov5647_parse_dt(struct ov5647 *sensor, struct device_node *np)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct device_node *ep;
	int ret;

	ep = of_graph_get_next_endpoint(np, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &bus_cfg);
	if (ret)
		goto out;

out:
	of_node_put(ep);

	return ret;
}

static int ov5647_configure_regulators(struct device *dev,
				       struct ov5647 *sensor)
{
	unsigned int i;

	for (i = 0; i < OV5647_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = ov5647_supply_name[i];

	return devm_regulator_bulk_get(dev, OV5647_NUM_SUPPLIES,
				       sensor->supplies);
}

static int ov5647_power_on(struct device *dev)
{
	struct ov5647 *sensor = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "OV5647 power on\n");

	ret = regulator_bulk_enable(OV5647_NUM_SUPPLIES, sensor->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		return ret;
	}

	if (sensor->pwr_gpio) {
		gpiod_set_value_cansleep(sensor->pwr_gpio, 0);
		msleep(PWDN_ACTIVE_DELAY_MS);
	}

	return ret;
}

static int ov5647_probe2(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct device *dev = &client->dev;
	struct ov5647 *sensor;
	struct v4l2_subdev *sd;
	u32 xclk_freq;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF) && np) {
		ret = ov5647_parse_dt(sensor, np);
		if (ret) {
			printk( "DT parsing error: %d\n", ret);
			return ret;
		}
	}

	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk)) {
		printk( "could not get xclk");
		return PTR_ERR(sensor->xclk);
	}

	xclk_freq = clk_get_rate(sensor->xclk);
	if (xclk_freq != 25000000) {
		printk( "Unsupported clock frequency: %u\n", xclk_freq);
		return -EINVAL;
	}

	/* Request the power down GPIO asserted. */
	sensor->pwr_gpio = devm_gpiod_get_optional(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->pwr_gpio)) {
		printk( "Failed to get 'pwdn' gpio\n");
		return -EINVAL;
	}

	ret = ov5647_configure_regulators(dev, sensor);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sensor->mutex);

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &subdev_ops);
	sd->internal_ops = &ov5647_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = ov5647_power_on(dev);
	if (ret)
		return -1;
	
	return 0;
}

//---------------------------

static const struct of_device_id ov5647_dt_ids[] = {
	{ .compatible = "ovti,ov5647" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov5647_dt_ids);

// static const struct dev_pm_ops imx219_pm_ops = {
// 	SET_SYSTEM_SLEEP_PM_OPS(imx219_suspend, imx219_resume)
// 	SET_RUNTIME_PM_OPS(imx219_power_off, imx219_power_on, NULL)
// };

static struct i2c_driver ov5647_i2c_driver = {
	.driver = {
		.name = "ov5647",
		.of_match_table	= ov5647_dt_ids,
		// .pm = &imx219_pm_ops,
	},
	.probe_new = ov5647_probe,
	// .remove = imx219_remove,
};

module_i2c_driver(ov5647_i2c_driver);

MODULE_AUTHOR("Mohamed Mahmoud <mohamednabil940@gmail.com>");
MODULE_DESCRIPTION("ov5647 sensor driver");
MODULE_LICENSE("GPL v2");