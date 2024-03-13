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

#define OV5647_SW_STANDBY		0x0100
#define OV5647_SW_RESET			0x0103

/* Chip ID */
#define OV5647_REG_CHIP_ID_HIGH		0x300A
#define OV5647_REG_CHIP_ID_LOW	    0x300B
#define OV5647_CHIP_ID_HIGH			0x56
#define OV5647_CHIP_ID_LOW	        0x47

/* i2c(SCCB) slave addr*/
#define OV5647_SCCB_SLAVE_ID        0x6c

/* External clock frequency is 25.0M */
#define OV5647_XCLK_FREQ		25000000

#define OV5647_REG_VTS_HI		0x380e
#define OV5647_REG_VTS_LO		0x380f

#define OV5647_REG_GAIN_HI		0x350a
#define OV5647_REG_GAIN_LO		0x350b

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

#define OV5647_VBLANK_MIN		24
#define OV5647_VTS_MAX			32767

/* Analog gain control */
#define OV564_REG_ANALOG_GAIN1		0x350A
#define OV564_REG_ANALOG_GAIN0		0x350B
#define OV564_ANA_GAIN_MIN			16
#define OV564_ANA_GAIN_MAX			1023
#define OV564_ANA_GAIN_STEP			1
#define OV564_ANA_GAIN_DEFAULT		32

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
#define OV5647_REG_FRAME_OFF_NUMBER		0x4202
#define OV5640_REG_PAD_OUT				0x300d

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
	struct v4l2_subdev 			sd;
	struct media_pad			pad;
	struct v4l2_mbus_framefmt 	fmt;

	struct clk *xclk; /* system clock to ov5647 */
	uint32_t xclk_freq;

	struct gpio_desc *pwr_gpio;
	struct regulator_bulk_data supplies[OV5647_NUM_SUPPLIES];
	bool clock_ncont;

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

static const int64_t ov5647_link_freq_menu[] = {
	OV5647_DEFAULT_LINK_FREQ,
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

	if (i2c_master_send(client, buf, 3) != 3) {
		printk("error in write reg 8 bit");
		return -EINVAL;
	}

	return 0;
}

static int ov5647_write_regs(struct ov5647 *ov5647, const struct ov5647_reg *regs, int len) 
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5647->sd);
	unsigned int i;
	int ret;

	printk("ov5647_write_regs:: len : %d", len);
	for (i = 0; i < len; i++) {
		printk("%d : attempting to write %d to reg 0x%4.4x.", i, regs[i].val, regs[i].address);
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
	} else {
		printk("ov5647 :: chip is high match");
	}

    if (val_low != OV5647_CHIP_ID_LOW ) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			OV5647_CHIP_ID_LOW, val_low);
		return -EIO;
	} else {
		printk("ov5647 :: chip is low match");
	}
	return 0;
}

// //----- TODO --------
// static int set_binning(struct ov5647 *ov5647)
// {
// 	enum binning_mode binning = BINNING_NONE;
// 	// int ret = imx219_resolve_binning(ov5647, &binning); // TODO implement this
// 	// if (ret < 0)
// 	// 	return ret;
    
// 	switch (binning) {
//         case BINNING_NONE:
//             return ov5647_write_reg_8bit(ov5647, 
//                     OV5647_REG_VER_BIN_FLIP_MIR, OV5647_VER_BINNING_DISABLE) 
//                 + ov5647_write_reg_8bit(ov5647, 
//                     OV5647_REG_HOR_BIN_FLIP_MIR, OV5647_HOR_BINNING_DISABLE);
//         case BINNING_BOTH:
//             return ov5647_write_reg_8bit(ov5647, 
//                     OV5647_REG_VER_BIN_FLIP_MIR, OV5647_VER_BINNING_EN);  
//                 + ov5647_write_reg_8bit(ov5647, 
//                     OV5647_REG_HOR_BIN_FLIP_MIR, OV5647_HOR_BINNING_EN);
//         case BINNING_VER:
//             return ov5647_write_reg_8bit(ov5647, 
//                     OV5647_REG_VER_BIN_FLIP_MIR, OV5647_VER_BINNING_EN);  
//         case BINNING_HOR:
//             return ov5647_write_reg_8bit(ov5647, 
//                     OV5647_REG_HOR_BIN_FLIP_MIR, OV5647_HOR_BINNING_EN);
// 	}
// 	return -EINVAL;
// }

static int ov5647_set_virtual_channel(struct ov5647 *ov5647, int channel)
{
	u8 channel_id;
	int ret;

	ret = ov5647_read_reg_8bit(ov5647, OV5647_REG_MIPI_CTRL14, &channel_id);
	if (ret < 0)
		return ret;

	channel_id &= ~(3 << 6);

	return ov5647_write_reg_8bit(ov5647, OV5647_REG_MIPI_CTRL14,
			    							channel_id | (channel << 6));
}

static int ov5647_start_streaming(struct ov5647 *ov5647)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5647->sd);
	const struct ov5647_reg_list *reg_list;
	uint8_t val = MIPI_CTRL00_BUS_IDLE;
	int ret;

	printk(" ov5647_start_streaming");

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	/* Apply default values of current mode */
	reg_list = &ov5647->mode->reg_list;
	ret = ov5647_write_regs(ov5647, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		printk( "%s failed to set mode\n", __func__);
		goto err_rpm_put;
	}

	ret = ov5647_set_virtual_channel(ov5647, 0);
	if (ret < 0)
		return ret;

	/* set stream on register */
	printk("Device is set in SW standby");
	ret = ov5647_write_reg_8bit(ov5647, OV5647_SW_STANDBY, 0x01);
	if (ret)
		goto err_rpm_put;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(ov5647->vflip, true);
	__v4l2_ctrl_grab(ov5647->hflip, true);

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(ov5647->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;

	if (ov5647->clock_ncont) 
		val |= MIPI_CTRL00_CLOCK_LANE_GATE | MIPI_CTRL00_LINE_SYNC_ENABLE;

	ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_MIPI_CTRL00, val);
	if (ret < 0)
		return ret;

	ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_FRAME_OFF_NUMBER, 0x00);
	if (ret < 0)
		return ret;

	ret = ov5647_write_reg_8bit(ov5647, OV5640_REG_PAD_OUT, 0x00);
	if (ret < 0)
		return ret;

	printk(" ov5647_start_streaming :: return 0");
	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void ov5647_stop_streaming(struct ov5647 *ov5647)
{
	printk(" ov5647_stop_streaming");
	return;
}

static int ov5647_set_stream(struct v4l2_subdev *sd, int enable) 
{

	struct ov5647 *ov5647 = to_ov5647(sd);
	int ret ;

	printk(" ov5647_set_stream starting : %d", enable); 
	mutex_lock(&ov5647->mutex);
	if (ov5647->streaming == enable) {
		mutex_unlock(&ov5647->mutex);
		return 0;
	}

	if (enable) {
		if (ov5647_start_streaming(ov5647))
			goto err_unlock;
	} else {
		ov5647_stop_streaming(ov5647);
	}
	ov5647->streaming = enable;

err_unlock:
	mutex_unlock(&ov5647->mutex);
	printk(" ov5647_set_stream:: return 0");
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
	try_fmt_img->width = supported_modes[3].width;
	try_fmt_img->height = supported_modes[3].height;
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

static int get_regulators(struct ov5647 *ov5647, struct device *dev)
{
	unsigned int i;

	for (i = 0; i < OV5647_NUM_SUPPLIES; i++) {
		ov5647->supplies[i].supply = ov5647_supply_name[i];
	}

	printk("get_regulators::devm_regulator_bulk_get");
	return devm_regulator_bulk_get(dev,
					OV5647_NUM_SUPPLIES,
					ov5647->supplies);
}

/* Power/clock management functions */
static int power_on(struct device *dev)
{
	struct ov5647 *ov5647 = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "OV5647 power on\n");

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

	msleep(PWDN_ACTIVE_DELAY_MS);

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
	fmt->width = supported_modes[3].width;
	fmt->height = supported_modes[3].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5647 *ov5647 = container_of(ctrl->handler, struct ov5647, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ov5647->sd);
	int ret;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = ov5647->mode->height + ctrl->val - 4;
		exposure_def = (exposure_max < OV5647_EXPOSURE_DEFAULT) ?
			exposure_max : OV5647_EXPOSURE_DEFAULT;
		__v4l2_ctrl_modify_range(ov5647->exposure,
					 ov5647->exposure->minimum,
					 exposure_max, ov5647->exposure->step,
					 exposure_def);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;
	
	switch (ctrl->id) {
		case V4L2_CID_ANALOGUE_GAIN:
			ret = ov5647_write_reg_8bit(ov5647, OV564_REG_ANALOG_GAIN0, 
												(uint8_t)(ctrl->val & 0xff));
			if (ret == 0) {
				ret = ov5647_write_reg_8bit(ov5647, OV564_REG_ANALOG_GAIN1, 
											(uint8_t)((ctrl->val >> 8) & 0x3));
			} else {
				ret = -EINVAL;
			}
			break;
		case V4L2_CID_EXPOSURE: {
			ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_EXPOSURE0, (uint8_t)(ctrl->val & 0xff));
			if (ret == 0) {
				ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_EXPOSURE1, 
								(uint8_t)((ctrl->val >> 8) && 0xff));
				if (ret == 0) {
					ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_EXPOSURE2, 
							(uint8_t)((ctrl->val >> 16) && 0xf));
				} else {
					ret = -EINVAL;
				}
			} else {
				ret = -EINVAL;
			}
			break;
		}
		case V4L2_CID_HFLIP: 
			ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_HOR_BIN_FLIP_MIR, !ctrl->val);
			break;
		
		case V4L2_CID_VFLIP:
			ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_VER_BIN_FLIP_MIR, !ctrl->val);
			break;

		case V4L2_CID_VBLANK:
			ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_VTS_HI,
												ov5647->mode->height + ctrl->val);
			break;

		case V4L2_CID_HBLANK:
			break;

		case V4L2_CID_AUTOGAIN: {
			uint8_t reg;
			ret = ov5647_read_reg_8bit(ov5647, OV5647_REG_MANUAL_CTRL, &reg);
			if (ret == 0) {
				ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_MANUAL_CTRL, 
						ctrl->val ? reg & ~BIT(1) : reg | BIT(1));
			} else {
				ret = -EINVAL;
			}
			break;
		}

		case V4L2_CID_EXPOSURE_AUTO: {
			uint8_t reg;
			ret = ov5647_read_reg_8bit(ov5647, OV5647_REG_MANUAL_CTRL, &reg);
			if (ret == 0) {
				ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_MANUAL_CTRL, 
						ctrl->val == V4L2_EXPOSURE_MANUAL ? reg | BIT(0) : reg & ~BIT(0));
			} else {
				ret = -EINVAL;
			}
			break;
		}

		case V4L2_CID_AUTO_WHITE_BALANCE:
			ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_MIPI_AWB, ctrl->val ? 1 : 0);
			break;

		case V4L2_CID_PIXEL_RATE:
			break;

		default:
			dev_info(&client->dev,
					"ctrl(id:0x%x,val:0x%x) is not handled\n",
					ctrl->id, ctrl->val);
			ret = -EINVAL;
			break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops _ctrl_ops = {
	.s_ctrl = set_ctrl,
};

/* Initialize control handlers */
static int init_controls(struct ov5647 *ov5647)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov5647->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	unsigned int height = ov5647->mode->height;
	struct v4l2_fwnode_device_properties props;
	int exposure_max, exposure_def, hblank;
	int ret;

	ctrl_hdlr = &ov5647->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 12);
	if (ret)
		return ret;

	printk("init_controls:: mutex_init ");
	mutex_init(&ov5647->mutex);
	ctrl_hdlr->lock = &ov5647->mutex;

	/* By default, PIXEL_RATE is read only */
	ov5647->pixel_rate = 
		v4l2_ctrl_new_std(ctrl_hdlr, &_ctrl_ops,
			V4L2_CID_PIXEL_RATE, ov5647->mode->pixel_rate, 
			ov5647->mode->pixel_rate, 1, ov5647->mode->pixel_rate);

	ov5647->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &_ctrl_ops, V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(ov5647_link_freq_menu) - 1, 0,
				       ov5647_link_freq_menu);
	if (ov5647->link_freq)
		ov5647->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* Initial vblank/hblank/exposure parameters based on current mode */
	ov5647->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &_ctrl_ops,
					   V4L2_CID_VBLANK, OV5647_VBLANK_MIN,
					   OV5647_VTS_MAX - height, 1,
					   ov5647->mode->vts_def - height);

	hblank = ov5647->mode->hts_def - ov5647->mode->width;
	ov5647->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank,
					   1, hblank);

	exposure_max = ov5647->mode->vts_def - 4;
	exposure_def = (exposure_max < OV5647_EXPOSURE_DEFAULT) ?
							exposure_max : OV5647_EXPOSURE_DEFAULT;
	ov5647->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &_ctrl_ops, V4L2_CID_EXPOSURE,
					     OV5647_EXPOSURE_MIN, exposure_max,
					     OV5647_EXPOSURE_STEP, exposure_def);
						
	v4l2_ctrl_new_std(ctrl_hdlr, &_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OV564_ANA_GAIN_MIN, OV564_ANA_GAIN_MAX,
			  OV564_ANA_GAIN_STEP, OV564_ANA_GAIN_DEFAULT);
	
	// DIGITAL GAIN TODO

	ov5647->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (ov5647->hflip)
		ov5647->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	ov5647->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (ov5647->vflip)
		ov5647->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std(ctrl_hdlr, &_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, 0);

	v4l2_ctrl_new_std(ctrl_hdlr, &_ctrl_ops,
			  V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu(ctrl_hdlr, &_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
			       0, V4L2_EXPOSURE_MANUAL);

	// TEST PATTERNS TODO

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	ov5647->sd.ctrl_handler = ctrl_hdlr;
	return 0;
	

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&ov5647->mutex);

	return ret;

}

//-------------------------------------

static int enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov5647 *ov5647 = to_ov5647(sd);
	printk("ov5647 :: enum_mbus_code");

	if (code->index > 0) {
		printk("ov5647 :: enum_mbus_code - code->index > 0");
		return -EINVAL;
	}

	if (code->pad == 0) { //IMAGE_PAD
		printk("ov5647 :: enum_mbus_code - code->pad == 0");
		mutex_lock(&ov5647->mutex);
		code->code = MEDIA_BUS_FMT_SBGGR10_1X10;
		mutex_unlock(&ov5647->mutex);
	} else {
		printk("ov5647 :: enum_mbus_code - code->pad != 0");
		return -EINVAL;
	}
	printk("ov5647 :: enum_mbus_code - return 0");
	return 0;
}

static void _reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void _update_image_pad_format( const struct ov5647_mode *mode,
					struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	_reset_colorspace(&fmt->format);
}

static int _get_pad_format(struct ov5647 *ov5647,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *try_fmt;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		printk("ov5647 :: get_pad_format");
		try_fmt = v4l2_subdev_get_try_format(&ov5647->sd, sd_state, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == 0 ? 
				MEDIA_BUS_FMT_SRGGB10_1X10 :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == 0) {
			_update_image_pad_format(ov5647->mode, fmt);
			fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
		} else {
			printk("ov5647 :: _get_pad_format - fmt->pad != 0");
			return -EINVAL;
		}
	}
	return 0;
}

static int get_pad_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	struct ov5647 *ov5647 = to_ov5647(sd);
	int ret = 0;

	printk("ov5647 :: get_pad_format");

	mutex_lock(&ov5647->mutex);
	ret = _get_pad_format(ov5647, sd_state, fmt);
	mutex_unlock(&ov5647->mutex);

	printk("ov5647 :: get_pad_format : %d", ret );
	return ret;
}

static int set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt) 
{
	struct ov5647 *ov5647 = to_ov5647(sd);
	const struct ov5647_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	int exposure_max, exposure_def, hblank;

	printk("ov5647 :: set_pad_format");
	mutex_lock(&ov5647->mutex);

	if (fmt->pad == 0) {
		fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
		mode = v4l2_find_nearest_size(supported_modes,
					ARRAY_SIZE(supported_modes),
					width, height,
					fmt->format.width,
					fmt->format.height);
		_update_image_pad_format(mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			printk("ov5647 :: set_pad_format - V4L2_SUBDEV_FORMAT_TRY");
			framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
			*framefmt = fmt->format;
		} else if (ov5647->mode != mode || ov5647->fmt.code != fmt->format.code) {
			printk("ov5647 :: set_pad_format - ov5647->mode != mode || ov5647->fmt.code != fmt->format.code");
			ov5647->fmt = fmt->format;
			ov5647->mode = mode;

			/* Update limits and set FPS to default */
			__v4l2_ctrl_modify_range(ov5647->vblank, OV5647_VBLANK_MIN,
									OV5647_VTS_MAX - mode->height, 1, 
									mode->vts_def - mode->height);
			__v4l2_ctrl_s_ctrl(ov5647->vblank, mode->vts_def - mode->height);

			hblank = mode->hts_def - mode->width;
			__v4l2_ctrl_modify_range(ov5647->hblank, hblank, hblank, 1, hblank);
			__v4l2_ctrl_s_ctrl(ov5647->hblank, hblank);

			/* Update max exposure while meeting expected vblanking */
			exposure_max = mode->vts_def - 4;
			exposure_def = (exposure_max < OV5647_EXPOSURE_DEFAULT) ?
								exposure_max : OV5647_EXPOSURE_DEFAULT;
			__v4l2_ctrl_modify_range(ov5647->exposure, ov5647->exposure->minimum,
						 exposure_max, ov5647->exposure->step, exposure_def);

			/* Scale the pixel rate based on the mode specific factor */
			__v4l2_ctrl_modify_range(ov5647->pixel_rate, ov5647->mode->pixel_rate,
								ov5647->mode->pixel_rate, 1, ov5647->mode->pixel_rate);
		}
	} else {
		printk("ov5647 :: set_pad_format - pad != 0");
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			printk("ov5647 :: set_pad_format - fmt which is not format try");
			return -EINVAL;
		}
	}

	mutex_unlock(&ov5647->mutex);
	printk("ov5647 :: set_pad_format : 0");
	return 0;
}

static const struct v4l2_rect *_get_pad_crop(struct ov5647 *ov5647,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
		case V4L2_SUBDEV_FORMAT_TRY:
			return v4l2_subdev_get_try_crop(&ov5647->sd, sd_state, pad);
		case V4L2_SUBDEV_FORMAT_ACTIVE:
			return &ov5647->mode->crop;
	}

	return NULL;
}

static int get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	struct ov5647 *ov5647;
	printk("ov5647 :: get_selection");

	switch (sel->target) {
		case V4L2_SEL_TGT_CROP: {
			printk("ov5647 :: get_selection - V4L2_SEL_TGT_CROP, attempting to lock");
			ov5647 = to_ov5647(sd);
			mutex_lock(&ov5647->mutex);
			sel->r = *_get_pad_crop(ov5647, sd_state, sel->pad, sel->which);
			mutex_unlock(&ov5647->mutex);
			printk("ov5647 :: get_selection - V4L2_SEL_TGT_CROP, unlocked | sel rect width : %d", sel->r.width);
			printk("ov5647 :: get_selection - return 0");
			return 0;
		}

		case V4L2_SEL_TGT_NATIVE_SIZE:
			printk("ov5647 :: get_selection - V4L2_SEL_TGT_NATIVE_SIZE");
			sel->r.top = 0;
			sel->r.left = 0;
			sel->r.width = OV5647_NATIVE_WIDTH;
			sel->r.height = OV5647_NATIVE_HEIGHT;
			printk("ov5647 :: get_selection - return 0");
			return 0;

		case V4L2_SEL_TGT_CROP_DEFAULT:
		case V4L2_SEL_TGT_CROP_BOUNDS:
			printk("ov5647 :: get_selection - V4L2_SEL_TGT_CROP_DEFAULT or V4L2_SEL_TGT_CROP_BOUNDS");
			sel->r.top = OV5647_PIXEL_ARRAY_TOP;
			sel->r.left = OV5647_PIXEL_ARRAY_LEFT;
			sel->r.width = OV5647_PIXEL_ARRAY_WIDTH;
			sel->r.height = OV5647_PIXEL_ARRAY_HEIGHT;
			printk("ov5647 :: get_selection - return 0");
			return 0;
	}
	printk("ov5647 :: get_selection - invalid target :  %d", sel->target);
	printk("ov5647 :: get_selection - return -EINVAL");
	return -EINVAL;
}

static int enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov5647 *ov5647 = to_ov5647(sd);
	printk("ov5647 :: enum_frame_size");

	if (fse->pad == 0) {
		printk("ov5647 :: enum_frame_size - fse->pad == 0");
		if (fse->index >= ARRAY_SIZE(supported_modes)) {
			printk("ov5647 :: enum_frame_size - fse->index > supported modes number");
			return -EINVAL;
		}

		mutex_lock(&ov5647->mutex);
		if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10) {
			printk("ov5647 :: enum_frame_size - fse->code != MEDIA_BUS_FMT_SBGGR10_1X10");
			return -EINVAL;
		}
		mutex_unlock(&ov5647->mutex);
		
		fse	->min_width = supported_modes[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = supported_modes[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		printk("ov5647 :: enum_frame_size - fse->pad != 0");
		return -EINVAL;
	}

	printk("ov5647 :: enum_frame_size - return 0");
	return 0;
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
	.enum_mbus_code = enum_mbus_code,
	.get_fmt = get_pad_format,
	.set_fmt = set_pad_format,
	.get_selection = get_selection,
	.enum_frame_size = enum_frame_size,
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
static int check_hwcfg(struct ov5647 *ov5647, struct device_node *np)
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

	ov5647->clock_ncont = bus_cfg.bus.mipi_csi2.flags &
			      V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK;

out:
	of_node_put(ep);

	return ret;
}

static int ov5647_probe(struct i2c_client *client)
{
	struct device* dev;
	struct ov5647* ov5647;
	struct device_node *np;
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

	np = client->dev.of_node;
	if (IS_ENABLED(CONFIG_OF) && np) {
		printk("ov5647_probe:: check_hwcfg ");
		ret = check_hwcfg(ov5647, np);
		if (ret) {
			dev_err(dev, "DT parsing error: %d\n", ret);
			return ret;
		}
	}

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

	printk("ov5647_probe:: get_regulators ");
	ret = get_regulators(ov5647, dev); 
	if (ret) {
		printk("ov5647_probe::failed to get regulators\n");
		return ret;
	}

	/* Set default mode to max resolution */
	printk("ov5647_probe:: Set default mode ");
	ov5647->mode = &supported_modes[3];

	printk("ov5647_probe:: init_controls ");
	ret = init_controls(ov5647);
	if (ret)
		goto error_power_off;

	printk("ov5647_probe:: power_on ");
	ret = power_on(dev);
	if (ret)
		return ret;

	printk("ov5647_probe:: ov5647_identify_module ");
	ret = ov5647_identify_module(ov5647);
	if (ret)
		goto error_power_off;


	/* sensor doesn't enter LP-11 state upon power up until and unless
	 * streaming is started, so upon power up switch the modes to:
	 * streaming -> standby
	 */
	ret = ov5647_write_reg_8bit(ov5647, OV5647_REG_MIPI_CTRL00, MIPI_CTRL00_CLOCK_LANE_GATE 
									| MIPI_CTRL00_BUS_IDLE | MIPI_CTRL00_CLOCK_LANE_DISABLE);
	if (ret < 0)
		goto error_power_off;

		/* Initialize default format */
	set_default_format(ov5647);

/* Initialize source pad */
	ov5647->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ov5647->sd.entity, 1, &ov5647->pad);
	if (ret < 0)
		goto error_handler_free;

	ret = v4l2_async_register_subdev_sensor(&ov5647->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	printk("ov5647_probe :: error_media_entity ");
	media_entity_cleanup(&ov5647->sd.entity);

error_handler_free:
	printk("ov5647_probe :: error_handler_free ");
	free_controls(ov5647);

error_power_off:
	printk("ov5647_probe :: error_power_off ");
	power_off(dev);

	return ret;
}

static void ov5647_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5647 *ov5647 = to_ov5647(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	free_controls(ov5647);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

//---------------------------

static const struct of_device_id ov5647_dt_ids[] = {
	{ .compatible = "ovti,ov5647" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov5647_dt_ids);

static const struct dev_pm_ops ov5647_pm_ops = {
	SET_RUNTIME_PM_OPS(power_off, power_on, NULL)
};

static struct i2c_driver ov5647_i2c_driver = {
	.driver = {
		.name = "ov5647",
		.of_match_table	= ov5647_dt_ids,
		.pm = &ov5647_pm_ops,
	},
	.probe_new = ov5647_probe,
	.remove = ov5647_remove,
};

module_i2c_driver(ov5647_i2c_driver);

MODULE_AUTHOR("Mohamed Mahmoud <mohamednabil940@gmail.com>");
MODULE_DESCRIPTION("ov5647 sensor driver");
MODULE_LICENSE("GPL v2");