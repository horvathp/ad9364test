#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/mman.h>

#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif

unsigned char * user_ftr = NULL;
unsigned int user_ftr_len = 0;

extern unsigned char mimocpm_5mhz_ftr[];
extern unsigned int mimocpm_5mhz_ftr_len;
extern unsigned char mimocpm_10mhz_ftr[];
extern unsigned int mimocpm_10mhz_ftr_len;
extern unsigned char mimocpm_20mhz_ftr[];
extern unsigned int mimocpm_20mhz_ftr_len;

extern unsigned char mimo5_ftr[];
extern unsigned int mimo5_ftr_len;
extern unsigned char mimo10_ftr[];
extern unsigned int mimo10_ftr_len;
extern unsigned char mimo20_ftr[];
extern unsigned int mimo20_ftr_len;
extern unsigned char mimo2_ftr[];
extern unsigned int mimo2_ftr_len;

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

#define ASSERT(expr) { \
    if (!(expr)) { \
        (void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
        (void) abort(); \
    } \
}

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
  __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

/* RX is input, TX is output */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg {
    long long bw_hz; // Analog banwidth in Hz
    long long fs_hz; // Baseband sample rate in Hz
    long long lo_hz; // Local oscillator frequency in Hz
    const char* rfport; // Port name
};

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

static bool stop;

/* cleanup and exit */
static void shutdown()
{
    printf("* Destroying buffers\n");
    if (rxbuf) { iio_buffer_destroy(rxbuf); }
    if (txbuf) { iio_buffer_destroy(txbuf); }

    printf("* Disabling streaming channels\n");
    if (rx0_i) { iio_channel_disable(rx0_i); }
    if (rx0_q) { iio_channel_disable(rx0_q); }
    if (tx0_i) { iio_channel_disable(tx0_i); }
    if (tx0_q) { iio_channel_disable(tx0_q); }

    printf("* Destroying context\n");
    if (ctx) { iio_context_destroy(ctx); }
    exit(0);
}

static void handle_sig(int sig)
{
    printf("Waiting for process to finish...\n");
    stop = true;
}

/* check return value of attr_write function */
static void errchk(int v, const char* what) {
     if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); shutdown(); }
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
    errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
    errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
    snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
    return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(struct iio_context *ctx)
{
    struct iio_device *dev =  iio_context_find_device(ctx, "ad9361-phy");
    ASSERT(dev && "No ad9361-phy found");
    return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
    switch (d) {
    case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
    case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");  return *dev != NULL;
    default: ASSERT(0); return false;
    }
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
    *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
    if (!*chn)
        *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
    return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
    switch (d) {
    case RX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), false); return *chn != NULL;
    case TX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), true);  return *chn != NULL;
    default: ASSERT(0); return false;
    }
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn)
{
    switch (d) {
     // LO chan is always output, i.e. true
    case RX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 0), true); return *chn != NULL;
    case TX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 1), true); return *chn != NULL;
    default: ASSERT(0); return false;
    }
}

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid)
{
    struct iio_channel *chn = NULL;

    // Configure phy and lo channels
    printf("* Acquiring AD9361 phy channel %d\n", chid);
    if (!get_phy_chan(ctx, type, chid, &chn)) {    return false; }
    wr_ch_str(chn, "rf_port_select",     cfg->rfport);
    wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
    wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

    // Configure LO channel
    printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
    if (!get_lo_chan(ctx, type, &chn)) { return false; }
    wr_ch_lli(chn, "frequency", cfg->lo_hz);
    return true;
}

static bool switch_tx_source(struct iio_device *tx_dev, uint32_t value)  
{
    int ret = 0;
    /* value == 0x02: external "DMA"
     4'h9: dac_data_out_int <= dac_pn_data;
     4'h8: dac_data_out_int <= adc_data;
     4'h3: dac_data_out_int <= 12'd0;
     4'h2: dac_data_out_int <= dma_data[15:4];
     4'h1: dac_data_out_int <= dac_pat_data[15:4];
     default: dac_data_out_int <= dac_dds_data[15:4];
    */
    ret += iio_device_reg_write(tx_dev, 0x418, value);
    ret += iio_device_reg_write(tx_dev, 0x458, value);
    ret += iio_device_reg_write(tx_dev, 0x498, value);
    ret += iio_device_reg_write(tx_dev, 0x4d8, value);
    return ret == 0;
}

static bool ad9361_set_trx_fir_enable(struct iio_device *dev, int enable)
{
	int ret = iio_device_attr_write_bool(dev,
					 "in_out_voltage_filter_fir_en", !!enable);
    if(ret < 0)  {
        printf("ad9361_set_trx_fir_enable returned %d\n", ret);
    }
	return ret;
}

int ad9361_get_trx_fir_enable(struct iio_device *dev, int *enable)
{
	bool value;

	int ret = iio_device_attr_read_bool(dev, "in_out_voltage_filter_fir_en", &value);

	if (!ret)
		*enable	= value;

	return ret;
}

static bool ad9361_set_ensm_mode(struct iio_device *dev, const char *mode)
{
	int ret = iio_device_attr_write_raw(dev, "ensm_mode", mode, strlen(mode));
    if(ret < 0)  {
        printf("ad9361_set_ensm_mode returned %d\n", ret);
    }
	return ret == 2;
}

int ad9361_set_bb_config(struct iio_device *dev, int filter_config)
{
	int ret, enable, buflen;
    char *buf;

	ret = ad9361_get_trx_fir_enable(dev, &enable);
	if (ret < 0)  {
        printf("ad9361_get_trx_fir_enable failed\n");
		return ret;
    }

	if (enable) {
		ret = ad9361_set_trx_fir_enable(dev, false);
		if (ret < 0)
			return ret;
	}
    if ((user_ftr != NULL) && (user_ftr_len > 0)) {
        buf = user_ftr;
        buflen = user_ftr_len;
    } else {
        switch(filter_config)  {
            case 0: buf = mimo5_ftr; buflen = mimo5_ftr_len; break;
            case 1: buf = mimo10_ftr; buflen = mimo10_ftr_len; break;
            case 2: buf = mimo20_ftr; buflen = mimo20_ftr_len; break;
            case 3: buf = mimo2_ftr; buflen = mimo2_ftr_len; break;
            case 4: buf = mimocpm_5mhz_ftr; buflen = mimocpm_5mhz_ftr_len; break;
            case 5: buf = mimocpm_10mhz_ftr; buflen = mimocpm_10mhz_ftr_len; break;
            case 6: buf = mimocpm_20mhz_ftr; buflen = mimocpm_20mhz_ftr_len; break;
        }
    }

    printf("* Writing filter configuration:\n");
    printf("%s", buf);
    
    ret = iio_device_attr_write_raw(dev, "filter_fir_config", buf, buflen);
	if (ret < 0)  {
        printf("iio_device_attr_write_raw failed\n");
		return ret;
    }

	ret = ad9361_set_trx_fir_enable(dev, true);
	if (ret < 0)  {
        printf("ad9361_set_trx_fir_enable failed\n");
		return ret;
    }
	return 0;
}

static int verify_fpga_version()
{
    int fd;
    void *map_base, *virt_addr; 
    off_t target = 0x83c00010;
    uint32_t read_result;
	unsigned page_size, mapped_size, offset_in_page;

    //if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
    if((fd = open("/dev/mem", O_RDONLY | O_SYNC)) == -1) {
        FATAL;
    }
    mapped_size = page_size = getpagesize();
	offset_in_page = (unsigned)target & (page_size - 1);
    /* Map one page */
    map_base = mmap(0, mapped_size, PROT_READ, MAP_SHARED, fd, target & ~(off_t)(page_size - 1));
    if(map_base == MAP_FAILED)  {
        FATAL;
    }
	virt_addr = map_base + offset_in_page;
    printf("/dev/mem opened, page_size = %d, map_base = 0x%X, offset in page = 0x%X.\n", page_size, map_base, offset_in_page);

    read_result = *((volatile uint32_t *) virt_addr);
    printf("Value at address 0x%X (%p): %d\n", target, virt_addr, read_result); 
    
    if(munmap(map_base, mapped_size) == -1)  {
        FATAL;
    }
    close(fd);
}

static int verify_fpga_version2()
{
    int fd;
    void *map_base, *virt_addr; 
    off_t target = 0x83c00010;
    uint32_t read_result;
	unsigned page_size, mapped_size;

    //if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
    if((fd = open("/dev/mem", O_RDONLY | O_SYNC)) == -1) {
        FATAL;
    }
    mapped_size = page_size = getpagesize();
    map_base = mmap(0, mapped_size, PROT_READ, MAP_SHARED, fd, 0x83c00000);
    if(map_base == MAP_FAILED)  {
        FATAL;
    }
	virt_addr = map_base + 0x10;
    read_result = *((volatile uint32_t *) virt_addr);
    printf("Value at address 0x%X (%p): %d\n", target, virt_addr, read_result); 
    
    if(munmap(map_base, mapped_size) == -1)  {
        FATAL;
    }
    close(fd);
}


int main (int argc, char *argv[])
{
    enum opmode {BW5, BW10, BW20, BW1};
    enum opmode mode;

    // PHY device
    struct iio_device *ad9361_phy;
    // Streaming devices
    struct iio_device *tx;
    //struct iio_device *rx;

    // RX and TX sample counters
    //size_t nrx = 0;
    size_t ntx = 0;

    // Stream configurations
    //struct stream_cfg rxcfg;
    struct stream_cfg txcfg;

    long long samp_rate;
    long long tx_bw;
    //long long rx_bw;
    
    int bb_config = -1;
    int prbs_src = 1;
    int axi_src = 0;
    long long fc = -1;

    int cmdopt;
    extern char *optarg;
    FILE *f;
    long fsize;

    verify_fpga_version();
    verify_fpga_version2();

    while( (cmdopt = getopt(argc, argv, "b:f:c:sp")) != -1) {
        switch(cmdopt)  {
            case 'b' :
                bb_config = atoi(optarg);
                printf("optarg = %d\n", bb_config);
                if(bb_config == 0)  {
                    printf("* Setting 5 MHz/OFDM configuration values\n");
                    mode = BW5;
                    samp_rate = MHZ(6.144);
                    tx_bw = MHZ(6);
                    //rx_bw = MHZ(6);
                }
                else if(bb_config == 1)   {
                    printf("* Setting 10 MHz/OFDM configuration values\n");
                    mode = BW10;
                    samp_rate = MHZ(2 * 6.144);
                    tx_bw = MHZ(12);
                    //rx_bw = MHZ(12);
                }
                else if(bb_config == 2)  {
                    printf("* Setting 20 MHz/OFDM configuration values\n");
                    mode = BW20;
                    samp_rate = MHZ(4 * 6.144);
                    tx_bw = MHZ(22);
                    //rx_bw = MHZ(22);
                }
                else if(bb_config == 3)  {
                    printf("* Setting 2 MHz/OFDM configuration values\n");
                    mode = BW1;
                    samp_rate = MHZ(2.304);
                    tx_bw = MHZ(2);
                    //rx_bw = MHZ(1.2);
                }
                else if(bb_config == 4)  {
                    printf("* Setting 5 MHz/CPM configuration values\n");
                    mode = BW1;
                    samp_rate = MHZ(6.144);
                    tx_bw = MHZ(3);
                    //rx_bw = MHZ(1.2);
                }
                else if(bb_config == 5)  {
                    printf("* Setting 10 MHz/CPM configuration values\n");
                    mode = BW1;
                    samp_rate = MHZ(2 * 6.144);
                    tx_bw = MHZ(6);
                    //rx_bw = MHZ(1.2);
                }
                else if(bb_config == 6)  {
                    printf("* Setting 20 MHz/CPM configuration values\n");
                    mode = BW1;
                    samp_rate = MHZ(4 * 6.144);
                    tx_bw = MHZ(12);
                    //rx_bw = MHZ(1.2);
                }
                else  {
                    printf("Bandwidth option must be [0...6]\n"); 
                    exit(-1);
                }
                break;
            case 'f' :
                    f = fopen(optarg, "rb");
                    if (f == NULL) {
                        printf("Unable to open the filter file!\n");
                        exit(-1);
                    }
                    fseek(f, 0, SEEK_END);
                    fsize = ftell(f);
                    fseek(f, 0, SEEK_SET);
                    user_ftr = malloc(fsize + 1);
                    fread(user_ftr, 1, fsize, f);
                    fclose(f);
                    user_ftr[fsize] = 0;
                    user_ftr_len = fsize;
                break;

            case 'c':  // carrier frequency
                fc = atoll(optarg);
                break;

            case 's':  // sample source test gen
                axi_src = 1;
                prbs_src = 0;
                break;

            case 'p':  // sample source PRBS
                prbs_src = 1;
                break;

            default:
                printf("Usage: %s -b <bandwidth> -f <ftr_file> -c <carrier_freq_hz> -sp\n", argv[0]);
                printf("Opmodes:\n0: 5 MHz/OFDM\n1: 10 MHz/OFDM\n2: 20 MHz/OFDM\n3: 2 MHz/OFDM\n4: 5 MHz/CPM\n");

                exit(-1);
        }
    }

    if(bb_config == -1)  {
        printf("Bandwidth (-b) must be specified!\n");
        exit(-1);
    }

    if(prbs_src && axi_src)  {
        printf("Only one of -p or -s might be specified!\n");
        exit(-1);
    }

    if(prbs_src)  {
        printf("Activating the PRBS source\n"); 
    }

    if(axi_src)  {
        printf("Activating the AXIS source\n"); 
    }

    if(fc > 200e6 && fc < 6000e6)  {
        printf("Setting carrier frequency %.3lf MHz\n", fc/1e6);
    }
    else  {
        printf("Using default carrier frequency 2.45 GHz\n");
        fc = 2450e6;
    }

    // Listen to ctrl+c and ASSERT
    signal(SIGINT, handle_sig);

    printf("* Sampling rate: %lld\n", samp_rate);
    printf("* TX RF BW: %lld\n", tx_bw);
    //printf("* RX RF BW: %lld\n", rx_bw);
    
    // RX stream config
    //rxcfg.bw_hz = rx_bw;
    //rxcfg.fs_hz = samp_rate;
    //rxcfg.lo_hz = fc; //GHZ(2.45); // 2.45 GHz rf frequency
    //rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)

    // TX stream config
    txcfg.bw_hz = tx_bw; 
    txcfg.fs_hz = samp_rate;   // 2.5 MS/s tx sample rate
    txcfg.lo_hz = fc; // GHZ(2.45); // 2.5 GHz rf frequency
    txcfg.rfport = "A"; // port A (select for rf freq.)

    printf("* Acquiring IIO context\n");
    ASSERT((ctx = iio_create_default_context()) && "No context");
    ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

    printf("* Acquiring the AD9361 device\n");
    ASSERT((ad9361_phy = get_ad9361_phy(ctx)) && "No AD9361-PHY device found");

    printf("* Acquiring AD9361 streaming devices\n");
    ASSERT(get_ad9361_stream_dev(ctx, TX, &tx) && "No tx dev found");
    //ASSERT(get_ad9361_stream_dev(ctx, RX, &rx) && "No rx dev found");

    printf("* Configuring AD9361 for streaming\n");
    //ASSERT(cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0) && "RX port 0 not found");
    ASSERT(cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0) && "TX port 0 not found");

    ASSERT((ad9361_set_bb_config(ad9361_phy, bb_config) == 0) && "Baseband filter configuration failed");

    printf("* Initializing AD9361 IIO streaming channels\n");
    //ASSERT(get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i) && "RX chan i not found");
    //ASSERT(get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q) && "RX chan q not found");
    ASSERT(get_ad9361_stream_ch(ctx, TX, tx, 0, &tx0_i) && "TX chan i not found");
    ASSERT(get_ad9361_stream_ch(ctx, TX, tx, 1, &tx0_q) && "TX chan q not found");

    if(axi_src)  {
        ASSERT(switch_tx_source(tx, 2) && "Switching the sample stream to FPGA failed");    
    }
    else  {
        ASSERT(switch_tx_source(tx, 9) && "Switching the sample stream to PRBS failed");    
    }

    printf("* Switching ENSM to TX\n");
    ASSERT(ad9361_set_ensm_mode(ad9361_phy, "tx") && "ENSM switch to TX failed");

    printf("* Enabling IIO streaming channels\n");
    //iio_channel_enable(rx0_i);
    //iio_channel_enable(rx0_q);
    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);

    printf("* Starting IO streaming (press CTRL+C to cancel)\n");

    while (!stop)  {
	    sleep(1);
    }

    printf("* Switching ENSM to RX\n");
    ASSERT(ad9361_set_ensm_mode(ad9361_phy, "rx") && "ENSM switch to RX failed");
    printf("* Shutdown\n");
    shutdown();

    return 0;
} 

