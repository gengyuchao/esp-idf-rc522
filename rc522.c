#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "rc522.h"

static const char* TAG = "ESP-RC522";

int extra_delay=0;	
struct rc522 {
    bool running;
    rc522_config_t* config;
    spi_device_handle_t spi;
    TaskHandle_t task_handle;
    bool scan_started;
    bool tag_was_present_last_time;
};

typedef struct rc522* rc522_handle_t;

static rc522_handle_t hndl = NULL;

#define rc522_fw_version() rc522_read(0x37)

bool rc522_is_inited() {
    return hndl != NULL;
}

static esp_err_t rc522_spi_init() {
    if(! hndl || ! hndl->config) {
        ESP_LOGE(TAG, "Fail to init SPI. Invalid handle");
        return ESP_ERR_INVALID_STATE;
    }

    if(hndl->spi) {
        ESP_LOGW(TAG, "SPI already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    spi_bus_config_t buscfg = {
        .miso_io_num = hndl->config->miso_io,
        .mosi_io_num = hndl->config->mosi_io,
        .sclk_io_num = hndl->config->sck_io,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5000000,
        .mode = 0,
        .spics_io_num = hndl->config->sda_io,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX
    };

    esp_err_t err = spi_bus_initialize(hndl->config->spi_host_id, &buscfg, 0);

    if(err != ESP_OK) {
        return err;
    }

    err = spi_bus_add_device(hndl->config->spi_host_id, &devcfg, &hndl->spi);

    if(err != ESP_OK) {
        spi_bus_free(hndl->config->spi_host_id);
        hndl->spi = NULL;
    }

    return err;
}

static esp_err_t rc522_write_n(uint8_t addr, uint8_t n, uint8_t *data) {
    uint8_t* buffer = (uint8_t*) malloc(n + 1);
    buffer[0] = (addr << 1) & 0x7E;

    for (uint8_t i = 1; i <= n; i++) {
        buffer[i] = data[i-1];
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = 8 * (n + 1);
    t.tx_buffer = buffer;

    esp_err_t ret = spi_device_transmit(hndl->spi, &t);

    free(buffer);

    return ret;
}

static esp_err_t rc522_write(uint8_t addr, uint8_t val) {
    return rc522_write_n(addr, 1, &val);
}

static uint8_t* rc522_read_n(uint8_t addr, uint8_t n) {
    if (n <= 0) {
        return NULL;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    uint8_t* buffer = (uint8_t*) malloc(n);
    
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;
    t.tx_data[0] = ((addr << 1) & 0x7E) | 0x80;
    t.rxlength = 8 * n;
    t.rx_buffer = buffer;

    esp_err_t ret = spi_device_transmit(hndl->spi, &t);
    assert(ret == ESP_OK);

    return buffer;
}

static uint8_t rc522_read(uint8_t addr) {
    uint8_t* buffer = rc522_read_n(addr, 1);
    uint8_t res = buffer[0];
    free(buffer);

    return res;
}

static esp_err_t rc522_set_bitmask(uint8_t addr, uint8_t mask) {
    return rc522_write(addr, rc522_read(addr) | mask);
}

static esp_err_t rc522_clear_bitmask(uint8_t addr, uint8_t mask) {
    return rc522_write(addr, rc522_read(addr) & ~mask);
}

static esp_err_t rc522_antenna_on() {
    esp_err_t ret;

    if(~ (rc522_read(0x14) & 0x03)) {
        ret = rc522_set_bitmask(0x14, 0x03);

        if(ret != ESP_OK) {
            return ret;
        }
    }

    return rc522_write(0x26, 0x60); // 43dB gain
}

static void rc522_task(void* arg);

esp_err_t rc522_init(rc522_config_t* config) {
    if(! config) {
        return ESP_ERR_INVALID_ARG;
    }

    if(hndl) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if(! (hndl = calloc(1, sizeof(struct rc522)))) {
        return ESP_ERR_NO_MEM;
    }

    if(! (hndl->config = calloc(1, sizeof(rc522_config_t)))) {
        rc522_destroy();
        return ESP_ERR_NO_MEM;
    }

    // copy config considering defaults
    hndl->config->callback         = config->callback;
    hndl->config->miso_io          = config->miso_io == 0 ? RC522_DEFAULT_MISO : config->miso_io;
    hndl->config->mosi_io          = config->mosi_io == 0 ? RC522_DEFAULT_MOSI : config->mosi_io;
    hndl->config->sck_io           = config->sck_io == 0 ? RC522_DEFAULT_SCK : config->sck_io;
    hndl->config->sda_io           = config->sda_io == 0 ? RC522_DEFAULT_SDA : config->sda_io;
    hndl->config->spi_host_id      = config->spi_host_id == 0 ? RC522_DEFAULT_SPI_HOST : config->spi_host_id;
    hndl->config->scan_interval_ms = config->scan_interval_ms < 50 ? RC522_DEFAULT_SCAN_INTERVAL_MS : config->scan_interval_ms;
    hndl->config->task_stack_size  = config->task_stack_size == 0 ? RC522_DEFAULT_TACK_STACK_SIZE : config->task_stack_size;
    hndl->config->task_priority    = config->task_priority == 0 ? RC522_DEFAULT_TACK_STACK_PRIORITY : config->task_priority;

    esp_err_t err = rc522_spi_init();

    if(err != ESP_OK) {
        rc522_destroy();
        return err;
    }
    
    // ---------- RW test ------------
    const uint8_t test_addr = 0x24, test_val = 0x25;
    for(uint8_t i = test_val; i < test_val + 2; i++) {
        if((err = rc522_write(test_addr, i)) != ESP_OK || rc522_read(test_addr) != i) {
            ESP_LOGE(TAG, "RW test fail");
            rc522_destroy();
            return err;
        }
    }
    // ------- End of RW test --------

    rc522_write(0x01, 0x0F);
    rc522_write(0x2A, 0x8D);
    rc522_write(0x2B, 0x3E);
    rc522_write(0x2D, 0x1E);
    rc522_write(0x2C, 0x00);
    rc522_write(0x15, 0x40);
    rc522_write(0x11, 0x3D);

    rc522_antenna_on();

    hndl->running = true;
    if (xTaskCreate(rc522_task, "rc522_task", hndl->config->task_stack_size, NULL, hndl->config->task_priority, &hndl->task_handle) != pdTRUE) {
        ESP_LOGE(TAG, "Fail to create rc522 task");
        rc522_destroy();
        return err;
    }

    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Fail to create timer");
        rc522_destroy();
        return err;
    }

    ESP_LOGI(TAG, "Initialized (firmware: 0x%x)", rc522_fw_version());
    return ESP_OK;
}

uint64_t rc522_sn_to_u64(uint8_t* sn) {
    if(!sn) {
        return 0;
    }

    uint64_t result = 0;
    for(int i = 4; i >= 0; i--) {
        result |= ((uint64_t) sn[i] << (i * 8));
    }

    return result;
}

/* Returns pointer to dynamically allocated array of two element */
static uint8_t* rc522_calculate_crc(uint8_t *data, uint8_t n) {
    rc522_clear_bitmask(0x05, 0x04);
    rc522_set_bitmask(0x0A, 0x80);

    rc522_write_n(0x09, n, data);

    rc522_write(0x01, 0x03);

    uint8_t i = 255;
    uint8_t nn = 0;

    for(;;) {
        nn = rc522_read(0x05);
        i--;

        if(! (i != 0 && ! (nn & 0x04))) {
            break;
        }
    }

    uint8_t* res = (uint8_t*) malloc(2); 
    
    res[0] = rc522_read(0x22);
    res[1] = rc522_read(0x21);

    return res;
}
// n : num bytes to sent
// res_n : num bits stored in receive buffer
static uint8_t rc522_card_write(uint8_t cmd, uint8_t *send_data, uint8_t n, uint8_t **recv_data, uint8_t* res_n) {
    uint8_t *result = NULL;
    uint8_t irq = 0x00;
    uint8_t irq_wait = 0x00;
    uint8_t last_bits = 0;
    uint8_t nn = 0;
    char   status = TAG_ERR;		// status code
    
    if(cmd == PCD_AUTHENT) {
        irq = 0x12;         // enable Idle and low alert interrupts
        irq_wait = 0x10;    // expect interrupt to continue (idle interrupt)
    }
    else if(cmd == PCD_TRANSCEIVE) {
        irq = 0x77;         // enable all alert except high alert interrupt
        irq_wait = 0x30;    // expect interrupt to continue (Idle and received end of valid send_ stream)
    } else {
        printf("Fatal internal error in rc522_card_write. invalide command 0x%x\n",cmd);
    }

    /*
     * B7   1 IRQ inverted with respect to Status1reg IRq bit
     * B6   1 allows transmitter interrupt request
     * B5   1 Allow receiver interrupt request
     * B4   1 allow idle interrupt request
     * B3   1 allow high alert interrupt request
     * B2   1 allows low alert interrupt request
     * B1   1 allow error interrupt request
     * B0   1 allows time interrupt request
     */
    rc522_write(ComIEnReg, irq | 0x80);
    
    /* clear IRQ inverted respect to status1reg 
     * indicates that the marked bits in the ComIrqReg register are set */
    rc522_clear_bitmask(ComIrqReg, 0x80);
    
    /* Flushbuffer / clear FIFO*/
    rc522_set_bitmask(FIFOLevelReg, 0x80);
    
    /* sent command to set RC522 to Idle & cancel any pending execution */
    rc522_write(CommandReg, PCD_IDLE);

    /* write data in FIFO data for the length of buffer*/
    rc522_write_n(FIFODataReg, n, send_data);

    /* sent the command */
    rc522_write(CommandReg, cmd);

    /* start the transmission of data in Transceive mode*/
    if(cmd == PCD_TRANSCEIVE) {
        rc522_set_bitmask(BitFramingReg, 0x80);
    }

    uint16_t i = 1000;

    /* as long as not count down
     * Time out has not happened
     * and other possible wait for interrupts
     * n&01 = time out has happened (timeout was set in PcdReset as 24ms)
     * n&waitfor = good interrupts to happen
     */

    for(;;) {
        /* read interrupt register */
        nn = rc522_read(ComIrqReg);
        i--;

        if(! (i != 0 && (((nn & 0x01) == 0) && ((nn & irq_wait) == 0)))) {
            break;
        }
    }

    /* removing start data transmission in transceive */
    if(cmd == PCD_TRANSCEIVE) {
        rc522_clear_bitmask(BitFramingReg, 0x80);
    }

    if(i != 0) {
        uint8_t err_state = rc522_read(ErrorReg);
        if((err_state & 0x1B) == 0x00) {
            status = TAG_OK;
            if(cmd == PCD_TRANSCEIVE) {
                nn = rc522_read(FIFOLevelReg);
                last_bits = rc522_read(ControlReg) & 0x07;

                if (last_bits != 0) {
                    *res_n = (nn - 1) + last_bits;
                } else {
                    *res_n = nn;
                }

                result = (uint8_t*) malloc(*res_n);

                for(i = 0; i < *res_n; i++) {
                    result[i] = rc522_read(FIFODataReg);
                }
                /* Given the comment above, an extra delay can now be requested on the command
                 * line with -E. The 80us mentioned above was resulting in 400us or 500us depending
                 * on the number of databytes that were received. 
                 * 
                 * allow for 500us extra delay for slow card to recover */
                // if (extra_delay) usleep(500);	//ets_delay_us(500);
            }
        } else {
            status = TAG_ERR;
        }

       /*
        BIT 0 ProtocolErr
            Set to logic 1, if one out of the following cases occur:
            • Set to logic 1 if the SOF is incorrect. It is cleared automatically at receiver start-up phase. The bit is only valid for 106 kbit/s.
            • During the MFAuthent Command, bit ProtocolErr is set to logic 1, if the number of bytes received 
            in one data stream is incorrect.
        BIT 1 ParityErr
            Set to logic 1, if the parity check has failed. It is cleared automatically at receiver start-up phase. Only valid for ISO/IEC 14443A/MIFARE® communication at 106 kbit/s.
        BIT 2 CRCErr
            Set to logic 1, if bit RxCRCEn in register RxModeReg is set and the CRC calculation fails. It is cleared to logic 0 automatically at receiver start-up phase.
        BIT 3 CollErr
            Set to logic 1, if a bit-collision is detected. It is cleared automatically at receiver start-up phase. This bit is only valid during the bitwise anticollision at 106 kbit/s. During communication schemes at 212, 
            424 and 848 kbit/s this bit is always set to logic 0.
        BIT 4 BufferOvfl
            Set to logic 1, if the host or a MFRC522’s internal state machine (e.g. receiver) tries to write data into the FIFO buffer although the FIFO buffer is already full.
        BIT 5
            Reserved for future use.
        BIT 6 TempErr[1]
            Set to logic 1, if the internal temperature sensor detects overheating. 
            In this case, the antenna drivers are switched off automatically.
        BIT 7 WrErr
            Set to logic 1, when data is written into the FIFO by the host during the MFAuthent command or if data is written into the FIFO by the host during the time between sending the last bit on the RF interface and receiving the last bit on the RF interface. 
       */
        if (err_state & 0x01) {
            printf("Protocol error\n");
        }
        if (err_state & 0x02) {
            printf("Parity error\n");
        }
        if (err_state & 0x04) {
            printf("CRC error\n");
        }
        if (err_state & 0x08) {
            printf("Collision error\n");
            status = TAG_ERR;
        }
        if (err_state & 0x10) {
            printf("Buffer overflow\n");
        }
        if (err_state & 0x20) {
            printf("Temperature error\n");
        }
        if (err_state & 0x40) {
            printf("Write error\n");
        }
    }
    rc522_set_bitmask(ControlReg, 0x80); // stop timer now
    // printf("%s : status %02x, length of message %d, PCD err %02x\n"
    // ,__func__, status,*pOutLenBit, PcdErr);
    *recv_data = result;
    return status;
}

static uint8_t* rc522_request(uint8_t* res_n) {
    uint8_t* result = NULL;
    rc522_write(BitFramingReg, 0x07);

    uint8_t req_mode = PICC_REQIDL; //PICC_REQIDL/PICC_REQALL
    rc522_card_write(PCD_TRANSCEIVE, &req_mode, 1, &result, res_n);

    if(*res_n * 8 != 0x10) {
        free(result);
        return NULL;
    }

    return result;
}

/* Perform anticollision to select a single card and get it's UID
 * sequence:
 *
 * 0x93  0x20   unique identifier is sent by card
 * 0x93  0x70 UID + 2 CRC to select te card (pcdselect handled))
 * 0x95 = cascade level 2 ( 7 bytes)
 * 0x97 = cascade level 3 ( 10 bytes)
 */
static uint8_t* rc522_anticoll(uint8_t cascade) {
    uint8_t res_n;
    uint8_t collbits=0;

    /* 
     * LSB of the received bit is stored at bit position 0, the second on stored on ps 1
     * all bits of the last byte will be transferred */
    rc522_write(BitFramingReg, 0x00);

    // bits received after collision are cleared
    rc522_write(CollReg, 0x80);
    /* anti collision commands 0x93, 0x95 or 0x97 */
    /* NVB = number of valid bits/bytes
     * 
     * NVB : 0x20 = no part of UID is sent = anticollision command
     * 
     * 0x20 => top 4 bits is byte count = 2 valid bytes to transmit: cascade (or SEL or CL) + NVB
     * byte count 2 = minimum, 7 max
     * 
     * 0x20 => bottom 4 bits is bit count = 0 : number of valid data bits (include SEL+NVB) modulo 8
     *     
     * normal way to connect at first, second the UID will be asked
     * with collbits set to the correct number of UID bits
     */
    uint8_t* result=NULL;
    rc522_card_write(PCD_TRANSCEIVE, (uint8_t[]) { cascade, 0x20 + collbits }, 2, &result,&res_n);

    if(result && res_n != 5) { // all cards/tags serial numbers is 5 bytes long (?)
        free(result);
        return NULL;
    }

    return result;
}

static uint8_t* rc522_get_tag() {
    uint8_t* result = NULL;
    uint8_t* res_data = NULL;
    uint8_t res_data_n;

    res_data = rc522_request(&res_data_n);

    if(res_data != NULL) {
        free(res_data);

        result = rc522_anticoll(PICC_ANTICOLL1);

        if(result != NULL) {
            uint8_t buf[] = { 0x50, 0x00, 0x00, 0x00 };
            uint8_t* crc = rc522_calculate_crc(buf, 2);

            buf[2] = crc[0];
            buf[3] = crc[1];

            free(crc);

            rc522_card_write(0x0C, buf, 4, &res_data,&res_data_n);

            free(res_data);

            rc522_clear_bitmask(0x08, 0x08);

            return result;
        }
    }

    return NULL;
}

esp_err_t rc522_start(rc522_start_args_t start_args) {
    esp_err_t err = rc522_init(&start_args);
    return err != ESP_OK ? err : rc522_start2();
}

esp_err_t rc522_start2() {
    if(! hndl) { return ESP_ERR_INVALID_STATE; }

    hndl->scan_started = true;

    return ESP_OK;
}

esp_err_t rc522_pause() {
    if(! hndl) { return ESP_ERR_INVALID_STATE; }

    if(! hndl->scan_started) {
        return ESP_OK;
    }

    hndl->scan_started = false;

    return ESP_OK;
}

void rc522_destroy() {
    if(! hndl) { return; }

    rc522_pause(); // stop timer
    hndl->running = false; // task will delete itself

    if(hndl->spi) {
        spi_bus_remove_device(hndl->spi);
        spi_bus_free(hndl->config->spi_host_id);
        hndl->spi = NULL;
    }

    free(hndl->config);
    hndl->config = NULL;

    free(hndl);
    hndl = NULL;
}

static void rc522_task(void* arg) {
    while(hndl->running) {
        if(!hndl->scan_started) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        uint8_t* serial_no = rc522_get_tag();

        if(serial_no && ! hndl->tag_was_present_last_time) {
            rc522_tag_callback_t cb = hndl->config->callback;
            if(cb) { cb(serial_no); }
        }
        
        if((hndl->tag_was_present_last_time = (serial_no != NULL))) {
            free(serial_no);
            serial_no = NULL;
        }

        int delay_interval_ms = hndl->config->scan_interval_ms;

        if(hndl->tag_was_present_last_time) {
            delay_interval_ms *= 2; // extra scan-bursting prevention
        }

        vTaskDelay(delay_interval_ms / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}


// auth 
// https://github.com/song940/RFID-RC522/blob/master/RFID.cpp
