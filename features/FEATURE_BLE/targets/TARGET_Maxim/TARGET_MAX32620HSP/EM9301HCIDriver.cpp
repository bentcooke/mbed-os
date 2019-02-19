#include <stdio.h>
#include "CordioHCIDriver.h"
#include "mbed.h"
#include "hci_api.h"
#include "hci_cmd.h"
#include "hci_core.h"
#include "bstream.h"
#include "hci_mbed_os_adaptation.h"

#define HCI_RESET_RAND_CNT        4

namespace ble { 
namespace vendor {
namespace em9301 { 

class TransportDriver : public cordio::CordioHCITransportDriver { 
public: 
    TransportDriver(PinName mosi, PinName miso, PinName sclk, PinName ncs, PinName irq)
        : mosi(mosi, 1), miso(miso, PullDown), sck(sclk, 0), cs(ncs), irq(irq) { }

    virtual ~TransportDriver() { }

    virtual void initialize() { 
        irq.disable_irq();
        irq.fall(NULL);
        irq.rise(callback(this, &TransportDriver::on_controller_irq));
        irq.enable_irq();
    }

    virtual void terminate() {  }
    
    virtual uint16_t write(uint8_t type, uint16_t len, uint8_t *pData) { 
        irq.disable_irq();
        mosi = 1;
        wait_ns(100);
        cs = 0;
        wait_ns(100);

        uint8_t i = 0;
        while (i < len + 1) {
            while (miso == 0) { };
            uint8_t to_write = i == 0 ? type : pData[i - 1];
            uint8_t status = spi_write(to_write);
            if (status == 0xFF) {
                ++i;
            }
        }
        cs = 1;
        irq.enable_irq();
        return len;
    }

private:
    uint8_t spi_write(uint8_t out)  { 
        uint8_t in = 0;
        for (uint8_t i = 0; i < 8; ++i) { 
            // start sending bit 
            sck = 0;
            mosi = ((out >> (7 - i)) & 1);
            wait_ns(100);
            // capture miso, change clock
            sck = 1; 
            wait_ns(1);
            in |= ((miso.read() & 1) << (7 - i));
            wait_ns(100);
        }
        sck = 0;
        return in;
    }

    void wait_ns(size_t time_to_wait) { 
        for (size_t i = 0; i < ((time_to_wait / 10) + 1); ++i) { 
            __NOP();
        }
    }

    void on_controller_irq() { 
        // set MOSI to 0 ...
        mosi = 0; 
        wait_ns(100); 
        cs = 0; 
        wait_ns(100);

        while (irq.read() == 1) { 
            // set MOSI to 0 during the transaction 
            uint8_t value_read = spi_write(0);
            // push the byte in the stack
            on_data_received(&value_read, 1);
        }
        cs = 1;
    }

    DigitalOut mosi;
    DigitalIn miso;
    DigitalOut sck;
    DigitalOut cs;
    InterruptIn irq;
};

class HCIDriver : public cordio::CordioHCIDriver 
{
public:
    HCIDriver(TransportDriver& transport_driver, PinName rst) :
        cordio::CordioHCIDriver(transport_driver), reset(rst) { }

    virtual ~HCIDriver() { }

    virtual void do_initialize() { 
        printf("em9301 HCIDriver initialization\r\n");
    }

    virtual void do_terminate() { }
    
    virtual void handle_reset_sequence(uint8_t *pMsg) { 
        uint16_t       opcode;
        static uint8_t randCnt;

        /* if event is a command complete event */
        if (*pMsg == HCI_CMD_CMPL_EVT)
        {
            /* parse parameters */
            pMsg += HCI_EVT_HDR_LEN;
            pMsg++;                   /* skip num packets */
            BSTREAM_TO_UINT16(opcode, pMsg);
            pMsg++;                   /* skip status */

            /* decode opcode */
            switch (opcode)
            {
            case HCI_OPCODE_RESET:
                /* initialize rand command count */
                randCnt = 0;

                /* send next command in sequence */
                HciSetEventMaskCmd((uint8_t *) hciEventMask);
                break;

            case HCI_OPCODE_SET_EVENT_MASK:
                /* send next command in sequence */
                HciLeSetEventMaskCmd((uint8_t *) hciLeEventMask);
                break;

            case HCI_OPCODE_LE_SET_EVENT_MASK:
                /* send next command in sequence */
                HciReadBdAddrCmd();
                break;

            case HCI_OPCODE_READ_BD_ADDR:
                /* parse and store event parameters */
                BdaCpy(hciCoreCb.bdAddr, pMsg);

                /* send next command in sequence */
                HciLeReadBufSizeCmd();
                break;

            case HCI_OPCODE_LE_READ_BUF_SIZE:
                /* parse and store event parameters */
                BSTREAM_TO_UINT16(hciCoreCb.bufSize, pMsg);
                BSTREAM_TO_UINT8(hciCoreCb.numBufs, pMsg);

                /* initialize ACL buffer accounting */
                hciCoreCb.availBufs = hciCoreCb.numBufs;

                /* send next command in sequence */
                HciLeReadSupStatesCmd();
                break;

            case HCI_OPCODE_LE_READ_SUP_STATES:
                /* parse and store event parameters */
                memcpy(hciCoreCb.leStates, pMsg, HCI_LE_STATES_LEN);

                /* send next command in sequence */
                HciLeReadWhiteListSizeCmd();
                break;

            case HCI_OPCODE_LE_READ_WHITE_LIST_SIZE:
                /* parse and store event parameters */
                BSTREAM_TO_UINT8(hciCoreCb.whiteListSize, pMsg);

                /* send next command in sequence */
                HciLeReadLocalSupFeatCmd();
                break;

            case HCI_OPCODE_LE_READ_LOCAL_SUP_FEAT:
                /* parse and store event parameters */
                BSTREAM_TO_UINT16(hciCoreCb.leSupFeat, pMsg);

                /* send next command in sequence */
                hciCoreReadResolvingListSize();
                break;

            case HCI_OPCODE_LE_READ_RES_LIST_SIZE:
                /* parse and store event parameters */
                BSTREAM_TO_UINT8(hciCoreCb.resListSize, pMsg);

                /* send next command in sequence */
                hciCoreReadMaxDataLen();
                break;

            case HCI_OPCODE_LE_READ_MAX_DATA_LEN:
                {
                    uint16_t maxTxOctets;
                    uint16_t maxTxTime;

                    BSTREAM_TO_UINT16(maxTxOctets, pMsg);
                    BSTREAM_TO_UINT16(maxTxTime, pMsg);

                    /* use Controller's maximum supported payload octets and packet duration times
                    * for transmission as Host's suggested values for maximum transmission number
                    * of payload octets and maximum packet transmission time for new connections.
                    */
                    HciLeWriteDefDataLen(maxTxOctets, maxTxTime);
                }
                break;

            case HCI_OPCODE_LE_WRITE_DEF_DATA_LEN:
                if (hciCoreCb.extResetSeq)
                {
                    /* send first extended command */
                    (*hciCoreCb.extResetSeq)(pMsg, opcode);
                }
                else
                {
                    /* initialize extended parameters */
                    hciCoreCb.maxAdvDataLen = 0;
                    hciCoreCb.numSupAdvSets = 0;
                    hciCoreCb.perAdvListSize = 0;

                    /* send next command in sequence */
                    HciLeRandCmd();
                }
                break;
                
            case HCI_OPCODE_LE_READ_MAX_ADV_DATA_LEN:
            case HCI_OPCODE_LE_READ_NUM_SUP_ADV_SETS:
            case HCI_OPCODE_LE_READ_PER_ADV_LIST_SIZE:
                if (hciCoreCb.extResetSeq)
                {
                    /* send next extended command in sequence */
                    (*hciCoreCb.extResetSeq)(pMsg, opcode);
                }
                break;

            case HCI_OPCODE_LE_RAND:
                /* check if need to send second rand command */
                if (randCnt < (HCI_RESET_RAND_CNT-1))
                {
                    randCnt++;
                    HciLeRandCmd();
                }
                else
                {
                    signal_reset_sequence_done();
                }
                break;

            default:
                break;
            }
        }
    }

private:
    void hciCoreReadResolvingListSize(void)
    {
        /* if LL Privacy is supported by Controller and included */
        if ((hciCoreCb.leSupFeat & HCI_LE_SUP_FEAT_PRIVACY) &&
            (hciLeSupFeatCfg & HCI_LE_SUP_FEAT_PRIVACY))
        {
            /* send next command in sequence */
            HciLeReadResolvingListSize();
        }
        else
        {
            hciCoreCb.resListSize = 0;

            /* send next command in sequence */
            hciCoreReadMaxDataLen();
        }
    }

    void hciCoreReadMaxDataLen(void)
    {
    /* if LE Data Packet Length Extensions is supported by Controller and included */
        if ((hciCoreCb.leSupFeat & HCI_LE_SUP_FEAT_DATA_LEN_EXT) &&
            (hciLeSupFeatCfg & HCI_LE_SUP_FEAT_DATA_LEN_EXT))
        {
            /* send next command in sequence */
            HciLeReadMaxDataLen();
        }
        else
        {
            /* send next command in sequence */
            HciLeRandCmd();
        }
    }

    DigitalOut reset;
};

} // namespace em9301
} // namespace vendor 
} // namespace ble 

ble::vendor::cordio::CordioHCIDriver& ble_cordio_get_hci_driver() { 
    static ble::vendor::em9301::TransportDriver transport_driver(
        HCI_MOSI, HCI_MISO, HCI_SCK, HCI_CSN, HCI_IRQ
    );
    static ble::vendor::em9301::HCIDriver hci_driver(
        transport_driver, HCI_RST
    );
    return hci_driver;
}
