/*
************************************************************************************************************************
*                                                   CONVERTED CODE
*                                                   
*                                This code is converted from uCOS-III template code
*
************************************************************************************************************************
*/
/*
************************************************************************************************************************
*
*                                               MODULE - Data Logger
*
*                                                   STM32F407VGT6
*                                                        on
*                                              UTMRBC Mainboard v2.1.1
*
* Filename      : dtlog.c
* Version       : V1.00
* Programmer(s) : HW
* Note(s)       : (1) The format of the data packet:
*
************************************************************************************************************************
*/

/*
************************************************************************************************************************
*                                                   INCLUDE FILES
************************************************************************************************************************
*/
#include  "dtlog.h"

/*
************************************************************************************************************************
*                                                       DEFINES
************************************************************************************************************************
*/
#define DTLOG_FRAME_H1  0x01
#define DTLOG_FRAME_H2  0x02
#define DTLOG_FRAME_F1  0x03
#define DTLOG_FRAME_F2  0x04

/*
************************************************************************************************************************
*                                                   LOCAL CONSTANTS
************************************************************************************************************************
*/
                                                            /* pass buff length to bsp uart                           */
#if MOD_CFG_DTLOG_UART == 2
OS_MSG_SIZE BSP_UART2_INT_RX_BUFLEN = MOD_DTLOG_INT_RX_BUFF;
#elif MOD_CFG_DTLOG_UART == 3
OS_MSG_SIZE BSP_UART3_INT_RX_BUFLEN = MOD_DTLOG_INT_RX_BUFF;
#elif MOD_CFG_DTLOG_UART == 4
OS_MSG_SIZE BSP_UART4_INT_RX_BUFLEN = MOD_DTLOG_INT_RX_BUFF;
#elif MOD_CFG_DTLOG_UART == 5
OS_MSG_SIZE BSP_UART5_INT_RX_BUFLEN = MOD_DTLOG_INT_RX_BUFF;
#endif

/*
************************************************************************************************************************
*                                                      CONSTANTS
************************************************************************************************************************
*/
static const uint32_t dtlog_size[10] =
{
    1u, 1u, 2u, 2u, 4u, 4u, 8u, 8u, 4u, 8u
};


/*
************************************************************************************************************************
*                                                      DATA TYPES
*
* Note(s)     : (1) There is no support for string, as it should stay in msglog
************************************************************************************************************************
*/
typedef enum                                                /* List of supported data types                           */
{
    UINT08 = 0,
    SINT08 = 1,
    UINT16 = 2,
    SINT16 = 3,
    UINT32 = 4,
    SINT32 = 5,
    UINT64 = 6,
    SINT64 = 7,
    FLOT32 = 8,
    FLOT64 = 9,
} dtlog_type_t;

typedef struct                                              /* Cell of element of the list of data                    */
{
    const char*      name;
    dtlog_type_t     type;
    void*            data;
} dtlog_block_t;

typedef struct                                              /* Cell of element of the list of data                    */
{
    const char*      name;
    dtlog_type_t     type;
    void*            data;
    void*            buff;                                  /* to allow deploy all changes all together               */
} dtlog_rx_block_t;


/*
************************************************************************************************************************
*                                                     CUSTOM AREA
*                               (TODO: Please edit here, mod_dtlog.h, and also client side)
*
* Note(s)     : (1) The name is best to be less than 10 words, excessive character will be ignored.
************************************************************************************************************************
*/

extern float     IMUroll;
extern float     IMUpitch;
extern float     IMUyaw;
extern uint32_t  tick;

extern uint32_t   qeiA;
extern uint32_t   qeiB;
extern uint32_t   qeiC;
extern uint32_t   qeiD;
extern double     freq;

uint32_t   buf_qeiA;
uint32_t   buf_qeiB;
uint32_t   buf_qeiC;
uint32_t   buf_qeiD;
double     buf_freq;

#define DTLOG_TX_TOTAL     8
dtlog_block_t  DTLOG_TX_FORMAT [DTLOG_TX_TOTAL] =
{
   /* NAME          TYPE    DATAPTR    */
    { "tick"      , UINT32, &tick    },
    { "roll"      , FLOT32, &IMUroll },
    { "pitch"     , FLOT32, &IMUpitch},
    { "yaw"       , FLOT32, &IMUyaw  },
    { "qeiA"      , UINT32, &qeiA    },
    { "qeiB"      , UINT32, &qeiB    },
    { "qeiC"      , UINT32, &qeiC    },
    { "qeiD"      , UINT32, &qeiD    },
};

#define DTLOG_RX_TOTAL     4
dtlog_rx_block_t  DTLOG_RX_FORMAT [DTLOG_RX_TOTAL] =
{
   /* NAME          TYPE    DATAPTR    BUFFER   */
    { "qeiA"      , UINT32, &qeiA    , &buf_qeiA},
    { "qeiB"      , UINT32, &qeiB    , &buf_qeiB},
    { "qeiC"      , UINT32, &qeiC    , &buf_qeiC},
    { "yaw"       , FLOT32, &IMUyaw  , &buf_qeiD},
};

/*
************************************************************************************************************************
*                                                   LOCAL VARIABLES
************************************************************************************************************************
*/
uint8_t  cum[DTLOG_RX_TOTAL];                        /* cumulative length of each RX variable                  */
uint8_t  st;                                         /* state of frame recognition                             */
uint8_t  ct;                                         /* count of received character for data region            */
uint8_t  id;                                         /* index of the data on list                              */
uint8_t  pt;                                         /* pointer to current writing byte of the specific data   */

/*
************************************************************************************************************************
*                                                   LOCAL FUNCTION
************************************************************************************************************************
*/

static void data_load (void);

/*
************************************************************************************************************************
*                                                    DTlog Tx Task
*
* Description : Send a frame of variables raw data whenever data is ready
* Argument(s) : p_arg   arg of a OSTask, not used
* Return(s)   : none.
* Caller(s)   : APP.
************************************************************************************************************************
*/
void ModTaskDtlogInit (void)
{
    uint8_t i;

    tx_pool[0]  = DTLOG_FRAME_H1;
    tx_pool[1]  = DTLOG_FRAME_H2;
    tx_pool[DTLOG_TX_RAW_LEN+2]  = DTLOG_FRAME_F1;
    tx_pool[DTLOG_TX_RAW_LEN+3]  = DTLOG_FRAME_F2;

    st       = 0;
    ct       = 0;
    id       = 0;
    pt       = 0;

    for (i = 0; i != DTLOG_RX_TOTAL; i++) {                 /* Calculate cumulative length                            */
        if (i == 0) {
            cum[0] = dtlog_size[DTLOG_RX_FORMAT[0].type];
        } else {
            cum[i] = cum[i-1] + dtlog_size[DTLOG_RX_FORMAT[i].type];
        }
    }

}

void ModTaskDtlogTx (void)
{
    uint8_t  i;                                          /* multipurpose counter                                   */
    uint8_t  a;                                          /* multipurpose counter                                   */


	i = 0;
	a = 2;
	for (i = 0; i != DTLOG_TX_TOTAL; i++) {
		memcpy((void   *)&tx_pool[a],
			   (void   *)DTLOG_TX_FORMAT[i].data,
			   (size_t  )dtlog_size[DTLOG_TX_FORMAT[i].type]);
		a += dtlog_size[DTLOG_TX_FORMAT[i].type];
	}
	HAL_UART_Transmit_IT(&DTLOG_UART, tx_pool, MOD_DTLOG_INT_TX_BUFF);
}

void ModTaskDtlogRx (void)
{
    uint8_t  i;                                          /* multipurpose counter                                   */
    uint8_t *ptr;
    uint8_t  in;


   ptr = &rx_pool[0];
   for (i = 0; i != MOD_DTLOG_INT_RX_BUFF; i++) {
	   in = *ptr;
	   switch (st) {
		case 0:
			if (in == DTLOG_FRAME_H1) {
				st++;
			}
			break;
		case 1:
			if (in == DTLOG_FRAME_H2) {
				st++;
				ct = 0;                                 /* reset count and index to start receiving data          */
				id = 0;
				pt = 0;
			} else if (in == DTLOG_FRAME_H1) { st = 1; }
			else { st = 0; }
			break;
		case 2:
			((uint8_t*)(DTLOG_RX_FORMAT[id].buff))[pt] = in;
			ct++;
			pt++;
			if (ct == cum[id]) {                        /* current data has finished, proceed                     */
				id++;                                   /* next data                                              */
				pt = 0;                                 /* reset pointer to new data                              */
			}
			if (id == DTLOG_RX_TOTAL) {
				st++;                                   /* if all data has been transfer, can proceed to footer   */
			}
			break;
		case 3:
			if (in == DTLOG_FRAME_F1) { st++; }
			else if (in == DTLOG_FRAME_H1) { st = 1; }
			else { st = 0; }
			break;
		case 4:
			if (in == DTLOG_FRAME_F2) {                     /* get complete frame here                                */
				st = 0;
				data_load();
			} else if (in == DTLOG_FRAME_H1) {
				st = 1;
			} else {
				st = 0;
			}
			break;
		default:
			break;
		}
	   ptr++;
    }
    
}
static void data_load (void)
{
    for (int i = 0; i != DTLOG_RX_TOTAL; i++) {
        switch(DTLOG_RX_FORMAT[i].type) {
        case UINT08:
        case SINT08:
            *((uint8_t*)(DTLOG_RX_FORMAT[i].data)) = *((uint8_t*)(DTLOG_RX_FORMAT[i].buff));
            break;
        case UINT16:
        case SINT16:
            *((uint16_t*)(DTLOG_RX_FORMAT[i].data)) = *((uint16_t*)(DTLOG_RX_FORMAT[i].buff));
            break;
        case FLOT64:
        case UINT64:
        case SINT64:
            *((uint64_t*)(DTLOG_RX_FORMAT[i].data)) = *((uint64_t*)(DTLOG_RX_FORMAT[i].buff));
            break;
        default:
            *((uint32_t*)(DTLOG_RX_FORMAT[i].data)) = *((uint32_t*)(DTLOG_RX_FORMAT[i].buff));
            break;
        }
    }
}

