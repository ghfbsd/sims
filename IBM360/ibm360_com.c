/* ibm360_com.c: IBM 360 2703 communications controller

   Copyright (c) 2017-2020, Richard Cornwell

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   RICHARD CORNWELL BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


*/

#include "ibm360_defs.h"
#include "sim_sock.h"
#include "sim_tmxr.h"

#ifdef NUM_DEVS_COM
#define UNIT_COM           0

#define UNIT_V_DIRECT      (UNIT_V_UF + 0)
#define UNIT_DIRECT        (1 << UNIT_V_DIRECT)


/* u3 */
#define CMD_WR             0x01       /* Write data to com line */
#define CMD_RD             0x02       /* Read in data from com line */
#define CMD_NOP            0x03       /* Nop command */
#define CMD_PREP           0x06       /* Wait for incoming data  */
#define CMD_POLL           0x09       /* Poll (BSC) */
#define CMD_INH            0x0A       /* Read data without timeout  */
#define CMD_BRK            0x0D       /* Send break signal  */
#define CMD_SRCH           0x0E       /* Wait for EOT character  */
#define CMD_SETM           0x23       /* Set Mode (BSC) */
#define CMD_ENB            0x27       /* Enable line */
#define CMD_DIAL           0x29       /* Dial call */
#define CMD_DIS            0x2F       /* Disable line */

/* u3 second byte */
#define RECV               0x00100    /* Recieving data */
#define SEND               0x00200    /* Sending data */
#define ENAB               0x00400    /* Line enabled */
#define POLL               0x00800    /* Waiting for connection */
#define INPUT              0x01000    /* Input ready */
#define BREAK              0x02000    /* Return unit exception */
#define ADDR               0x04000    /* Address request recieved. */
#define BSCDLE             0x04000    /* BSC DLE char output */
#define ATTN               0x08000    /* Send attention signal */
#define BSCXPR             0x08000    /* BSC transparent mode */
#define ADDR9              0x10000    /* Address char 9 recieved */
#define BSCTXT             0x10000    /* BSC text mode */
#define BYPASS             0x20000    /* Don't echo output */
#define BSCEIB             0x20000    /* Send BSC EIB byte after ITB/ETB/ETX */

/* Upper 11 bits of u3 hold the device address */

/* u4 */
/* Where we are reading from */

/* u5 */
/* Sense byte 0 */
#define SNS_CMDREJ         0x80       /* Command reject */
#define SNS_INTVENT        0x40       /* Unit intervention required */
#define SNS_BUSCHK         0x20       /* Parity error on bus */
#define SNS_EQUCHK         0x10       /* Equipment check */
#define SNS_DATCHK         0x08       /* Data Check */
#define SNS_OVRRUN         0x04       /* Data overrun */
#define SNS_RECV           0x02       /* Receiving */
#define SNS_TIMEOUT        0x01       /* Timeout */

/* u6 */
/* Pointer into buffer */

#define CMD        u3
#define IPTR       u4
#define SNS        u5
#define BPTR       u6
#define TIME       recsize

/* BSC characters */
#define SOH 0x01
#define STX 0x02
#define ETX 0x03
#define HT  0x05
#define DLE 0x10
#define EOM 0x19
#define IBC 0x1f
#define ETB 0x26
#define ESC 0x27
#define ENQ 0x2d
#define SYN 0x32
#define EOT 0x37
#define NAK 0x3d
#define ACK0 0x61
#define ACK1 0x70

uint8       coml_startcmd(UNIT *uptr, uint16 chan,  uint8 cmd) ;
uint8       coml_haltio(UNIT *uptr);
t_stat      coml_srv(UNIT *uptr);
t_stat      com_reset(DEVICE *dptr);
t_stat      com_scan(UNIT *uptr);
t_stat      com_attach(UNIT *uptr, CONST char *);
t_stat      com_detach(UNIT *uptr);
t_stat      com_help (FILE *, DEVICE *, UNIT *, int32, const char *);
const char *com_description (DEVICE *);
t_stat      bsc_reset(DEVICE *dptr);
t_stat      bsc_attach(UNIT *uptr, CONST char *);
t_stat      bsc_detach(UNIT *uptr);
t_stat      bsc_help (FILE *, DEVICE *, UNIT *, int32, const char *);
const char *bsc_description (DEVICE *);

uint8       com_buf[NUM_UNITS_COM][256];
TMLN        com_ldsc[NUM_UNITS_COM];
TMXR        com_desc = { NUM_UNITS_COM, 0, 0, com_ldsc};
TMXR        bsc_desc = { NUM_UNITS_BSC, 0, 0, com_ldsc+NUM_UNITS_COM-NUM_UNITS_BSC};
int32       tmxr_poll = 10000;


MTAB                com_mod[] = {
    {0}
};

MTAB                coml_mod[] = {
    {MTAB_XTD | MTAB_VUN | MTAB_VALR, 0, "DEV", "DEV", &set_dev_addr,
        &show_dev_addr, NULL},
    {UNIT_DIRECT, 0, "DIALUP", "DIALUP", NULL, NULL, NULL, "Dialup line" },
    {UNIT_DIRECT, UNIT_DIRECT, "NODIAL", "NODIAL", NULL, NULL, NULL,
               "Hard wired line" },
    {0}
};

UNIT                com_unit[] = {
    {UDATA(&com_scan, UNIT_ATTABLE | UNIT_IDLE, 0)},        /* Line scanner */
};

UNIT                bsc_unit[] = {
    {UDATA(&com_scan, UNIT_ATTABLE | UNIT_IDLE, 0)},        /* Line scanner */
};

UNIT                coml_unit[] = {
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x030)},       /* 0 */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x031)},       /* 1 */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x032)},       /* 2 */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x033)},       /* 3 */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x034)},       /* 4 */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x035)},       /* 5 */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x036)},       /* 6 */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x037)},       /* 7 */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x038)},       /* 8 */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x039)},       /* 9 */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x03A)},       /* A */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x03B)},       /* B */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x03C)},       /* C */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x03D)},       /* D */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x03E)},       /* E */
    {UDATA(&coml_srv, UNIT_COM, 0), 0, UNIT_ADDR(0x03F)},       /* F */
};

struct dib com_dib = { 0xF0, NUM_UNITS_COM, NULL, coml_startcmd,
    coml_haltio, coml_unit, NULL};

DEVICE              com_dev = {
    "COM", com_unit, NULL, com_mod,
    NUM_DEVS_COM, 8, 15, 1, 8, 8,
    NULL, NULL, com_reset, NULL, &com_attach, &com_detach,
    NULL, DEV_MUX | DEV_DISABLE | DEV_DEBUG, 0, dev_debug,
    NULL, NULL, &com_help, NULL, NULL, &com_description
};

DEVICE              coml_dev = {
    "COML", coml_unit, NULL, coml_mod,
    NUM_UNITS_COM, 8, 15, 1, 8, 8,
    NULL, NULL, NULL, NULL, NULL, NULL,
    &com_dib, DEV_DISABLE | DEV_DEBUG, 0, dev_debug
};

DEVICE              bsc_dev = {
    "BSC", bsc_unit, NULL, com_mod,
    NUM_DEVS_BSC, 8, 15, 1, 8, 8,
    NULL, NULL, bsc_reset, NULL, &bsc_attach, &bsc_detach,
    NULL, DEV_MUX | DEV_DISABLE | DEV_DEBUG, 0, dev_debug,
    NULL, NULL, &bsc_help, NULL, NULL, &bsc_description
};


 /* Cent = 0xa0 */
static const uint8 com_2741_in[128] = {
   /*     SOH   STX   ETX   EOT   ENQ   ACK   BEL */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     /*0-37*/
   /* 8    9     A     B     C     D     E     F */
   /*BS   HT    LF    VT    FF    CR    SO    SI */
    0xDD, 0xFA, 0xB5, 0x00, 0x00, 0x5b, 0x00, 0x00,
   /*DLE  DC1   DC2   DC3   DC4   NAK   SYN   ETB */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   /*CAN  EM    SUB   ESC   FS    GS    RS    US */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   /*  sp    !    "     #     $     %     &     ' */
    0x81, 0xD7, 0x96, 0x16, 0x57, 0x8B, 0x61, 0x8D,    /* 40 - 77 */
   /*  (     )    *     +     ,     -     .     / */
    0x93, 0x95, 0x90, 0xE1, 0x37, 0x40, 0x76, 0x23,
   /*  0    1     2     3     4     5     6     7 */
    0x15, 0x02, 0x04, 0x07, 0x08, 0x0B, 0x0D, 0x0E,
   /*  8    9     :     ;     <     =     >     ? */
    0x10, 0x13, 0x88, 0x87, 0x84, 0x82, 0x8E, 0xA3,
   /*  @    A     B     C     D     E     F     G */
    0x20, 0xE2, 0xE4, 0xE7, 0xE8, 0xEB, 0xED, 0xEE,    /* 100 - 137 */
   /*  H    I     J     K     L     M     N     O */
    0xF0, 0xF3, 0xC3, 0xC5, 0xC6, 0xC9, 0xCA, 0xCC,
   /*  P    Q     R     S     T     U     V     W */
    0xCF, 0xD1, 0xD2, 0xA5, 0xA6, 0xA9, 0xAA, 0xAC,
   /*  X    Y     Z     [     \     ]     ^     _ */
    0xAF, 0xB1, 0xB2, 0x00, 0x00, 0x00, 0x00, 0xC0,
   /*  `    a     b     c     d     e     f     g */
    0x00, 0x62, 0x64, 0x67, 0x68, 0x6B, 0x6D, 0x6E,    /* 140 - 177 */
   /*  h    i     j     k     l     m     n     o */
    0x70, 0x73, 0x43, 0x45, 0x46, 0x49, 0x4A, 0x4C,
   /*  p    q     r     s     t     u     v     w */
    0x4F, 0x51, 0x52, 0x25, 0x26, 0x29, 0x2A, 0x2C,
   /*  x    y     z     {     |     }     ~   del*/
    0x2F, 0x31, 0x32, 0x00, 0xB7, 0x00, 0xF6, 0x00,
};

static const uint8 com_2741_out[256] = {
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
    0xff,  ' ',  '1', 0xff,  '2', 0xff, 0xff,  '3',       /* 0x0x */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
     '4', 0xff, 0xff,  '5', 0xff,  '6',  '7', 0xff,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
     '8', 0xff, 0xff,  '9', 0xff,  '0',  '#', 0xff,       /* 0x1x */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
     '@', 0xff, 0xff,  '/', 0xff,  's',  't', 0xff,       /* 0x2x */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff,  'u',  'v', 0xff,  'w', 0xff, 0xff,  'x',
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
    0xff,  'y',  'z', 0xff, 0xff, 0xff, 0xff,  ',',       /* 0x3x */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
     '-', 0xff, 0xff,  'j', 0xff,  'k',  'l', 0xff,       /* 0x4x */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff,  'm',  'n', 0xff,  'o', 0xff, 0xff,  'p',
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
    0xff,  'q',  'r', 0xff, 0xff, 0xff, 0xff,  '$',       /* 0x5x */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff, 0xff, 0xff, 0x0a, 0xff, 0x08, 0xff, 0xff,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
    0xff,  '&',  'a', 0xff,  'b', 0xff, 0xff,  'c',       /* 0x6x */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
     'd', 0xff, 0xff,  'e', 0xff,  'f',  'g', 0xff,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
     'h', 0xff, 0xff,  'i', 0xff, 0xff,  '.', 0xff,       /* 0x7x */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff, 0xff, 0x09, 0xff, 0xff, 0xff, 0xff, 0x7f,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
    0xff,  ' ',  '=', 0xff,  '<', 0xff, 0xff,  ';',       /* 0x8x */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
     ':', 0xff, 0xff,  '%', 0xff, '\'',  '>', 0xff,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
     '*', 0xff, 0xff,  '(', 0xff,  ')',  '"', 0xff,       /* 0x9x */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
    0xff, 0xff, 0xff,  '?', 0xff,  'S',  'T', 0xff,       /* 0xAx */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff,  'U',  'V', 0xff,  'W', 0xff, 0xff,  'X',
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
    0xff,  'Y',  'Z', 0xff, 0xff, 0xff, 0xff,  '|',       /* 0xBx */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
     '_',  '-', 0xff,  'J', 0xff,  'K',  'L', 0xff,       /* 0xCx */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff,  'M',  'N', 0xff,  'O', 0xff, 0xff,  'P',
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
    0xff,  'Q',  'R', 0xff, 0xff, 0xff, 0xff,  '!',       /* 0xDx */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff, 0xff, 0xff, 0x0a, 0xff, 0x08, 0xff, 0xff,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
    0xff,  '+',  'A', 0xff,  'B', 0xff, 0xff,  'C',       /* 0xEx */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
     'D', 0xff, 0xff,  'E', 0xff,  'F',  'G', 0xff,
   /*  0,    1,    2,    3,    4,    5,    6,    7,  */
     'H', 0xff, 0xff,  'I', 0xff, 0xff,  '~', 0xff,       /* 0xFx */
   /*  8,    9,    A,    B,    C,    D,    E,    F,  */
    0xff, 0xff, 0x09, 0xff, 0xff, 0xff, 0xff, 0x7f
};

/*
 * Issue a command to the 2701 controller.
 */
uint8  coml_startcmd(UNIT *uptr, uint16 chan,  uint8 cmd) {
    uint16         addr = GET_UADDR(uptr->CMD);
    DEVICE         *dptr = find_dev_from_unit(uptr);
    int            unit = (uptr - dptr->units);

    sim_debug(DEBUG_CMD, dptr, "CMD unit=%d %x\n", unit, cmd);
    if ((uptr->CMD & 0xff) != 0) {
       return SNS_BSY;
    }


    switch (cmd & 0x3) {
    case 0x3:              /* Control */
         if ((cmd == CMD_NOP) || (cmd & 0x10))
             /* 2703 treats 2701 SADxxx commands as NOP */
             return SNS_CHNEND|SNS_DEVEND;
    case 0x2:              /* Read command */
    case 0x1:              /* Write command */
         uptr->CMD |= cmd;
         uptr->SNS = 0;
         uptr->TIME = sim_os_msec();
         sim_activate(uptr, 200);
         return 0;

    case 0x0:               /* Status */
         if (cmd == 0x4) {  /* Sense */
            uptr->CMD |= cmd;
            sim_activate(uptr, 200);
            return 0;
         }
         break;
    }
    if (uptr->SNS & 0xff)
        return SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK;
    return SNS_CHNEND|SNS_DEVEND;
}

/*
 * Handle halt I/O instruction by stoping running command.
 */
uint8  coml_haltio(UNIT *uptr) {
    uint16         addr = GET_UADDR(uptr->CMD);
    DEVICE         *dptr = find_dev_from_unit(uptr);
    int            unit = (uptr - dptr->units);
    int            cmd = uptr->CMD & 0xff;

    sim_debug(DEBUG_CMD, dptr, "HLTIO unit=%d %x\n", unit, cmd);
    if ((com_unit[0].flags & UNIT_ATT) == 0)              /* attached? */
        return 3;

    switch (cmd) {
    case 0:
    case CMD_DIS:        /* Disable line */
    case CMD_DIAL:       /* Dial call */
    case 0x4:
         /* Short commands nothing to do */
         break;

    case CMD_PREP:       /* Wait for incoming data  */
         uptr->CMD &= ~(ADDR9|ADDR|BSCTXT|BSCXPR|BSCDLE|0xff);
         chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITEXP);
         break;

    case CMD_SETM:       /* Set mode */
    case CMD_POLL:       /* Poll */
    case CMD_INH:        /* Read data without timeout  */
    case CMD_RD:         /* Read in data from com line */
    case CMD_WR:         /* Write data to com line */
    case CMD_BRK:        /* Send break signal  */
    case CMD_SRCH:       /* Wait for EOT character  */
         uptr->CMD &= ~(ADDR9|ADDR|BSCTXT|BSCXPR|BSCDLE|0xff);
         chan_end(addr, SNS_CHNEND|SNS_DEVEND);
         break;
    case CMD_ENB:        /* Enable line */
         /* Terminate the operation */
         uptr->CMD &= ~(POLL|ADDR9|ADDR|BSCTXT|BSCXPR|BSCDLE|0xff);
         (void)tmxr_reset_ln(&com_ldsc[unit]);
         chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITEXP);
         return 1;
    }
    return 1;
}

/* Handle per unit commands */
t_stat coml_srv(UNIT * uptr)
{
    uint16              addr = GET_UADDR(uptr->CMD);
    DEVICE             *dptr = find_dev_from_unit(uptr);
    int                 unit = (uptr - dptr->units);
    int                 cmd = uptr->CMD & 0xff;
    uint8               ch;
    uint8               bsc = unit >= com_desc.lines-NUM_UNITS_BSC;


    if (bsc) sim_debug(DEBUG_DETAIL, dptr, "BSC stat %02x XPR=%s TXT=%s DLE=%s ENB=%d RCV=%d INP=%d MOD=%s\n",
        uptr->CMD & 0xff,
        uptr->CMD & BSCXPR ? "ON":"OFF",
        uptr->CMD & BSCTXT ? "ON":"OFF",
        uptr->CMD & BSCDLE ? "ON":"OFF",
        (uptr->CMD & ENAB) != 0, (uptr->CMD & RECV) != 0, (uptr->CMD & INPUT) != 0,
        uptr->CMD & BSCEIB ? "EIB":"NONE");

    switch (cmd) {
    case 0:
         break;

    case 0x4:
         ch = uptr->SNS & 0xff;
         sim_debug(DEBUG_DETAIL, dptr, "sense unit=%d 1 %x\n", unit, ch);
         chan_write_byte(addr, &ch) ;
         uptr->CMD &= ~0xff;
         chan_end(addr, SNS_CHNEND|SNS_DEVEND);
         break;

    case CMD_DIAL:       /* Dial call */
         uptr->SNS = SNS_CMDREJ;
         chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK);
         break;

    case CMD_INH:        /* Read data without timeout  */
    case CMD_RD:         /* Read in data from com line */
         uptr->SNS = 0;
         if (uptr->CMD & ENAB) {
             uptr->CMD |= RECV;
             if (com_ldsc[unit].conn == 0) {
                 uptr->CMD &= ~(0xff|BREAK|INPUT|ENAB|POLL|RECV);
                 uptr->SNS = SNS_INTVENT;
                 uptr->BPTR = 0;
                 uptr->IPTR = 0;
                 chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITEXP);
                 return SCPE_OK;
             }
#if 0
             /* Timeouts are not strictly needed for BSC read protocol */
             if (bsc && ((uptr->CMD & 0xff) == CMD_RD)) {
                 if (sim_os_msec() - uptr->TIME > 3000) {
                     /* 3 sec timeout on bisync read */
                     sim_debug(DEBUG_CMD, dptr, "COM: unit=%d timeout %08x %08x\n", unit, uptr->TIME, sim_os_msec());
                     uptr->CMD &= ~(0xff|BSCXPR|BSCTXT|BSCDLE|INPUT|RECV);
                     uptr->SNS = SNS_TIMEOUT;
                     uptr->BPTR = 0;
                     uptr->IPTR = 0;
                     chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK);
                     return SCPE_OK;
                 }
             }
#endif
             if (!bsc && (uptr->CMD & ADDR) && (uptr->BPTR == 0)) {
                 ch = 0x16;
                 sim_debug(DEBUG_CMD, dptr, "COM: unit=%d addr %02x\n", unit, ch);
                 uptr->CMD &= ~ADDR;
                 if (chan_write_byte( addr, &ch)) {
                     uptr->CMD &= ~(ADDR9|0xff);
                     chan_end(addr, SNS_CHNEND|SNS_DEVEND);
                     return SCPE_OK;
                 }
                 if (uptr->CMD & ADDR9) {
                     uptr->CMD &= ~(ADDR9|0xff);
                     sim_debug(DEBUG_CMD, dptr, "COM: unit=%d addr9 %02x\n", unit, ch);
                     chan_end(addr, SNS_CHNEND|SNS_DEVEND);
                     return SCPE_OK;
                 }
             } else if (uptr->CMD & BREAK) {
                 uptr->CMD &= ~(0xff|BREAK|INPUT|RECV);
                 uptr->SNS = SNS_INTVENT;
                 uptr->BPTR = 0;
                 uptr->IPTR = 0;
                 chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK|SNS_UNITEXP);
                 return SCPE_OK;
             } else if (uptr->CMD & INPUT) {
                 uint8 ack = bsc && (uptr->IPTR>0) && (com_buf[unit][uptr->IPTR-1] != DLE);
                 if (uptr->BPTR == uptr->IPTR) {
                     uptr->CMD &= ~(0xff|INPUT|RECV);
                     uptr->BPTR = 0;
                     uptr->IPTR = 0;
                     chan_end(addr, SNS_CHNEND|SNS_DEVEND);
                     return SCPE_OK;
                 }
                 ch = com_buf[unit][uptr->IPTR++];
                 if (!bsc && ch == 0x1f)
                     uptr->CMD |= ADDR;
                 if (ack && ((ch == ACK0) || (ch == ACK1))) {
                     uptr->CMD &= ~(0xff|INPUT|RECV);
                     uptr->IPTR = 0;
                     uptr->BPTR = 0;
                     chan_end(addr, SNS_CHNEND|SNS_DEVEND|(ch == ACK1 ? SNS_UNITEXP:0));
                     return SCPE_OK;
                 }
                 if (chan_write_byte( addr, &ch)) {
                     uptr->CMD &= ~(0xff|INPUT|RECV);
                     uptr->IPTR = 0;
                     uptr->BPTR = 0;
                     chan_end(addr, SNS_CHNEND|SNS_DEVEND);
                     return SCPE_OK;
                 }
             }
             sim_activate(uptr, 200);
         } else {
             sim_debug(DEBUG_CMD, dptr, "COM: unit=%d read error\n", unit);
             uptr->CMD &= ~0xff;
             chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITEXP);
         }
         break;

    case CMD_SETM:       /* Set mode */
         uptr->SNS = 0;
         uptr->CMD &= ~(0xff|BSCTXT|BSCXPR|BSCDLE|BSCEIB);
         if (!chan_read_byte (addr, &ch))
             ch = 0;
         uptr->CMD |= ch & 0x40 ? BSCEIB : 0;
#if 1
         if (uptr->CMD & ENAB) {
             /* bisync: If line already enabled, set mode CCW is restart of
                device.  Tell 2780 that previous transmission ends.  */
             tmxr_putc_ln( &com_ldsc[unit], SYN);
             tmxr_putc_ln( &com_ldsc[unit], EOT);
         }
#endif
         sim_debug(DEBUG_CMD, dptr, "COM: unit=%d set mode (%02x)\n", unit, ch);
         chan_end(addr, SNS_CHNEND|SNS_DEVEND);
         break;

    case CMD_POLL:       /* Modified write to com line */
    case CMD_WR:         /* Write data to com line */
         uptr->SNS = 0;
         if (uptr->CMD & ENAB) {
             if (com_ldsc[unit].conn == 0) {
                 uptr->CMD &= ~(0xff|BREAK|INPUT|ENAB|POLL);
                 uptr->SNS = SNS_INTVENT;
                 uptr->BPTR = 0;
                 uptr->IPTR = 0;
                 chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITEXP);
                 return SCPE_OK;
             }
             if (!bsc && (uptr->CMD & BREAK)) {
                 sim_debug(DEBUG_CMD, dptr, "COM: unit=%d attn write\n", unit);
                 uptr->CMD &= ~(0xff|BREAK);
                 uptr->SNS |= SNS_INTVENT;
                 chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK);
                 return SCPE_OK;
             }
             sim_debug(DEBUG_CMD, dptr, "COM: unit=%d write\n", unit);
             if (chan_read_byte (addr, &ch)) {
                 uptr->CMD &= ~0xff;
                 chan_end(addr, SNS_CHNEND|SNS_DEVEND);
                 if (bsc) 
                     tmxr_send_buffered_data(&com_ldsc[unit]);/* flush it */
             } else if (bsc) {
                 if (uptr->CMD & BSCXPR) {
                     if (ch == DLE)
                         tmxr_putc_ln( &com_ldsc[unit], DLE);
                 } else if (ch == DLE) {
                     uptr->CMD |= BSCDLE;
                 } else {
                     if ((uptr->CMD & BSCDLE) && (ch == ETX))
                         uptr->CMD |= BSCXPR;
                     uptr->CMD &= ~BSCDLE;
                 }
                 sim_debug(DEBUG_CMD, dptr, "COM: unit=%d BSC send %02x\n",
                     unit, ch);
                 tmxr_putc_ln( &com_ldsc[unit], ch);
                 sim_activate(uptr, 200);
             } else {
                 int32 data;
                 data = com_2741_out[ch];
                 sim_debug(DEBUG_CMD, dptr, "COM: unit=%d send %02x %02x '%c'\n",
                              unit, ch, data, isprint(data)? data: '^');
                 if (ch == 0x1f) {  /* Check for address character */
                     uptr->CMD |= ADDR;
                 } else if (ch == 0x16 && (uptr->CMD & ADDR)) {
                     uptr->CMD &= ~ADDR;
                 } else if (ch == 0xb8) {   /* Bypass */
                     uptr->CMD |= BYPASS;
                 } else if (ch == 0x58) {   /* Restore */
                     uptr->CMD &= ~(BYPASS|ADDR|ADDR9);
                 } else if ((uptr->CMD & ADDR) != 0 && ch == 0x13) {
                     uptr->CMD |= ADDR9;
                 } else if ((uptr->CMD & ADDR) == 0) {
                     if (ch == 0xf6) { /* UTF-8 logical not */
                         tmxr_putc_ln( &com_ldsc[unit], 0xC2);
                         tmxr_putc_ln( &com_ldsc[unit], 0xAC);
                     } else if (ch == 0xa0) { /* UTF-8 cent sign */
                         tmxr_putc_ln( &com_ldsc[unit], 0xC2);
                         tmxr_putc_ln( &com_ldsc[unit], 0xA2);
                     } else if (data != 0xff) 
                         tmxr_putc_ln( &com_ldsc[unit], data);
                     if (ch == 0x5b || ch == 0xdb)
                         tmxr_putc_ln( &com_ldsc[unit], '\r');
                 }
                 sim_activate(uptr, 2000);
             }
         } else {
             sim_debug(DEBUG_CMD, dptr, "COM: unit=%d write error\n", unit);
             uptr->CMD &= ~0xff;
             chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITEXP);
         }
         break;

    case CMD_BRK:        /* Send break signal  */
         uptr->CMD &= ~0xff;
         uptr->CMD |= ADDR;  /* also puts TA(I) in control mode */
         uptr->SNS = 0;
         chan_end(addr, SNS_CHNEND|SNS_DEVEND);
         break;

    case CMD_PREP:       /* Wait for incoming data  */
         uptr->SNS = 0;
         if (uptr->CMD & ENAB) {
             if (com_ldsc[unit].conn == 0) {
                 uptr->CMD &= ~(0xff|BREAK|INPUT|ENAB|POLL);
                 uptr->SNS = SNS_INTVENT;
                 uptr->BPTR = 0;
                 uptr->IPTR = 0;
                 chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITEXP);
                 return SCPE_OK;
             }
             uptr->CMD |= RECV;
             uptr->CMD &= ~(ADDR|ADDR9);
             if (uptr->CMD & (INPUT|BREAK)) {
                 uptr->CMD &= ~0xff;
                 chan_end(addr, SNS_CHNEND|SNS_DEVEND);
             } else {
                 sim_activate(uptr, 200);
             }
         } else {
             uptr->CMD &= ~0xff;
             chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITEXP);
         }
         break;

    case CMD_SRCH:       /* Wait for EOT character (NOP for the nonce) */
         sim_debug(DEBUG_CMD, dptr, "COM: unit=%d search\n", unit);
         uptr->CMD &= ~0xff;
         uptr->SNS = 0;
         chan_end(addr, SNS_CHNEND|SNS_DEVEND);
         break;

    case CMD_ENB:        /* Enable line */
         uptr->SNS = 0;
         if ((uptr->CMD & (POLL|ENAB)) == ENAB) {
             uptr->CMD &= ~0xff;
             uptr->BPTR = 0;
             uptr->IPTR = 0;
             sim_debug(DEBUG_CMD, dptr, "COM: unit=%d enable connect\n", unit);
             chan_end(addr, SNS_CHNEND|SNS_DEVEND);
         } else if ((uptr->CMD & POLL) == 0) {
             sim_debug(DEBUG_CMD, dptr, "COM: unit=%d enable\n", unit);
             (void)tmxr_set_get_modem_bits(&com_ldsc[unit], TMXR_MDM_DTR,
                   0, NULL);
             uptr->CMD |= POLL;
         }
         break;

    case CMD_DIS:        /* Disable line */
         uptr->SNS = 0;
         sim_debug(DEBUG_CMD, dptr, "COM: unit=%d disable\n", unit);
         (void)tmxr_set_get_modem_bits(&com_ldsc[unit], 0, TMXR_MDM_DTR, NULL);
         if (!bsc)
             (void)tmxr_reset_ln(&com_ldsc[unit]);
         uptr->CMD &= ~(0xff|POLL|ENAB) ;
         chan_end(addr, SNS_CHNEND|SNS_DEVEND);
         break;

    default:
         uptr->SNS = SNS_CMDREJ;
         uptr->CMD &= ~(0xff|BREAK|INPUT|RECV|SEND|POLL);
         chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK);
         break;
    }

    if ((uptr->CMD & (ENAB|RECV)) == (ENAB|RECV)) {
       int32 data = tmxr_getc_ln(&com_ldsc[unit]);
       if (bsc && uptr->BPTR<sizeof(com_buf[unit])) {
           /* BSC operation, in contrast to async (2741) operation, ingests
              as many characters from the connection as required to determine
              the ending status from the input CCW -- otherwise, there is too much
              state to carry between calls to the service routine.  
              The ending status is carried in the last byte in the input
              buffer: ACK0 - CE+DE, ACK1 - CE+DE+UE.  The last ACKx byte is
              not transmitted to storage.
           */
           uint8 gotdle;
           do {
               if ((data & TMXR_VALID) == 0) break;
               ch = data & 0xff;
               sim_debug(DEBUG_DATA, dptr, "unit=%d BSC read %02x\n", unit, ch);
               gotdle = (uptr->BPTR > 0) && (com_buf[unit][uptr->BPTR-1] == DLE);
               if (uptr->CMD & BSCTXT) {
                   /* text mode */
                   if (uptr->CMD & BSCXPR) {
                       if (gotdle) switch (ch) {
                       case DLE:
                           com_buf[unit][uptr->BPTR++] = ch;
                           break;
                       case SYN:
                           break;
                       case ETB:
                           (void) tmxr_getc_ln(&com_ldsc[unit]); /* BCC */
                           (void) tmxr_getc_ln(&com_ldsc[unit]); /* BCC */
                       case EOT:
                       case ETX:
                           /* End transparent mode, end read */
                       case IBC:
                           /* End transparent mode, continue read */
                           uptr->CMD &= ~BSCXPR;
                           com_buf[unit][uptr->BPTR++] = DLE;
                           com_buf[unit][uptr->BPTR++] = ch;
                           if (uptr->CMD & BSCEIB)
                               com_buf[unit][uptr->BPTR++] = 0;
                           if (ch != IBC) {
                               com_buf[unit][uptr->BPTR++] = ACK0;
                               uptr->CMD |= INPUT;
                               uptr->CMD &= ~RECV;
                               sim_activate(uptr, 200);
                               return SCPE_OK;
                           }
                           break;
                       case ENQ:
                           uptr->CMD &= ~(BSCXPR|BSCTXT);
                           com_buf[unit][uptr->BPTR++] = DLE;
                           com_buf[unit][uptr->BPTR++] = ch;
                           com_buf[unit][uptr->BPTR++] = ACK0;
                           uptr->CMD |= INPUT;
                           uptr->CMD &= ~RECV;
                           sim_activate(uptr, 200);
                           return SCPE_OK;
                       default:
                           com_buf[unit][uptr->BPTR++] = DLE;
                           com_buf[unit][uptr->BPTR++] = ch;
                           break;
                       }
                       else switch(ch) {
                       case SYN:
                           break;
                       case ETB:
                           (void) tmxr_getc_ln(&com_ldsc[unit]); /* BCC */
                           (void) tmxr_getc_ln(&com_ldsc[unit]); /* BCC */
                       case EOT:
                       case ETX:
                       case ENQ: /* End read */
                       case IBC: /* Continue read */
                           if (uptr->CMD & BSCEIB)
                               com_buf[unit][uptr->BPTR++] = 0;
                           com_buf[unit][uptr->BPTR++] = ch;
                           if (ch != IBC) {
                               com_buf[unit][uptr->BPTR++] = ACK0;
                               uptr->CMD |= INPUT;
                               uptr->CMD &= ~RECV;
                               sim_activate(uptr, 200);
                               return SCPE_OK;
                           }
                       case DLE:
                       default:
                           com_buf[unit][uptr->BPTR++] = ch;
                           break;
                       }
                   } else {
                       if (ch != SYN) {
                           com_buf[unit][uptr->BPTR++] = ch;
                           switch(ch) {
                           case ETB:
                               (void) tmxr_getc_ln(&com_ldsc[unit]); /* BCC */
                               (void) tmxr_getc_ln(&com_ldsc[unit]); /* BCC */
                           case EOT:
                           case ETX:
                           case ENQ: /* End read */
                           case IBC: /* Continue read */
                               if (uptr->CMD & BSCEIB)
                                   com_buf[unit][uptr->BPTR++] = 0;
                               if (ch != IBC) {
                                   com_buf[unit][uptr->BPTR++] = ACK0;
                                   uptr->CMD |= INPUT;
                                   uptr->CMD &= ~RECV;
                                   sim_activate(uptr, 200);
                                   return SCPE_OK;
                               }
                           case 0x60:
                           case 0x61:
                           case 0x70: /* Check if 2 char seq. DLE x (ACK0, ACK1) */
                               if (gotdle) {
                                   com_buf[unit][uptr->BPTR++] = ACK0;
                                   uptr->CMD |= INPUT;
                                   uptr->CMD &= ~RECV;
                                   sim_activate(uptr, 200);
                                   return SCPE_OK;
                               }
                           default:
                               break;
                           }
                       }
                   }
               } else {
                   /* not text mode */
                   if (ch != SYN) {
                       if (gotdle) { /* Check if 2 char seq. DLE x (ACK0, ACK1) */
                           if ((ch & 0xf0) == 0x60 || (ch & 0xf0) == 0x70) {
                               com_buf[unit][uptr->BPTR++] = ch;
                               com_buf[unit][uptr->BPTR++] = ACK0;
                               uptr->CMD |= INPUT;
                               uptr->CMD &= ~RECV;
                               sim_activate(uptr, 200);
                               return SCPE_OK;
                           }
                           if (ch == STX) { /* DLE STX - transparent mode */
                               com_buf[unit][uptr->BPTR++] = ch;
                               uptr->CMD |= BSCTXT|BSCXPR;
                           }
                       } else {
                           com_buf[unit][uptr->BPTR++] = ch;
                           switch (ch) {
                           case EOT:
                               com_buf[unit][uptr->BPTR++] = ACK1;
                               uptr->CMD |= INPUT;
                               uptr->CMD &= ~RECV;
                               sim_activate(uptr, 200);
                               return SCPE_OK;
                           case ENQ:
                           case ETX:
                           case NAK:
                               com_buf[unit][uptr->BPTR++] = ACK0;
                               uptr->CMD |= INPUT;
                               uptr->CMD &= ~RECV;
                               sim_activate(uptr, 200);
                               return SCPE_OK;
                           case SOH:
                           case STX:
                               uptr->CMD |= BSCTXT;
                           default:
                               break;
                           }
                       }
                   }
               }
               data = tmxr_getc_ln(&com_ldsc[unit]);
           } while(1);

       } else if ((data & TMXR_VALID) != 0) {
           ch = com_2741_in[data & 0x7f];
           sim_debug(DEBUG_DATA, dptr, "COML: unit=%d read '%c' %02x\n", unit, data, ch);
           if (data & SCPE_BREAK) {
                uptr->CMD |= BREAK;
                return SCPE_OK;
           }
           /* Handle end of buffer */
           switch (data & 0x7f) {
           case '\r':
           case '\n':
                 com_buf[unit][uptr->BPTR++] = 0x5b;
                 com_buf[unit][uptr->BPTR++] = 0x1f;
                 uptr->CMD |= INPUT;
                 uptr->CMD &= ~RECV;
                 uptr->IPTR = 0;
                 tmxr_putc_ln( &com_ldsc[unit], '\r');
                 tmxr_putc_ln( &com_ldsc[unit], '\n');
                 break;

           case 0177:
           case '\b':
                 if (uptr->BPTR != 0) {
                      uptr->BPTR--;
                      tmxr_putc_ln( &com_ldsc[unit], '\b');
                      tmxr_putc_ln( &com_ldsc[unit], ' ');
                      tmxr_putc_ln( &com_ldsc[unit], '\b');
                 }
                 break;

           case 025: /* ^U clear line */
                 while(uptr->BPTR > 0) {
                     tmxr_putc_ln( &com_ldsc[unit], '\b');
                     tmxr_putc_ln( &com_ldsc[unit], ' ');
                     tmxr_putc_ln( &com_ldsc[unit], '\b');
                     uptr->BPTR--;
                 }
                 break;

           case 03:  /* ^C */
                 uptr->CMD |= BREAK;
                 uptr->CMD &= ~RECV;
                 break;

           default:
                 if (uptr->BPTR < 253) {
                     if (ch == 0x00) {
                        sim_putchar('\007');
                     } else {
                        com_buf[unit][uptr->BPTR++] = ch;
                        if ((uptr->CMD & BYPASS) == 0)
                            tmxr_putc_ln( &com_ldsc[unit], data);
                     }
                 } else {
                     com_buf[unit][uptr->BPTR++] = 0x5b;
                     com_buf[unit][uptr->BPTR++] = 0x1f;
                     uptr->CMD |= INPUT;
                     uptr->CMD &= ~RECV;
                     uptr->BPTR &= 0xff;
                 }
           }
       }
    }
    return SCPE_OK;
}

/* Scan for new connections, flush and poll for data */
t_stat com_scan(UNIT * uptr)
{
    UNIT      *line;
    int32      ln;
    uint8      bsc = uptr == bsc_unit ? 1:0;
    TMXR      *desc = bsc ? &bsc_desc : &com_desc;

    sim_activate(uptr, tmxr_poll);          /* continue poll */
    if ((uptr->flags & UNIT_ATT) == 0)              /* attached? */
        return SCPE_OK;
    ln = tmxr_poll_conn (desc);                     /* look for connect */
    if (ln >= 0) {                                  /* got one? rcv enb*/
        sim_debug(DEBUG_DETAIL, &com_dev, "%s line connect %d\n",
            bsc?"BSC":"COM",ln);
        ln += bsc ?  NUM_UNITS_COM-NUM_UNITS_BSC : 0;
        line = &coml_unit[ln];
        if (line->CMD & ENAB)                        /* Already connected */
            return SCPE_OK;
        if ((line->CMD & POLL) == 0) {               /* Check if not polling */
            if (line->flags & UNIT_DIRECT) {
                 set_devattn(GET_UADDR(line->CMD), SNS_ATTN);
                 line->CMD |= ENAB|ADDR;
                 com_ldsc[ln].rcve = 1;                 /* Mark as ok */
                 sim_activate(line, 200);
            } else {
                 (void)tmxr_set_get_modem_bits(&com_ldsc[ln], 0, TMXR_MDM_DTR, NULL);
                 (void)tmxr_reset_ln(&com_ldsc[ln]);
            }
        } else {
             com_ldsc[ln].rcve = 1;                 /* Mark as ok */
             line->CMD &= ~POLL;
             line->CMD |= ENAB;
             sim_activate(line, 200);
        }
    }

    /* See if a line is disconnected with no command on it. */
    for (ln = 0; ln < com_desc.lines; ln++) {
        line = &coml_unit[ln];
        if ((line->CMD & (RECV|ENAB)) == ENAB && tmxr_rqln(&com_ldsc[ln]) > 0) {
            set_devattn(GET_UADDR(line->CMD), SNS_ATTN);
        }
    }
    tmxr_poll_tx(desc);
    tmxr_poll_rx(desc);
    return SCPE_OK;
}

t_stat
com_reset(DEVICE * dptr)
{
    sim_activate(&com_unit[0], tmxr_poll);
    return SCPE_OK;
}


t_stat
com_attach(UNIT * uptr, CONST char *cptr)
{
    t_stat        r;
    int           i;

    if ((r = tmxr_attach(&com_desc, uptr, cptr)) != SCPE_OK)
       return r;
    for (i = 0; i< com_desc.lines; i++) {
        coml_unit[i].CMD &= ~0x3ffff;
    }
    sim_activate(uptr, tmxr_poll);
    return SCPE_OK;
}

t_stat
com_detach(UNIT * uptr)
{
    t_stat        r;
    int           i;

    for (i = 0; i< com_desc.lines; i++) {
        (void)tmxr_set_get_modem_bits(&com_ldsc[i], 0, TMXR_MDM_DTR, NULL);
        (void)tmxr_reset_ln(&com_ldsc[i]);
        coml_unit[i].CMD &= ~0x3ffff;
    }
    sim_cancel(uptr);
    r = tmxr_detach(&com_desc, uptr);
    return r;
}

t_stat com_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag,
    const char *cptr)
{
fprint_set_help (st, dptr);
fprint_show_help (st, dptr);
return SCPE_OK;
}

const char *com_description (DEVICE *dptr)
{
return "IBM 2703 communications controller";
}

t_stat
bsc_reset(DEVICE * dptr)
{
    sim_activate(&bsc_unit[0], tmxr_poll);
    (void)tmxr_set_notelnet (&bsc_desc);
    return SCPE_OK;
}

t_stat
bsc_attach(UNIT * uptr, CONST char *cptr)
{
    t_stat        r;
    int           i;

    if ((r = tmxr_attach(&bsc_desc, uptr, cptr)) != SCPE_OK)
       return r;
    for (i = com_desc.lines-bsc_desc.lines; i< com_desc.lines; i++) {
        coml_unit[i].CMD &= ~0x3ffff;
        coml_unit[i].flags |= UNIT_DIRECT;
    }
    sim_activate(uptr, tmxr_poll);
    return SCPE_OK;
}

t_stat
bsc_detach(UNIT * uptr)
{
    t_stat        r;
    int           i;

    for (i = com_desc.lines-bsc_desc.lines; i< com_desc.lines; i++) {
        (void)tmxr_set_get_modem_bits(&com_ldsc[i], 0, TMXR_MDM_DTR, NULL);
        (void)tmxr_reset_ln(&com_ldsc[i]);
        coml_unit[i].CMD &= ~0x3ffff;
        coml_unit[i].flags &= ~UNIT_DIRECT;
    }
    sim_cancel(uptr);
    r = tmxr_detach(&bsc_desc, uptr);
    return r;
}

t_stat bsc_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag,
    const char *cptr)
{
fprint_set_help (st, dptr);
fprint_show_help (st, dptr);
return SCPE_OK;
}

const char *bsc_description (DEVICE *dptr)
{
return "IBM 2703 communications controller (bisync line)";
}

#endif
