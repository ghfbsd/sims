/* ibm360_cdr.c: IBM 360 Card Reader.

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

   This is the standard card reader.

   These units each buffer one record in local memory and signal
   ready when the buffer is full or empty. The channel must be
   ready to recieve/transmit data when they are activated since
   they will transfer their block during chan_cmd. All data is
   transmitted as BCD characters.

*/

#include "ibm360_defs.h"
#include "sim_defs.h"
#include "sim_card.h"

#ifdef NUM_DEVS_CDR
#define UNIT_CDR       UNIT_ATTABLE | UNIT_RO | UNIT_DISABLE | MODE_029


#define CHN_SNS        0x04       /* Sense command */

/* Device status information stored in u3 */
#define CDR_RD         0x02       /* Read command */
#define CDR_FEED       0x03       /* Feed next card */
#define CDR_CMDMSK     0x27       /* Mask command part. */
#define CDR_MODE       0x20       /* Mode operation */
#define CDR_STKMSK     0xC0       /* Mask for stacker */
#define CDP_WR         0x09       /* Punch command */
#define CDR_CARD       0x100      /* Unit has card in buffer */
#define CDR_EOF        0x200      /* EOF indicator */

/* Upper 11 bits of u3 hold the device address */

/* u4 holds current column, */

/* in u5 packs sense byte 0,1 and 3 */
/* Sense byte 0 */
#define SNS_CMDREJ     0x80       /* Command reject */
#define SNS_INTVENT    0x40       /* Unit intervention required */
#define SNS_BUSCHK     0x20       /* Parity error on bus */
#define SNS_EQUCHK     0x10       /* Equipment check */
#define SNS_DATCHK     0x08       /* Data Check */
#define SNS_OVRRUN     0x04       /* Data overrun */
#define SNS_SEQUENCE   0x02       /* Unusual sequence */
#define SNS_CHN9       0x01       /* Channel 9 on printer */

#define CMD    u3
#define COL    u4
#define SNS    u5


/* std devices. data structures

   cdr_dev       Card Reader device descriptor
   cdr_unit      Card Reader unit descriptor
   cdr_reg       Card Reader register list
   cdr_mod       Card Reader modifiers list
*/


uint8               cdr_startcmd(UNIT *, uint16,  uint8);
t_stat              cdr_boot(int32, DEVICE *);
t_stat              cdr_srv(UNIT *);
t_stat              cdr_reset(DEVICE *);
t_stat              cdr_attach(UNIT *, CONST char *);
t_stat              cdr_detach(UNIT *);
t_stat              cdr_help(FILE *, DEVICE *, UNIT *, int32, const char *);
const char         *cdr_description(DEVICE *);


UNIT                cdr_unit[] = {
   {UDATA(cdr_srv, UNIT_CDR, 0), 300, UNIT_ADDR(0x0C)},
#if NUM_DEVS_CDR > 1
   {UDATA(cdr_srv, UNIT_CDR | UNIT_DIS, 0), 300, UNIT_ADDR(0x1C)},
#if NUM_DEVS_CDR > 2
   {UDATA(cdr_srv, UNIT_CDR | UNIT_DIS, 0), 300, UNIT_ADDR(0x40C)},
#if NUM_DEVS_CDR > 3
   {UDATA(cdr_srv, UNIT_CDR | UNIT_DIS, 0), 300, UNIT_ADDR(0x41C)},
#endif
#endif
#endif
};

MTAB                cdr_mod[] = {
    {MTAB_XTD | MTAB_VUN, 0, "FORMAT", "FORMAT",
               &sim_card_set_fmt, &sim_card_show_fmt, NULL},
    {MTAB_XTD | MTAB_VUN | MTAB_VALR, 0, "DEV", "DEV", &set_dev_addr,
        &show_dev_addr, NULL},
    {0}
};

struct dib cdr_dib = { 0xFF, 1, NULL, cdr_startcmd, NULL, cdr_unit};

DEVICE              cdr_dev = {
    "CDR", cdr_unit, NULL, cdr_mod,
    NUM_DEVS_CDR, 8, 15, 1, 8, 8,
    NULL, NULL, NULL, &cdr_boot, &cdr_attach, &cdr_detach,
    &cdr_dib, DEV_UADDR | DEV_DISABLE | DEV_DEBUG | DEV_CARD, 0, crd_debug
};


/*
 * Start card reader to read in one card.
 */
uint8  cdr_startcmd(UNIT *uptr, uint16 chan,  uint8 cmd) {
    DEVICE         *dptr = find_dev_from_unit(uptr);
    int            unit = (uptr - dptr->units);

    if ((uptr->CMD & CDR_CMDMSK) != 0) {
        if ((uptr->flags & UNIT_ATT) != 0)
            return SNS_BSY;
        return SNS_DEVEND;
    }

    sim_debug(DEBUG_CMD, dptr, "CMD unit=%d %x\n", unit, cmd);
    if (cmd != 4 && sim_card_eof(uptr) == 1) {
        uint16   *image = (uint16 *)(uptr->up7);
        uptr->SNS = SNS_INTVENT;
        sim_read_card(uptr, image);   /* Read in the EOF */
        return SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK;
    }
    if (cmd != 4 && (uptr->flags & UNIT_ATT) == 0) {
        uptr->SNS = SNS_INTVENT;
        return SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK;
    }
    switch (cmd & 0x7) {
    case 2:              /* Read command */
         if ((cmd & 0xc0) != 0xc0)
             uptr->CMD &= ~CDR_CARD;
         if (uptr->CMD & CDR_EOF) {
             uptr->CMD &= ~CDR_EOF;
             uptr->SNS &= ~SNS_INTVENT;
             return SNS_CHNEND|SNS_DEVEND|SNS_UNITEXP;
         }
         uptr->CMD &= ~(CDR_CMDMSK);
         uptr->CMD |= (cmd & CDR_CMDMSK);
         sim_activate(uptr, 1000);       /* Start unit off */
         uptr->COL = 0;
         uptr->SNS = 0;
         return 0;

    case 3:              /* Control */
         uptr->SNS = 0;
         uptr->CMD &= ~(CDR_CMDMSK|CDR_CARD);
         if (cmd == 0x3)
             return SNS_CHNEND|SNS_DEVEND;
         if ((cmd & 0x30) != 0x20 || (cmd & 0xc0) == 0xc0) {
             uptr->SNS |= SNS_CMDREJ;
             return SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK;
         }
         uptr->CMD |= (cmd & CDR_CMDMSK);
         uptr->COL = 0;
         sim_activate(uptr, 1000);       /* Start unit off */
         return 0;

    case 0:               /* Status */
         break;

    case 4:               /* Sense */
         uptr->CMD &= ~(CDR_CMDMSK);
         uptr->CMD |= (cmd & CDR_CMDMSK);
         sim_activate(uptr, 10);
         return 0;

    default:              /* invalid command */
         uptr->SNS |= SNS_CMDREJ;
         break;
    }

    if (uptr->SNS)
        return SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK;
    return SNS_CHNEND|SNS_DEVEND;
}

/* Handle transfer of data for card reader */
t_stat
cdr_srv(UNIT *uptr) {
    int       addr = GET_UADDR(uptr->CMD);
    uint16   *image = (uint16 *)(uptr->up7);

    if ((uptr->CMD & CDR_CMDMSK) == CHN_SNS) {
         uint8 ch = uptr->SNS;
         if (ch == 0 && (uptr->flags & UNIT_ATT) == 0)
             ch = SNS_INTVENT;
         else if (sim_card_eof(uptr))
             ch |= SNS_INTVENT;
         chan_write_byte(addr, &ch);
         chan_end(addr, SNS_CHNEND|SNS_DEVEND);
         uptr->CMD &= ~(CDR_CMDMSK);
         uptr->SNS &= ~(SNS_CMDREJ+SNS_INTVENT);
         return SCPE_OK;
    }

    /* Check if new card requested if not status poll. */
    if ((uptr->CMD & CDR_CARD) == 0 && (uptr->CMD & CDR_CMDMSK)) {
       int u = uptr-cdr_unit;
       switch(sim_read_card(uptr, image)) {
       case CDSE_EMPTY:
            uptr->CMD |= CDR_EOF;
       case CDSE_EOF:
            uptr->CMD &= ~CDR_CMDMSK;
            sim_debug(DEBUG_CMD, &cdr_dev, "CMD unit=%d %x: %s\n",
                u, uptr->CMD, uptr->CMD & CDR_EOF ? "EMPTY":"EOF");
            if (((uptr->CMD & CDR_CMDMSK) & ~CDR_MODE) == CDR_RD) {
                /* Only give UE on a read cmd, not control.  From
                   2821 manual: "After the last card has been read
                   from the buffer and stacked in the selected
                   stacker, unit-exception status is given at
                   initial selection of the next read command." */
                chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITEXP);
                return SCPE_OK;
            }
            chan_end(addr, SNS_CHNEND|SNS_DEVEND);
            break;
       case CDSE_ERROR:
            uptr->SNS = SNS_INTVENT;
            uptr->CMD &= ~CDR_CMDMSK;
            chan_end(addr, SNS_CHNEND|SNS_DEVEND|SNS_UNITCHK);
            sim_debug(DEBUG_CMD, &cdr_dev, "CMD unit=%d %x: ERROR\n", u,
                uptr->CMD);
            return SCPE_OK;
       case CDSE_OK:
            uptr->CMD |= CDR_CARD;
            if (((uptr->CMD & CDR_CMDMSK) & ~CDR_MODE) == CDR_FEED) {
                chan_end(addr, SNS_CHNEND|SNS_DEVEND);
                uptr->CMD &= ~(CDR_CMDMSK);
                return SCPE_OK;
            }
            break;
       }
       sim_activate(uptr, 10000);       /* Start unit off */
       return SCPE_OK;
    }

    /* Copy next column over */
    if (((uptr->CMD & CDR_CMDMSK) & ~CDR_MODE) == CDR_RD) {
        int                  u = uptr-cdr_unit;
        uint16               xlat;
        uint8                ch = 0;

        xlat = sim_hol_to_ebcdic(image[uptr->COL]);

        if (xlat == 0x100) {
            uptr->SNS |= SNS_DATCHK;
            ch = 0x00;
        } else
            ch = (uint8)(xlat&0xff);
        if (chan_write_byte(addr, &ch)) {
           uptr->CMD &= ~(CDR_CMDMSK);
           chan_end(addr, SNS_CHNEND|SNS_DEVEND|(uptr->SNS ? SNS_UNITCHK:0));
           return SCPE_OK;
       } else {
           uptr->COL++;
           sim_debug(DEBUG_DATA, &cdr_dev, "%d: Char > %02x\n", u, ch);
        }
        if (uptr->COL == 80) {
            uptr->CMD &= ~(CDR_CMDMSK);
            chan_end(addr, SNS_CHNEND|SNS_DEVEND|(uptr->SNS ? SNS_UNITCHK:0));
        }
        sim_activate(uptr, 100);
    }
    return SCPE_OK;
}

/* Boot from given device */
t_stat
cdr_boot(int32 unit_num, DEVICE * dptr)
{
    UNIT               *uptr = &dptr->units[unit_num];

    if ((uptr->flags & UNIT_ATT) == 0)
       return SCPE_UNATT;       /* attached? */
    return chan_boot(GET_UADDR(uptr->CMD), dptr);
}

t_stat
cdr_attach(UNIT * uptr, CONST char *file)
{
    int                 addr = GET_UADDR(uptr->CMD);
    t_stat              r;

    if ((r = sim_card_attach(uptr, file)) != SCPE_OK)
       return r;
    if (uptr->up7 == 0)
        uptr->up7 = malloc(sizeof(uint16)*80);
    set_devattn(addr, SNS_DEVEND);
    uptr->CMD &= ~(CDR_CARD);
    uptr->COL = 0;
    uptr->u6 = 0;
    return SCPE_OK;
}

t_stat
cdr_detach(UNIT * uptr)
{
    if (uptr->up7 != 0)
        free(uptr->up7);
    uptr->up7 = 0;
    uptr->SNS = 0;
    return sim_card_detach(uptr);
}

t_stat
cdr_help(FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, const char *cptr)
{
   fprintf (st, "2540R Card Reader\n\n");
   sim_card_attach_help(st, dptr, uptr, flag, cptr);
   fprint_set_help(st, dptr);
   fprint_show_help(st, dptr);
   return SCPE_OK;
}

const char *
cdr_description(DEVICE *dptr)
{
   return "2540R Card Reader";
}
#endif
