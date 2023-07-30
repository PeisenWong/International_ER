
#include "dongbei.h"

void DONGBEIInit(dongbei_data_handler dh)
{
    DONGBEIHandleData = dh;
}

void DONGBEIProcessData(uint8_t input)
{
    static uint8_t datacount;
    static uint8_t datacursor;
    switch(datacount) {
    case 0:
        if (input == 0x0d) datacount++;
        break;
    case 1:
        if (input == 0x0a) {
            datacursor = 0;
            datacount++;
        } else if (input != 0x0d) {
            datacount = 0;
        }
        break;
    case 2:
        DONGBEI_DATAFRAME.raw[datacursor] = input;
        datacursor++;
        if (datacursor == 24) {
            datacursor = 0;
            datacount++;
        }
        break;
    case 3:
        if (input == 0x0a) datacount++;
        else datacount = 0;
        break;
    case 4:
        if (input == 0x0d) datacount++;
        else datacount = 0;
        break;
    case 5:
        DONGBEIHandleData();
        datacount = 0;
        break;
    }
}

uint8_t* DONGBEICmd(dongbei_cmd_t cmd, float f1, float f2, float f3)
{
    static uint8_t buffer[18];
    struct {
        float val;
        uint8_t raw[4];
    }tempfloat;

    buffer[0] = 'A';
    buffer[1] = 'C';
    buffer[2] = 'T';

    switch(cmd) {
    case DONGBEI_CALIBRATE:
        return "ACTR";
    case DONGBEI_RESET:
        return "ACT0";
    case DONGBEI_SET_YAW:
        buffer[3] = 'J';
        tempfloat.val = f1;
        buffer[4] = tempfloat.raw[0];
        buffer[5] = tempfloat.raw[1];
        buffer[6] = tempfloat.raw[2];
        buffer[7] = tempfloat.raw[3];
        buffer[8] = '\n';
        buffer[9] = 0;
        return buffer;
    case DONGBEI_SET_X:
        buffer[3] = 'X';
        tempfloat.val = f1;
        buffer[4] = tempfloat.raw[0];
        buffer[5] = tempfloat.raw[1];
        buffer[6] = tempfloat.raw[2];
        buffer[7] = tempfloat.raw[3];
        buffer[8] = '\n';
        buffer[9] = 0;
        return buffer;
    case DONGBEI_SET_Y:
        buffer[3] = 'Y';
        tempfloat.val = f1;
        buffer[4] = tempfloat.raw[0];
        buffer[5] = tempfloat.raw[1];
        buffer[6] = tempfloat.raw[2];
        buffer[7] = tempfloat.raw[3];
        buffer[8] = '\n';
        buffer[9] = 0;
        return buffer;
    case DONGBEI_SET_DOUBLE:
        buffer[3] = 'D';
        tempfloat.val = f1;
        buffer[4] = tempfloat.raw[0];
        buffer[5] = tempfloat.raw[1];
        buffer[6] = tempfloat.raw[2];
        buffer[7] = tempfloat.raw[3];
        tempfloat.val = f2;
        buffer[ 8] = tempfloat.raw[0];
        buffer[ 9] = tempfloat.raw[1];
        buffer[10] = tempfloat.raw[2];
        buffer[11] = tempfloat.raw[3];
        buffer[12] = '\n';
        buffer[13] = 0;
        return buffer;
    case DONGBEI_SET_ALL:
        buffer[3] = 'A';
        tempfloat.val = f1;
        buffer[4] = tempfloat.raw[0];
        buffer[5] = tempfloat.raw[1];
        buffer[6] = tempfloat.raw[2];
        buffer[7] = tempfloat.raw[3];
        tempfloat.val = f2;
        buffer[ 8] = tempfloat.raw[0];
        buffer[ 9] = tempfloat.raw[1];
        buffer[10] = tempfloat.raw[2];
        buffer[11] = tempfloat.raw[3];
        tempfloat.val = f3;
        buffer[12] = tempfloat.raw[0];
        buffer[13] = tempfloat.raw[1];
        buffer[14] = tempfloat.raw[2];
        buffer[15] = tempfloat.raw[3];
        buffer[16] = '\n';
        buffer[17] = 0;
        return buffer;
    }
}
