// test the CCS series USB driver

#include <stdio.h>
#include "CCS_Series_Drv.h"


void showerr(unsigned long inst, int retcode, char* funcname) {
        char errdesc[CCS_SERIES_ERR_DESCR_BUFFER_SIZE];
        CCSseries_errorMessage(inst, retcode, errdesc);
        printf("%s returned %d: %s\n", funcname, retcode, errdesc);
}


void status(unsigned long inst) {
        long stat;
        int ret;

        ret = CCSseries_getDeviceStatus(inst, &stat);
        showerr(inst, ret, "getstat");

        printf("status %lx = ", stat);
        if (stat & CCS_SERIES_STATUS_SCAN_IDLE) printf("idle,");
        if (stat & CCS_SERIES_STATUS_SCAN_TRIGGERED) printf("triggered,");
        if (stat & CCS_SERIES_STATUS_SCAN_START_TRANS) printf("starting,");
        if (stat & CCS_SERIES_STATUS_SCAN_TRANSFER) printf("awaiting_xfer,");
        if (stat & CCS_SERIES_STATUS_WAIT_FOR_EXT_TRIG) printf("armed,");
        printf("\n");
}


int main(int argc, char* argv[]) {
    unsigned long inst; //instrument handle  (ViPSession = unsigned long*)
    ViStatus ret;

    ret = CCSseries_init(argv[1],   VI_ON,  VI_ON,      &inst); // read usb vid:pid from command line
    //                   resource name, IDQuery, resetDevice, instrumentHandle
    //ret = CCSseries_init("1313:8081",   VI_OFF,  VI_OFF,      &inst);
    //ret = CCSseries_init("1313:8087",   VI_ON,  VI_ON,      &inst);
    //ret = CCSseries_init("1313:8081",   VI_ON,  VI_ON,      &inst);
    showerr(inst, ret, "init");   // if nonzero must exit, as we'll get a segfault if we proceed

    char vendor[CCS_SERIES_BUFFER_SIZE];
    char device[CCS_SERIES_BUFFER_SIZE];
    char serial[CCS_SERIES_BUFFER_SIZE];
    char revision[CCS_SERIES_BUFFER_SIZE];
    char driver_revision[CCS_SERIES_BUFFER_SIZE];
    ret = CCSseries_identificationQuery(inst, vendor, device, serial, revision, driver_revision);
    showerr(inst, ret, "ID");
    status(inst);
    printf("V=%s D=%s S=%s R=%s DR=%s\n", vendor, device, serial, revision, driver_revision);

    double inttime, newinttime;
    ret = CCSseries_getIntegrationTime(inst, &inttime);
    showerr(inst, ret, "getinttime");
    printf("inttime read as %f\n", inttime);

    printf("inttime?\n");
    scanf("%lf", &newinttime);
    ret = CCSseries_setIntegrationTime(inst, newinttime);
    showerr(inst, ret, "setinttime");

    ret = CCSseries_getIntegrationTime(inst, &inttime);
    showerr(inst, ret, "get2inttime");
    printf("inttime read as %f after setting %f\n", inttime, newinttime);
    status(inst);


    /// TODO a flush read before starting?

    ret = CCSseries_close(inst);
    showerr(inst, ret, "close");

    return 0;
}

