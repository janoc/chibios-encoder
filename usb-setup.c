
#include "usb-setup.h"

SerialUSBDriver SDU1;

void usb_lld_disconnect_bus(USBDriver *usbp)
{
    palClearPort(USB_GPIO_PORT, (1<<USBDM_BIT) | (1<<USBDP_BIT));
    palSetPadMode(USB_GPIO_PORT, USBDM_BIT, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(USB_GPIO_PORT, USBDP_BIT, PAL_MODE_OUTPUT_PUSHPULL);
}

void usb_lld_connect_bus(USBDriver *usbp)
{
    palClearPort(USB_GPIO_PORT, (1<<USBDM_BIT) | (1<<USBDP_BIT));
    palSetPadMode(USB_GPIO_PORT, USBDM_BIT, PAL_MODE_ALTERNATE(14));
    palSetPadMode(USB_GPIO_PORT, USBDP_BIT, PAL_MODE_ALTERNATE(14));
}


/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

    (void)usbp;
    (void)lang;
    switch (dtype) {
        case USB_DESCRIPTOR_DEVICE:
            return &vcom_device_descriptor;
        case USB_DESCRIPTOR_CONFIGURATION:
            return &vcom_configuration_descriptor;
        case USB_DESCRIPTOR_STRING:
            if (dindex < 4)
                return &vcom_strings[dindex];
    }
    return NULL;
}

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {

    switch (event) {
        case USB_EVENT_RESET:
            return;
        case USB_EVENT_ADDRESS:
            return;
        case USB_EVENT_CONFIGURED:
            chSysLockFromIsr();

            /* Enables the endpoints specified into the configuration.
               Note, this callback is invoked from an ISR so I-Class functions
               must be used.*/
            usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &ep1config);
            usbInitEndpointI(usbp, USBD1_INTERRUPT_REQUEST_EP, &ep2config);

            /* Resetting the state of the CDC subsystem.*/
            sduConfigureHookI(&SDU1);

            chSysUnlockFromIsr();
            return;
        case USB_EVENT_SUSPEND:
            return;
        case USB_EVENT_WAKEUP:
            return;
        case USB_EVENT_STALLED:
            return;
    }
    return;
}

const USBConfig usbcfg = {
    usb_event,
    get_descriptor,
    sduRequestsHook,
    NULL
};

