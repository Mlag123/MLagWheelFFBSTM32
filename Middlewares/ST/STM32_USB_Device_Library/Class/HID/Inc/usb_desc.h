/*
 * usb_desc.h
 *
 *  Created on: Jul 22, 2025
 *      Author: MLag
 */

#ifndef ST_STM32_USB_DEVICE_LIBRARY_CLASS_HID_INC_USB_DESC_H_
#define ST_STM32_USB_DEVICE_LIBRARY_CLASS_HID_INC_USB_DESC_H_

#define DESK_WHEEL_SIZE 67U


__ALIGN_BEGIN uint8_t WHEEL_HID_DESC[DESK_WHEEL_SIZE] __ALIGN_END = {
	    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	    0x09, 0x04,                    // USAGE (Joystick)
	    0xa1, 0x01,                    // COLLECTION (Application)
	    0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
	    0x09, 0xc8,                    //   USAGE (Steering)
	    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	    0x26, 0x00, 0x10,              //   LOGICAL_MAXIMUM (4096)
	    0x75, 0x10,                    //   REPORT_SIZE (16)
	    0x95, 0x01,                    //   REPORT_COUNT (1)
	    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	    0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
	    0x09, 0xbb,                    //   USAGE (Throttle)
	    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	    0x26, 0x00, 0x10,              //   LOGICAL_MAXIMUM (4096)
	    0x75, 0x10,                    //   REPORT_SIZE (16)
	    0x95, 0x01,                    //   REPORT_COUNT (1)
	    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	    0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
	    0x09, 0xc5,                    //   USAGE (Brake)
	    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	    0x26, 0x00, 0x10,              //   LOGICAL_MAXIMUM (4096)
	    0x75, 0x10,                    //   REPORT_SIZE (16)
	    0x95, 0x01,                    //   REPORT_COUNT (1)
	    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	    0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
	    0x09, 0xc6,                    //   USAGE (Clutch)
	    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	    0x26, 0x00, 0x10,              //   LOGICAL_MAXIMUM (4096)
	    0x75, 0x10,                    //   REPORT_SIZE (16)
	    0x95, 0x01,                    //   REPORT_COUNT (1)
	    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	    0xc0                           // END_COLLECTION
};

#endif /* ST_STM32_USB_DEVICE_LIBRARY_CLASS_HID_INC_USB_DESC_H_ */
