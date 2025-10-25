/*
 * usb_desc.h
 *
 *  Created on: Jul 22, 2025
 *      Author: MLag
 */

#ifndef ST_STM32_USB_DEVICE_LIBRARY_CLASS_HID_INC_USB_DESC_H_
#define ST_STM32_USB_DEVICE_LIBRARY_CLASS_HID_INC_USB_DESC_H_

#define DESK_WHEEL_SIZE 83U
#define DESK_FFB_SIZE 142U

#define HID_EPIN_ADDR     0x81  // IN endpoint 1
#define HID_EPOUT_ADDR    0x01  // OUT endpoint 1
#define HID_EPOUT_SIZE    64    // размер OUT endpoint для FFB
#define HID_FS_BINTERVAL  0x05  // пример интервала


__ALIGN_BEGIN uint8_t WHEEL_HID_DESC[DESK_WHEEL_SIZE] __ALIGN_END = {
0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
0x09, 0x04,        // USAGE (Game Pad)
0xA1, 0x01,        // COLLECTION (Application)

// Кнопки (16 buttons) - ДОЛЖНЫ БЫТЬ ПЕРВЫМИ!
0x05, 0x09,        //   USAGE_PAGE (Button)
0x19, 0x01,        //   USAGE_MINIMUM (Button 1)
0x29, 0x10,        //   USAGE_MAXIMUM (Button 16)
0x15, 0x00,        //   LOGICAL_MINIMUM (0)
0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
0x75, 0x01,        //   REPORT_SIZE (1)
0x95, 0x10,        //   REPORT_COUNT (16)
0x81, 0x02,        //   INPUT (Data,Var,Abs)

// Steering (X-axis)
0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
0x09, 0x30,        //   USAGE (X)
0x15, 0x00,        //   LOGICAL_MINIMUM (0)
0x26, 0xFF, 0x7F,  //   LOGICAL_MAXIMUM (32767) // ИЗМЕНЕНО НА 32767!
0x75, 0x10,        //   REPORT_SIZE (16)
0x95, 0x01,        //   REPORT_COUNT (1)
0x81, 0x02,        //   INPUT (Data,Var,Abs)

// Throttle (Y-axis)
0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
0x09, 0x31,        //   USAGE (Y)
0x15, 0x00,        //   LOGICAL_MINIMUM (0)
0x26, 0xFF, 0x0F,  //   LOGICAL_MAXIMUM (4095) // ИЗМЕНЕНО НА 4095!
0x75, 0x10,        //   REPORT_SIZE (16)
0x95, 0x01,        //   REPORT_COUNT (1)
0x81, 0x02,        //   INPUT (Data,Var,Abs)

// Brake (Z-axis)
0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
0x09, 0x32,        //   USAGE (Z)
0x15, 0x00,        //   LOGICAL_MINIMUM (0)
0x26, 0xFF, 0x0F,  //   LOGICAL_MAXIMUM (4095)
0x75, 0x10,        //   REPORT_SIZE (16)
0x95, 0x01,        //   REPORT_COUNT (1)
0x81, 0x02,        //   INPUT (Data,Var,Abs)

// Clutch (Rz-axis)
0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
0x09, 0x33,        //   USAGE (Rz)
0x15, 0x00,        //   LOGICAL_MINIMUM (0)
0x26, 0xFF, 0x0F,  //   LOGICAL_MAXIMUM (4095)
0x75, 0x10,        //   REPORT_SIZE (16)
0x95, 0x01,        //   REPORT_COUNT (1)
0x81, 0x02,        //   INPUT (Data,Var,Abs)

0xC0              // END_COLLECTION

};


//work desc

//size  83U
/*


	0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
0x09, 0x05,        // USAGE (Game Pad)
0xA1, 0x01,        // COLLECTION (Application)

// Кнопки (16 buttons) - ДОЛЖНЫ БЫТЬ ПЕРВЫМИ!
0x05, 0x09,        //   USAGE_PAGE (Button)
0x19, 0x01,        //   USAGE_MINIMUM (Button 1)
0x29, 0x10,        //   USAGE_MAXIMUM (Button 16)
0x15, 0x00,        //   LOGICAL_MINIMUM (0)
0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
0x75, 0x01,        //   REPORT_SIZE (1)
0x95, 0x10,        //   REPORT_COUNT (16)
0x81, 0x02,        //   INPUT (Data,Var,Abs)

// Steering (X-axis)
0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
0x09, 0x30,        //   USAGE (X)
0x15, 0x00,        //   LOGICAL_MINIMUM (0)
0x26, 0xFF, 0x7F,  //   LOGICAL_MAXIMUM (32767) // ИЗМЕНЕНО НА 32767!
0x75, 0x10,        //   REPORT_SIZE (16)
0x95, 0x01,        //   REPORT_COUNT (1)
0x81, 0x02,        //   INPUT (Data,Var,Abs)

// Throttle (Y-axis)
0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
0x09, 0x31,        //   USAGE (Y)
0x15, 0x00,        //   LOGICAL_MINIMUM (0)
0x26, 0xFF, 0x0F,  //   LOGICAL_MAXIMUM (4095) // ИЗМЕНЕНО НА 4095!
0x75, 0x10,        //   REPORT_SIZE (16)
0x95, 0x01,        //   REPORT_COUNT (1)
0x81, 0x02,        //   INPUT (Data,Var,Abs)

// Brake (Z-axis)
0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
0x09, 0x32,        //   USAGE (Z)
0x15, 0x00,        //   LOGICAL_MINIMUM (0)
0x26, 0xFF, 0x0F,  //   LOGICAL_MAXIMUM (4095)
0x75, 0x10,        //   REPORT_SIZE (16)
0x95, 0x01,        //   REPORT_COUNT (1)
0x81, 0x02,        //   INPUT (Data,Var,Abs)

// Clutch (Rx-axis)
0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
0x09, 0x33,        //   USAGE (Rx)
0x15, 0x00,        //   LOGICAL_MINIMUM (0)
0x26, 0xFF, 0x0F,  //   LOGICAL_MAXIMUM (4095)
0x75, 0x10,        //   REPORT_SIZE (16)
0x95, 0x01,        //   REPORT_COUNT (1)
0x81, 0x02,        //   INPUT (Data,Var,Abs)

0xC0              // END_COLLECTION

*/

__ALIGN_BEGIN static uint8_t HID_FFB_ReportDesc[DESK_FFB_SIZE] __ALIGN_END = {
	0x05, 0x01,                    // Usage Page (Generic Desktop)
	0x09, 0x05,                    // Usage (Game Pad)
	0xA1, 0x01,                    // Collection (Application)
	
	// Feature report: Effect Block Index
	0x85, 0x01,                    //   Report ID (1)
	0x09, 0x21,                    //   Usage (PID State Report)
	0xA1, 0x02,                    //   Collection (Logical)
	0x09, 0x22,                    //     Usage (Effect Block Index)
	0x15, 0x01,                    //     Logical Minimum (1)
	0x25, 0x28,                    //     Logical Maximum (40)
	0x75, 0x08,                    //     Report Size (8)
	0x95, 0x01,                    //     Report Count (1)
	0x81, 0x02,                    //     Input (Data, Variable, Absolute)
	
	// Effect Type
	0x09, 0x25,                    //     Usage (Effect Type)
	0x15, 0x00,                    //     Logical Minimum (0)
	0x25, 0x16,                    //     Logical Maximum (22)
	0x75, 0x08,                    //     Report Size (8)
	0x95, 0x01,                    //     Report Count (1)
	0x81, 0x02,                    //     Input (Data, Variable, Absolute)
  
	// Duration (in milliseconds)
	0x09, 0x26,                    //     Usage (Duration)
	0x16, 0xFF, 0x00,              //     Logical Minimum (0)
	0x26, 0xFF, 0x7F,              //     Logical Maximum (32767)
	0x75, 0x10,                    //     Report Size (16)
	0x95, 0x01,                    //     Report Count (1)
	0x81, 0x02,                    //     Input (Data, Variable, Absolute)
  
	// Trigger Repeat Interval
	0x09, 0x27,                    //     Usage (Trigger Repeat Interval)
	0x16, 0xFF, 0x00,              //     Logical Minimum (0)
	0x26, 0xFF, 0x7F,              //     Logical Maximum (32767)
	0x75, 0x10,                    //     Report Size (16)
	0x95, 0x01,                    //     Report Count (1)
	0x81, 0x02,                    //     Input (Data, Variable, Absolute)
  
	// Sample Period
	0x09, 0x28,                    //     Usage (Sample Period)
	0x16, 0xFF, 0x00,              //     Logical Minimum (0)
	0x26, 0xFF, 0x7F,              //     Logical Maximum (32767)
	0x75, 0x10,                    //     Report Size (16)
	0x95, 0x01,                    //     Report Count (1)
	0x81, 0x02,                    //     Input (Data, Variable, Absolute)
  
	// Gain
	0x09, 0x2A,                    //     Usage (Gain)
	0x15, 0x00,                    //     Logical Minimum (0)
	0x26, 0xFF, 0x00,              //     Logical Maximum (255)
	0x75, 0x08,                    //     Report Size (8)
	0x95, 0x01,                    //     Report Count (1)
	0x81, 0x02,                    //     Input (Data, Variable, Absolute)
  
	// Trigger Button
	0x09, 0x2B,                    //     Usage (Trigger Button)
	0x15, 0x00,                    //     Logical Minimum (0)
	0x25, 0x01,                    //     Logical Maximum (1)
	0x75, 0x01,                    //     Report Size (1)
	0x95, 0x01,                    //     Report Count (1)
	0x81, 0x02,                    //     Input (Data, Variable, Absolute)
	
	0x75, 0x07,                    //     Report Size (7) - padding
	0x95, 0x01,                    //     Report Count (1)
	0x81, 0x03,                    //     Input (Constant, Variable, Absolute)
  
	// Start Delay
	0x09, 0x2C,                    //     Usage (Start Delay)
	0x16, 0xFF, 0x00,              //     Logical Minimum (0)
	0x26, 0xFF, 0x7F,              //     Logical Maximum (32767)
	0x75, 0x10,                    //     Report Size (16)
	0x95, 0x01,                    //     Report Count (1)
	0x81, 0x02,                    //     Input (Data, Variable, Absolute)
  
	0xC0,                         //   End Collection
  
	// Output report: Effect Block Data (for Set Effect report)
	0x85, 0x02,                   // Report ID (2)
	0x09, 0x22,                   // Usage (Effect Block Index)
	0x15, 0x01,                   // Logical Minimum (1)
	0x25, 0x28,                   // Logical Maximum (40)
	0x75, 0x08,                   // Report Size (8)
	0x95, 0x01,                   // Report Count (1)
	0x91, 0x02,                   // Output (Data, Variable, Absolute)
  
	0x09, 0x25,                   // Usage (Effect Type)
	0x15, 0x00,                   // Logical Minimum (0)
	0x25, 0x16,                   // Logical Maximum (22)
	0x75, 0x08,                   // Report Size (8)
	0x95, 0x01,                   // Report Count (1)
	0x91, 0x02,                   // Output (Data, Variable, Absolute)
  
	// Effect Parameters (example - Ramp Force)
	0x09, 0x30,                   // Usage (Direction)
	0x15, 0x00,                   // Logical Minimum (0)
	0x26, 0xFF, 0x00,             // Logical Maximum (255)
	0x75, 0x08,                   // Report Size (8)
	0x95, 0x01,                   // Report Count (1)
	0x91, 0x02,                   // Output (Data, Variable, Absolute)
  
	// Additional parameters omitted for brevity
	0xC0                          // End Collection
  };
  

#endif
