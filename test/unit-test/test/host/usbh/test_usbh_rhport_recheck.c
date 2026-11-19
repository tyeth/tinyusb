/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 TinyUSB contributors
 * SPDX-License-Identifier: MIT
 */

#include "unity.h"

// Include the production implementation to drive the recheck state machine directly.
#include "host/usbh.c"

TEST_SOURCE_FILE("tusb.c")
TEST_SOURCE_FILE("tusb_fifo.c")
TEST_SOURCE_FILE("usbh_enum_xfer_retry_hcd_close.c")

enum {
  RHPORT = 0,
  DADDR  = 1,
};

static uint32_t now_ms;
static bool     port_connected;
static uint32_t port_reset_count;
static uint32_t setup_send_count;
static uint8_t  setup_send_daddr;
static bool     setup_send_result;

//--------------------------------------------------------------------+
// HCD stubs
//--------------------------------------------------------------------+
uint32_t tusb_time_millis_api(void) {
  return now_ms;
}

bool hcd_init(uint8_t rhport, const tusb_rhport_init_t *rh_init) {
  (void) rhport;
  (void) rh_init;
  return true;
}

void hcd_int_handler(uint8_t rhport, bool in_isr) {
  (void) rhport;
  (void) in_isr;
}

void hcd_int_enable(uint8_t rhport) {
  (void) rhport;
}

void hcd_int_disable(uint8_t rhport) {
  (void) rhport;
}

bool hcd_port_connect_status(uint8_t rhport) {
  return rhport == RHPORT && port_connected;
}

void hcd_port_reset(uint8_t rhport) {
  (void) rhport;
  port_reset_count++;
}

void hcd_port_reset_end(uint8_t rhport) {
  (void) rhport;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport) {
  (void) rhport;
  return TUSB_SPEED_FULL;
}

void hcd_device_close(uint8_t rhport, uint8_t daddr) {
  (void) rhport;
  (void) daddr;
}

bool hcd_setup_send(uint8_t rhport, uint8_t daddr, const uint8_t setup_packet[8]) {
  (void) rhport;
  (void) setup_packet;
  setup_send_count++;
  setup_send_daddr = daddr;
  return setup_send_result;
}

bool hcd_edpt_open(uint8_t rhport, uint8_t daddr, const tusb_desc_endpoint_t *ep_desc) {
  (void) rhport;
  (void) daddr;
  (void) ep_desc;
  return true;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t daddr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen) {
  (void) rhport;
  (void) daddr;
  (void) ep_addr;
  (void) buffer;
  (void) buflen;
  return true;
}

bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t daddr, uint8_t ep_addr) {
  (void) rhport;
  (void) daddr;
  (void) ep_addr;
  return true;
}

bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t daddr, uint8_t ep_addr) {
  (void) rhport;
  (void) daddr;
  (void) ep_addr;
  return true;
}

//--------------------------------------------------------------------+
// Helpers
//--------------------------------------------------------------------+
static const tusb_desc_device_t desc_device = {
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = 0x0200,
  .bDeviceClass       = 0,
  .bDeviceSubClass    = 0,
  .bDeviceProtocol    = 0,
  .bMaxPacketSize0    = 64,
  .idVendor           = 0xCafe,
  .idProduct          = 0x4000,
  .bcdDevice          = 0x0100,
  .iManufacturer      = 0,
  .iProduct           = 0,
  .iSerialNumber      = 0,
  .bNumConfigurations = 1,
};

// A device enumerated and mounted on the root port at DADDR, as usbh leaves it
static void mount_root_device(void) {
  usbh_device_t *dev    = get_device(DADDR);
  dev->bus_info.rhport  = RHPORT;
  dev->bus_info.hub_addr = 0;
  dev->bus_info.hub_port = 0;
  dev->bus_info.speed   = TUSB_SPEED_FULL;
  memcpy(&dev->desc_device, (const uint8_t *) &desc_device + offsetof(tusb_desc_device_t, bcdUSB),
         sizeof(desc_device_noheader_t));
  dev->connected  = 1;
  dev->addressed  = 1;
  dev->configured = 1;
  dev->mounted    = 1;
  port_connected  = true;
}

// Run the pending enumeration delay as tuh_task_ext() does once it is due
static void fire_delay(void) {
  const tusb_defer_func_t func = _usbh_data.call_after.func;
  TEST_ASSERT_NOT_NULL(func);
  now_ms                     = _usbh_data.call_after.at_ms;
  _usbh_data.call_after.func = NULL;
  func(_usbh_data.call_after.arg);
}

// Handle the next queued host event as tuh_task_ext() does
static void process_next_event(void) {
  hcd_event_t event;
  TEST_ASSERT_TRUE(osal_queue_receive(_usbh_q, &event, 0));
  TEST_ASSERT_TRUE(usbh_process_event(&event, false));
}

// The device answers the request in flight with the given data
static void request_succeeds(const void *data, uint16_t len) {
  const uint8_t daddr = _usbh_data.ctrl_xfer_info.daddr;
  usbh_control_xfer_cb(daddr, 0, XFER_RESULT_SUCCESS, 0); // SETUP
  memcpy(_usbh_data.ctrl_xfer_info.buffer, data, len);
  usbh_control_xfer_cb(daddr, 0x80, XFER_RESULT_SUCCESS, len); // DATA IN
  usbh_control_xfer_cb(daddr, 0x00, XFER_RESULT_SUCCESS, 0);   // STATUS
}

// The device does not answer the request in flight: usbh resends it until USBH_CONTROL_RETRY_MAX, then fails it
static void request_fails(void) {
  const uint8_t daddr = _usbh_data.ctrl_xfer_info.daddr;
  while (_usbh_data.ctrl_xfer_info.stage != CONTROL_STAGE_IDLE) {
    usbh_control_xfer_cb(daddr, 0, XFER_RESULT_FAILED, 0);
  }
}

// Ask for a check on a mounted device and send its probe
static void probe_mounted_device(void) {
  mount_root_device();
  TEST_ASSERT_TRUE(tuh_rhport_recheck(RHPORT));
  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_PROBING, _usbh_data.recheck_state);
  TEST_ASSERT_EQUAL_UINT32(1, setup_send_count);
  TEST_ASSERT_EQUAL_UINT8(DADDR, setup_send_daddr);
  TEST_ASSERT_EQUAL_UINT8(TUSB_REQ_GET_DESCRIPTOR, _usbh_epbuf.request.bRequest);
  TEST_ASSERT_EQUAL_UINT16(TUSB_DESC_DEVICE << 8, tu_le16toh(_usbh_epbuf.request.wValue));
  TEST_ASSERT_EQUAL_UINT16(sizeof(tusb_desc_device_t), tu_le16toh(_usbh_epbuf.request.wLength));
}

// The check re-enumerates the port: the old device is removed and a new enumeration starts with the attach debounce
static void assert_reenumeration_started(void) {
  TEST_ASSERT_EQUAL(RECHECK_REENUM, _usbh_data.recheck_state);
  TEST_ASSERT_FALSE(get_device(DADDR)->connected);
  TEST_ASSERT_EQUAL_UINT8(0, _usbh_data.enumerating_daddr);
  TEST_ASSERT_EQUAL_PTR(enum_delay_async, _usbh_data.call_after.func);
  TEST_ASSERT_EQUAL(ENUM_AFTER_DEBOUNCING_DELAY, _usbh_data.call_after.arg);
  TEST_ASSERT_TRUE(tu_bit_test(_usbh_data.attach_debouncing_bm, RHPORT));
  TEST_ASSERT_TRUE(tuh_rhport_recheck_busy(RHPORT));
}

// Nothing is attached any more: the debounce finds the port empty and the check completes
static void finish_with_empty_port(void) {
  port_connected = false;
  fire_delay();
  TEST_ASSERT_EQUAL_UINT8(TUSB_INDEX_INVALID_8, _usbh_data.enumerating_daddr);
  TEST_ASSERT_TRUE(tuh_rhport_recheck_busy(RHPORT));
  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_IDLE, _usbh_data.recheck_state);
  TEST_ASSERT_FALSE(tuh_rhport_recheck_busy(RHPORT));
  TEST_ASSERT_FALSE(tu_bit_test(_usbh_data.attach_debouncing_bm, RHPORT));
}

//--------------------------------------------------------------------+
// Setup
//--------------------------------------------------------------------+
void setUp(void) {
  tu_memclr(&_usbh_data, sizeof(_usbh_data));
  tu_memclr(_usbh_devices, sizeof(_usbh_devices));
  now_ms                       = 1000;
  port_connected               = false;
  port_reset_count             = 0;
  setup_send_count             = 0;
  setup_send_daddr             = TUSB_INDEX_INVALID_8;
  setup_send_result            = true;
  _usbh_data.enumerating_daddr = TUSB_INDEX_INVALID_8;
  _usbh_controller_id          = RHPORT;
  _usbh_q                      = osal_queue_create(&_usbh_qdef);
  tu_fifo_clear(&_usbh_pending_ctrl_q);
}

void tearDown(void) {
  _usbh_controller_id = TUSB_INDEX_INVALID_8;
}

//--------------------------------------------------------------------+
// Tests
//--------------------------------------------------------------------+
// Before tuh_init() nothing is active: the check is refused instead of touching uninitialized state
void test_rejected_before_init(void) {
  _usbh_controller_id = TUSB_INDEX_INVALID_8;
  TEST_ASSERT_FALSE(tuh_rhport_recheck(TUSB_INDEX_INVALID_8));
  TEST_ASSERT_FALSE(tuh_rhport_recheck_busy(TUSB_INDEX_INVALID_8));
  TEST_ASSERT_EQUAL(RECHECK_IDLE, _usbh_data.recheck_state);
}

// The same device answers: it stays mounted, after one control transfer and no port reset
void test_same_device_stays_mounted(void) {
  probe_mounted_device();
  TEST_ASSERT_TRUE(tuh_rhport_recheck_busy(RHPORT));

  request_succeeds(&desc_device, sizeof(desc_device));

  TEST_ASSERT_EQUAL(RECHECK_IDLE, _usbh_data.recheck_state);
  TEST_ASSERT_FALSE(tuh_rhport_recheck_busy(RHPORT));
  TEST_ASSERT_TRUE(get_device(DADDR)->connected);
  TEST_ASSERT_TRUE(get_device(DADDR)->mounted);
  TEST_ASSERT_EQUAL_UINT32(0, port_reset_count);
  TEST_ASSERT_TRUE(osal_queue_empty(_usbh_q));
}

// Another device answers at the address: the port is enumerated again
void test_other_device_is_enumerated(void) {
  probe_mounted_device();
  tusb_desc_device_t other = desc_device;
  other.idProduct          = 0x4001;
  request_succeeds(&other, sizeof(other));
  TEST_ASSERT_EQUAL(RECHECK_REENUM_REQ, _usbh_data.recheck_state);
  TEST_ASSERT_TRUE(get_device(DADDR)->connected); // the callback only records the verdict

  recheck_process();
  assert_reenumeration_started();
  finish_with_empty_port();
}

// Nothing answers at the address (a new or reset device waits at address 0): the port is enumerated again
void test_no_answer_is_enumerated(void) {
  probe_mounted_device();
  request_fails();
  TEST_ASSERT_EQUAL(RECHECK_REENUM_REQ, _usbh_data.recheck_state);

  recheck_process();
  assert_reenumeration_started();
  finish_with_empty_port();
}

// The control slot is busy: the probe waits for it, and the task is not woken for the check meanwhile
void test_probe_waits_for_busy_control_slot(void) {
  mount_root_device();
  _usbh_data.ctrl_xfer_info.stage = CONTROL_STAGE_DATA;
  _usbh_data.ctrl_xfer_info.daddr = 2;
  TEST_ASSERT_TRUE(tuh_rhport_recheck(RHPORT));

  TEST_ASSERT_FALSE(recheck_due());
  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_PENDING, _usbh_data.recheck_state);
  TEST_ASSERT_EQUAL_UINT32(0, setup_send_count);

  _usbh_data.ctrl_xfer_info.stage = CONTROL_STAGE_IDLE;
  TEST_ASSERT_TRUE(recheck_due());
  TEST_ASSERT_TRUE(tuh_task_event_ready());
  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_PROBING, _usbh_data.recheck_state);
  TEST_ASSERT_EQUAL_UINT32(1, setup_send_count);
}

// The probe cannot be sent to the device at all (the HCD refuses the SETUP): the port is enumerated again
void test_probe_not_sent_is_enumerated(void) {
  mount_root_device();
  setup_send_result = false;
  TEST_ASSERT_TRUE(tuh_rhport_recheck(RHPORT));
  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_REENUM_REQ, _usbh_data.recheck_state);
}

// Port events queued before the re-enumeration are handled first, so an older detach cannot undo it, and the attach
// debounce bit is not left set
void test_reenumeration_waits_for_queued_events(void) {
  probe_mounted_device();
  request_fails();
  TEST_ASSERT_EQUAL(RECHECK_REENUM_REQ, _usbh_data.recheck_state);

  // a detach the HCD saw before the check acted
  port_connected = false;
  hcd_event_device_remove(RHPORT, false);
  TEST_ASSERT_FALSE(recheck_due());
  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_REENUM_REQ, _usbh_data.recheck_state);
  TEST_ASSERT_FALSE(tu_bit_test(_usbh_data.attach_debouncing_bm, RHPORT));

  process_next_event(); // the old device is removed
  TEST_ASSERT_FALSE(get_device(DADDR)->connected);

  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_REENUM, _usbh_data.recheck_state);
  TEST_ASSERT_EQUAL_UINT8(0, _usbh_data.enumerating_daddr);
  finish_with_empty_port();

  // a later attach is not dropped
  port_connected = true;
  hcd_event_device_attach(RHPORT, false);
  TEST_ASSERT_FALSE(osal_queue_empty(_usbh_q));
}

// A detach handled during the re-enumeration's debounce cancels that delay: the debounce bit must not stay set, or the
// port would drop every later attach and detach
void test_detach_during_reenumeration_debounce_clears_bit(void) {
  probe_mounted_device();
  request_fails();
  recheck_process();
  assert_reenumeration_started();

  hcd_event_t event         = {0};
  event.rhport              = RHPORT;
  event.event_id            = HCD_EVENT_DEVICE_REMOVE;
  event.connection.hub_addr = 0;
  event.connection.hub_port = 0;
  TEST_ASSERT_TRUE(usbh_process_event(&event, false));

  TEST_ASSERT_NULL(_usbh_data.call_after.func);
  TEST_ASSERT_FALSE(tu_bit_test(_usbh_data.attach_debouncing_bm, RHPORT));

  port_connected = true;
  hcd_event_device_attach(RHPORT, false);
  TEST_ASSERT_FALSE(osal_queue_empty(_usbh_q));
}

// A device attached to the port but not enumerated (its attach used up its retries) is enumerated again
void test_unenumerated_device_is_enumerated(void) {
  port_connected = true;
  TEST_ASSERT_TRUE(tuh_rhport_recheck(RHPORT));
  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_REENUM_REQ, _usbh_data.recheck_state);
  TEST_ASSERT_EQUAL_UINT32(0, setup_send_count);

  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_REENUM, _usbh_data.recheck_state);
  TEST_ASSERT_EQUAL_UINT8(0, _usbh_data.enumerating_daddr);
}

// An empty port: nothing to do
void test_empty_port_completes_at_once(void) {
  TEST_ASSERT_TRUE(tuh_rhport_recheck(RHPORT));
  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_IDLE, _usbh_data.recheck_state);
  TEST_ASSERT_FALSE(tuh_rhport_recheck_busy(RHPORT));
  TEST_ASSERT_EQUAL_UINT32(0, setup_send_count);
  TEST_ASSERT_NULL(_usbh_data.call_after.func);
}

// A check asked for while one runs is not lost: another follows
void test_check_asked_while_busy_runs_again(void) {
  probe_mounted_device();
  TEST_ASSERT_TRUE(tuh_rhport_recheck(RHPORT));

  request_succeeds(&desc_device, sizeof(desc_device));
  TEST_ASSERT_EQUAL(RECHECK_PENDING, _usbh_data.recheck_state);

  recheck_process();
  TEST_ASSERT_EQUAL(RECHECK_PROBING, _usbh_data.recheck_state);
  TEST_ASSERT_EQUAL_UINT32(2, setup_send_count);
  request_succeeds(&desc_device, sizeof(desc_device));
  TEST_ASSERT_EQUAL(RECHECK_IDLE, _usbh_data.recheck_state);
}

// The device is removed while its probe is in flight: the failed probe completes the check, it does not re-enumerate
void test_device_removed_during_probe(void) {
  probe_mounted_device();

  port_connected = false;
  hcd_event_device_remove(RHPORT, false);
  process_next_event(); // the removal fails the probe in flight

  TEST_ASSERT_EQUAL(RECHECK_IDLE, _usbh_data.recheck_state);
  TEST_ASSERT_FALSE(tuh_rhport_recheck_busy(RHPORT));
  TEST_ASSERT_NULL(_usbh_data.call_after.func);
}
