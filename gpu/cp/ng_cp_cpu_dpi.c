/* ng_cp_cpu_dpi.c -- behavioural Ibex stand-in for simulation.
 *
 * The real CP firmware (fw/cp/src, built with -DNGFW_BUS_HOOK) runs here as a
 * coroutine on the simulation host: every MMIO access of the firmware
 * suspends it and is handed to the RTL bus master in ng_cp_cpu_stub.sv, which
 * performs it on the CP private bus and resumes the firmware with the read
 * data.  `wfi` suspends the firmware until the CPU interrupt line is high.
 * It is NOT cycle accurate (firmware instructions take zero time between
 * bus accesses) -- it checks function, not timing.
 *
 * Coroutine: Win32 fibers (xsim on Windows) or ucontext (Linux).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "ngfw_ibex_port.h"

#ifdef _WIN32
#include <windows.h>
typedef LPVOID co_t;
#else
#include <ucontext.h>
typedef ucontext_t co_t;
#endif

#define CO_STACK (256u * 1024u)

static co_t g_main_co;
#ifdef _WIN32
static co_t g_fw_co;
#else
static co_t g_fw_co_storage;
static co_t *g_fw_co;
static void *g_stack;
#endif
static int g_have_fw;
static int g_waiting_rsp;
static int g_irq;
static int g_op_valid, g_op_we;
static uint32_t g_op_addr, g_op_wdata, g_rsp;

static void to_main(void) {
#ifdef _WIN32
  SwitchToFiber(g_main_co);
#else
  swapcontext(g_fw_co, &g_main_co);
#endif
}

static void to_fw(void) {
#ifdef _WIN32
  SwitchToFiber(g_fw_co);
#else
  swapcontext(&g_main_co, g_fw_co);
#endif
}

/* ---- the firmware's view of the hardware ------------------------------- */
uint32_t ngfw_bus_rd32(uint32_t addr) {
  g_op_valid = 1;
  g_op_we = 0;
  g_op_addr = addr;
  g_op_wdata = 0;
  to_main();
  return g_rsp;
}

void ngfw_bus_wr32(uint32_t addr, uint32_t val) {
  g_op_valid = 1;
  g_op_we = 1;
  g_op_addr = addr;
  g_op_wdata = val;
  to_main();
}

void ngfw_bus_wfi(void) {
  while (!g_irq) {
    g_op_valid = 0;
    to_main();
  }
}

#ifdef _WIN32
static VOID CALLBACK fw_body(LPVOID arg) {
  (void)arg;
  ngfw_ibex_main();
}
#else
static void fw_body(void) { ngfw_ibex_main(); }
#endif

static void co_destroy(void) {
  if (!g_have_fw) return;
#ifdef _WIN32
  DeleteFiber(g_fw_co);
  g_fw_co = NULL;
#else
  free(g_stack);
  g_stack = NULL;
#endif
  g_have_fw = 0;
}

static void co_create(void) {
#ifdef _WIN32
  if (!g_main_co) g_main_co = IsThreadAFiber() ? GetCurrentFiber() : ConvertThreadToFiber(NULL);
  g_fw_co = CreateFiber(CO_STACK, fw_body, NULL);
#else
  g_stack = malloc(CO_STACK);
  g_fw_co = &g_fw_co_storage;
  getcontext(g_fw_co);
  g_fw_co->uc_stack.ss_sp = g_stack;
  g_fw_co->uc_stack.ss_size = CO_STACK;
  g_fw_co->uc_link = &g_main_co;
  makecontext(g_fw_co, fw_body, 0);
#endif
  g_have_fw = 1;
  g_waiting_rsp = 0;
}

/* ---- DPI entry points ------------------------------------------------------ */

/* Core reset asserted: the firmware dies, it restarts from ngfw_ibex_main. */
void ng_cpu_reset(void) {
  co_destroy();
  ngfw_ibex_quiet = 0;
}

/* Resume the firmware.  `rsp_valid` delivers the data of the outstanding bus
 * access.  Returns 1 and the next bus access, or 0 when the firmware sleeps
 * (or still waits for a response).  `quiet` mirrors ngfw_ibex_quiet. */
int ng_cpu_step(int irq, int rsp_valid, int rsp_data, int *op_we, int *op_addr, int *op_wdata,
                int *quiet) {
  g_irq = irq;
  if (!g_have_fw) co_create();
  if (g_waiting_rsp) {
    if (!rsp_valid) {
      *quiet = 0;
      return 0;
    }
    g_rsp = (uint32_t)rsp_data;
    g_waiting_rsp = 0;
  }
  g_op_valid = 0;
  to_fw();
  *quiet = (int)ngfw_ibex_quiet;
  if (g_op_valid) {
    g_waiting_rsp = 1;
    *op_we = g_op_we;
    *op_addr = (int)g_op_addr;
    *op_wdata = (int)g_op_wdata;
    return 1;
  }
  return 0;
}
