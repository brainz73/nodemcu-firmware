// Module for interfacing with GPIO
//#define NODE_DEBUG

#include "module.h"
#include "lauxlib.h"
#include "lmem.h"
#include "platform.h"
#include "pin_map.h"
#include "user_interface.h"
#include "c_types.h"
#include "c_string.h"
#include "gpio.h"
#include "hw_timer.h"

#define delayMicroseconds os_delay_us

#define MAX_WRITE_BYTES 200
#define MAX_WRITE_SEQ 3
#define DELAY_TABLE_MAX_LEN 600

typedef struct
{
  unsigned pin;
  int index;
  uint8 seq;
  uint8 seqstart;
  uint8 numseq;
  uint8 numbits[MAX_WRITE_SEQ];
  uint8 bytes[MAX_WRITE_BYTES];
  uint32 header;
  uint32 headerspace;
  uint32 mark;
  uint32 space0;
  uint32 space1;
  uint32 pause;
  uint32 *delay_table;
  uint32 tablelen;
  task_handle_t done_taskid;
  task_handle_t write_taskid;
  int lua_done_ref; // callback when transmission is done
  uint32_t mark_time; 
  uint32 pulse;
  uint32 min_delay;
  uint32 max_delay;
  uint32 start_time;
} irwrite_t;
static irwrite_t irwrite;
#define TIMER_OWNER ((os_param_t) 'i') // "ir"

static void write_done (task_param_t arg)
{
  lua_State *L = lua_getstate();

  if (irwrite.delay_table) {
    luaM_freearray(L, irwrite.delay_table, irwrite.tablelen, uint32);
    irwrite.delay_table = NULL;
  }

  if (irwrite.lua_done_ref != LUA_NOREF) {
    lua_rawgeti (L, LUA_REGISTRYINDEX, irwrite.lua_done_ref);
    luaL_unref (L, LUA_REGISTRYINDEX, irwrite.lua_done_ref);
    irwrite.lua_done_ref = LUA_NOREF;
    lua_pushinteger( L, arg);
    lua_pushinteger( L, irwrite.min_delay);
    lua_pushinteger( L, irwrite.max_delay);
    if (lua_pcall(L, 3, 0, 0)) {
      // Uncaught Error. Print instead of sudden reset
      luaL_error(L, "error: %s", lua_tostring(L, -1));
    }
  }
}

static void ICACHE_RAM_ATTR irwritebits_cb(os_param_t p) {
//  if (!irwrite_do_markspace())
//    return;
  if (irwrite.index < 0) {
    irwrite.index++;
//    irwrite_start_markspace(irwrite.header,irwrite.headerspace);
  } else if (irwrite.index < irwrite.numbits[irwrite.seq]) {
    unsigned level = (irwrite.bytes[irwrite.seqstart+irwrite.index/8] >> (irwrite.seqstart+(7-irwrite.index%8))) & 1;
    irwrite.index++;
    NODE_DBG("%d",level);
//    irwrite_start_markspace(irwrite.mark,level == 1 ? irwrite.space1 : irwrite.space0);
  } else if (++irwrite.seq < irwrite.numseq) {
    NODE_DBG(" ");
    irwrite.seqstart += (irwrite.index-1)/8+1;
    irwrite.index = -1;
//    irwrite_start_markspace(irwrite.mark,irwrite.pause);
  } else {
    platform_hw_timer_close(TIMER_OWNER);
//    irwrite_start_markspace(irwrite.mark,0);
    task_post_low (irwrite.done_taskid, (task_param_t)0);
  }
}

static int lir_config( lua_State* L )
{
  irwrite.header = luaL_checkinteger( L, 1 );
  irwrite.headerspace = luaL_checkinteger( L, 2 );
  irwrite.mark = luaL_checkinteger( L, 3 );
  irwrite.space0 = luaL_checkinteger( L, 4 );
  irwrite.space1 = luaL_checkinteger( L, 5 );
  irwrite.pause = luaL_optinteger( L, 6, 0 );

  return 0;
}

static int lir_writebits( lua_State* L )
{
  irwrite.pin = luaL_checkinteger(L, 1);
  luaL_argcheck(L, platform_gpio_exists(irwrite.pin), 1, "Invalid pin");

  unsigned param = 2;
  irwrite.numseq = 0;
  unsigned numbytes = 0;

  while (lua_isnumber(L, param)) {
    irwrite.numbits[irwrite.numseq] = luaL_checkinteger( L, param++ );
    irwrite.numseq++;
    luaL_argcheck(L, lua_istable( L, param ),8,"Ivalid bits table");
    unsigned tablelen = lua_objlen( L, param );
    luaL_argcheck(L, (tablelen+numbytes) < MAX_WRITE_BYTES, 8, "Too many write bits");
    for(unsigned i = 0; i < tablelen; i++ )  {
      lua_rawgeti( L, param, i + 1 );
      irwrite.bytes[numbytes++] = (unsigned) luaL_checkinteger( L, -1 );
      lua_pop( L, 1 );
    }
    param++;
  }

  luaL_unref (L, LUA_REGISTRYINDEX, irwrite.lua_done_ref);
  if (!lua_isnoneornil(L, param)) {
    lua_pushvalue(L, param);
    irwrite.lua_done_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  } else {
    irwrite.lua_done_ref = LUA_NOREF;
  }

  if (!platform_hw_timer_init(TIMER_OWNER, FRC1_SOURCE, FALSE)) {
    // Failed to init the timer
    luaL_error(L, "Unable to initialize timer");
  }

  platform_hw_timer_set_func(TIMER_OWNER, irwritebits_cb, 0);
  irwrite.index = -1;
  irwrite.seq = 0;
  irwrite.seqstart = 0;
//  irwrite_init_markspace();
  irwritebits_cb(0);

  lua_pushinteger( L, 1 );
  return 1;
}

static void irwrite_task(task_param_t arg) {
  if (irwrite.index == 0) {
    irwrite.start_time = system_get_time();
  }
  if (irwrite.index > 0 && irwrite.index < irwrite.tablelen) {
    uint32_t space = irwrite.delay_table[irwrite.index];
    uint32 system_time = system_get_time();
    uint32_t elapsed = system_time > irwrite.mark_time ? system_time-irwrite.mark_time : system_time+(0xffffffff-irwrite.mark_time);
    if (elapsed > irwrite.max_delay)
      irwrite.max_delay = elapsed;
    if (elapsed < irwrite.min_delay)
      irwrite.min_delay = elapsed;
    if (elapsed < space) {
      if (space-elapsed > 10000) {
        delayMicroseconds(5000);
        task_post_high(irwrite.write_taskid,(task_param_t)0);
        return;
      } 
      irwrite.index++;
      delayMicroseconds(space-elapsed);
    }
  }  
  if (irwrite.index < irwrite.tablelen) {
    uint32_t time = system_get_time();
    uint32_t mark = irwrite.delay_table[irwrite.index++];
    while(system_get_time()-time < mark) {
      GPIO_OUTPUT_SET(GPIO_ID_PIN(pin_num[irwrite.pin]), 1);
      delayMicroseconds(irwrite.pulse);
      GPIO_OUTPUT_SET(GPIO_ID_PIN(pin_num[irwrite.pin]), 0);
      delayMicroseconds(irwrite.pulse);
    }
    irwrite.mark_time = system_get_time();
  }
  if (irwrite.index < irwrite.tablelen) {
    task_post_high(irwrite.write_taskid,(task_param_t)0);
  } else {
    platform_hw_timer_close(TIMER_OWNER);
    uint32 system_time = system_get_time();
    uint32_t elapsed = system_time > irwrite.start_time ? system_time-irwrite.start_time : system_time+(0xffffffff-irwrite.start_time);
    task_post_low (irwrite.done_taskid, (task_param_t)elapsed);
  }
}

static int lir_write( lua_State *L ) 
{
  irwrite.pin = luaL_checkinteger(L, 1);
  luaL_argcheck(L, platform_gpio_exists(irwrite.pin), 1, "Invalid pin");

  luaL_unref (L, LUA_REGISTRYINDEX, irwrite.lua_done_ref);
  if (!lua_isnoneornil(L, 3)) {
    if (lua_isnumber(L, 3)) {
      irwrite.lua_done_ref = LUA_NOREF;
    } else {
      lua_pushvalue(L, 3);
      irwrite.lua_done_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    }
  } else {
    irwrite.lua_done_ref = LUA_NOREF;
  }

  if (irwrite.delay_table) {
    luaM_freearray(L, irwrite.delay_table, irwrite.tablelen, uint32);
    irwrite.delay_table = NULL;
  }

  luaL_argcheck(L, lua_istable( L, 2 ) &&
                   ((irwrite.tablelen = lua_objlen( L, 2 )) < DELAY_TABLE_MAX_LEN), 2, "Invalid delay_times" );

  irwrite.pulse = luaL_optinteger( L, 4, 13 )-4;

  irwrite.delay_table = luaM_newvector(L, irwrite.tablelen, uint32);
  for(unsigned i = 0; i < irwrite.tablelen; i++ )  {
    lua_rawgeti( L, 2, i + 1 );
    long delay = (long) luaL_checkinteger( L, -1 );
    irwrite.delay_table[i] = delay;
    lua_pop( L, 1 );
  }

//  platform_hw_timer_set_func(TIMER_OWNER, irwrite_cb, 0);
  irwrite.index = 0;
  task_post_high(irwrite.write_taskid,(task_param_t)0);
//  irwrite._mark = irwrite._space = 0;
//  irwrite.max_delay = irwrite.min_delay = irwrite.pulse;
  irwrite.max_delay = 0;
  irwrite.min_delay = 0xffffffff;
//  irwrite_cb(0);

  lua_pushinteger( L, 1 );
  return 1;
}

#ifdef GPIO_INTERRUPT_ENABLE

#define IRREAD_BUF_LEN 600
typedef struct
{
  unsigned pin;
  int buflen;
  uint32_t lasttime;
  uint32 buf[IRREAD_BUF_LEN];
  os_timer_t timeout_cb;
  task_handle_t done_taskid;
  int lua_done_ref; // callback when transmission is done
  uint32_t timeout;
  uint32_t gap_min;
  int running;
} irread_t;
static irread_t irread;

static void read_done (task_param_t arg) 
{
  NODE_DBG("irread done call: %d\n",irread.buflen);
  lua_State *L = lua_getstate();
  if (irread.lua_done_ref != LUA_NOREF) {
    lua_rawgeti (L, LUA_REGISTRYINDEX, irread.lua_done_ref);
    luaL_unref (L, LUA_REGISTRYINDEX, irread.lua_done_ref);
    irread.lua_done_ref = LUA_NOREF;
    if (irread.buflen <= 0) {
      lua_pushnil(L);
    } else {
      NODE_DBG("creating table\n");
      lua_createtable( L , irread.buflen, 0);
      for (int i = 0;i < irread.buflen;i++) {
        NODE_DBG("pushinteger\n");
        lua_pushinteger( L, irread.buf[i]);
        NODE_DBG("rawseti\n");
        lua_rawseti( L, -2, i+1);
      }
    }
    lua_pushinteger(L,arg);
    NODE_DBG("pcall");
    if (lua_pcall(L, 2, 0, 0)) {
      // Uncaught Error. Print instead of sudden reset
      luaL_error(L, "error: %s", lua_tostring(L, -1));
    }
  }
}

static uint32_t  ICACHE_RAM_ATTR read_intr(uint32_t ret_gpio_status);

static void ICACHE_RAM_ATTR read_timeout_cb(os_param_t p)
{
  uint32_t time = system_get_time();
  uint32_t elapsed = time > irread.lasttime ? time-irread.lasttime : time+(0xffffffff-irread.lasttime);
  os_timer_disarm(&irread.timeout_cb);
  platform_gpio_register_intr_hook(0,read_intr);
  task_post_low(irread.done_taskid, elapsed);
  irread.running = 0;
}

static uint32_t  ICACHE_RAM_ATTR read_intr(uint32_t ret_gpio_status) 
{
  uint32_t time = system_get_time();
  uint32_t elapsed = time > irread.lasttime ? time-irread.lasttime : time+(0xffffffff-irread.lasttime);
  irread.lasttime = time;

  uint32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  int level = 0x1 & GPIO_INPUT_GET(GPIO_ID_PIN(pin_num[irread.pin]));
  //disable interrupt
  gpio_pin_intr_state_set(GPIO_ID_PIN(pin_num[irread.pin]), GPIO_PIN_INTR_DISABLE);
  //clear interrupt status
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(pin_num[irread.pin]));

  if ((irread.buflen >= 0 && elapsed > irread.gap_min && level == 0) ||
      irread.buflen >= IRREAD_BUF_LEN) {
    read_timeout_cb(0);
  } else if (irread.buflen >= 0) {
    irread.buf[irread.buflen++] = elapsed;
  } else if (elapsed > irread.gap_min && level == 0) {
	irread.buflen++;
  }
  if (irread.buflen < IRREAD_BUF_LEN) {
    platform_gpio_intr_init(irread.pin, GPIO_PIN_INTR_ANYEDGE);
  }
  return ret_gpio_status & (~BIT(pin_num[irread.pin]));
}

static int lir_read( lua_State* L )
{
  irread.pin = luaL_checkinteger( L, 1 );
  irread.gap_min = luaL_checkinteger( L, 2 );
  irread.timeout = luaL_checkinteger( L, 3 );
  luaL_unref (L, LUA_REGISTRYINDEX, irread.lua_done_ref);
  if (!lua_isnoneornil(L, 4)) {
    lua_pushvalue(L, 4);
    irread.lua_done_ref = luaL_ref(L, LUA_REGISTRYINDEX);
  } else {
    irread.lua_done_ref = LUA_NOREF;
  }
  
  unsigned hooked = platform_gpio_register_intr_hook(BIT(pin_num[irread.pin]),read_intr);
  if (!hooked) {
    lua_pushinteger( L, 0 );
    return 1;
  }
  if (irread.running) {
	  lua_pushinteger( L, 0 );
	  return 1;
  }

  irread.running = 1;
  lua_pushinteger( L, hooked);
  platform_gpio_mode( irread.pin, PLATFORM_GPIO_INT, PLATFORM_GPIO_FLOAT );
  platform_gpio_intr_init(irread.pin, GPIO_PIN_INTR_ANYEDGE);
  irread.lasttime = system_get_time();
  irread.buflen = -1;
  os_timer_setfn(&irread.timeout_cb, (os_timer_func_t *)read_timeout_cb, NULL);
  os_timer_arm(&irread.timeout_cb, irread.timeout, 0);

  lua_pushinteger( L, irread.timeout );
  return 1;
}

#endif

// Module function map
static const LUA_REG_TYPE ir_map[] = {
  { LSTRKEY( "write" ), LFUNCVAL( lir_write ) },
  { LSTRKEY( "writebits" ), LFUNCVAL( lir_writebits ) },
#ifdef GPIO_INTERRUPT_ENABLE
  { LSTRKEY( "read" ), LFUNCVAL( lir_read ) },
#endif
  { LSTRKEY( "config" ), LFUNCVAL( lir_config ) },
  { LNILKEY, LNILVAL }
};

int luaopen_ir( lua_State *L ) {
#ifdef GPIO_INTERRUPT_ENABLE
  irread.done_taskid = task_get_id((task_callback_t) read_done);
  irread.lua_done_ref = LUA_NOREF;
#endif
  irwrite.write_taskid = task_get_id((task_callback_t) irwrite_task); 
  irwrite.done_taskid = task_get_id((task_callback_t) write_done);
  irwrite.lua_done_ref = LUA_NOREF;
  irwrite.header = 8000;
  irwrite.headerspace = 4000;
  irwrite.mark = 550;
  irwrite.space0 = 550;
  irwrite.space1 = 1650;
  irwrite.pause = 40000;

  return 0;
}

NODEMCU_MODULE(IR, "ir", ir_map, luaopen_ir);
