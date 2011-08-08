/* Copyright (C) 2007-2008 The Android Open Source Project
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
*/
#include "qemu_file.h"
#include "goldfish_device.h"

enum {
	/* irq_enable */
	INT_ENABLE  = 0x00,
	/* accelerometer */
	ACCEL_X	    = 0x04,
	ACCEL_Y     = 0x08,
        ACCEL_Z     = 0x0c,
	/* compass */
        COMPASS_X   = 0x10,
        COMPASS_Y   = 0x14,
        COMPASS_Z   = 0x18,
        /* orientation */
        ORIENT_X   = 0x1c,
        ORIENT_Y   = 0x20,
        ORIENT_Z   = 0x24,
};


struct goldfish_sensor_state {
    struct goldfish_device dev;
    // IRQs
    uint32_t int_status;
    // irq enable mask for int_status
    uint32_t int_enable;

    int accel_x,accel_y,accel_z;
    int compass_x,compass_y,compass_z;
    int orient_x,orient_y,orient_z;
};

/* update this each time you update the sensor_state struct */
#define  SENSOR_STATE_SAVE_VERSION  1

#define  QFIELD_STRUCT  struct goldfish_sensor_state
QFIELD_BEGIN(goldfish_sensor_fields)
    QFIELD_INT32(int_status),
    QFIELD_INT32(int_enable),
    QFIELD_INT32(accel_x),
    QFIELD_INT32(accel_y),
    QFIELD_INT32(accel_z),
    QFIELD_INT32(compass_x),
    QFIELD_INT32(compass_y),
    QFIELD_INT32(compass_z),
    QFIELD_INT32(orient_x),
    QFIELD_INT32(orient_y),
    QFIELD_INT32(orient_z),
QFIELD_END

static void  goldfish_sensor_save(QEMUFile*  f, void* opaque)
{
    struct goldfish_sensor_state*  s = opaque;

    qemu_put_struct(f, goldfish_sensor_fields, s);
}

static int   goldfish_sensor_load(QEMUFile*  f, void*  opaque, int  version_id)
{
    struct goldfish_sensor_state*  s = opaque;

    if (version_id != SENSOR_STATE_SAVE_VERSION)
        return -1;

    return qemu_get_struct(f, goldfish_sensor_fields, s);
}

static struct goldfish_sensor_state *sensor_state;

static uint32_t goldfish_sensor_read(void *opaque, target_phys_addr_t offset)
{
    uint32_t ret;
    struct goldfish_sensor_state *s = opaque;
    offset -= s->dev.base;
    switch(offset) {
        case INT_ENABLE:
            // return current buffer status flags
            ret = s->int_status & s->int_enable;
            if (ret) {
                goldfish_device_set_irq(&s->dev, 0, 0);
                s->int_status = 0;
            }
            return ret;

        case ACCEL_X:
            return s->accel_x;
        case ACCEL_Y:
            return s->accel_y;
        case ACCEL_Z:
            return s->accel_z;
        case COMPASS_X:
            return s->compass_x;
        case COMPASS_Y:
            return s->compass_y;
        case COMPASS_Z:
            return s->compass_z;
        case ORIENT_X:
            return s->orient_x;
        case ORIENT_Y:
            return s->orient_y;
        case ORIENT_Z:
            return s->orient_z;
        default:
            cpu_abort (cpu_single_env, "goldfish_sensor_read: Bad offset %x\n", offset);
            return 0;
    }
}

static void goldfish_sensor_write(void *opaque, target_phys_addr_t offset, uint32_t val)
{
    struct goldfish_sensor_state *s = opaque;
    offset -= s->dev.base;

    switch(offset) {
        case INT_ENABLE:
            /* enable interrupts */
            s->int_enable = val;
            break;

        default:
            cpu_abort (cpu_single_env, "goldfish_sensor_write: Bad offset %x\n", offset);
    }
}

static CPUReadMemoryFunc *goldfish_sensor_readfn[] = {
    goldfish_sensor_read,
    goldfish_sensor_read,
    goldfish_sensor_read
};


static CPUWriteMemoryFunc *goldfish_sensor_writefn[] = {
    goldfish_sensor_write,
    goldfish_sensor_write,
    goldfish_sensor_write
};

void goldfish_sensor_init()
{
    struct goldfish_sensor_state *s;

    s = (struct goldfish_sensor_state *)qemu_mallocz(sizeof(*s));
    s->dev.name = "goldfish_sensor";
    s->dev.base = 0;    // will be allocated dynamically
    s->dev.size = 0x1000;
    s->dev.irq_count = 1;

    // default values for the sensor
    s->int_enable = 0;
    s->int_status = 0;

    sensor_state = s;

    goldfish_device_add(&s->dev, goldfish_sensor_readfn, goldfish_sensor_writefn, s);

    register_savevm( "sensor_state", 0, SENSOR_STATE_SAVE_VERSION,
                     goldfish_sensor_save, goldfish_sensor_load, s);
}

void goldfish_sensor_set_prop(int sensor, int x,int y,int z)
{

    switch (sensor) {
        case 0:
            sensor_state->accel_x=x;
            sensor_state->accel_y=y;
            sensor_state->accel_z=z;
            break;
        case 1:
            sensor_state->compass_x=x;
            sensor_state->compass_y=y;
            sensor_state->compass_z=z;
            break;

        case 2:
            sensor_state->orient_x=x;
            sensor_state->orient_y=y;
            sensor_state->orient_z=z;
            break;	
    }
    sensor_state->int_status |= (1<<sensor);
    goldfish_device_set_irq(&sensor_state->dev, 0, (sensor_state->int_status & sensor_state->int_enable));
}

