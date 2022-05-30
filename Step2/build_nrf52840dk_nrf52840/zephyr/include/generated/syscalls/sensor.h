
/* auto-generated by gen_syscalls.py, don't edit */
#ifndef Z_INCLUDE_SYSCALLS_SENSOR_H
#define Z_INCLUDE_SYSCALLS_SENSOR_H


#include <tracing/tracing_syscall.h>

#ifndef _ASMLANGUAGE

#include <syscall_list.h>
#include <syscall.h>

#include <linker/sections.h>


#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic push
#endif

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#if !defined(__XCC__)
#pragma GCC diagnostic ignored "-Warray-bounds"
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern int z_impl_sensor_attr_set(const struct device * dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value * val);

__pinned_func
static inline int sensor_attr_set(const struct device * dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value * val)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		/* coverity[OVERRUN] */
		return (int) arch_syscall_invoke4(*(uintptr_t *)&dev, *(uintptr_t *)&chan, *(uintptr_t *)&attr, *(uintptr_t *)&val, K_SYSCALL_SENSOR_ATTR_SET);
	}
#endif
	compiler_barrier();
	return z_impl_sensor_attr_set(dev, chan, attr, val);
}

#if (CONFIG_TRACING_SYSCALL == 1)
#ifndef DISABLE_SYSCALL_TRACING

#define sensor_attr_set(dev, chan, attr, val) ({ 	int retval; 	sys_port_trace_syscall_enter(K_SYSCALL_SENSOR_ATTR_SET, sensor_attr_set, dev, chan, attr, val); 	retval = sensor_attr_set(dev, chan, attr, val); 	sys_port_trace_syscall_exit(K_SYSCALL_SENSOR_ATTR_SET, sensor_attr_set, dev, chan, attr, val, retval); 	retval; })
#endif
#endif


extern int z_impl_sensor_attr_get(const struct device * dev, enum sensor_channel chan, enum sensor_attribute attr, struct sensor_value * val);

__pinned_func
static inline int sensor_attr_get(const struct device * dev, enum sensor_channel chan, enum sensor_attribute attr, struct sensor_value * val)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		/* coverity[OVERRUN] */
		return (int) arch_syscall_invoke4(*(uintptr_t *)&dev, *(uintptr_t *)&chan, *(uintptr_t *)&attr, *(uintptr_t *)&val, K_SYSCALL_SENSOR_ATTR_GET);
	}
#endif
	compiler_barrier();
	return z_impl_sensor_attr_get(dev, chan, attr, val);
}

#if (CONFIG_TRACING_SYSCALL == 1)
#ifndef DISABLE_SYSCALL_TRACING

#define sensor_attr_get(dev, chan, attr, val) ({ 	int retval; 	sys_port_trace_syscall_enter(K_SYSCALL_SENSOR_ATTR_GET, sensor_attr_get, dev, chan, attr, val); 	retval = sensor_attr_get(dev, chan, attr, val); 	sys_port_trace_syscall_exit(K_SYSCALL_SENSOR_ATTR_GET, sensor_attr_get, dev, chan, attr, val, retval); 	retval; })
#endif
#endif


extern int z_impl_sensor_sample_fetch(const struct device * dev);

__pinned_func
static inline int sensor_sample_fetch(const struct device * dev)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		/* coverity[OVERRUN] */
		return (int) arch_syscall_invoke1(*(uintptr_t *)&dev, K_SYSCALL_SENSOR_SAMPLE_FETCH);
	}
#endif
	compiler_barrier();
	return z_impl_sensor_sample_fetch(dev);
}

#if (CONFIG_TRACING_SYSCALL == 1)
#ifndef DISABLE_SYSCALL_TRACING

#define sensor_sample_fetch(dev) ({ 	int retval; 	sys_port_trace_syscall_enter(K_SYSCALL_SENSOR_SAMPLE_FETCH, sensor_sample_fetch, dev); 	retval = sensor_sample_fetch(dev); 	sys_port_trace_syscall_exit(K_SYSCALL_SENSOR_SAMPLE_FETCH, sensor_sample_fetch, dev, retval); 	retval; })
#endif
#endif


extern int z_impl_sensor_sample_fetch_chan(const struct device * dev, enum sensor_channel type);

__pinned_func
static inline int sensor_sample_fetch_chan(const struct device * dev, enum sensor_channel type)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		/* coverity[OVERRUN] */
		return (int) arch_syscall_invoke2(*(uintptr_t *)&dev, *(uintptr_t *)&type, K_SYSCALL_SENSOR_SAMPLE_FETCH_CHAN);
	}
#endif
	compiler_barrier();
	return z_impl_sensor_sample_fetch_chan(dev, type);
}

#if (CONFIG_TRACING_SYSCALL == 1)
#ifndef DISABLE_SYSCALL_TRACING

#define sensor_sample_fetch_chan(dev, type) ({ 	int retval; 	sys_port_trace_syscall_enter(K_SYSCALL_SENSOR_SAMPLE_FETCH_CHAN, sensor_sample_fetch_chan, dev, type); 	retval = sensor_sample_fetch_chan(dev, type); 	sys_port_trace_syscall_exit(K_SYSCALL_SENSOR_SAMPLE_FETCH_CHAN, sensor_sample_fetch_chan, dev, type, retval); 	retval; })
#endif
#endif


extern int z_impl_sensor_channel_get(const struct device * dev, enum sensor_channel chan, struct sensor_value * val);

__pinned_func
static inline int sensor_channel_get(const struct device * dev, enum sensor_channel chan, struct sensor_value * val)
{
#ifdef CONFIG_USERSPACE
	if (z_syscall_trap()) {
		/* coverity[OVERRUN] */
		return (int) arch_syscall_invoke3(*(uintptr_t *)&dev, *(uintptr_t *)&chan, *(uintptr_t *)&val, K_SYSCALL_SENSOR_CHANNEL_GET);
	}
#endif
	compiler_barrier();
	return z_impl_sensor_channel_get(dev, chan, val);
}

#if (CONFIG_TRACING_SYSCALL == 1)
#ifndef DISABLE_SYSCALL_TRACING

#define sensor_channel_get(dev, chan, val) ({ 	int retval; 	sys_port_trace_syscall_enter(K_SYSCALL_SENSOR_CHANNEL_GET, sensor_channel_get, dev, chan, val); 	retval = sensor_channel_get(dev, chan, val); 	sys_port_trace_syscall_exit(K_SYSCALL_SENSOR_CHANNEL_GET, sensor_channel_get, dev, chan, val, retval); 	retval; })
#endif
#endif


#ifdef __cplusplus
}
#endif

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic pop
#endif

#endif
#endif /* include guard */
