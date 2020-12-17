#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <linux/unistd.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <sys/syscall.h>
#include <pthread.h>

#define SCHED_DEADLINE 6
/* XXX use the proper syscall numbers */
#ifdef __x86_64__
#define __NR_sched_setattr 314
#define __NR_sched_getattr 315
#endif
#ifdef __i386__
#define __NR_sched_setattr 351
#define __NR_sched_getattr 352
#endif
#ifdef __arm__
#define __NR_sched_setattr 380
#define __NR_sched_getattr 381
#endif

struct sched_attr {
__u32 size;
__u32 sched_policy;
__u64 sched_flags;
/* SCHED_NORMAL, SCHED_BATCH */
__s32 sched_nice;
/* SCHED_FIFO, SCHED_RR */
__u32 sched_priority;
/* SCHED_DEADLINE (nsec) */
__u64 sched_runtime;
__u64 sched_deadline;
__u64 sched_period;
};

int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags);
/*
int sched_setattr(pid_t pid,
const struct sched_attr *attr,
unsigned int flags)
{
return syscall(__NR_sched_setattr, pid, attr, flags);
}
int sched_getattr(pid_t pid,
struct sched_attr *attr,
unsigned int size,
unsigned int flags)
{
return syscall(__NR_sched_getattr, pid, attr, size, flags);
}
*/
