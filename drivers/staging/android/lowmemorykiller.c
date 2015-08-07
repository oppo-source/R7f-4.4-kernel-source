/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*******************change log***********************/
/* jiemin.zhu@oppo.com, 2014-10-23
 * 1. add kill duplicate tasks
 * 2. add kill burst
 * 3. add kernel thread to calculate
 * 4. other minor changes
 */
/*********************end***********************/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt


#ifndef VENDOR_EDIT
#undef CONFIG_ENHANCED_LMK
#endif /* VENDOR_EDIT */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/rcupdate.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/swap.h>
#include <linux/fs.h>
#ifdef VENDOR_EDIT
//Lycan.Wang@Prd.BasicDrv, 2014-07-29 Add for workaround method for SHM memleak
#include <linux/ipc_namespace.h>
#endif /* VENDOR_EDIT */
#ifdef CONFIG_ENHANCED_LMK
//jiemin.zhu@oppo.com, 2014-10-23 Add for kill duplicate tasks
#include <linux/version.h>
#include <linux/list.h>
#include <linux/slab.h>
#endif /* CONFIG_ENHANCED_LMK */

#ifdef CONFIG_HIGHMEM
#define _ZONE ZONE_HIGHMEM
#else
#define _ZONE ZONE_NORMAL
#endif
#ifdef CONFIG_ENHANCED_LMK
//jiemin.zhu@oppo.com, 2014-10-23 Add for kill duplicate tasks
//you should reconfig your kernel for using this feature:
//devices drivers->staging drivers->android->enhanced lowmemorykiller 

#define MAX_PROCESS 2048	/* maximun kinds of processes in the system */
#define MAX_DUPLICATE_TASK   20 /* threshold */
#define LOWMEM_DEATHPENDING_DEPTH	5 /* default NUM kill in one burst */
#define MIN_ENHANCE_ADJ		176//117  /* default adj is 0 58 117 176 529 1000 */
#define MAX_PERSISTENT_ATTEMPT	10

#endif /* CONFIG_ENHANCED_LMK */

#ifndef CONFIG_ENHANCED_LMK
#ifdef VENDOR_EDIT
//xuanzhi.qin@Swdp.Android.FrameworkUi, 2015/01/29, add  to kill bad process which maybe  lead to memleak
static uint32_t goto_oom_level = 2;
#endif /* VENDOR_EDIT */
#endif /* CONFIG_ENHANCED_LMK */

static uint32_t lowmem_debug_level = 1;
static short lowmem_adj[6] = {
	0,
	1,
	6,
	12,
};
static int lowmem_adj_size = 4;
static int lowmem_minfree[6] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};
static int lowmem_minfree_size = 4;
static int lmk_fast_run = 1;

static unsigned long lowmem_deathpending_timeout;

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_info(x);			\
	} while (0)

static int test_task_flag(struct task_struct *p, int flag)
{
	struct task_struct *t = p;

	do {
		task_lock(t);
		if (test_tsk_thread_flag(t, flag)) {
			task_unlock(t);
			return 1;
		}
		task_unlock(t);
	} while_each_thread(p, t);

	return 0;
}

static DEFINE_MUTEX(scan_mutex);

int can_use_cma_pages(gfp_t gfp_mask)
{
	int can_use = 0;
	int mtype = allocflags_to_migratetype(gfp_mask);
	int i = 0;
	int *mtype_fallbacks = get_migratetype_fallbacks(mtype);

	if (is_migrate_cma(mtype)) {
		can_use = 1;
	} else {
		for (i = 0;; i++) {
			int fallbacktype = mtype_fallbacks[i];

			if (is_migrate_cma(fallbacktype)) {
				can_use = 1;
				break;
			}

			if (fallbacktype == MIGRATE_RESERVE)
				break;
		}
	}
	return can_use;
}

void tune_lmk_zone_param(struct zonelist *zonelist, int classzone_idx,
					int *other_free, int *other_file,
					int use_cma_pages)
{
	struct zone *zone;
	struct zoneref *zoneref;
	int zone_idx;

	for_each_zone_zonelist(zone, zoneref, zonelist, MAX_NR_ZONES) {
		zone_idx = zonelist_zone_idx(zoneref);
		if (zone_idx == ZONE_MOVABLE) {
			if (!use_cma_pages)
				*other_free -=
				    zone_page_state(zone, NR_FREE_CMA_PAGES);
			continue;
		}

		if (zone_idx > classzone_idx) {
			if (other_free != NULL)
				*other_free -= zone_page_state(zone,
							       NR_FREE_PAGES);
			if (other_file != NULL)
				*other_file -= zone_page_state(zone,
							       NR_FILE_PAGES)
					      - zone_page_state(zone, NR_SHMEM);
		} else if (zone_idx < classzone_idx) {
			if (zone_watermark_ok(zone, 0, 0, classzone_idx, 0)) {
				if (!use_cma_pages) {
					*other_free -= min(
					  zone->lowmem_reserve[classzone_idx] +
					  zone_page_state(
					    zone, NR_FREE_CMA_PAGES),
					  zone_page_state(
					    zone, NR_FREE_PAGES));
				} else {
					*other_free -=
					  zone->lowmem_reserve[classzone_idx];
				}
			} else {
				*other_free -=
					   zone_page_state(zone, NR_FREE_PAGES);
			}
		}
	}
}

#ifdef CONFIG_HIGHMEM
void adjust_gfp_mask(gfp_t *gfp_mask)
{
	struct zone *preferred_zone;
	struct zonelist *zonelist;
	enum zone_type high_zoneidx;

	if (current_is_kswapd()) {
		zonelist = node_zonelist(0, *gfp_mask);
		high_zoneidx = gfp_zone(*gfp_mask);
		first_zones_zonelist(zonelist, high_zoneidx, NULL,
				&preferred_zone);

		if (high_zoneidx == ZONE_NORMAL) {
			if (zone_watermark_ok_safe(preferred_zone, 0,
					high_wmark_pages(preferred_zone), 0,
					0))
				*gfp_mask |= __GFP_HIGHMEM;
		} else if (high_zoneidx == ZONE_HIGHMEM) {
			*gfp_mask |= __GFP_HIGHMEM;
		}
	}
}
#else
void adjust_gfp_mask(gfp_t *unused)
{
}
#endif

void tune_lmk_param(int *other_free, int *other_file, struct shrink_control *sc)
{
	gfp_t gfp_mask;
	struct zone *preferred_zone;
	struct zonelist *zonelist;
	enum zone_type high_zoneidx, classzone_idx;
	unsigned long balance_gap;
	int use_cma_pages;

	gfp_mask = sc->gfp_mask;
	adjust_gfp_mask(&gfp_mask);

	zonelist = node_zonelist(0, gfp_mask);
	high_zoneidx = gfp_zone(gfp_mask);
	first_zones_zonelist(zonelist, high_zoneidx, NULL, &preferred_zone);
	classzone_idx = zone_idx(preferred_zone);
	use_cma_pages = can_use_cma_pages(gfp_mask);

	balance_gap = min(low_wmark_pages(preferred_zone),
			  (preferred_zone->present_pages +
			   KSWAPD_ZONE_BALANCE_GAP_RATIO-1) /
			   KSWAPD_ZONE_BALANCE_GAP_RATIO);

	if (likely(current_is_kswapd() && zone_watermark_ok(preferred_zone, 0,
			  high_wmark_pages(preferred_zone) + SWAP_CLUSTER_MAX +
			  balance_gap, 0, 0))) {
		if (lmk_fast_run)
			tune_lmk_zone_param(zonelist, classzone_idx, other_free,
				       other_file, use_cma_pages);
		else
			tune_lmk_zone_param(zonelist, classzone_idx, other_free,
				       NULL, use_cma_pages);

		if (zone_watermark_ok(preferred_zone, 0, 0, _ZONE, 0)) {
			if (!use_cma_pages) {
				*other_free -= min(
				  preferred_zone->lowmem_reserve[_ZONE]
				  + zone_page_state(
				    preferred_zone, NR_FREE_CMA_PAGES),
				  zone_page_state(
				    preferred_zone, NR_FREE_PAGES));
			} else {
				*other_free -=
				  preferred_zone->lowmem_reserve[_ZONE];
			}
		} else {
			*other_free -= zone_page_state(preferred_zone,
						      NR_FREE_PAGES);
		}

		lowmem_print(4, "lowmem_shrink of kswapd tunning for highmem "
			     "ofree %d, %d\n", *other_free, *other_file);
	} else {
		tune_lmk_zone_param(zonelist, classzone_idx, other_free,
			       other_file, use_cma_pages);

		if (!use_cma_pages) {
			*other_free -=
			  zone_page_state(preferred_zone, NR_FREE_CMA_PAGES);
		}

		lowmem_print(4, "lowmem_shrink tunning for others ofree %d, "
			     "%d\n", *other_free, *other_file);
	}
}

#ifdef CONFIG_ENHANCED_LMK
//jiemin.zhu@oppo.com, 2014-10-21 Add for sort tasks in rb tree and kill duplicate tasks
static bool protected_apps(char *comm)
{
	if (strcmp(comm, "system_server") == 0 ||
		strcmp(comm, "m.android.phone") == 0 ||
		strcmp(comm, "d.process.media") == 0 ||
		strcmp(comm, "ureguide.custom") == 0 ||
		strcmp(comm, "ndroid.systemui") == 0 ||
		strcmp(comm, "nManagerService") == 0) {
		return 1;
	}
	return 0;
}

static int resident_task_kill(int *rem)
{
	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	int tasksize;
	int selected_tasksize = 0;
	int selected_oom_score_adj;
	int ret = 0;

	for_each_process(tsk) {
		struct task_struct *p;
		int oom_score_adj;

		if (tsk->flags & PF_KTHREAD)
			continue;

		/* if task no longer has any memory ignore it */
		if (test_task_flag(tsk, TIF_MM_RELEASED))
			continue;	

		if (test_task_flag(tsk, TIF_MEMDIE) && tsk->exit_state != EXIT_ZOMBIE) {
			continue;
		}

		if (protected_apps(tsk->comm)) {
			continue;
		}

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		oom_score_adj = p->signal->oom_score_adj;
		tasksize = get_mm_rss(p->mm);
		task_unlock(p);
		if (oom_score_adj >= 0 || oom_score_adj == -941)
			continue;
		if (tasksize <= 0)
			continue;

		if (selected) {
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			if (oom_score_adj == selected_oom_score_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
		lowmem_print(1, "select persistent task %d (%s), adj %d, size %d, to kill\n",
			     p->pid, p->comm, oom_score_adj, tasksize);
	}
	
	if (selected) {
		lowmem_print(1, "Killing persistent task %d (%s), adj %d, size %d from %s\n",
			     selected->pid, selected->comm,
			     selected_oom_score_adj, selected_tasksize, current->comm);

		send_sig(SIGKILL, selected, 0);
		set_tsk_thread_flag(selected, TIF_MEMDIE);
		*rem = *rem - selected_tasksize;
		ret++;
	}

	return ret;
}

void print_all_process_tasksize(void)
{
	struct task_struct *tsk;
	int tasksize;
	int ltzero_total = 0;
	int eqzero_total = 0;
	int lgzero_total = 0;    
	int total = 0;

	lowmem_print(1, "\n--------------print all process tasksize------------\n");

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		int oom_score_adj;

		if (tsk->flags & PF_KTHREAD)
			continue;

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		oom_score_adj = p->signal->oom_score_adj;
		//tasksize = get_mm_rss(p->mm);
		tasksize = get_mm_counter(p->mm, MM_ANONPAGES);
		total += tasksize;        
		if (oom_score_adj > 0) lgzero_total += tasksize;
		if (oom_score_adj < 0) ltzero_total += tasksize;
		if (oom_score_adj == 0) eqzero_total += tasksize;

		task_unlock(p);       

		/* if task no longer has any memory ignore it */
		if (test_task_flag(tsk, TIF_MM_RELEASED)) {
			lowmem_print(2, "###Process(TIF_MM_RELEASED) %d (%s), adj %d, size %d\n",
					p->pid, p->comm, oom_score_adj, tasksize);
		} else {        
			lowmem_print(1, "###Process %d (%s), ppid %d adj %d, size %dKB\n",
					p->pid, p->comm, p->parent->pid, oom_score_adj, tasksize * 4);
		}     
	}
	rcu_read_unlock();

	lowmem_print(1, "###print_all_process_tasksize, total tasksize: (>0)%dKB (=0)%dKB (<0)%dKB, total %dKB\n", 
			lgzero_total * 4, eqzero_total * 4,
			ltzero_total * 4, total * 4);     
}

#endif /* CONFIG_ENHANCED_LMK */

#ifdef VENDOR_EDIT
//jiemin.zhu@oppo.com, 2014-12-26, modify for 8916/8939 master
static void orphan_foreground_task_kill(struct task_struct *task, short adj, short min_score_adj)
{
	if (min_score_adj == 0)
		return;

	if (task->parent->pid == 1 && adj == 0) {
		lowmem_print(1, "kill orphan foreground task %s, pid %d, adj %hd, min_score_adj %hd\n", 
					task->comm, task->pid, adj, min_score_adj);
		send_sig(SIGKILL, task, 0);
	}
}

#endif /* VENDOR_EDIT */

static int lowmem_shrink(struct shrinker *s, struct shrink_control *sc)
{
	struct task_struct *tsk;
#ifdef CONFIG_ENHANCED_LMK
	//jiemin.zhu@oppo.com, 2014-10-21 Add for kill tasks burst
	struct task_struct *selected[LOWMEM_DEATHPENDING_DEPTH] = {NULL,};
	int lowmem_enhance_threshold = 1;
	int have_selected = 0;
	int persistent_num = 0;	
#else
	struct task_struct *selected = NULL;
#endif	/* CONFIG_ENHANCED_LMK */
	int rem = 0;
	int tasksize;
	int i;
#ifdef CONFIG_ENHANCED_LMK
//jiemin.zhu@oppo.com, 2014-10-21 Add for kill tasks burst
	int min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int selected_tasksize[LOWMEM_DEATHPENDING_DEPTH] = {0,};
	int selected_oom_score_adj[LOWMEM_DEATHPENDING_DEPTH] = {OOM_ADJUST_MAX,};
	int all_selected_oom = 0;
	int max_selected_oom_idx = 0;
#else
	short min_score_adj = OOM_SCORE_ADJ_MAX + 1;	
	int selected_tasksize = 0;
	short selected_oom_score_adj;
#endif	/* CONFIG_ENHANCED_LMK */
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free;
	int other_file;
	int minfree = 0;
	unsigned long nr_to_scan = sc->nr_to_scan;

#ifndef CONFIG_ENHANCED_LMK
#ifdef VENDOR_EDIT
	//xuanzhi.qin@Swdp.Android.FrameworkUi, 2015/01/29, add  to kill bad process which maybe  lead to memleak
	static unsigned long kill_foreground_count;
	static unsigned long oom_deathpending_timeout;
#endif /* VENDOR_EDIT */
#endif /* CONFIG_ENHANCED_LMK */

	if (nr_to_scan > 0) {
		if (mutex_lock_interruptible(&scan_mutex) < 0)
			return 0;
	}
	other_free = global_page_state(NR_FREE_PAGES);

	if (global_page_state(NR_SHMEM) + total_swapcache_pages() <
		global_page_state(NR_FILE_PAGES))
		other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM) -
						total_swapcache_pages();
	else
		other_file = 0;

	tune_lmk_param(&other_free, &other_file, sc);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		minfree = lowmem_minfree[i];
		if (other_free < minfree && other_file < minfree) {
			min_score_adj = lowmem_adj[i];
			break;
		}
	}
	if (nr_to_scan > 0)
		lowmem_print(3, "lowmem_shrink %lu, %x, ofree %d %d, ma %hd\n",
				nr_to_scan, sc->gfp_mask, other_free,
				other_file, min_score_adj);
	rem = global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
	if (nr_to_scan <= 0 || min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		lowmem_print(5, "lowmem_shrink %lu, %x, return %d\n",
			     nr_to_scan, sc->gfp_mask, rem);

		if (nr_to_scan > 0)
			mutex_unlock(&scan_mutex);

		return rem;
	}
#ifdef CONFIG_ENHANCED_LMK
//jiemin.zhu@oppo.com, 2014-10-21 Add for kill tasks burst
	for (i = 0; i < LOWMEM_DEATHPENDING_DEPTH; i++)
		selected_oom_score_adj[i] = min_score_adj;
	if (min_score_adj <= MIN_ENHANCE_ADJ)
		lowmem_enhance_threshold = LOWMEM_DEATHPENDING_DEPTH;
#else
	selected_oom_score_adj = min_score_adj;
#endif	/* CONFIG_ENHANCED_LMK */	
	
	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;
#ifdef CONFIG_ENHANCED_LMK
//jiemin.zhu@oppo.com, 2014-10-21 Add for kill tasks burst
		int is_exist_oom_task = 0;
#endif /* CONFIG_ENHANCED_LMK */		

		if (tsk->flags & PF_KTHREAD)
			continue;

		/* if task no longer has any memory ignore it */
		if (test_task_flag(tsk, TIF_MM_RELEASED))
			continue;

		if (time_before_eq(jiffies, lowmem_deathpending_timeout)) {
#ifdef CONFIG_ENHANCED_LMK
/* jiemin.zhu@oppo.com 
 * When the task has been set to TIF_MEMDIE,and tsk->exit_state is not
 * EXIT_ZOMBIE,return 0, to avoid low efficiency that lmk kill process.
 */
			if (test_task_flag(tsk, TIF_MEMDIE) && tsk->exit_state != EXIT_ZOMBIE) {
#else			
			if (test_task_flag(tsk, TIF_MEMDIE)) {
#endif
				rcu_read_unlock();
				/* give the system time to free up the memory */
				msleep_interruptible(20);
				mutex_unlock(&scan_mutex);
				lowmem_print(1, "LMK-return, %d (%s), exit_state %d, task flag 0x%x, task state %ld, jiffies %ld lowmem_deathpending_timeout %ld\n",
							tsk->pid, tsk->comm, tsk->exit_state, tsk->flags, tsk->state, jiffies, lowmem_deathpending_timeout);
				return 0;
			}
		}

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		oom_score_adj = p->signal->oom_score_adj;
		if (oom_score_adj < min_score_adj) {
#ifdef VENDOR_EDIT
//xuanzhi.qin@oppo.com, move task_unlock(p) to here advoid above func call  NULL poiter (p->mm == NULL)  lead to system panic 			
			tasksize = get_mm_rss(p->mm);
#endif
			task_unlock(p);
#ifdef VENDOR_EDIT
//jiemin.zhu@oppo.com, add for tencent native process			
			if (tasksize > 0)
				orphan_foreground_task_kill(p, oom_score_adj, min_score_adj);
#endif			
			continue;
		}
		tasksize = get_mm_rss(p->mm);
		task_unlock(p);
		if (tasksize <= 0)
			continue;

#ifdef CONFIG_ENHANCED_LMK
//jiemin.zhu@oppo.com, 2014-10-21 Add for kill tasks burst
		if (all_selected_oom < lowmem_enhance_threshold) {
			for (i = 0; i < lowmem_enhance_threshold; i++) {
				if (!selected[i]) {
					is_exist_oom_task = 1;
					max_selected_oom_idx = i;
					break;
				}
			}
		} else if (selected_oom_score_adj[max_selected_oom_idx] < oom_score_adj ||
			(selected_oom_score_adj[max_selected_oom_idx] == oom_score_adj &&
			selected_tasksize[max_selected_oom_idx] < tasksize)) {
			is_exist_oom_task = 1;
		}

		if (is_exist_oom_task) {
			selected[max_selected_oom_idx] = p;
			selected_tasksize[max_selected_oom_idx] = tasksize;
			selected_oom_score_adj[max_selected_oom_idx] = oom_score_adj;

			if (all_selected_oom < lowmem_enhance_threshold)
				all_selected_oom++;

			if (all_selected_oom == lowmem_enhance_threshold) {
				for (i = 0; i < lowmem_enhance_threshold; i++) {
					if (selected_oom_score_adj[i] < selected_oom_score_adj[max_selected_oom_idx])
						max_selected_oom_idx = i;
					else if (selected_oom_score_adj[i] == selected_oom_score_adj[max_selected_oom_idx] &&
						selected_tasksize[i] < selected_tasksize[max_selected_oom_idx])
						max_selected_oom_idx = i;
				}
			}

			lowmem_print(2, "select %d (%s), adj %d, \
					size %d, to kill\n",
				p->pid, p->comm, oom_score_adj, tasksize);
		}
	}
	for (i = 0; i < lowmem_enhance_threshold; i++) {
		if (selected[i]) {
			if (selected_oom_score_adj[i] == 0) {
				if (persistent_num == 0) {
					lowmem_print(1, "try to kill foreground task, then goto kill persistent task\n");
					persistent_num = resident_task_kill(&rem);
					have_selected += persistent_num;
					break;
				}
				continue;
			}
			lowmem_print(1, "Killing '%s' (%d), adj %hd,\n" \
					"   to free %dkB on behalf of '%s' (%d) because\n" \
					"   cache %dkB is below limit %dkB for oom_score_adj %hd\n" \
					"   Free memory is %dkB above reserved\n",
			     	selected[i]->comm, selected[i]->pid,
			     	selected_oom_score_adj[i],
			    	 selected_tasksize[i] * 4,
			     	current->comm, current->pid,
			     	other_file * 4,
			     	minfree * 4,
			     	min_score_adj,
			     	other_free * 4);
			send_sig(SIGKILL, selected[i], 0);
			set_tsk_thread_flag(selected[i], TIF_MEMDIE);	
			rem -= selected_tasksize[i];
			have_selected++;
		}
	}
	rcu_read_unlock();
	if (have_selected) {
#ifdef VENDOR_EDIT
        //Lycan.Wang@Prd.BasicDrv, 2014-07-29 Add for workaround method for SHM memleak	
		if (totalram_pages / global_page_state(NR_SHMEM) < 10) {
			struct ipc_namespace *ns = current->nsproxy->ipc_ns;
			int backup_shm_rmid_forced = ns->shm_rmid_forced;

			lowmem_print(1, "Shmem too large (%ldKB)\n Try to release IPC shmem !\n", global_page_state(NR_SHMEM) * (long)(PAGE_SIZE / 1024));
			ns->shm_rmid_forced = 1;
			shm_destroy_orphaned(ns);
			ns->shm_rmid_forced = backup_shm_rmid_forced;
		}
#endif /* VENDOR_EDIT */
		lowmem_deathpending_timeout = jiffies + HZ;
		for (i = 0; i < lowmem_enhance_threshold; i++) {
			if (selected[i]) {
				if (selected_oom_score_adj[i] == 0 && lowmem_debug_level > 1) {
					print_all_process_tasksize();
					break;
				}
			}
		}
		/* give the system time to free up the memory */
		msleep_interruptible(20);
	}
#else
		if (selected) {
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			if (oom_score_adj == selected_oom_score_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
		lowmem_print(2, "select '%s' (%d), adj %hd, size %d, to kill\n",
			     p->comm, p->pid, oom_score_adj, tasksize);
	}
	if (selected) {
		lowmem_print(1, "Killing '%s' (%d), adj %hd,\n" \
				"   to free %ldkB on behalf of '%s' (%d) because\n" \
				"   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n" \
				"   Free memory is %ldkB above reserved\n",
			     selected->comm, selected->pid,
			     selected_oom_score_adj,
			     selected_tasksize * (long)(PAGE_SIZE / 1024),
			     current->comm, current->pid,
			     other_file * (long)(PAGE_SIZE / 1024),
			     minfree * (long)(PAGE_SIZE / 1024),
			     min_score_adj,
			     other_free * (long)(PAGE_SIZE / 1024));
		lowmem_deathpending_timeout = jiffies + HZ;
		send_sig(SIGKILL, selected, 0);
		set_tsk_thread_flag(selected, TIF_MEMDIE);
		rem -= selected_tasksize;
		rcu_read_unlock();
#ifdef VENDOR_EDIT
        //Lycan.Wang@Prd.BasicDrv, 2014-07-29 Add for workaround method for SHM memleak
        if (totalram_pages / global_page_state(NR_SHMEM) < 10) {
            struct ipc_namespace *ns = current->nsproxy->ipc_ns;
            int backup_shm_rmid_forced = ns->shm_rmid_forced;

            lowmem_print(1, "Shmem too large (%ldKB)\n Try to release IPC shmem !\n", global_page_state(NR_SHMEM) * (long)(PAGE_SIZE / 1024));
            ns->shm_rmid_forced = 1;
            shm_destroy_orphaned(ns);
            ns->shm_rmid_forced = backup_shm_rmid_forced;
        }
#endif /* VENDOR_EDIT */
		/* give the system time to free up the memory */
		msleep_interruptible(20);
#ifdef VENDOR_EDIT
		//xuanzhi.qin@Swdp.Android.FrameworkUi, 2015/01/29, add  to kill bad process which maybe  lead to memleak
		if (selected_oom_score_adj == 0) {
			if (kill_foreground_count == 0)
				oom_deathpending_timeout = jiffies + HZ;
			kill_foreground_count++;
		}
		if (time_before_eq(jiffies, oom_deathpending_timeout)) {
			if (kill_foreground_count >= goto_oom_level) {
				out_of_memory(NULL, 0, 0, NULL, true);
				kill_foreground_count = 0;
			}
		} else
			kill_foreground_count = 0;
#endif /* VENDOR_EDIT */
	} else
		rcu_read_unlock();
#endif /* CONFIG_ENHANCED_LMK */	

	lowmem_print(4, "lowmem_shrink %lu, %x, return %d\n",
		     nr_to_scan, sc->gfp_mask, rem);

	mutex_unlock(&scan_mutex);
	
	return rem;
}

static struct shrinker lowmem_shrinker = {
	.shrink = lowmem_shrink,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
	register_shrinker(&lowmem_shrinker);
	return 0;
}

static void __exit lowmem_exit(void)
{
	unregister_shrinker(&lowmem_shrinker);
}

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static short lowmem_oom_adj_to_oom_score_adj(short oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	short oom_adj;
	short oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_short,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
__module_param_call(MODULE_PARAM_PREFIX, adj,
		    &lowmem_adj_array_ops,
		    .arr = &__param_arr_adj,
		    S_IRUGO | S_IWUSR, -1);
__MODULE_PARM_TYPE(adj, "array of short");
#else
module_param_array_named(adj, lowmem_adj, short, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);
module_param_named(lmk_fast_run, lmk_fast_run, int, S_IRUGO | S_IWUSR);

#ifndef CONFIG_ENHANCED_LMK
#ifdef VENDOR_EDIT
//xuanzhi.qin@Swdp.Android.FrameworkUi, 2015/01/29, add  to kill bad process which maybe  lead to memleak
module_param_named(lmk_oom_level, goto_oom_level, uint, S_IRUGO | S_IWUSR);
#endif /* VENDOR_EDIT */
#endif /*CONFIG_ENHANCED_LMK*/

module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

