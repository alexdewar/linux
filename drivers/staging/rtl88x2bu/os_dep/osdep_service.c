/******************************************************************************
 *
 * Copyright(c) 2007 - 2019 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/


#define _OSDEP_SERVICE_C_

#include <drv_types.h>

#define RT_TAG	'1178'



/*
* Translate the OS dependent @param error_code to OS independent RTW_STATUS_CODE
* @return: one of RTW_STATUS_CODE
*/
inline int RTW_STATUS_CODE(int error_code)
{
	if (error_code >= 0)
		return _SUCCESS;

	switch (error_code) {
	/* case -ETIMEDOUT: */
	/*	return RTW_STATUS_TIMEDOUT; */
	default:
		return _FAIL;
	}
}

u32 rtw_atoi(u8 *s)
{

	int num = 0, flag = 0;
	int i;
	for (i = 0; i <= strlen(s); i++) {
		if (s[i] >= '0' && s[i] <= '9')
			num = num * 10 + s[i] - '0';
		else if (s[0] == '-' && i == 0)
			flag = 1;
		else
			break;
	}

	if (flag == 1)
		num = num * -1;

	return num;

}

inline void *_rtw_vmalloc(u32 sz)
{
	void *pbuf;
	pbuf = vmalloc(sz);



	return pbuf;
}

inline void *_rtw_zvmalloc(u32 sz)
{
	void *pbuf;
	pbuf = _rtw_vmalloc(sz);
	if (pbuf != NULL)
		memset(pbuf, 0, sz);

	return pbuf;
}

inline void _rtw_vmfree(void *pbuf, u32 sz)
{
	vfree(pbuf);

}

void *_rtw_malloc(u32 sz)
{
	void *pbuf = NULL;

#ifdef RTK_DMP_PLATFORM
	if (sz > 0x4000)
		pbuf = dvr_malloc(sz);
	else
#endif
		pbuf = kmalloc(sz, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);



	return pbuf;

}


void *_rtw_zmalloc(u32 sz)
{
	void *pbuf = _rtw_malloc(sz);

	if (pbuf != NULL) {

		memset(pbuf, 0, sz);


	}

	return pbuf;
}

void _rtw_mfree(void *pbuf, u32 sz)
{

#ifdef RTK_DMP_PLATFORM
	if (sz > 0x4000)
		dvr_free(pbuf);
	else
#endif
		kfree(pbuf);



}


inline struct sk_buff *_rtw_skb_alloc(u32 sz)
{
	return __dev_alloc_skb(sz, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);

}

inline void _rtw_skb_free(struct sk_buff *skb)
{
	dev_kfree_skb_any(skb);
}

inline struct sk_buff *_rtw_skb_copy(const struct sk_buff *skb)
{
	return skb_copy(skb, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);

}

inline struct sk_buff *_rtw_skb_clone(struct sk_buff *skb)
{
	return skb_clone(skb, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);

}
inline struct sk_buff *_rtw_pskb_copy(struct sk_buff *skb)
{
	return pskb_copy(skb, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);

}

inline int _rtw_netif_rx(_nic_hdl ndev, struct sk_buff *skb)
{
	skb->dev = ndev;
	return netif_rx(skb);
}

inline int _rtw_netif_receive_skb(_nic_hdl ndev, struct sk_buff *skb)
{
	skb->dev = ndev;
	return netif_receive_skb(skb);
}

inline gro_result_t _rtw_napi_gro_receive(struct napi_struct *napi, struct sk_buff *skb)
{
	return napi_gro_receive(napi, skb);
}

void _rtw_skb_queue_purge(struct sk_buff_head *list)
{
	struct sk_buff *skb;

	while ((skb = skb_dequeue(list)) != NULL)
		_rtw_skb_free(skb);
}

inline void *_rtw_usb_buffer_alloc(struct usb_device *dev, size_t size, dma_addr_t *dma)
{
	return usb_alloc_coherent(dev, size, (in_interrupt() ? GFP_ATOMIC : GFP_KERNEL), dma);

}
inline void _rtw_usb_buffer_free(struct usb_device *dev, size_t size, void *addr, dma_addr_t dma)
{
	usb_free_coherent(dev, size, addr, dma);

}


void *rtw_malloc2d(int h, int w, size_t size)
{
	int j;

	void **a = (void **) rtw_zmalloc(h * sizeof(void *) + h * w * size);
	if (a == NULL) {
		RTW_INFO("%s: alloc memory fail!\n", __FUNCTION__);
		return NULL;
	}

	for (j = 0; j < h; j++)
		a[j] = ((char *)(a + h)) + j * w * size;

	return a;
}

void rtw_mfree2d(void *pbuf, int h, int w, int size)
{
	rtw_mfree((u8 *)pbuf, h * sizeof(void *) + w * h * size);
}

inline void rtw_os_pkt_free(_pkt *pkt)
{
	rtw_skb_free(pkt);
}

inline _pkt *rtw_os_pkt_copy(_pkt *pkt)
{
	return rtw_skb_copy(pkt);
}

inline void *rtw_os_pkt_data(_pkt *pkt)
{
	return pkt->data;
}

inline u32 rtw_os_pkt_len(_pkt *pkt)
{
	return pkt->len;
}

void _rtw_memcpy(void *dst, const void *src, u32 sz)
{


	memcpy(dst, src, sz);



}

inline void _rtw_memmove(void *dst, const void *src, u32 sz)
{
	memmove(dst, src, sz);
}

int	_rtw_memcmp(const void *dst, const void *src, u32 sz)
{

	/* under Linux/GNU/GLibc, the return value of memcmp for two same mem. chunk is 0 */

	if (!(memcmp(dst, src, sz)))
		return _TRUE;
	else
		return _FALSE;





}

void _rtw_memset(void *pbuf, int c, u32 sz)
{


	memset(pbuf, c, sz);



}



void _rtw_init_listhead(_list *list)
{


	INIT_LIST_HEAD(list);



}


/*
For the following list_xxx operations,
caller must guarantee the atomic context.
Otherwise, there will be racing condition.
*/
u32	rtw_is_list_empty(_list *phead)
{


	if (list_empty(phead))
		return _TRUE;
	else
		return _FALSE;





}

void rtw_list_insert_head(_list *plist, _list *phead)
{

	list_add(plist, phead);


}

void rtw_list_insert_tail(_list *plist, _list *phead)
{


	list_add_tail(plist, phead);


}

inline void rtw_list_splice(_list *list, _list *head)
{
	list_splice(list, head);
}

inline void rtw_list_splice_init(_list *list, _list *head)
{
	list_splice_init(list, head);
}

inline void rtw_list_splice_tail(_list *list, _list *head)
{
	list_splice_tail(list, head);
}

inline void rtw_hlist_head_init(rtw_hlist_head *h)
{
	INIT_HLIST_HEAD(h);
}

inline void rtw_hlist_add_head(rtw_hlist_node *n, rtw_hlist_head *h)
{
	hlist_add_head(n, h);
}

inline void rtw_hlist_del(rtw_hlist_node *n)
{
	hlist_del(n);
}

inline void rtw_hlist_add_head_rcu(rtw_hlist_node *n, rtw_hlist_head *h)
{
	hlist_add_head_rcu(n, h);
}

inline void rtw_hlist_del_rcu(rtw_hlist_node *n)
{
	hlist_del_rcu(n);
}

void rtw_init_timer(_timer *ptimer, void *padapter, void *pfunc, void *ctx)
{
	_adapter *adapter = (_adapter *)padapter;

	_init_timer(ptimer, adapter->pnetdev, pfunc, ctx);
}

/*

Caller must check if the list is empty before calling rtw_list_delete

*/


void _rtw_init_sema(_sema	*sema, int init_val)
{


	sema_init(sema, init_val);



}

void _rtw_free_sema(_sema	*sema)
{

}

void _rtw_up_sema(_sema	*sema)
{


	up(sema);


}

u32 _rtw_down_sema(_sema *sema)
{

#if 0
	if (down_interruptible(sema))
		return _FAIL;
	else
		return _SUCCESS;
#else
	int res;

	res = down_interruptible(sema);
	if (res)
		RTW_ERR("%s: unexpected interrupted! res=%d\n",
			__FUNCTION__, res);
	return _SUCCESS;
#endif

}

inline void thread_exit(_completion *comp)
{
	complete_and_exit(comp, 0);



}

inline void _rtw_init_completion(_completion *comp)
{
	init_completion(comp);
}
inline void _rtw_wait_for_comp_timeout(_completion *comp)
{
	wait_for_completion_timeout(comp, msecs_to_jiffies(3000));
}
inline void _rtw_wait_for_comp(_completion *comp)
{
	wait_for_completion(comp);
}

void	_rtw_mutex_init(_mutex *pmutex)
{

	mutex_init(pmutex);


}

void	_rtw_mutex_free(_mutex *pmutex);
void	_rtw_mutex_free(_mutex *pmutex)
{

	mutex_destroy(pmutex);




}

void	_rtw_spinlock_init(_lock *plock)
{


	spin_lock_init(plock);


}

void	_rtw_spinlock_free(_lock *plock)
{


}


void	_rtw_spinlock(_lock	*plock)
{


	spin_lock(plock);


}

void	_rtw_spinunlock(_lock *plock)
{


	spin_unlock(plock);

}


void	_rtw_spinlock_ex(_lock	*plock)
{


	spin_lock(plock);


}

void	_rtw_spinunlock_ex(_lock *plock)
{


	spin_unlock(plock);

}



void _rtw_init_queue(_queue *pqueue)
{
	_rtw_init_listhead(&(pqueue->queue));
	_rtw_spinlock_init(&(pqueue->lock));
}

void _rtw_deinit_queue(_queue *pqueue)
{
	_rtw_spinlock_free(&(pqueue->lock));
}

u32	  _rtw_queue_empty(_queue	*pqueue)
{
	return rtw_is_list_empty(&(pqueue->queue));
}


u32 rtw_end_of_queue_search(_list *head, _list *plist)
{
	if (head == plist)
		return _TRUE;
	else
		return _FALSE;
}


systime _rtw_get_current_time(void)
{

	return jiffies;
}

inline u32 _rtw_systime_to_ms(systime stime)
{
	return jiffies_to_msecs(stime);
}

inline systime _rtw_ms_to_systime(u32 ms)
{
	return msecs_to_jiffies(ms);
}

inline systime _rtw_us_to_systime(u32 us)
{
	return usecs_to_jiffies(us);
}

/* the input parameter start use the same unit as returned by rtw_get_current_time */
inline s32 _rtw_get_passing_time_ms(systime start)
{
	return _rtw_systime_to_ms(_rtw_get_current_time() - start);
}

inline s32 _rtw_get_remaining_time_ms(systime end)
{
	return _rtw_systime_to_ms(end - _rtw_get_current_time());
}

inline s32 _rtw_get_time_interval_ms(systime start, systime end)
{
	return _rtw_systime_to_ms(end - start);
}

inline bool _rtw_time_after(systime a, systime b)
{
	return time_after(a, b);
}

void rtw_sleep_schedulable(int ms)
{


	u32 delta;

	delta = (ms * HZ) / 1000; /* (ms) */
	if (delta == 0) {
		delta = 1;/* 1 ms */
	}
	set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(delta);
	return;



}


void rtw_msleep_os(int ms)
{

	if (ms < 20) {
		unsigned long us = ms * 1000UL;
		usleep_range(us, us + 1000UL);
	} else
		msleep((unsigned int)ms);



}
void rtw_usleep_os(int us)
{

	/* msleep((unsigned int)us); */
	usleep_range(us, us + 1);



}


void rtw_mdelay_os(int ms)
{


	mdelay((unsigned long)ms);



}
void rtw_udelay_os(int us)
{


	udelay((unsigned long)us);


}

void rtw_yield_os(void)
{
	yield();
}

bool rtw_macaddr_is_larger(const u8 *a, const u8 *b)
{
	u32 va, vb;

	va = be32_to_cpu(*((u32 *)a));
	vb = be32_to_cpu(*((u32 *)b));
	if (va > vb)
		return 1;
	else if (va < vb)
		return 0;

	return be16_to_cpu(*((u16 *)(a + 4))) > be16_to_cpu(*((u16 *)(b + 4)));
}

#define RTW_SUSPEND_LOCK_NAME "rtw_wifi"
#define RTW_SUSPEND_TRAFFIC_LOCK_NAME "rtw_wifi_traffic"
#define RTW_SUSPEND_RESUME_LOCK_NAME "rtw_wifi_resume"
#ifdef CONFIG_WAKELOCK
static struct wake_lock rtw_suspend_lock;
static struct wake_lock rtw_suspend_traffic_lock;
static struct wake_lock rtw_suspend_resume_lock;
#elif defined(CONFIG_ANDROID_POWER)
static android_suspend_lock_t rtw_suspend_lock = {
	.name = RTW_SUSPEND_LOCK_NAME
};
static android_suspend_lock_t rtw_suspend_traffic_lock = {
	.name = RTW_SUSPEND_TRAFFIC_LOCK_NAME
};
static android_suspend_lock_t rtw_suspend_resume_lock = {
	.name = RTW_SUSPEND_RESUME_LOCK_NAME
};
#endif

inline void rtw_suspend_lock_init(void)
{
#ifdef CONFIG_WAKELOCK
	wake_lock_init(&rtw_suspend_lock, WAKE_LOCK_SUSPEND, RTW_SUSPEND_LOCK_NAME);
	wake_lock_init(&rtw_suspend_traffic_lock, WAKE_LOCK_SUSPEND, RTW_SUSPEND_TRAFFIC_LOCK_NAME);
	wake_lock_init(&rtw_suspend_resume_lock, WAKE_LOCK_SUSPEND, RTW_SUSPEND_RESUME_LOCK_NAME);
#elif defined(CONFIG_ANDROID_POWER)
	android_init_suspend_lock(&rtw_suspend_lock);
	android_init_suspend_lock(&rtw_suspend_traffic_lock);
	android_init_suspend_lock(&rtw_suspend_resume_lock);
#endif
}

inline void rtw_suspend_lock_uninit(void)
{
#ifdef CONFIG_WAKELOCK
	wake_lock_destroy(&rtw_suspend_lock);
	wake_lock_destroy(&rtw_suspend_traffic_lock);
	wake_lock_destroy(&rtw_suspend_resume_lock);
#elif defined(CONFIG_ANDROID_POWER)
	android_uninit_suspend_lock(&rtw_suspend_lock);
	android_uninit_suspend_lock(&rtw_suspend_traffic_lock);
	android_uninit_suspend_lock(&rtw_suspend_resume_lock);
#endif
}

inline void rtw_lock_suspend(void)
{
#ifdef CONFIG_WAKELOCK
	wake_lock(&rtw_suspend_lock);
#elif defined(CONFIG_ANDROID_POWER)
	android_lock_suspend(&rtw_suspend_lock);
#endif

#if  defined(CONFIG_WAKELOCK) || defined(CONFIG_ANDROID_POWER)
	/* RTW_INFO("####%s: suspend_lock_count:%d####\n", __FUNCTION__, rtw_suspend_lock.stat.count); */
#endif
}

inline void rtw_unlock_suspend(void)
{
#ifdef CONFIG_WAKELOCK
	wake_unlock(&rtw_suspend_lock);
#elif defined(CONFIG_ANDROID_POWER)
	android_unlock_suspend(&rtw_suspend_lock);
#endif

#if  defined(CONFIG_WAKELOCK) || defined(CONFIG_ANDROID_POWER)
	/* RTW_INFO("####%s: suspend_lock_count:%d####\n", __FUNCTION__, rtw_suspend_lock.stat.count); */
#endif
}

inline void rtw_resume_lock_suspend(void)
{
#ifdef CONFIG_WAKELOCK
	wake_lock(&rtw_suspend_resume_lock);
#elif defined(CONFIG_ANDROID_POWER)
	android_lock_suspend(&rtw_suspend_resume_lock);
#endif

#if  defined(CONFIG_WAKELOCK) || defined(CONFIG_ANDROID_POWER)
	/* RTW_INFO("####%s: suspend_lock_count:%d####\n", __FUNCTION__, rtw_suspend_lock.stat.count); */
#endif
}

inline void rtw_resume_unlock_suspend(void)
{
#ifdef CONFIG_WAKELOCK
	wake_unlock(&rtw_suspend_resume_lock);
#elif defined(CONFIG_ANDROID_POWER)
	android_unlock_suspend(&rtw_suspend_resume_lock);
#endif

#if  defined(CONFIG_WAKELOCK) || defined(CONFIG_ANDROID_POWER)
	/* RTW_INFO("####%s: suspend_lock_count:%d####\n", __FUNCTION__, rtw_suspend_lock.stat.count); */
#endif
}

inline void rtw_lock_suspend_timeout(u32 timeout_ms)
{
#ifdef CONFIG_WAKELOCK
	wake_lock_timeout(&rtw_suspend_lock, rtw_ms_to_systime(timeout_ms));
#elif defined(CONFIG_ANDROID_POWER)
	android_lock_suspend_auto_expire(&rtw_suspend_lock, rtw_ms_to_systime(timeout_ms));
#endif
}


inline void rtw_lock_traffic_suspend_timeout(u32 timeout_ms)
{
#ifdef CONFIG_WAKELOCK
	wake_lock_timeout(&rtw_suspend_traffic_lock, rtw_ms_to_systime(timeout_ms));
#elif defined(CONFIG_ANDROID_POWER)
	android_lock_suspend_auto_expire(&rtw_suspend_traffic_lock, rtw_ms_to_systime(timeout_ms));
#endif
	/* RTW_INFO("traffic lock timeout:%d\n", timeout_ms); */
}

inline void rtw_set_bit(int nr, unsigned long *addr)
{
	set_bit(nr, addr);
}

inline void rtw_clear_bit(int nr, unsigned long *addr)
{
	clear_bit(nr, addr);
}

inline int rtw_test_and_clear_bit(int nr, unsigned long *addr)
{
	return test_and_clear_bit(nr, addr);
}

inline void ATOMIC_SET(ATOMIC_T *v, int i)
{
	atomic_set(v, i);
}

inline int ATOMIC_READ(ATOMIC_T *v)
{
	return atomic_read(v);
}

inline void ATOMIC_ADD(ATOMIC_T *v, int i)
{
	atomic_add(i, v);
}
inline void ATOMIC_SUB(ATOMIC_T *v, int i)
{
	atomic_sub(i, v);
}

inline void ATOMIC_INC(ATOMIC_T *v)
{
	atomic_inc(v);
}

inline void ATOMIC_DEC(ATOMIC_T *v)
{
	atomic_dec(v);
}

inline int ATOMIC_ADD_RETURN(ATOMIC_T *v, int i)
{
	return atomic_add_return(i, v);
}

inline int ATOMIC_SUB_RETURN(ATOMIC_T *v, int i)
{
	return atomic_sub_return(i, v);
}

inline int ATOMIC_INC_RETURN(ATOMIC_T *v)
{
	return atomic_inc_return(v);
}

inline int ATOMIC_DEC_RETURN(ATOMIC_T *v)
{
	return atomic_dec_return(v);
}

inline bool ATOMIC_INC_UNLESS(ATOMIC_T *v, int u)
{
	return atomic_add_unless(v, 1, u);
}

/*
* Open a file with the specific @param path, @param flag, @param mode
* @param fpp the pointer of struct file pointer to get struct file pointer while file opening is success
* @param path the path of the file to open
* @param flag file operation flags, please refer to linux document
* @param mode please refer to linux document
* @return Linux specific error code
*/
static int openFile(struct file **fpp, const char *path, int flag, int mode)
{
	struct file *fp;

	fp = filp_open(path, flag, mode);
	if (IS_ERR(fp)) {
		*fpp = NULL;
		return PTR_ERR(fp);
	} else {
		*fpp = fp;
		return 0;
	}
}

/*
* Close the file with the specific @param fp
* @param fp the pointer of struct file to close
* @return always 0
*/
static int closeFile(struct file *fp)
{
	filp_close(fp, NULL);
	return 0;
}

static int readFile(struct file *fp, char *buf, int len)
{
	int rlen = 0, sum = 0;

	if (!(fp->f_mode & FMODE_CAN_READ))
		return -EPERM;

	while (sum < len) {
		rlen = kernel_read(fp, buf + sum, len - sum, &fp->f_pos);
		if (rlen > 0)
			sum += rlen;
		else if (0 != rlen)
			return rlen;
		else
			break;
	}

	return  sum;

}

static int writeFile(struct file *fp, char *buf, int len)
{
	int wlen = 0, sum = 0;

	if (!(fp->f_mode & FMODE_CAN_WRITE))
		return -EPERM;

	while (sum < len) {
		wlen = kernel_write(fp, buf + sum, len - sum, &fp->f_pos);
		if (wlen > 0)
			sum += wlen;
		else if (0 != wlen)
			return wlen;
		else
			break;
	}

	return sum;

}

/*
* Test if the specifi @param pathname is a direct and readable
* If readable, @param sz is not used
* @param pathname the name of the path to test
* @return Linux specific error code
*/
static int isDirReadable(const char *pathname, u32 *sz)
{
	struct path path;
	int error = 0;

	return kern_path(pathname, LOOKUP_FOLLOW, &path);
}

/*
* Test if the specifi @param path is a file and readable
* If readable, @param sz is got
* @param path the path of the file to test
* @return Linux specific error code
*/
static int isFileReadable(const char *path, u32 *sz)
{
	struct file *fp;
	int ret = 0;
	mm_segment_t oldfs;
	char buf;

	fp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(fp))
		ret = PTR_ERR(fp);
	else {
		oldfs = get_fs();
		set_fs(KERNEL_DS);

		if (1 != readFile(fp, &buf, 1))
			ret = PTR_ERR(fp);

		if (ret == 0 && sz) {
			*sz = i_size_read(fp->f_path.dentry->d_inode);
		}

		set_fs(oldfs);
		filp_close(fp, NULL);
	}
	return ret;
}

/*
* Open the file with @param path and retrive the file content into memory starting from @param buf for @param sz at most
* @param path the path of the file to open and read
* @param buf the starting address of the buffer to store file content
* @param sz how many bytes to read at most
* @return the byte we've read, or Linux specific error code
*/
static int retriveFromFile(const char *path, u8 *buf, u32 sz)
{
	int ret = -1;
	mm_segment_t oldfs;
	struct file *fp;

	if (path && buf) {
		ret = openFile(&fp, path, O_RDONLY, 0);
		if (0 == ret) {
			RTW_INFO("%s openFile path:%s fp=%p\n", __FUNCTION__, path , fp);

			oldfs = get_fs();
			set_fs(KERNEL_DS);
			ret = readFile(fp, buf, sz);
			set_fs(oldfs);
			closeFile(fp);

			RTW_INFO("%s readFile, ret:%d\n", __FUNCTION__, ret);

		} else
			RTW_INFO("%s openFile path:%s Fail, ret:%d\n", __FUNCTION__, path, ret);
	} else {
		RTW_INFO("%s NULL pointer\n", __FUNCTION__);
		ret =  -EINVAL;
	}
	return ret;
}

/*
* Open the file with @param path and wirte @param sz byte of data starting from @param buf into the file
* @param path the path of the file to open and write
* @param buf the starting address of the data to write into file
* @param sz how many bytes to write at most
* @return the byte we've written, or Linux specific error code
*/
static int storeToFile(const char *path, u8 *buf, u32 sz)
{
	int ret = 0;
	mm_segment_t oldfs;
	struct file *fp;

	if (path && buf) {
		ret = openFile(&fp, path, O_CREAT | O_WRONLY, 0666);
		if (0 == ret) {
			RTW_INFO("%s openFile path:%s fp=%p\n", __FUNCTION__, path , fp);

			oldfs = get_fs();
			set_fs(KERNEL_DS);
			ret = writeFile(fp, buf, sz);
			set_fs(oldfs);
			closeFile(fp);

			RTW_INFO("%s writeFile, ret:%d\n", __FUNCTION__, ret);

		} else
			RTW_INFO("%s openFile path:%s Fail, ret:%d\n", __FUNCTION__, path, ret);
	} else {
		RTW_INFO("%s NULL pointer\n", __FUNCTION__);
		ret =  -EINVAL;
	}
	return ret;
}

/*
* Test if the specifi @param path is a direct and readable
* @param path the path of the direct to test
* @return _TRUE or _FALSE
*/
int rtw_is_dir_readable(const char *path)
{
	if (isDirReadable(path, NULL) == 0)
		return _TRUE;
	else
		return _FALSE;
}

/*
* Test if the specifi @param path is a file and readable
* @param path the path of the file to test
* @return _TRUE or _FALSE
*/
int rtw_is_file_readable(const char *path)
{
	if (isFileReadable(path, NULL) == 0)
		return _TRUE;
	else
		return _FALSE;
}

/*
* Test if the specifi @param path is a file and readable.
* If readable, @param sz is got
* @param path the path of the file to test
* @return _TRUE or _FALSE
*/
int rtw_is_file_readable_with_size(const char *path, u32 *sz)
{
	if (isFileReadable(path, sz) == 0)
		return _TRUE;
	else
		return _FALSE;
}

/*
* Test if the specifi @param path is a readable file with valid size.
* If readable, @param sz is got
* @param path the path of the file to test
* @return _TRUE or _FALSE
*/
int rtw_readable_file_sz_chk(const char *path, u32 sz)
{
	u32 fsz;

	if (rtw_is_file_readable_with_size(path, &fsz) == _FALSE)
		return _FALSE;

	if (fsz > sz)
		return _FALSE;
	
	return _TRUE;
}

/*
* Open the file with @param path and retrive the file content into memory starting from @param buf for @param sz at most
* @param path the path of the file to open and read
* @param buf the starting address of the buffer to store file content
* @param sz how many bytes to read at most
* @return the byte we've read
*/
int rtw_retrieve_from_file(const char *path, u8 *buf, u32 sz)
{
	int ret = retriveFromFile(path, buf, sz);
	return ret >= 0 ? ret : 0;
}

/*
* Open the file with @param path and wirte @param sz byte of data starting from @param buf into the file
* @param path the path of the file to open and write
* @param buf the starting address of the data to write into file
* @param sz how many bytes to write at most
* @return the byte we've written
*/
int rtw_store_to_file(const char *path, u8 *buf, u32 sz)
{
	int ret = storeToFile(path, buf, sz);
	return ret >= 0 ? ret : 0;
}

struct net_device *rtw_alloc_etherdev_with_old_priv(int sizeof_priv, void *old_priv)
{
	struct net_device *pnetdev;
	struct rtw_netdev_priv_indicator *pnpi;

	pnetdev = alloc_etherdev_mq(sizeof(struct rtw_netdev_priv_indicator), 4);
	if (!pnetdev)
		goto RETURN;

	pnpi = netdev_priv(pnetdev);
	pnpi->priv = old_priv;
	pnpi->sizeof_priv = sizeof_priv;

RETURN:
	return pnetdev;
}

struct net_device *rtw_alloc_etherdev(int sizeof_priv)
{
	struct net_device *pnetdev;
	struct rtw_netdev_priv_indicator *pnpi;

	pnetdev = alloc_etherdev_mq(sizeof(struct rtw_netdev_priv_indicator), 4);
	if (!pnetdev)
		goto RETURN;

	pnpi = netdev_priv(pnetdev);

	pnpi->priv = rtw_zvmalloc(sizeof_priv);
	if (!pnpi->priv) {
		free_netdev(pnetdev);
		pnetdev = NULL;
		goto RETURN;
	}

	pnpi->sizeof_priv = sizeof_priv;
RETURN:
	return pnetdev;
}

void rtw_free_netdev(struct net_device *netdev)
{
	struct rtw_netdev_priv_indicator *pnpi;

	if (!netdev)
		goto RETURN;

	pnpi = netdev_priv(netdev);

	if (!pnpi->priv)
		goto RETURN;

	free_netdev(netdev);

RETURN:
	return;
}

int rtw_change_ifname(_adapter *padapter, const char *ifname)
{
	struct dvobj_priv *dvobj;
	struct net_device *pnetdev;
	struct net_device *cur_pnetdev;
	struct rereg_nd_name_data *rereg_priv;
	int ret;
	u8 rtnl_lock_needed;

	if (!padapter)
		goto error;

	dvobj = adapter_to_dvobj(padapter);
	cur_pnetdev = padapter->pnetdev;
	rereg_priv = &padapter->rereg_nd_name_priv;

	/* free the old_pnetdev */
	if (rereg_priv->old_pnetdev) {
		free_netdev(rereg_priv->old_pnetdev);
		rereg_priv->old_pnetdev = NULL;
	}

	rtnl_lock_needed = rtw_rtnl_lock_needed(dvobj);

	if (rtnl_lock_needed)
		unregister_netdev(cur_pnetdev);
	else
		unregister_netdevice(cur_pnetdev);

	rereg_priv->old_pnetdev = cur_pnetdev;

	pnetdev = rtw_init_netdev(padapter);
	if (!pnetdev)  {
		ret = -1;
		goto error;
	}

	SET_NETDEV_DEV(pnetdev, dvobj_to_dev(adapter_to_dvobj(padapter)));

	rtw_init_netdev_name(pnetdev, ifname);

	_rtw_memcpy(pnetdev->dev_addr, adapter_mac_addr(padapter), ETH_ALEN);

	if (rtnl_lock_needed)
		ret = register_netdev(pnetdev);
	else
		ret = register_netdevice(pnetdev);

	if (ret != 0) {
		goto error;
	}

	return 0;

error:

	return -1;

}


#ifdef CONFIG_PLATFORM_SPRD
	#ifdef do_div
		#undef do_div
	#endif
	#include <asm-generic/div64.h>
#endif

u64 rtw_modular64(u64 x, u64 y)
{
	return do_div(x, y);
}

u64 rtw_division64(u64 x, u64 y)
{
	do_div(x, y);
	return x;
}

inline u32 rtw_random32(void)
{
	return prandom_u32();
}

void rtw_buf_free(u8 **buf, u32 *buf_len)
{
	u32 ori_len;

	if (!buf || !buf_len)
		return;

	ori_len = *buf_len;

	if (*buf) {
		u32 tmp_buf_len = *buf_len;
		*buf_len = 0;
		rtw_mfree(*buf, tmp_buf_len);
		*buf = NULL;
	}
}

void rtw_buf_update(u8 **buf, u32 *buf_len, u8 *src, u32 src_len)
{
	u32 ori_len = 0, dup_len = 0;
	u8 *ori = NULL;
	u8 *dup = NULL;

	if (!buf || !buf_len)
		return;

	if (!src || !src_len)
		goto keep_ori;

	/* duplicate src */
	dup = rtw_malloc(src_len);
	if (dup) {
		dup_len = src_len;
		_rtw_memcpy(dup, src, dup_len);
	}

keep_ori:
	ori = *buf;
	ori_len = *buf_len;

	/* replace buf with dup */
	*buf_len = 0;
	*buf = dup;
	*buf_len = dup_len;

	/* free ori */
	if (ori && ori_len > 0)
		rtw_mfree(ori, ori_len);
}


/**
 * rtw_cbuf_full - test if cbuf is full
 * @cbuf: pointer of struct rtw_cbuf
 *
 * Returns: _TRUE if cbuf is full
 */
inline bool rtw_cbuf_full(struct rtw_cbuf *cbuf)
{
	return (cbuf->write == cbuf->read - 1) ? _TRUE : _FALSE;
}

/**
 * rtw_cbuf_empty - test if cbuf is empty
 * @cbuf: pointer of struct rtw_cbuf
 *
 * Returns: _TRUE if cbuf is empty
 */
inline bool rtw_cbuf_empty(struct rtw_cbuf *cbuf)
{
	return (cbuf->write == cbuf->read) ? _TRUE : _FALSE;
}

/**
 * rtw_cbuf_push - push a pointer into cbuf
 * @cbuf: pointer of struct rtw_cbuf
 * @buf: pointer to push in
 *
 * Lock free operation, be careful of the use scheme
 * Returns: _TRUE push success
 */
bool rtw_cbuf_push(struct rtw_cbuf *cbuf, void *buf)
{
	if (rtw_cbuf_full(cbuf))
		return _FAIL;

	if (0)
		RTW_INFO("%s on %u\n", __func__, cbuf->write);
	cbuf->bufs[cbuf->write] = buf;
	cbuf->write = (cbuf->write + 1) % cbuf->size;

	return _SUCCESS;
}

/**
 * rtw_cbuf_pop - pop a pointer from cbuf
 * @cbuf: pointer of struct rtw_cbuf
 *
 * Lock free operation, be careful of the use scheme
 * Returns: pointer popped out
 */
void *rtw_cbuf_pop(struct rtw_cbuf *cbuf)
{
	void *buf;
	if (rtw_cbuf_empty(cbuf))
		return NULL;

	if (0)
		RTW_INFO("%s on %u\n", __func__, cbuf->read);
	buf = cbuf->bufs[cbuf->read];
	cbuf->read = (cbuf->read + 1) % cbuf->size;

	return buf;
}

/**
 * rtw_cbuf_alloc - allocte a rtw_cbuf with given size and do initialization
 * @size: size of pointer
 *
 * Returns: pointer of srtuct rtw_cbuf, NULL for allocation failure
 */
struct rtw_cbuf *rtw_cbuf_alloc(u32 size)
{
	struct rtw_cbuf *cbuf;

	cbuf = (struct rtw_cbuf *)rtw_malloc(sizeof(*cbuf) + sizeof(void *) * size);

	if (cbuf) {
		cbuf->write = cbuf->read = 0;
		cbuf->size = size;
	}

	return cbuf;
}

/**
 * rtw_cbuf_free - free the given rtw_cbuf
 * @cbuf: pointer of struct rtw_cbuf to free
 */
void rtw_cbuf_free(struct rtw_cbuf *cbuf)
{
	rtw_mfree((u8 *)cbuf, sizeof(*cbuf) + sizeof(void *) * cbuf->size);
}

/**
 * map_readN - read a range of map data
 * @map: map to read
 * @offset: start address to read
 * @len: length to read
 * @buf: pointer of buffer to store data read
 *
 * Returns: _SUCCESS or _FAIL
 */
int map_readN(const struct map_t *map, u16 offset, u16 len, u8 *buf)
{
	const struct map_seg_t *seg;
	int ret = _FAIL;
	int i;

	if (len == 0) {
		rtw_warn_on(1);
		goto exit;
	}

	if (offset + len > map->len) {
		rtw_warn_on(1);
		goto exit;
	}

	_rtw_memset(buf, map->init_value, len);

	for (i = 0; i < map->seg_num; i++) {
		u8 *c_dst, *c_src;
		u16 c_len;

		seg = map->segs + i;
		if (seg->sa + seg->len <= offset || seg->sa >= offset + len)
			continue;

		if (seg->sa >= offset) {
			c_dst = buf + (seg->sa - offset);
			c_src = seg->c;
			if (seg->sa + seg->len <= offset + len)
				c_len = seg->len;
			else
				c_len = offset + len - seg->sa;
		} else {
			c_dst = buf;
			c_src = seg->c + (offset - seg->sa);
			if (seg->sa + seg->len >= offset + len)
				c_len = len;
			else
				c_len = seg->sa + seg->len - offset;
		}
			
		_rtw_memcpy(c_dst, c_src, c_len);
	}

exit:
	return ret;
}

/**
 * map_read8 - read 1 byte of map data
 * @map: map to read
 * @offset: address to read
 *
 * Returns: value of data of specified offset. map.init_value if offset is out of range
 */
u8 map_read8(const struct map_t *map, u16 offset)
{
	const struct map_seg_t *seg;
	u8 val = map->init_value;
	int i;

	if (offset + 1 > map->len) {
		rtw_warn_on(1);
		goto exit;
	}

	for (i = 0; i < map->seg_num; i++) {
		seg = map->segs + i;
		if (seg->sa + seg->len <= offset || seg->sa >= offset + 1)
			continue;

		val = *(seg->c + offset - seg->sa);
		break;
	}

exit:
	return val;
}

int rtw_blacklist_add(_queue *blist, const u8 *addr, u32 timeout_ms)
{
	struct blacklist_ent *ent;
	_list *list, *head;
	u8 exist = _FALSE, timeout = _FALSE;

	enter_critical_bh(&blist->lock);

	head = &blist->queue;
	list = get_next(head);
	while (rtw_end_of_queue_search(head, list) == _FALSE) {
		ent = LIST_CONTAINOR(list, struct blacklist_ent, list);
		list = get_next(list);

		if (_rtw_memcmp(ent->addr, addr, ETH_ALEN) == _TRUE) {
			exist = _TRUE;
			if (rtw_time_after(rtw_get_current_time(), ent->exp_time))
				timeout = _TRUE;
			ent->exp_time = rtw_get_current_time()
				+ rtw_ms_to_systime(timeout_ms);
			break;
		}

		if (rtw_time_after(rtw_get_current_time(), ent->exp_time)) {
			rtw_list_delete(&ent->list);
			rtw_mfree(ent, sizeof(struct blacklist_ent));
		}
	}

	if (exist == _FALSE) {
		ent = rtw_malloc(sizeof(struct blacklist_ent));
		if (ent) {
			_rtw_memcpy(ent->addr, addr, ETH_ALEN);
			ent->exp_time = rtw_get_current_time()
				+ rtw_ms_to_systime(timeout_ms);
			rtw_list_insert_tail(&ent->list, head);
		}
	}

	exit_critical_bh(&blist->lock);

	return (exist == _TRUE && timeout == _FALSE) ? RTW_ALREADY : (ent ? _SUCCESS : _FAIL);
}

int rtw_blacklist_del(_queue *blist, const u8 *addr)
{
	struct blacklist_ent *ent = NULL;
	_list *list, *head;
	u8 exist = _FALSE;

	enter_critical_bh(&blist->lock);
	head = &blist->queue;
	list = get_next(head);
	while (rtw_end_of_queue_search(head, list) == _FALSE) {
		ent = LIST_CONTAINOR(list, struct blacklist_ent, list);
		list = get_next(list);

		if (_rtw_memcmp(ent->addr, addr, ETH_ALEN) == _TRUE) {
			rtw_list_delete(&ent->list);
			rtw_mfree(ent, sizeof(struct blacklist_ent));
			exist = _TRUE;
			break;
		}

		if (rtw_time_after(rtw_get_current_time(), ent->exp_time)) {
			rtw_list_delete(&ent->list);
			rtw_mfree(ent, sizeof(struct blacklist_ent));
		}
	}

	exit_critical_bh(&blist->lock);

	return exist == _TRUE ? _SUCCESS : RTW_ALREADY;
}

int rtw_blacklist_search(_queue *blist, const u8 *addr)
{
	struct blacklist_ent *ent = NULL;
	_list *list, *head;
	u8 exist = _FALSE;

	enter_critical_bh(&blist->lock);
	head = &blist->queue;
	list = get_next(head);
	while (rtw_end_of_queue_search(head, list) == _FALSE) {
		ent = LIST_CONTAINOR(list, struct blacklist_ent, list);
		list = get_next(list);

		if (_rtw_memcmp(ent->addr, addr, ETH_ALEN) == _TRUE) {
			if (rtw_time_after(rtw_get_current_time(), ent->exp_time)) {
				rtw_list_delete(&ent->list);
				rtw_mfree(ent, sizeof(struct blacklist_ent));
			} else
				exist = _TRUE;
			break;
		}

		if (rtw_time_after(rtw_get_current_time(), ent->exp_time)) {
			rtw_list_delete(&ent->list);
			rtw_mfree(ent, sizeof(struct blacklist_ent));
		}
	}

	exit_critical_bh(&blist->lock);

	return exist;
}

void rtw_blacklist_flush(_queue *blist)
{
	struct blacklist_ent *ent;
	_list *list, *head;
	_list tmp;

	_rtw_init_listhead(&tmp);

	enter_critical_bh(&blist->lock);
	rtw_list_splice_init(&blist->queue, &tmp);
	exit_critical_bh(&blist->lock);

	head = &tmp;
	list = get_next(head);
	while (rtw_end_of_queue_search(head, list) == _FALSE) {
		ent = LIST_CONTAINOR(list, struct blacklist_ent, list);
		list = get_next(list);
		rtw_list_delete(&ent->list);
		rtw_mfree(ent, sizeof(struct blacklist_ent));
	}
}

void dump_blacklist(void *sel, _queue *blist, const char *title)
{
	struct blacklist_ent *ent = NULL;
	_list *list, *head;

	enter_critical_bh(&blist->lock);
	head = &blist->queue;
	list = get_next(head);

	if (rtw_end_of_queue_search(head, list) == _FALSE) {
		if (title)
			RTW_PRINT_SEL(sel, "%s:\n", title);
	
		while (rtw_end_of_queue_search(head, list) == _FALSE) {
			ent = LIST_CONTAINOR(list, struct blacklist_ent, list);
			list = get_next(list);

			if (rtw_time_after(rtw_get_current_time(), ent->exp_time))
				RTW_PRINT_SEL(sel, MAC_FMT" expired\n", MAC_ARG(ent->addr));
			else
				RTW_PRINT_SEL(sel, MAC_FMT" %u\n", MAC_ARG(ent->addr)
					, rtw_get_remaining_time_ms(ent->exp_time));
		}

	}
	exit_critical_bh(&blist->lock);
}

/**
* is_null -
*
* Return	TRUE if c is null character
*		FALSE otherwise.
*/
inline BOOLEAN is_null(char c)
{
	if (c == '\0')
		return _TRUE;
	else
		return _FALSE;
}

inline BOOLEAN is_all_null(char *c, int len)
{
	for (; len > 0; len--)
		if (c[len - 1] != '\0')
			return _FALSE;

	return _TRUE;
}

/**
* is_eol -
*
* Return	TRUE if c is represent for EOL (end of line)
*		FALSE otherwise.
*/
inline BOOLEAN is_eol(char c)
{
	if (c == '\r' || c == '\n')
		return _TRUE;
	else
		return _FALSE;
}

/**
* is_space -
*
* Return	TRUE if c is represent for space
*		FALSE otherwise.
*/
inline BOOLEAN is_space(char c)
{
	if (c == ' ' || c == '\t')
		return _TRUE;
	else
		return _FALSE;
}

/**
* IsHexDigit -
*
* Return	TRUE if chTmp is represent for hex digit
*		FALSE otherwise.
*/
inline BOOLEAN IsHexDigit(char chTmp)
{
	if ((chTmp >= '0' && chTmp <= '9') ||
		(chTmp >= 'a' && chTmp <= 'f') ||
		(chTmp >= 'A' && chTmp <= 'F'))
		return _TRUE;
	else
		return _FALSE;
}

/**
* is_alpha -
*
* Return	TRUE if chTmp is represent for alphabet
*		FALSE otherwise.
*/
inline BOOLEAN is_alpha(char chTmp)
{
	if ((chTmp >= 'a' && chTmp <= 'z') ||
		(chTmp >= 'A' && chTmp <= 'Z'))
		return _TRUE;
	else
		return _FALSE;
}

inline char alpha_to_upper(char c)
{
	if ((c >= 'a' && c <= 'z'))
		c = 'A' + (c - 'a');
	return c;
}

int hex2num_i(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

int hex2byte_i(const char *hex)
{
	int a, b;
	a = hex2num_i(*hex++);
	if (a < 0)
		return -1;
	b = hex2num_i(*hex++);
	if (b < 0)
		return -1;
	return (a << 4) | b;
}

int hexstr2bin(const char *hex, u8 *buf, size_t len)
{
	size_t i;
	int a;
	const char *ipos = hex;
	u8 *opos = buf;

	for (i = 0; i < len; i++) {
		a = hex2byte_i(ipos);
		if (a < 0)
			return -1;
		*opos++ = a;
		ipos += 2;
	}
	return 0;
}

