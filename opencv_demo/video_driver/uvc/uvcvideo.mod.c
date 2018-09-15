#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xa2f7d132, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x3356b90b, __VMLINUX_SYMBOL_STR(cpu_tss) },
	{ 0xe9423d3f, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x32064690, __VMLINUX_SYMBOL_STR(v4l2_event_unsubscribe) },
	{ 0x3fe2ccbe, __VMLINUX_SYMBOL_STR(memweight) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0xe6da44a, __VMLINUX_SYMBOL_STR(set_normalized_timespec) },
	{ 0x619cb7dd, __VMLINUX_SYMBOL_STR(simple_read_from_buffer) },
	{ 0x29439599, __VMLINUX_SYMBOL_STR(debugfs_create_dir) },
	{ 0x5e70676f, __VMLINUX_SYMBOL_STR(v4l2_event_queue_fh) },
	{ 0xf9661c36, __VMLINUX_SYMBOL_STR(vb2_mmap) },
	{ 0x376d6471, __VMLINUX_SYMBOL_STR(usb_debug_root) },
	{ 0xbe722a5b, __VMLINUX_SYMBOL_STR(v4l2_device_unregister) },
	{ 0x187e91b5, __VMLINUX_SYMBOL_STR(no_llseek) },
	{ 0xf1c62ff8, __VMLINUX_SYMBOL_STR(vb2_create_bufs) },
	{ 0x495ded38, __VMLINUX_SYMBOL_STR(usb_kill_urb) },
	{ 0x6729d3df, __VMLINUX_SYMBOL_STR(__get_user_4) },
	{ 0x448eac3e, __VMLINUX_SYMBOL_STR(kmemdup) },
	{ 0x9a5d9e04, __VMLINUX_SYMBOL_STR(vb2_ops_wait_prepare) },
	{ 0xfff7655e, __VMLINUX_SYMBOL_STR(__video_register_device) },
	{ 0x84cc4934, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x8531ece9, __VMLINUX_SYMBOL_STR(usb_autopm_get_interface) },
	{ 0xc3e30545, __VMLINUX_SYMBOL_STR(usb_enable_autosuspend) },
	{ 0xf8435c90, __VMLINUX_SYMBOL_STR(debugfs_create_file) },
	{ 0x1b5c0864, __VMLINUX_SYMBOL_STR(v4l2_ctrl_merge) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x36ac2370, __VMLINUX_SYMBOL_STR(debugfs_remove_recursive) },
	{ 0x26948d96, __VMLINUX_SYMBOL_STR(copy_user_enhanced_fast_string) },
	{ 0xe540fd5b, __VMLINUX_SYMBOL_STR(media_entity_cleanup) },
	{ 0xf453c7e1, __VMLINUX_SYMBOL_STR(v4l2_device_register) },
	{ 0xf116d4b5, __VMLINUX_SYMBOL_STR(copy_in_user) },
	{ 0xf97b4e14, __VMLINUX_SYMBOL_STR(input_event) },
	{ 0x4f8b5ddb, __VMLINUX_SYMBOL_STR(_copy_to_user) },
	{ 0xf9c0b663, __VMLINUX_SYMBOL_STR(strlcat) },
	{ 0xfb578fc5, __VMLINUX_SYMBOL_STR(memset) },
	{ 0x25abf3ad, __VMLINUX_SYMBOL_STR(vb2_vmalloc_memops) },
	{ 0x8bc3e1a7, __VMLINUX_SYMBOL_STR(usb_string) },
	{ 0x1916e38c, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_irqrestore) },
	{ 0xfd91fb91, __VMLINUX_SYMBOL_STR(usb_deregister) },
	{ 0xe807b57a, __VMLINUX_SYMBOL_STR(mutex_lock_interruptible) },
	{ 0x4e8ce6c0, __VMLINUX_SYMBOL_STR(v4l2_event_subscribe) },
	{ 0xc277280a, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xb77b0159, __VMLINUX_SYMBOL_STR(v4l2_prio_init) },
	{ 0x449ad0a7, __VMLINUX_SYMBOL_STR(memcmp) },
	{ 0x94d4e866, __VMLINUX_SYMBOL_STR(video_unregister_device) },
	{ 0xafb8c6ff, __VMLINUX_SYMBOL_STR(copy_user_generic_string) },
	{ 0xf8b0e7e3, __VMLINUX_SYMBOL_STR(usb_set_interface) },
	{ 0x2027cec, __VMLINUX_SYMBOL_STR(v4l2_fh_init) },
	{ 0xae96e9aa, __VMLINUX_SYMBOL_STR(vb2_plane_vaddr) },
	{ 0x8371a000, __VMLINUX_SYMBOL_STR(vb2_buffer_done) },
	{ 0xaafdc258, __VMLINUX_SYMBOL_STR(strcasecmp) },
	{ 0xf3341268, __VMLINUX_SYMBOL_STR(__clear_user) },
	{ 0x8460cfc, __VMLINUX_SYMBOL_STR(usb_control_msg) },
	{ 0x5792f848, __VMLINUX_SYMBOL_STR(strlcpy) },
	{ 0x42896f99, __VMLINUX_SYMBOL_STR(usb_driver_claim_interface) },
	{ 0x55701cc3, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0xa8a57028, __VMLINUX_SYMBOL_STR(vb2_qbuf) },
	{ 0xdf656215, __VMLINUX_SYMBOL_STR(usb_free_coherent) },
	{ 0xa9332df6, __VMLINUX_SYMBOL_STR(vb2_querybuf) },
	{ 0xc7fb73ab, __VMLINUX_SYMBOL_STR(media_entity_init) },
	{ 0x72a98fdb, __VMLINUX_SYMBOL_STR(copy_user_generic_unrolled) },
	{ 0xa0ef33de, __VMLINUX_SYMBOL_STR(usb_submit_urb) },
	{ 0x4b77c88d, __VMLINUX_SYMBOL_STR(v4l2_ctrl_replace) },
	{ 0xb19da169, __VMLINUX_SYMBOL_STR(vb2_streamon) },
	{ 0xf2882a90, __VMLINUX_SYMBOL_STR(usb_get_dev) },
	{ 0x20d1ffe9, __VMLINUX_SYMBOL_STR(video_devdata) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0xe9f603fe, __VMLINUX_SYMBOL_STR(vb2_expbuf) },
	{ 0xd533ea5c, __VMLINUX_SYMBOL_STR(input_register_device) },
	{ 0xa1ff976b, __VMLINUX_SYMBOL_STR(usb_put_dev) },
	{ 0x96b29254, __VMLINUX_SYMBOL_STR(strncasecmp) },
	{ 0xcd19f8de, __VMLINUX_SYMBOL_STR(usb_clear_halt) },
	{ 0x54983c6, __VMLINUX_SYMBOL_STR(usb_driver_release_interface) },
	{ 0x9cbce800, __VMLINUX_SYMBOL_STR(input_free_device) },
	{ 0x211f68f1, __VMLINUX_SYMBOL_STR(getnstimeofday64) },
	{ 0x8e63f95b, __VMLINUX_SYMBOL_STR(v4l2_device_register_subdev) },
	{ 0x480de3c9, __VMLINUX_SYMBOL_STR(vb2_reqbufs) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0x85d4c68, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0xfb7538c9, __VMLINUX_SYMBOL_STR(usb_get_intf) },
	{ 0x680ec266, __VMLINUX_SYMBOL_STR(_raw_spin_lock_irqsave) },
	{ 0xad78f338, __VMLINUX_SYMBOL_STR(v4l2_subdev_init) },
	{ 0x809b0871, __VMLINUX_SYMBOL_STR(__media_device_register) },
	{ 0x9161ce3, __VMLINUX_SYMBOL_STR(vb2_dqbuf) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x69acdf38, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0xbd1fb265, __VMLINUX_SYMBOL_STR(input_unregister_device) },
	{ 0xbffde8ec, __VMLINUX_SYMBOL_STR(compat_alloc_user_space) },
	{ 0x29effeaa, __VMLINUX_SYMBOL_STR(usb_match_one_id) },
	{ 0x4ca9669f, __VMLINUX_SYMBOL_STR(scnprintf) },
	{ 0x74b83f6a, __VMLINUX_SYMBOL_STR(usb_register_driver) },
	{ 0x5af2a839, __VMLINUX_SYMBOL_STR(vb2_ops_wait_finish) },
	{ 0x7b49d18b, __VMLINUX_SYMBOL_STR(v4l2_fh_add) },
	{ 0x776916f4, __VMLINUX_SYMBOL_STR(v4l2_fh_del) },
	{ 0xeda4b88d, __VMLINUX_SYMBOL_STR(usb_ifnum_to_if) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0xbe1babf8, __VMLINUX_SYMBOL_STR(vb2_poll) },
	{ 0x3113e264, __VMLINUX_SYMBOL_STR(usb_alloc_coherent) },
	{ 0xf8d75404, __VMLINUX_SYMBOL_STR(usb_get_current_frame_number) },
	{ 0xd56b5f64, __VMLINUX_SYMBOL_STR(ktime_get_ts64) },
	{ 0x720dc772, __VMLINUX_SYMBOL_STR(vb2_queue_release) },
	{ 0x4f6b400b, __VMLINUX_SYMBOL_STR(_copy_from_user) },
	{ 0xd9a7cc00, __VMLINUX_SYMBOL_STR(param_ops_uint) },
	{ 0xb224121f, __VMLINUX_SYMBOL_STR(media_entity_create_link) },
	{ 0xe2cfce4f, __VMLINUX_SYMBOL_STR(vb2_streamoff) },
	{ 0xfd808436, __VMLINUX_SYMBOL_STR(usb_free_urb) },
	{ 0xfa3e9354, __VMLINUX_SYMBOL_STR(media_device_unregister) },
	{ 0xe2207d63, __VMLINUX_SYMBOL_STR(video_ioctl2) },
	{ 0x5e9e4f48, __VMLINUX_SYMBOL_STR(usb_autopm_put_interface) },
	{ 0x77d19e0f, __VMLINUX_SYMBOL_STR(usb_alloc_urb) },
	{ 0xb5add92a, __VMLINUX_SYMBOL_STR(usb_put_intf) },
	{ 0xe914e41e, __VMLINUX_SYMBOL_STR(strcpy) },
	{ 0x8c8149f9, __VMLINUX_SYMBOL_STR(v4l2_fh_exit) },
	{ 0xbe485978, __VMLINUX_SYMBOL_STR(input_allocate_device) },
	{ 0x55efebd1, __VMLINUX_SYMBOL_STR(vb2_queue_init) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=videodev,videobuf2-core,videobuf2-v4l2,media,videobuf2-vmalloc";

MODULE_ALIAS("usb:v0416pA91Ad*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v0458p706Ed*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v045Ep00F8d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v045Ep0721d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v045Ep0723d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v046Dp08C1d*dc*dsc*dp*icFFisc01ip00in*");
MODULE_ALIAS("usb:v046Dp08C2d*dc*dsc*dp*icFFisc01ip00in*");
MODULE_ALIAS("usb:v046Dp08C3d*dc*dsc*dp*icFFisc01ip00in*");
MODULE_ALIAS("usb:v046Dp08C5d*dc*dsc*dp*icFFisc01ip00in*");
MODULE_ALIAS("usb:v046Dp08C6d*dc*dsc*dp*icFFisc01ip00in*");
MODULE_ALIAS("usb:v046Dp08C7d*dc*dsc*dp*icFFisc01ip00in*");
MODULE_ALIAS("usb:v046Dp082Dd*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v04F2pB071d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v058Fp3820d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v05A9p2640d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v05A9p2641d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v05A9p2643d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v05A9p264Ad*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v05A9p7670d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v05ACp8501d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v05C8p0403d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v05E3p0505d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v06F8p300Cd*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v0AC8p332Dd*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v0AC8p3410d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v0AC8p3420d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v0BD3p0555d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v0E8Dp0004d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v13D3p5103d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v152Dp0310d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v174Fp5212d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v174Fp5931d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v174Fp8A12d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v174Fp8A31d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v174Fp8A33d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v174Fp8A34d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v17DCp0202d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v17EFp480Bd*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v1871p0306d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v1871p0516d*dc*dsc*dp*icFFisc01ip00in*");
MODULE_ALIAS("usb:v18CDpCAFEd*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v18ECp3188d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v18ECp3288d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v18ECp3290d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v199Ep8102d*dc*dsc*dp*icFFisc01ip00in*");
MODULE_ALIAS("usb:v19ABp1000d012[0-6]dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v19ABp1000d01[0-1]*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v19ABp1000d00*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v1B3Bp2951d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v1C4Fp3000d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v2833p0201d*dc*dsc*dp*ic0Eisc01ip00in*");
MODULE_ALIAS("usb:v*p*d*dc*dsc*dp*ic0Eisc01ip00in*");

MODULE_INFO(srcversion, "91A65978787405A4C252F60");
