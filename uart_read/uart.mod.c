#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
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
	{ 0xae141548, "module_layout" },
	{ 0x2da418b5, "copy_to_user" },
	{ 0x38787e33, "cdev_add" },
	{ 0xf432cc41, "cdev_init" },
	{ 0x29537c9e, "alloc_chrdev_region" },
	{ 0xd8e484f0, "register_chrdev_region" },
	{ 0xadf42bd5, "__request_region" },
	{ 0x9bce482f, "__release_region" },
	{ 0x59d8223a, "ioport_resource" },
	{ 0x7485e15e, "unregister_chrdev_region" },
	{ 0x2152d374, "cdev_del" },
	{ 0xacdeb154, "__tracepoint_module_get" },
	{ 0x280f9f14, "__per_cpu_offset" },
	{ 0x9629486a, "per_cpu__cpu_number" },
	{ 0xabde244e, "module_put" },
	{ 0xb72397d5, "printk" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

