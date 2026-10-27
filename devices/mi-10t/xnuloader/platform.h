/*
 * The Xiaomi Mi 10T (apollo, Snapdragon 865 / SM8250) for xnuloader/loader.c, which includes this
 * after its own descriptions (struct platform is there).
 *
 * The SoC window, the GICv3 and the core numbering are the same as the POCO X5 Pro's: GIC distributor
 * 0x17A00000 and redistributors 0x17A60000 (the device tree's interrupt-controller@17a00000), eight
 * cores numbered in MPIDR's second affinity field. Cores 0-3 are Cortex-A55s ("small"), 4-6 Cortex-A77s,
 * 7 the fastest Cortex-A77.
 *
 * The phone is recognised by its screen (efi_main): its UEFI's GOP framebuffer is allocated at boot, so
 * there is no fixed base to match as on the POCO, and a 1080x2400 mode that is not the POCO's base is
 * taken to be this phone until its base is known.
 */
static const struct platform platform_apollo = {
	.name = "apollo", .io_base = 0x00400000, .io_size = 0x3FC00000,
	.gic_dist = 0x17A00000, .gic_redist = 0x17A60000, .uart = 0, .log_other_cpu = 1,
	.cores = 8, .core_shift = 8,
	.small_cores = 0x0f,
};

#define APOLLO_SCREEN_WIDTH  1080
#define APOLLO_SCREEN_HEIGHT 2400
