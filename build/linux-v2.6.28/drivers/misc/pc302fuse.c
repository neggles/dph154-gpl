#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/pc302fuse.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <mach/hardware.h>

#define PC302_FUSE_MINOR    ( 242 )

/** The constant programming time when blowing a fuse. */
#define PC302_FUSE_PROGRAM_TIME         ( 20 )

/** The time from raising the programming voltage to delay for before
 * initiating the programming. */
#define PC302_FUSE_VDDQ_HIGH_DEL_DFL    ( 10 )

/** The time from stopping programming until VDDQ goes low. This is only used
 * in estimating the VDDQ time. */
#define PC302_FUSE_VDDQ_LOW_DEL_DFL     ( 10 )

#define CARDNAME    "pc302-fuse"

static int test_mode;
module_param( test_mode, bool, S_IRUSR );

static unsigned vddq_high_delay = PC302_FUSE_VDDQ_HIGH_DEL_DFL;
module_param( vddq_high_delay, uint, S_IRUSR );

static unsigned vddq_low_delay = PC302_FUSE_VDDQ_LOW_DEL_DFL;
module_param( vddq_low_delay, uint, S_IRUSR );

static int pc302fuse_open( struct inode *inode,
                           struct file *filp );

static int pc302fuse_release( struct inode *inode,
                              struct file *filp );

static int pc302fuse_ioctl( struct inode *inode,
                            struct file *filp,
                            unsigned int cmd,
                            unsigned long arg );

static int pc302fuse_get_vddq( u32 *t );

/**
 * The file operations for the fuse driver.
 */
static const struct file_operations pc302fuse_fops = {
    .open = pc302fuse_open,
    .release = pc302fuse_release,
    .ioctl = pc302fuse_ioctl,
};

/**
 * Operation modes for the driver.
 */
typedef enum
{
    PC302_FUSE_OP_MODE_NORMAL = 0,  /*< Normal operation mode on hardware. */
    PC302_FUSE_OP_MODE_TEST,        /*< Software test mode. */

} pc302fuse_op_mode;

static struct 
{
    /**
     * The current operation mode. Used to select between real fuse
     * operations and a test mode for debugging.
     */
    pc302fuse_op_mode op_mode;

    /**
     * The character device used for ioctl() calls to read and blow fuses.
     */
    struct miscdevice dev;

    /**
     * The platform device that this driver interfaces to.
     */
    struct platform_device *pdev;

    /**
     * The platform driver that this device interfaces to.
     */
    struct platform_driver *pdrv;

    /**
     * Boolean flag for signalling the completion of device registration.
     */
    int registered;

    /**
     * The memory that has been mapped for the fuses. This is a union to allow
     * using real hardware memory or a software buffer.
     */
    union
    {
        /**
         * The hardware memory of the fuse shadow map. This field is only used
         * if op_mode is set to PC302_FUSE_OP_MODE_NORMAL.
         */
        void __iomem *hw;

        /**
         * The software memory that is used for testing the fuse driver. This
         * field is only used if op_mode is set to PC302_FUSE_OP_MODE_TEST.
         */
        void *sw;

    } mem_region;

    spinlock_t hw_lock;

} pc302fuse_int = {

    .op_mode = PC302_FUSE_OP_MODE_TEST,

    .dev = {
        .minor = PC302_FUSE_MINOR,
        .name = "picofuse",
        .fops = &pc302fuse_fops,
    },

    .pdev = NULL,

    .pdrv = NULL,

    .registered = 0,

    .mem_region.sw = NULL,

    .hw_lock = __SPIN_LOCK_UNLOCKED( pc302fuse_hw_lock ),
};

/**
 * Read the value of a fuse.
 *
 * \param indx The fuse number to read from.
 * \return Returns 1 if blown, 0 if not blown, negative on error.
 */
static int pc302fuse_read_fuse( unsigned indx );

/**
 * Blow a fuse.
 *
 * \param indx The index of the fuse to blow.
 * \return Returns 0 on success, non-zero on failure.
 */
static int pc302fuse_blow( unsigned indx );

static int
pc302fuse_read_byte( int indx )
{
    u8 val;

    if ( test_mode )
        val = ( ( u8 * )pc302fuse_int.mem_region.sw )[ indx / 8 ];
    else
        val = ioread8( pc302fuse_int.mem_region.hw + ( indx / 8 ) );

    return val;
}

static ssize_t
pc302fuse_print_range( char *buf, struct fuse_range_t range )
{
    int num_bytes = ( ( range.end - range.start ) + 1 ) / 8;
    char *pos = buf;
    int i;
    int val;

    WARN_ON( ( ( range.end - range.start ) + 1 ) % 8 );
    WARN_ON( range.start % 8 );

    for ( i = 0; i < num_bytes; ++i )
    {
        val = pc302fuse_read_byte( range.start + ( i * 8 ) );
        sprintf( pos, "%02x", val );
        pos += 2;
    }

    sprintf( pos, "\n" );
    ++pos;

    return pos - buf;
}

static ssize_t
pc302fuse_sysfs_fb_show( struct device *dev,
                               struct device_attribute *attr,
                               char *buf )
{
    const char *name = attr->attr.name;
    int val;

    if ( !strcmp( name, "secure_boot" ) )
        val = pc302fuse_read_fuse( PC302_FB_SECURE_BOOT );
    else if ( !strcmp( name, "disable_trustzone" ) )
        val = pc302fuse_read_fuse( PC302_FB_DISABLE_TRUSTZONE );
    else if ( !strcmp( name, "global_last_time_program" ) )
        val = pc302fuse_read_fuse( PC302_FB_GLOBAL_LAST_TIME_PROGRAM );
    else if ( !strcmp( name, "disable_debug" ) )
        val = pc302fuse_read_fuse( PC302_FB_DISABLE_DEBUG );
    else if ( !strcmp( name, "disable_isc" ) )
        val = pc302fuse_read_fuse( PC302_FB_DISABLE_ISC );
    else if ( !strcmp( name, "disable_jtag" ) )
        val = pc302fuse_read_fuse( PC302_FB_DISABLE_JTAG );
    else if ( !strcmp( name, "disable_invasive_debug_in_secure" ) )
        val = pc302fuse_read_fuse( PC302_FB_DISABLE_INVASIVE_DEBUG_IN_SECURE );
    else if ( !strcmp( name, "disable_non_invasive_debug_in_secure" ) )
        val = pc302fuse_read_fuse(
                PC302_FB_DISABLE_NON_INVASIVE_DEBUG_IN_SECURE );
    else if ( !strcmp( name, "disable_cp15_register" ) )
        val = pc302fuse_read_fuse( PC302_FB_DISABLE_CP15_REGISTER );
    else if ( !strcmp( name, "disable_memif_arm" ) )
        val = pc302fuse_read_fuse( PC302_FB_DUAL_SINGLE_MEMIF );
    else if ( !strcmp( name, "disable_non_secure_parallel_flash" ) )
        val = pc302fuse_read_fuse( PC302_FB_DISABLE_NON_SECURE_PARALLEL_FLASH );
    else if ( !strcmp( name, "key0_read_once_per_boot" ) )
        val = pc302fuse_read_fuse( PC302_FB_SECURE_BOOTSTRAP_ROPB );
    else if ( !strcmp( name, "key0_last_time_program" ) )
        val = pc302fuse_read_fuse( PC302_FB_SECURE_BOOTSTRAP_LTP );
    else if ( !strcmp( name, "key0_disable_jtag" ) )
        val = pc302fuse_read_fuse( PC302_FB_SECURE_BOOTSTRAP_DJRK );
    else if ( !strcmp( name, "counter_iv_read_once_per_boot" ) )
        val = pc302fuse_read_fuse( PC302_FB_COUNTER_IV_ROPB );
    else if ( !strcmp( name, "counter_iv_last_time_program" ) )
        val = pc302fuse_read_fuse( PC302_FB_COUNTER_IV_LTP );
    else if ( !strcmp( name, "counter_iv_disable_jtag" ) )
        val = pc302fuse_read_fuse( PC302_FB_COUNTER_IV_DJRK );
    else if ( !strcmp( name, "key2_read_once_per_boot" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY2_ROPB );
    else if ( !strcmp( name, "key2_last_time_program" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY2_LTP );
    else if ( !strcmp( name, "key2_disable_jtag" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY2_DJRK );
    else if ( !strcmp( name, "key3_read_once_per_boot" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY3_ROPB );
    else if ( !strcmp( name, "key3_last_time_program" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY3_LTP );
    else if ( !strcmp( name, "key3_disable_jtag" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY3_DJRK );
    else if ( !strcmp( name, "key4_read_once_per_boot" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY4_ROPB );
    else if ( !strcmp( name, "key4_last_time_program" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY4_LTP );
    else if ( !strcmp( name, "key4_disable_jtag" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY4_DJRK );
    else if ( !strcmp( name, "key5_read_once_per_boot" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY5_ROPB );
    else if ( !strcmp( name, "key5_last_time_program" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY5_LTP );
    else if ( !strcmp( name, "key5_disable_jtag" ) )
        val = pc302fuse_read_fuse( PC302_FB_KEY5_DJRK );
    else if ( !strcmp( name, "die_ident_read_once_per_boot" ) )
        val = pc302fuse_read_fuse( PC302_FB_DIE_IDENT_ROPB );
    else if ( !strcmp( name, "die_ident_last_time_program" ) )
        val = pc302fuse_read_fuse( PC302_FB_DIE_IDENT_LTP );
    else if ( !strcmp( name, "die_ident_disable_jtag" ) )
        val = pc302fuse_read_fuse( PC302_FB_DIE_IDENT_DJRK );
    else
        val = -1;

    return sprintf( buf, "%d\n", val );
}

static ssize_t
pc302fuse_sysfs_fr_show( struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf )
{
    const char *name = attr->attr.name;
    int val;
    int ret = 0;

    if ( !strcmp( name, "key0" ) )
        ret = pc302fuse_print_range( buf, PC302_FR_SECURE_BOOTSTRAP );
    if ( !strcmp( name, "key2" ) )
        ret = pc302fuse_print_range( buf, PC302_FR_KEY2 );
    if ( !strcmp( name, "key3" ) )
        ret = pc302fuse_print_range( buf, PC302_FR_KEY3 );
    if ( !strcmp( name, "key4" ) )
        ret = pc302fuse_print_range( buf, PC302_FR_KEY4 );
    if ( !strcmp( name, "key5" ) )
        ret = pc302fuse_print_range( buf, PC302_FR_KEY5 );
    if ( !strcmp( name, "die_ident" ) )
        ret = pc302fuse_print_range( buf, PC302_FR_DIE_IDENT );
    if ( !strcmp( name, "counter_iv" ) )
        ret = pc302fuse_print_range( buf, PC302_FR_COUNTER_IV );
    else
        val = -1;

    return ret;
}

static DEVICE_ATTR( secure_boot, S_IRUSR,pc302fuse_sysfs_fb_show, NULL );
static DEVICE_ATTR( disable_trustzone, S_IRUSR,pc302fuse_sysfs_fb_show, NULL );
static DEVICE_ATTR( global_last_time_program, S_IRUSR,
                    pc302fuse_sysfs_fb_show, NULL );
static DEVICE_ATTR( disable_debug, S_IRUSR,pc302fuse_sysfs_fb_show, NULL );
static DEVICE_ATTR( disable_isc, S_IRUSR,pc302fuse_sysfs_fb_show, NULL );
static DEVICE_ATTR( disable_jtag, S_IRUSR,pc302fuse_sysfs_fb_show, NULL );
static DEVICE_ATTR( disable_invasive_debug_in_secure, S_IRUSR,
                    pc302fuse_sysfs_fb_show, NULL );
static DEVICE_ATTR( disable_non_invasive_debug_in_secure, S_IRUSR,
                    pc302fuse_sysfs_fb_show, NULL );
static DEVICE_ATTR( disable_cp15_register, S_IRUSR,
                    pc302fuse_sysfs_fb_show, NULL );
static DEVICE_ATTR( disable_memif_arm, S_IRUSR,pc302fuse_sysfs_fb_show,
                    NULL );
static DEVICE_ATTR( disable_non_secure_parallel_flash, S_IRUSR,
                    pc302fuse_sysfs_fb_show, NULL );

#define PC302_FUSE_SYSFS_PARTITION( _fuse_name ) \
    static DEVICE_ATTR( _fuse_name, S_IRUSR, pc302fuse_sysfs_fr_show, NULL ); \
    static DEVICE_ATTR( _fuse_name##_read_once_per_boot, S_IRUSR, \
                        pc302fuse_sysfs_fb_show, NULL ); \
    static DEVICE_ATTR( _fuse_name##_last_time_program, S_IRUSR, \
                        pc302fuse_sysfs_fb_show, NULL ); \
    static DEVICE_ATTR( _fuse_name##_disable_jtag, S_IRUSR, \
                        pc302fuse_sysfs_fb_show, NULL )

PC302_FUSE_SYSFS_PARTITION( die_ident );
PC302_FUSE_SYSFS_PARTITION( key0 );
PC302_FUSE_SYSFS_PARTITION( counter_iv );
PC302_FUSE_SYSFS_PARTITION( key2 );
PC302_FUSE_SYSFS_PARTITION( key3 );
PC302_FUSE_SYSFS_PARTITION( key4 );
PC302_FUSE_SYSFS_PARTITION( key5 );

#define PC302_FUSE_SYSFS_PARTITION_ATTRIBUTES( _partition ) \
    &dev_attr_##_partition.attr, \
    &dev_attr_##_partition##_read_once_per_boot.attr, \
    &dev_attr_##_partition##_last_time_program.attr, \
    &dev_attr_##_partition##_disable_jtag.attr

static struct attribute *pc302fuse_device_attrs[] = {
    &dev_attr_secure_boot.attr,
    &dev_attr_disable_trustzone.attr,
    &dev_attr_global_last_time_program.attr,
    &dev_attr_disable_debug.attr,
    &dev_attr_disable_isc.attr,
    &dev_attr_disable_jtag.attr,
    &dev_attr_disable_invasive_debug_in_secure.attr,
    &dev_attr_disable_non_invasive_debug_in_secure.attr,
    &dev_attr_disable_cp15_register.attr,
    &dev_attr_disable_memif_arm.attr,
    &dev_attr_disable_non_secure_parallel_flash.attr,
    PC302_FUSE_SYSFS_PARTITION_ATTRIBUTES( counter_iv ),
    PC302_FUSE_SYSFS_PARTITION_ATTRIBUTES( key0 ),
    PC302_FUSE_SYSFS_PARTITION_ATTRIBUTES( key2 ),
    PC302_FUSE_SYSFS_PARTITION_ATTRIBUTES( key3 ),
    PC302_FUSE_SYSFS_PARTITION_ATTRIBUTES( key4 ),
    PC302_FUSE_SYSFS_PARTITION_ATTRIBUTES( key5 ),
    PC302_FUSE_SYSFS_PARTITION_ATTRIBUTES( die_ident ),
    NULL,
};

static struct attribute_group pc302fuse_device_attr_group = {
    .name = "fuses",
    .attrs = pc302fuse_device_attrs,
};

static ssize_t
pc302fuse_sysfs_test_mode_show( struct device_driver *dev,
                                char *buf )
{
    return sprintf( buf, "%d\n",
                    pc302fuse_int.op_mode == PC302_FUSE_OP_MODE_TEST ? 1 : 0 );
}

static DRIVER_ATTR( test_mode, S_IRUSR, pc302fuse_sysfs_test_mode_show, NULL );

static int
pc302fuse_sysfs_add( void )
{
    int ret;

    /* Create the driver attributes. */
    ret = driver_create_file( &pc302fuse_int.pdrv->driver,
                              &driver_attr_test_mode );
    /* Create the device attributes. */
    ret = sysfs_create_group( &pc302fuse_int.pdev->dev.kobj,
                              &pc302fuse_device_attr_group );

    return ret;
}

static void
pc302fuse_sysfs_del( void )
{
    driver_remove_file( &pc302fuse_int.pdrv->driver,
                        &driver_attr_test_mode );
    sysfs_remove_group( &pc302fuse_int.pdev->dev.kobj,
                        &pc302fuse_device_attr_group );
}

static int
pc302fuse_open( struct inode *inode,
                struct file *filp )
{
    return 0;
}

static int
pc302fuse_release( struct inode *inode,
                   struct file *filp )
{
    return 0;
}

static int
pc302fuse_get_vddq( u32 *t )
{
    unsigned i;
    int ret;
    unsigned num_blown = 0;

    for ( i = 0; i < PC302_FUSE_NUM_FUSES; ++i )
    {
        ret = pc302fuse_read_fuse( i );
        if ( ret < 0 )
            break;

        if ( 1 == ret )
            ++num_blown;
    }

    *t = num_blown * ( PC302_FUSE_PROGRAM_TIME + vddq_low_delay + 
                       vddq_high_delay );

    return ( ret >= 0 ) ? 0 : -1;
}

static int
pc302fuse_ioctl( struct inode *inode,
                 struct file *filp,
                 unsigned int cmd,
                 unsigned long arg )
{
    pcfuse_t fuse_struct;
    u32 vddq;
    int ret;

    if ( _IOC_TYPE( cmd ) != PC302_FUSE_IOCTL_BASE )
    {
        printk( KERN_DEBUG "pc302fuse: invalid command type\n" );
        return -ENOTTY;
    }

    if ( _IOC_NR( cmd ) >
            ( PC302_FUSE_IOCTL_START + PC302_FUSE_IOCTL_NUM_IOCTL ) ||
         _IOC_NR( cmd ) < ( PC302_FUSE_IOCTL_START ) )
    {
        printk( KERN_DEBUG "pc302fuse: invalid command\n" );
        return -ENOTTY;
    }

    ret = copy_from_user( &fuse_struct, ( void __user * )arg,
                          sizeof( fuse_struct ) );

    if ( ret )
    {
        printk( KERN_DEBUG "pc302fuse: failed to copy structure\n" );
        return -EFAULT;
    }

    switch ( cmd )
    {
        case PC302_FUSE_GET:
            ret = pc302fuse_read_fuse( fuse_struct.offset );
            fuse_struct.value = ret;
            /* If the read was successful, copy the result. */
            if ( ret >= 0 )
            {
                ret = copy_to_user( ( void __user * )arg, &fuse_struct,
                                    sizeof( fuse_struct ) );
            }
            break;

        case PC302_FUSE_BLOW:
            ret = pc302fuse_blow( fuse_struct.offset );
            if ( 0 == ret )
            {
                fuse_struct.value = 1;
                ret = copy_to_user( ( void __user * )arg, &fuse_struct,
                                    sizeof( fuse_struct ) );
            }
            break;

        case PC302_FUSE_GET_VDDQ:
            ret = pc302fuse_get_vddq( &vddq );
            if ( 0 == ret )
            {
                fuse_struct.value = vddq;
                ret = copy_to_user( ( void __user * )arg, &fuse_struct,
                                    sizeof( fuse_struct ) );
            }
            ret = 0;
            break;

        default:
            printk( KERN_DEBUG "pc302fuse: invalid ioctl(), cmd=%d\n", cmd );
            ret = -EINVAL;
            break;
    }

    return ret;
}

static int
pc302fuse_read_fuse_tm( unsigned indx )
{
    unsigned byte_off = indx / 8;
    unsigned bit_off = indx % 8;
    u8 val;

    val = ( ( u8 * )pc302fuse_int.mem_region.sw )[ byte_off ];

    return !!( val & ( 1 << bit_off ) );
}

static int
pc302fuse_read_fuse_hw( unsigned indx )
{
    unsigned byte_off = indx / 8;
    unsigned bit_off = indx % 8;
    u8 byte = ioread8( pc302fuse_int.mem_region.hw + byte_off );

    return !!( byte & ( 1 << bit_off ) );
}

/** Check if a fuse is in a segment. */
#define PC302_FUSE_IS_IN_SEGMENT( _seg, _indx ) \
    (( _indx >= PC302_FR_##_seg.start && _indx <= PC302_FR_##_seg.end ))

/** Check if a fuse in a segment can be blown. */
#define PC302_SEGMENT_CAN_BLOW( _seg ) \
    (( 0 == pc302fuse_read_fuse( PC302_FB_##_seg##_ROPB ) && \
       0 == pc302fuse_read_fuse( PC302_FB_##_seg##_LTP ) ))

static int
pc302fuse_can_blow( unsigned indx )
{
    int ret = 0;

    /* If the global last time program fuse is blown, we certainly can't blow
     * another fuse. */
    if ( 1 == pc302fuse_read_fuse( PC302_FB_GLOBAL_LAST_TIME_PROGRAM) )
        return 0;

    /* Check each segment to see if we are inside it and check the read once
     * per boot and last time program bits for that segment. */

    if ( PC302_FUSE_IS_IN_SEGMENT( SECURE_BOOTSTRAP, indx ) )
        ret = PC302_SEGMENT_CAN_BLOW( SECURE_BOOTSTRAP );
    else if ( PC302_FUSE_IS_IN_SEGMENT( COUNTER_IV, indx ) )
        ret = PC302_SEGMENT_CAN_BLOW( COUNTER_IV );
    else if ( PC302_FUSE_IS_IN_SEGMENT( KEY2, indx ) )
        ret = PC302_SEGMENT_CAN_BLOW( KEY2 );
    else if ( PC302_FUSE_IS_IN_SEGMENT( KEY3, indx ) )
        ret = PC302_SEGMENT_CAN_BLOW( KEY3 );
    else if ( PC302_FUSE_IS_IN_SEGMENT( KEY4, indx ) )
        ret = PC302_SEGMENT_CAN_BLOW( KEY4 );
    else if ( PC302_FUSE_IS_IN_SEGMENT( KEY5, indx ) )
        ret = PC302_SEGMENT_CAN_BLOW( KEY5 );
    else if ( PC302_FUSE_IS_IN_SEGMENT( DIE_IDENT, indx ) )
        ret = PC302_SEGMENT_CAN_BLOW( DIE_IDENT );
    else if ( PC302_FUSE_IS_IN_SEGMENT( PARTITION_1, indx ) )
        ret = PC302_SEGMENT_CAN_BLOW( PARTITION_1 );
    else if ( PC302_FUSE_IS_IN_SEGMENT( PARTITION_2, indx ) )
        ret = PC302_SEGMENT_CAN_BLOW( PARTITION_2 );
    else if ( PC302_FUSE_IS_IN_SEGMENT( PARTITION_3, indx ) )
        ret = PC302_SEGMENT_CAN_BLOW( PARTITION_3 );
    else
        ret = -EIO;

    return ret;
}

static int
pc302fuse_blow_tm( unsigned indx )
{
    unsigned byte_off = indx / 8;
    unsigned bit_off = indx % 8;

    /* Check that the fuse is in range. */
    if ( indx >= PC302_FUSE_NUM_FUSES )
        return -EFAULT;

    if ( !pc302fuse_can_blow( indx ) )
    {
        printk( KERN_INFO "pc302fuse: cannot blow fuse %d\n", indx );
        return -EIO;
    }

    ( ( u8 * )pc302fuse_int.mem_region.sw )[ byte_off ] |= ( 1 << bit_off );

    return 0;
}

static int
pc302fuse_blow_hw( unsigned indx )
{
    unsigned long flags;
    u32 ctrl_reg;

    /* Check that the fuse is in range. */
    if ( indx >= PC302_FUSE_NUM_FUSES )
        return -EFAULT;

    if ( !pc302fuse_can_blow( indx ) )
    {
        printk( KERN_INFO "pc302fuse: cannot blow fuse %d\n", indx );
        return -EIO;
    }

    spin_lock_irqsave( &pc302fuse_int.hw_lock, flags );

    /* Make sure that the fuse process is not already active. */
    do
    {
        ctrl_reg =
            ioread32( pc302fuse_int.mem_region.hw + PC302_FUSE_CTRL_REG );
    } while ( ctrl_reg & PC302_FUSE_CTRL_WRITE_BUSY );

    iowrite32( indx,
               pc302fuse_int.mem_region.hw + PC302_FUSE_WR_BIT_ADDRESS_REG );

    /* Set VDDQ high. */
    iowrite32( PC302_FUSE_WRITE_PAD_EN_VALUE,
               pc302fuse_int.mem_region.hw + PC302_FUSE_WRITE_PAD_EN_REG );
    iowrite32( PC302_FUSE_WRITE_PAD_VALUE,
               pc302fuse_int.mem_region.hw + PC302_FUSE_WRITE_PAD_REG );

    /* Wait for vddq_high_delay to allow VDDQ to raise to the required
     * voltage. */
    udelay( vddq_high_delay );

    /* Start the fuse blowing process. */
    iowrite32( PC302_FUSE_WR_PERFORM_VALUE,
               pc302fuse_int.mem_region.hw + PC302_FUSE_WR_PERFORM_REG );

    do
    {
        ctrl_reg =
            ioread32( pc302fuse_int.mem_region.hw + PC302_FUSE_CTRL_REG );
    } while ( ctrl_reg & PC302_FUSE_CTRL_WRITE_BUSY );

    /* Set VDDQ low. */
    iowrite32( 0, pc302fuse_int.mem_region.hw + PC302_FUSE_WRITE_PAD_REG );
    iowrite32( 0, pc302fuse_int.mem_region.hw + PC302_FUSE_WRITE_PAD_EN_REG );

    spin_unlock_irqrestore( &pc302fuse_int.hw_lock, flags );

    return 0;
}

static int
pc302fuse_read_fuse( unsigned indx )
{
    /* Check that the fuse is in range. If the read once per boot bit is set
     * then the value may be invalid, but it may also have been unset in the
     * shadow RAM so we have no way of knowing if we can perform the read so
     * we just do the read anyway. */
    if ( indx >= PC302_FUSE_NUM_FUSES )
        return -EFAULT;

    if ( PC302_FUSE_OP_MODE_TEST == pc302fuse_int.op_mode )
        return pc302fuse_read_fuse_tm( indx );
    else
        return pc302fuse_read_fuse_hw( indx );
}

static int
pc302fuse_blow( unsigned indx )
{
    /* Check that the fuse is in range. If the read once per boot bit is set
     * then the value may be invalid, but it may also have been unset in the
     * shadow RAM so we have no way of knowing if we can perform the write so
     * we just do the write anyway. */
    if ( indx >= PC302_FUSE_NUM_FUSES )
        return -EFAULT;

    /* Check that the fuse has not already been blown. */
    if ( 1 == pc302fuse_read_fuse( indx ) )
    {
        printk( KERN_INFO "pc302fuse: fuse already blown (%d)\n", indx );
        return -EIO;
    }

    if ( PC302_FUSE_OP_MODE_TEST == pc302fuse_int.op_mode )
        return pc302fuse_blow_tm( indx );
    else
        return pc302fuse_blow_hw( indx );
}

static int pc302fuse_probe( struct platform_device *pdev );
static int pc302fuse_remove( struct platform_device *pdev );

static struct platform_driver pc302fuse_driver = {
    .probe = pc302fuse_probe,
    .remove = pc302fuse_remove,
    .driver = {
        .name = CARDNAME,
    },
};

static int
pc302fuse_probe( struct platform_device *pdev )
{
    int ret;
    struct resource *mem = platform_get_resource( pdev, IORESOURCE_MEM, 0 );

    if ( !mem )
        return -EINVAL;

    pc302fuse_int.pdev = pdev;
    pc302fuse_int.pdrv = &pc302fuse_driver;

    pc302fuse_int.op_mode =
        test_mode == 1 ? PC302_FUSE_OP_MODE_TEST : PC302_FUSE_OP_MODE_NORMAL;

    if ( test_mode )
    {
        pc302fuse_int.mem_region.sw =
            kzalloc( PC302_FUSE_NUM_FUSES / 8, GFP_KERNEL );
        if ( !pc302fuse_int.mem_region.sw )
            return -ENOMEM;
    }
    else
    {
        if ( !request_mem_region( mem->start, ( mem->end - mem->start ) + 1,
                                  CARDNAME ) )
            return -EBUSY;
        pc302fuse_int.mem_region.hw =
            ioremap( mem->start, ( mem->end - mem->start ) + 1 );
        if ( !pc302fuse_int.mem_region.hw )
        {
            ret = -EBUSY;
            goto remap_failed;
        }
    }

    ret = misc_register( &pc302fuse_int.dev );

    if ( !ret )
    {
        pc302fuse_int.registered = 1;
        pc302fuse_sysfs_add();
    }
    else
        goto register_failed;

    return ret;

register_failed:
    if ( test_mode )
        kfree( pc302fuse_int.mem_region.sw );

remap_failed:
    if ( !test_mode )
        release_mem_region( mem->start, ( mem->end - mem->start ) + 1 );
    
    printk( KERN_ERR "PC302 Fuseblock driver registration failed\n" );
    
    return ret;
}

static int
pc302fuse_remove( struct platform_device *pdev )
{
    struct resource *mem = platform_get_resource( pdev, IORESOURCE_MEM, 0 );

    if ( pc302fuse_int.registered )
    {
        misc_deregister( &pc302fuse_int.dev );
        pc302fuse_sysfs_del();
    }

    if ( PC302_FUSE_OP_MODE_TEST == pc302fuse_int.op_mode )
    {
        if ( pc302fuse_int.mem_region.sw )
            kfree( pc302fuse_int.mem_region.sw );
    }
    else
    {
        if ( pc302fuse_int.mem_region.hw )
        {
            iounmap( pc302fuse_int.mem_region.hw );
            release_mem_region( mem->start, ( mem->end - mem->start ) + 1 );
        }
    }

    return 0;
}

static int
pc302fuse_init( void )
{
    int ret;

    /* Check the vddq_high_delay and vddq_low_delay values are sensible. If
     * we can't program all of the fuses in the maximum 1000000 uS, then don't
     * make things worse! */
    BUG_ON( PC302_FUSE_NUM_FUSES * 
                ( vddq_high_delay + vddq_low_delay +
                  PC302_FUSE_PROGRAM_TIME ) > 1000000 );

    ret = platform_driver_register( &pc302fuse_driver );

    return ret;
}

static void
pc302fuse_exit( void )
{
    platform_driver_unregister( &pc302fuse_driver );
}

module_init( pc302fuse_init );
module_exit( pc302fuse_exit );

MODULE_AUTHOR( "Jamie Iles" );
MODULE_DESCRIPTION( "PC302 Fuse Block Driver" );
MODULE_LICENSE( "GPL" );
