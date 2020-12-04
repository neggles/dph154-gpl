/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*
 * Copyright (c) 2009 picoChip Designs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All enquiries to support@picochip.com
 */

/*!
 * \file picoarray.h
 * \brief picoArray device and resource definition.
 *
 * This file defines a base class for a picoArray and a resource type used for
 * accessing picoArray resources. This base class should not be instantiated
 * directly but inherited from to create specific device types (using the
 * container_of()) macro.
 */

/*!
 * \page paAbstraction picoArray Class Hierarchy
 *
 * \section introduction Introduction
 *
 * Each type of picoArray is implemented as a class derived from the picoarray
 * base class and must contain the methods in picoarray_ops. The picoarray
 * structure consists of 3 main components:
 *  \li Resources
 *  \li Operations
 *  \li Common members
 *
 * The resources describe the DMA channels, GPRs and interrupt sources that
 * the picoArray has. Users should never directly access these resources -
 * they should use the get_resource and put_resource methods of the
 * picoarray_ops to get and release the resources. Using these resources,
 * upper layers such as transport methods can hold exclusive access to a
 * resource and prevent conflicts between transport instances and methods and
 * also prevent other user applications from disrupting a transport service.
 * Consider the HwIf transport as an example. There are several GPRs used for
 * signalling the amount of data available to transfer, the amount of data the
 * host processor has queued for transfer and for clearing interrupts. If a
 * user application did a register write to the count GPR, then the reference
 * design would lose synchronisation with the application and data
 * loss/corruption could occur. By adding these resources and locks, if the
 * transport needs exclusive access, then this is guaranteed by the resources.
 *
 * The operations define a set of primitive operations that effectively export
 * the basic services that the picoArray can provide. For some device types,
 * some services may be unavailable in which case the operations should still
 * be implemented but always return an error. These services include
 * primitives such as configuration bus access, GPR access, IRQ registration
 * and DMA to/from device methods. As an example use of these services, a DMA
 * transport module would be responsible for managing the data that needs to
 * be transferred to/from the picoArray and when a transfer needs to take
 * place, the module will call the dma_to_device() or dma_from_device()
 * operation in the relevant picoArray.
 *
 * The common members include the logical device number, spin locks for
 * concurrent access and pointers to the resources and operations. These
 * common members should contain no device specific information.
 *
 * \section new_devices Adding new devices
 *
 * To add a new device type, an implementation file for the device should be
 * created, for example pc202.c. This file contains the derived structure -
 * pc202, and has the picoarray structure embedded in it. The PC202 methods
 * are also defined, and these can get a pc202 pointer from a pointer to the
 * embedded picoarray structure with the container_of() macro:
 *
 * \code
 *  struct picoarray *pa;
 *  struct pc202 *pc202dev = container_of( pa, struct pc202, pa );
 * \endcode
 *
 * This file also needs to define the resources that the device has and must
 * provide init() and destructor functions that are used to register the
 * device with picoIf and remove the driver when picoIf is unloaded.
 */
#ifndef __PICOIF_PICOARRAY_H__
#define __PICOIF_PICOARRAY_H__

#include <linux/types.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <asm/atomic.h>

struct picoarray;

/*!
 * picoArray device types.
 */
enum picoarray_device_type
{
    PICOARRAY_PC202,
    PICOARRAY_PC203,
    PICOARRAY_PC302,
};

/*!
 * \brief Enumeration for picoArray resource types.
 *
 * This defines resource types for picoArray resources that may be requested
 * by functions. Each device type should have a corresponding enumeration for
 * each type that lists all resources the device has.
 */
enum pico_resource_type
{
    PICO_RES_DMA_CHANNEL = 1,  /*!< DMA channel. */
    PICO_RES_GPR,              /*!< General purpose register (GPR). */
    PICO_RES_IRQ,              /*!< IRQ source. */
};

/*!
 * \brief A resource descriptor describing a picoArray resource.
 *
 * This structure defines a resource for a picoArray and should not be
 * accessed directly.
 */
struct pico_resource
{
    enum pico_resource_type type;       /*!< The resource type. */

    unsigned                value;      /*!< The resource ID/value. */

    int                     exclusive;  /*!< Boolean flag to indicate the
                                         *  resource is exclusively held. */

    atomic_t                ref_count;  /*!< The reference count of the
                                         *  resource. */

    unsigned                offset;     /*!< For GPRs and DMA channels, this is
                                         *  the offset. For interrupts this
                                         *  fields usage is dependent on the
                                         *  IRQ type. */

    int                     metadata;   /*!< Metadata for the resource. For GPR
                                         * interrupts, this may be the GPR that
                                         * the interrupt is tied to. */
};

/*!
 * \brief Operations common to all picoArray devices.
 *
 * This structure defines a set of operations that all picoArrays should
 * support. These functions are used for configuration and transport but only
 * provide primitive services. More complicated transports should use these
 * functions to implement a higher level transport module.
 */
struct picoarray_ops
{
    /*!
     * Get the picoArray device type.
     *
     * @return Returns the picoArray device type.
     */
    enum picoarray_device_type ( *get_device_type )( struct picoarray *pa );

    /*!
     * Reset the device.
     *
     * @param pa The device to reset.
     * @return Returns zero on success, negative on failure.
     */
    int ( *reset )( struct picoarray *pa );

    /*!
     * Start the device.
     *
     * @param pa The device to start.
     * @return Returns zero on success, negative on failure.
     */
    int ( *start )( struct picoarray *pa );

    /*!
     * Sync the device.
     *
     * @param pa The device to sync.
     * @return Returns zero on success, negative on failure.
     */
    int ( *sync )( struct picoarray *pa );

    /*!
     * Stop the device.
     *
     * @param pa The device to stop.
     * @return Returns zero on success, negative on failure.
     */
    int ( *stop )( struct picoarray *pa );

    /*!
     * Perform a read from the configuration bus.
     *
     * @param pa The device to read from.
     * @param caeid The CAEID of the AE to read from.
     * @param address The address from within the AE to begin reading from.
     * @param count The number of 16-bit words to read.
     * @param data Pointer to the buffer to write the words read into.
     * @return Returns the number of 16-bit words read on success, negative on
     * failure.
     */
    int ( *config_read )( struct picoarray *pa,
                          u16 caeid,
                          u16 address,
                          u16 count,
                          u16 *data );

    /*!
     * Perform a write to the configuration bus.
     *
     * @param pa The device to write to.
     * @param caeid The CAEID of the AE to write to.
     * @param address The address from within the AE to begin writing from.
     * @param count The number of 16-bit words to write.
     * @param data Pointer to the buffer of words to write.
     * @return Returns the number of 16-bit words written on success, negative
     * on failure.
     */
    int ( *config_write )( struct picoarray *pa,
                           u16 caeid,
                           u16 address,
                           u16 count,
                           u16 *data );

    /*!
     * Perform multiple write to the configuration bus write register.
     *
     * @param pa The device to write to.
     * @param data Pointer to the buffer of words to write.
     * @param sgl The scatter gather list of the source data.
     * @return Returns the number of 32-bit words written on success, negative
     * on failure.
     */
    int ( *pa_load )( struct picoarray *pa,
                      u32 *data,
                      struct scatterlist *sgl );

    /*!
     * DMA a scatter gather list of memory from a kernel mapped scatterlist
     * into a picoArray DMA channel. After the DMA transfer has completed, the
     * callback function will be called with the cookie as the parameter. The
     * caller of this function is responsible for mapping and unmapping the
     * buffers to be transferred into a DMA capable region.
     *
     * @param pa The picoArray to DMA the data.
     * @param dma_channel The DMA channel to use as a destination.
     * @param sgl The scatter gather list of the source data.
     * @param nbytes The number of bytes in the scatter gather list.
     * @param callback The callback function to be called when the transfer
     * has completed. The parameter errno will be set to the status of the DMA
     * operation where 0 == success, negative == failure.
     * @param cookie The cookie to pass to the callback function.
     * @return Returns the number of bytes queued for transfer on success,
     * negative on failure. Note that it is possible that the number of bytes
     * queued may not be the same as nbytes if the device does not support
     * scatter gather operations.
     */
    int ( *dma_to_device )( struct picoarray *pa,
                            struct pico_resource *dma_channel,
                            struct scatterlist *sgl,
                            size_t nbytes,
                            int ( *callback )( void *cookie,
                                               int errno ),
                            void *cookie );

    /*!
     * DMA a scatter gather list of memory from a picoArray DMA channel. After
     * the DMA transfer has completed, the callback function will be called
     * with the cookie as the parameter. The caller of this function is
     * responsible for mapping and unmapping the buffers to be transferred
     * into a DMA capable region.
     *
     * @param pa The picoArray to DMA the data.
     * @param dma_channel The DMA channel to use as a source.
     * @param sgl The scatter gather list of the destination buffer.
     * @param nbytes The number of bytes to transfer.
     * @param callback The callback function to be called when the transfer
     * has completed. The parameter errno will be set to the status of the DMA
     * operation where 0 == success, negative == failure.
     * @param cookie The cookie to pass to the callback function.
     * @return Returns the number of bytes queued for transfer on success,
     * negative on failure. Note that it is possible that the number of bytes
     * queued may not be the same as nbytes if the device does not support
     * scatter gather operations.
     */
    int ( *dma_from_device )( struct picoarray *pa,
                              struct pico_resource *dma_channel,
                              struct scatterlist *sgl,
                              size_t nbytes,
                              int ( *callback )( void *cookie,
                                                 int errno ),
                              void *cookie );

    /*!
     * Open and enables a DMA channel
     *
     * @param pa The picoArray to DMA the data.
     * @param dma_channel The DMA channel to use as a source.
     * @param is_downlink 1 if the DMA channel is a downlink, 0 otherwise.
     * @return Returns 0 on success, negative on failure.
     */
    int ( *dma_open )( struct picoarray *pa,
                       struct pico_resource *dma_channel,
                       int is_downlink );

    /*!
     * Closes and disables a DMA channel
     *
     * @param pa The picoArray to DMA the data.
     * @param dma_channel The DMA channel to use as a source.
     * @return Returns 0 on success, negative on failure.
     */
    int ( *dma_close )( struct picoarray *pa,
                        struct pico_resource *dma_channel );

    /*!
     * Read the value of a general purpose register (GPR). The register is
     * referenced by the resource which must be retrieved with get_resource.
     * If the register is being used exclusively by a transport and the
     * get_resource fails, the register cannot be read.
     *
     * @param pa The device to read from.
     * @param reg The register to read.
     * @param value Pointer to the address the value should be stored in.
     * @return Returns the resource on success, NULL on failure.
     */
     int ( *register_read )( struct picoarray *pa,
                             struct pico_resource *reg,
                             u32 *value );

    /*!
     * Write the value of a general purpose register (GPR). The register is
     * referenced by the resource which must be retrieved with get_resource.
     * If the register is being used exclusively by a transport and the
     * get_resource fails, the register cannot be read.
     *
     * @param pa The device to write to.
     * @param reg The register to write.
     * @param value The value of the register to write.
     * @return Returns zero on success, negative on failure.
     */
    int ( *register_write )( struct picoarray *pa,
                             struct pico_resource *reg,
                             u32 value );

    /*!
     * Add a new IRQ handler. There must only be 1 IRQ handler for each
     * interrupt source so the resource must have the exclusive flag set. The
     * callback will be called when the interrupt is raised and must the
     * interrupt must be cleared by the callback. The callback will be called
     * in interrupt context and so should run as quickly as possible and may
     * not sleep. If more processing is required, this should be deferred to a
     * tasklet or work queue.
     *
     * @param pa The device to register the handler for.
     * @param irq The IRQ source to handle.
     * @param callback The callback function to call when the IRQ is raised.
     * @param cookie A cookie to pass to the callback function.
     * @return Returns zero on success, negative on failure.
     */
    int ( *add_irq_handler )( struct picoarray *pa,
                              struct pico_resource *irq,
                              int ( *callback )( struct pico_resource *irq,
                                                 void *cookie ),
                              void *cookie );

    /*!
     * Remove an interrupt handler from the device.
     *
     * @param pa The device to remove the interrupt handler from.
     * @param irq The interrupt source to remove the handler for.
     */
    void ( *remove_irq_handler )( struct picoarray *pa,
                                  struct pico_resource *irq );

    /*!
     * Get a resource from the device. This will increment the reference count
     * of the resource, and if it exclusive access is requested, the exclusive
     * flag will be set and further attempts to get the resource will fail
     * until the reference count has been decremented back to zero.
     *
     * @param pa The device to get the resource from.
     * @param resource_type The type of resource being requested.
     * @param resource_id The ID of the resource being requested.
     * @param exclusive Boolean flag indicating whether exclusive access is
     * required.
     * @return Returns a pointer to the resource on success, NULL on failure.
     */
    struct pico_resource * ( *get_resource )( struct picoarray *pa,
                                        enum pico_resource_type resource_type,
                                        unsigned resource_id,
                                        int exclusive );

    /*!
     * Decrement the reference count of a resource. If the resource is held
     * exclusively and the reference count is zero after decrementing, it will
     * become available to other users.
     *
     * @param pa The device the resource belongs to.
     * @param resource The resource to put.
     */
    void ( *put_resource )( struct picoarray *pa,
                            struct pico_resource *resource );

    /*!
     * Destructor for the device. This function is called when the module
     * releases the devices and should be used for any cleanup needed.
     *
     * @param pa The device to destroy.
     */
    void ( *destructor )( struct picoarray *pa );

    /*!
     * Get the ITM, ITS and procIF IRQ resources.
     * This function will return the pointers to the resources requested on success
     * or NULL if the resource is already allocated.
     *
     * @param pa The device to take to DMA the data.
     * @param its The ITS resource or NULL on failure
     * @param itm The ITM resource or NULL on failure
     * @param procif_irq The IRQ resource or NULL on failure
     *
     * @return 0 on success, non-zero on failure
     */
    int ( *get_procif_resource )( struct picoarray *pa,
                                  struct pico_resource **its,
                                  struct pico_resource **itm,
                                  struct pico_resource **procif_irq );
    
    /*!
     * Put the ITM, ITS and procIF IRQ resources.
     * This function will free any non NULL resources specified
     *
     * @param pa The device to take to DMA the data.
     * @param its The ITS resource
     * @param itm The ITM resource
     * @param procif_irq The IRQ resource
     */
    void ( *put_procif_resource )( struct picoarray *pa,
                                   struct pico_resource *its,
                                   struct pico_resource *itm,
                                   struct pico_resource *procif_irq );
};

/*!
 * \brief Base class for picoArray structures.
 *
 * This is an abstract base class for picoArray structures and there should
 * never be any standalone instances of this structure - they should be
 * contained in device specific structures to inherit from this using
 * container_of() macros.
 */
struct picoarray
{
    unsigned                dev_num;    /*!< The logical device number of the
                                         *   picoArray. */

    struct picoarray_ops    *ops;       /*!< Methods for the device. */

    struct pico_resource    *resources; /*!< Resources that the device has.
                                         *   This is an array of resources and
                                         *   should be terminated with a 0
                                         *   filled entry. */

    spinlock_t              lock;       /*!< Concurrency lock. This is used
                                         *   for protecting resources and
                                         *   hardware interlocks. */

    unsigned long           features;   /*!< Features bitmap. Used to indicate
                                         *   features that the device supports
                                         */
    unsigned long           max_dma_sz; /*!< Maximum DMA transfer length. */
};

#define PICOARRAY_HAS_DMA_LOAD          ( 1 << 0 )
#define PICOARRAY_HAS_DMA               ( 1 << 1 )

/*!
 * Check if a picoArray has a requested feature.
 *
 * \param pa The picoArray to query.
 * \param feature The feature to test for.
 * \return Returns non-zero if the device has the feature, zero otherwise.
 */
static inline int
picoarray_has_feature( const struct picoarray *pa,
                       unsigned long feature )
{
    return pa->features & feature;
}

/*!
 * Init function for the PC203 device family. This must be called by
 * picoif_main to allow the PC203 devices (if any) to be detected.
 *
 * @return Returns zero on success, negative on failure.
 */
int
pc203_init( void );

/*!
 * Init function for the PC202 device family. This must be called by
 * picoif_main to allow the PC202 devices (if any) to be detected.
 *
 * @return Returns zero on success, negative on failure.
 */
int
pc202_init( void );

/*!
 * Init function for the PC302 device family. This must be called by
 * picoif_main to allow the PC302 devices (if any) to be detected.
 *
 * @return Returns zero on success, negative on failure.
 */
int
pc302_init( void );

#endif /* !__PICOIF_PICOARRAY_H__ */
