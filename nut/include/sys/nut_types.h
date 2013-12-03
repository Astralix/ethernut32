#ifndef _SYS_NUT_TYPES_H_
#define _SYS_NUT_TYPES_H_

/*
 * nut-specific sys/types.h definitions that are not available on in unix sys/types.h
 */

/*!
 * \brief Unsigned register type.
 *
 * The size of this type is equal to the size of a register,
 * the hardware datapath or whatever might fit to give optimum
 * performance for values from 0 to 255.
 *
 * Typically 8 bit CPUs will use unsigned characters, 16 bit
 * CPUs will use unsigned shorts etc.
 */
#if defined(__arm__)
typedef unsigned short ureg_t;
#else
typedef unsigned short ureg_t;
#endif

/*!
 * \brief Signed register type.
 *
 * Similar to ureg_t, but for signed values from -128 to +127.
 */
#if defined(__arm__)
typedef unsigned short reg_t;
#else
typedef unsigned short reg_t;
#endif


/*!
 * \brief Unsigned pointer value type.
 *
 * For remaining MCUs GCC is assumed where __PTRDIFF_TYPE__ macro is defined
 */
typedef unsigned __PTRDIFF_TYPE__ uptr_t;

/*! \brief Void pointer */
typedef void *HANDLE;


#endif
