#ifndef BITMANIP_H
#define BITMANIP_H
/**
 * @file  bitmanip.h
 * @brief Macros for bit and bitfields manipulation
 * @note  Small cases in macro names to improve readability
 *
 * BIT(N)                Creates a bit mask with only the bit N set
 * BITFIELD(V,N)         Creates a bit mask with value V at position N
 * BITFIELDMASK(M,N)     Creates a bit mask with bits between M and N (inclusive) set to 1

 * Operations
 * BitSet(V,M)           Set all bits in V with corresponding bits set in M
 * BitClear(V,M)         Clears all bits in V with corresponding bits set in M
 * BitToggle(V,M)        Toggles all bits in V with corresponding bits set in M
 *
 * Bitfield manipulation
 * BitFieldSet(V,M,X)    Clear bits in V which corresponds to bits set in M and set them according X
 * BitFieldClear(V,M)    Clear bits in V which corresponds to bits set in M
 */

///@{
#define BIT(N)                     (1UL<<(N))
#define BITFIELD(V,N)              ((V)<<(N))
#define BITFIELDMASK(M,N)          ((BIT((M)-(N)+1)-1)<<(N))

#define BitSet(V,M)                (V)|=(M)
#define BitClear(V,M)              (V)&=~(M)
#define BitToggle(V,M)             (V)^=(M)

#define BitFieldSet(VAR,MASK,VAL)  (VAR) = ((VAR)&~(MASK))|(VAL)
#define BitFieldClear(VAR,MASK)    (VAR) &= ~(MASK)

///@}
#endif // BITMANIP_H
