#ifndef _ELF_RANDOMIZE_H
#define _ELF_RANDOMIZE_H

struct mm_struct;

#ifndef CONFIG_ARCH_HAS_ELF_RANDOMIZE
static inline unsigned long arch_mmap_rnd(void) { return 0; }
# if defined(arch_randomize_brk) && defined(CONFIG_COMPAT_BRK)
#  define compat_brk_randomized
# endif
# ifndef arch_randomize_brk
#  define arch_randomize_brk(mm)	(mm->brk)
#  define arch_randomize_brk_with_stack(mm, s)	(mm->brk)
# endif
#else
extern unsigned long arch_mmap_rnd(void);
# if defined(arch_randomize_brk_with_stack)
extern unsigned long arch_randomize_brk_with_stack(struct mm_struct *mm,
        unsigned long start_stack);
# define arch_randomize_brk(mm) arch_randomize_brk_with_stack(mm, 0)
# else
extern unsigned long arch_randomize_brk(struct mm_struct *mm);
#define arch_randomize_brk_with_stack(mm, s) arch_randomize_brk(mm)
#  ifdef CONFIG_COMPAT_BRK
#   define compat_brk_randomized
#  endif
# endif
#endif

#endif
