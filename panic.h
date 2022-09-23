// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2 -*-
#if !defined(PANIC_H)
#    define PANIC_H

void panic0_(char const *file, int line);
#    define panic0() panic0_(__FILE__, __LINE__)

#endif // defined(PANIC_H)
