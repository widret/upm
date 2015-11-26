%module pyupm_lps25h
%include "../upm.i"

%feature("autodoc", "3");

#ifdef DOXYGEN
%include "lps25h_doc.i"
#endif

%include "lps25h.h"
%{
    #include "lps25h.h"
%}
