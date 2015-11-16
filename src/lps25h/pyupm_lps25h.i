%module pyupm_lps25h
%include "../upm.i"
%include "cpointer.i"

%include "stdint.i"

%feature("autodoc", "3");

%pointer_functions(float, floatp);

%include "lps25h.h"
%{
    #include "lps25h.h"
%}

