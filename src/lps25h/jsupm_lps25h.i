%module jsupm_lps25h
%include "../upm.i"
%include "cpointer.i"

%pointer_functions(float, floatp);

%include "lps25h.h"
%{
    #include "lps25h.h"
%}

