# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Build the cppad/local/is_pod.hpp file.
# -----------------------------------------------------------------------------
# is_pod_main
SET(is_pod_main "
    int main(void) { return 0; }"
)
# is_pod_default
SET(is_pod_default
    "template <class T> inline bool is_pod(void) { return false; }"
)
# -----------------------------------------------------------------------------
# is_pod_specialize_98
SET(is_pod_specialize_98 "")
SET(pod_type_list "bool" "char" "wchar_t" "float" "double" "long double")
FOREACH(base_type "char" "short" "int" "long" )
    FOREACH(sign signed unsigned)
        add_to_list(pod_type_list "${sign} ${base_type}")
    ENDFOREACH(sign signed unsigned)
ENDFOREACH(base_type)
#
SET(CMAKE_REQUIRED_DERINITIONS "")
SET(CMAKE_REQUIRED_INCLUDES    "")
SET(CMAKE_REQUIRED_LIBRARIES   "")
SET(CMAKE_REQUIRED_FLAGS       "")
FOREACH(pod_type ${pod_type_list})
    STRING(REPLACE " " "_" pod_type_var "${pod_type}")
    SET(pod_type_var "pod_type_${pod_type_var}")
    SET(line "
    template <> inline bool is_pod<${pod_type}>(void) { return true; }"
    )
    SET(source
        "${is_pod_default} ${is_pod_specialize_98} ${line} ${is_pod_main}"
    )
    IF( DEFINED ${pod_type_var} )
        MESSAGE(ERROR "${pod_type_var} is defined before expected")
    ENDIF( DEFINED ${pod_type_var} )
    CHECK_CXX_SOURCE_RUNS("${source}" ${pod_type_var} )
    IF( ${pod_type_var} )
        SET(is_pod_specialize_98 "${is_pod_specialize_98}${line}")
    ENDIF( ${pod_type_var} )
ENDFOREACH(pod_type)
# -----------------------------------------------------------------------------
# is_pod_specialize_11
SET(is_pod_specialize_11 "")
SET(pod_type_list "char16_t" "char32_t")
FOREACH(base_type "long long" )
    FOREACH(sign signed unsigned)
        add_to_list(pod_type_list "${sign} ${base_type}")
    ENDFOREACH(sign signed unsigned)
ENDFOREACH(base_type)
#
FOREACH(pod_type ${pod_type_list})
    STRING(REPLACE " " "_" pod_type_var "${pod_type}")
    SET(pod_type_var "pod_type_${pod_type_var}")
    SET(line "
    template <> inline bool is_pod<${pod_type}>(void) { return true; }"
    )
    SET(source
        "${is_pod_default} ${is_pod_specialize_98} ${is_pod_specialize_11}
        ${line} ${is_pod_main}"
    )
    IF( DEFINED ${pod_type_var} )
        MESSAGE(ERROR "{pod_type_var} is defined before expected")
    ENDIF( DEFINED ${pod_type_var} )
    CHECK_CXX_SOURCE_RUNS("${source}" ${pod_type_var} )
    IF( ${pod_type_var} )
        SET(is_pod_specialize_11 "${is_pod_specialize_11}${line}")
    ENDIF( ${pod_type_var} )
ENDFOREACH(pod_type)
# -----------------------------------------------------------------------------
# is_pod.hpp
CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/is_pod.hpp.in
    ${CMAKE_CURRENT_SOURCE_DIR}/is_pod.hpp
)
